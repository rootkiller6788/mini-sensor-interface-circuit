/**
 * @file cap_touch_detection.c
 * @brief Touch Detection: Baseline Tracking, Threshold, State Machine, Debounce
 *
 * Implements the core touch detection pipeline:
 * 1. Baseline tracking (asymmetric exponential moving average)
 * 2. Noise estimation (running variance via exponential window)
 * 3. Adaptive threshold computation (Neyman-Pearson criterion)
 * 4. Touch state machine with hysteresis debounce
 * 5. Touch pressure estimation
 * 6. Water film detection
 * 7. Glove touch identification
 *
 * Knowledge Coverage: L1-L6 complete for touch detection algorithms.
 *
 * The asymmetric EMA baseline tracker is the most critical algorithm.
 * It must track slow environmental drift (temperature coefficient of
 * FR4 ≈ 300 ppm/degC, X7R capacitors ≈ ±15% over -55 to +125 degC)
 * while ignoring the fast capacitance changes from touch events.
 *
 * Ref: Microchip AN1478, Cypress AN2397, Atmel AVR4013
 */

#include "cap_touch_detection.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ==========================================================================
 * L1-L2: CONFIGURATION INITIALIZATION
 * ========================================================================== */

/**
 * cap_touch_detect_config_init
 *
 * Initializes touch detection configuration with proven defaults:
 *
 * Threshold: K=5 sigma (Neyman-Pearson, false-positive rate ~2.87e-7/sample)
 *   For K=5 at 100 Hz scan rate: ~1 false touch per 9.7 hours.
 *   For K=6: ~1 false touch per 122,000 hours (~14 years).
 *
 * Debounce: 3 samples (30 ms at 100 Hz) — prevents noise spikes
 *   from triggering false touches.
 *
 * Release debounce: 2 samples (20 ms) — prevents flicker during
 *   finger removal when deltaC crosses threshold multiple times.
 *
 * Adaptive threshold: enabled, range [5 fF, 50 pF].
 *   Prevents threshold from drifting to zero (falsing on noise)
 *   or growing too large (missing real touches).
 */
void cap_touch_detect_config_init(cap_touch_detect_config_t *cfg)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->touch_threshold_f = 100.0e-15;      /* 100 fF */
    cfg->release_threshold_f = 70.0e-15;      /* 70 fF (30% hysteresis) */
    cfg->proximity_threshold_f = 10.0e-15;    /* 10 fF */
    cfg->noise_sigma_f = 20.0e-15;            /* 20 fF initial estimate */
    cfg->threshold_sigma_mult = 5.0;          /* K=5, false-touch ~1e-7 */
    cfg->debounce_samples = 3;
    cfg->release_debounce = 2;
    cfg->fast_debounce_samples = 2;           /* Faster after first touch */
    cfg->adaptive_threshold = true;
    cfg->max_threshold_f = 50.0e-12;          /* 50 pF max */
    cfg->min_threshold_f = 5.0e-15;           /* 5 fF min */
}

/**
 * cap_baseline_tracker_config_init
 *
 * Initializes baseline tracker with asymmetric EMA coefficients.
 *
 * alpha_fast = 0.05: Time constant ~20 samples.
 *   For 100 Hz scan: baseline corrects within 200 ms after touch release.
 *
 * alpha_slow = 0.01: Time constant ~100 samples.
 *   Prevents baseline from tracking the finger during slow approaches.
 *
 * alpha_init = 0.2: Fast initial settling during calibration.
 *
 * max_drift = 10 fF/scan: Prevents sudden baseline jumps from interference.
 *
 * The asymmetry is critical: if alpha_slow ≈ alpha_fast, the baseline
 * will partially track the finger, reducing deltaC and potentially
 * causing touch loss during prolonged holds.
 *
 * Ref: Kim et al. (2011) "Adaptive Baseline Update for Capacitive Touch
 *      Sensors" IEEE Sensors Journal
 */
void cap_baseline_tracker_config_init(cap_baseline_tracker_config_t *cfg)
{
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->ema_alpha_slow = 0.01;
    cfg->ema_alpha_fast = 0.05;
    cfg->ema_alpha_init = 0.20;
    cfg->max_drift_per_scan_f = 10.0e-15;    /* 10 fF per scan */
    cfg->init_settle_samples = 50;            /* 500 ms at 100 Hz */
    cfg->baseline_frozen = false;
    cfg->freeze_delay_samples = 5;            /* Freeze 50 ms after touch */
}

/* ==========================================================================
 * L3: BASELINE TRACKING — ASYMMETRIC EMA
 * ========================================================================== */

/**
 * cap_baseline_update_ema
 *
 * Updates the baseline using an asymmetric exponential moving average:
 *
 *   if raw > baseline (possible touch/pickup):
 *     baseline[n] = baseline[n-1] + alpha_slow * (raw - baseline[n-1])
 *   else (raw <= baseline, no touch):
 *     baseline[n] = baseline[n-1] + alpha_fast * (raw - baseline[n-1])
 *
 * With touch_active==true, the baseline is frozen (no update) to prevent
 * the baseline from "eating" the touch signal.
 *
 * The EMA is equivalent to a first-order IIR low-pass filter:
 *   H(z) = alpha / (1 - (1-alpha)*z^{-1})
 *   Time constant: tau = -T_s / ln(1-alpha) ≈ T_s / alpha (for alpha << 1)
 *
 * For alpha=0.01, tau ≈ 100 samples.
 * For alpha=0.05, tau ≈ 20 samples.
 *
 * Complexity: O(1) — single multiply-add per update.
 *
 * @param baseline      Current baseline value (updated in-place)
 * @param raw           New raw measurement
 * @param cfg           Baseline tracker configuration
 * @param touch_active  If true, baseline is frozen
 * @return Updated baseline value
 */
uint32_t cap_baseline_update_ema(uint32_t baseline, uint32_t raw,
                                 const cap_baseline_tracker_config_t *cfg,
                                 bool touch_active)
{
    if (!cfg) return baseline;

    /* Freeze baseline during active touch */
    if (touch_active && cfg->baseline_frozen) {
        return baseline;
    }

    double alpha;
    if (touch_active) {
        /* During touch, use very slow update or freeze */
        alpha = cfg->baseline_frozen ? 0.0 : cfg->ema_alpha_slow * 0.1;
    } else if (raw > baseline) {
        /* Raw above baseline: possible approaching finger, update slowly */
        alpha = cfg->ema_alpha_slow;
    } else {
        /* Raw below baseline: recovery after touch, update fast */
        alpha = cfg->ema_alpha_fast;
    }

    double new_baseline_d = (double)baseline + alpha * ((double)raw - (double)baseline);

    /* Clamp maximum drift per scan.
     * max_drift_per_scan_f is in Farads. For a typical system where
     * 1 count represents approximately 1 fF (C_ref / 2^N ≈ 1e-15 F),
     * we convert: max_drift_counts = max_drift_f / 1e-15.
     * Clamp to prevent measurement glitches from corrupting baseline. */
    double drift = new_baseline_d - (double)baseline;
    if (cfg->max_drift_per_scan_f > 0.0) {
        double max_drift_counts = cfg->max_drift_per_scan_f / 1.0e-15;
        /* Ensure at least 1 count drift is allowed */
        if (max_drift_counts < 1.0) max_drift_counts = 1.0;
        if (fabs(drift) > max_drift_counts) {
            drift = (drift > 0) ? max_drift_counts : -max_drift_counts;
        }
        new_baseline_d = (double)baseline + drift;
    }

    return (uint32_t)(new_baseline_d + 0.5);
}

/**
 * cap_compute_delta
 *
 * Converts raw and baseline readings to a signed delta.
 * The sign convention depends on sensing mode:
 *
 * Self-capacitance (SELF_CAP):
 *   Touch ADDS capacitance (finger couples to ground in parallel with C_parasitic)
 *   delta = raw - baseline  → positive = touch
 *
 * Mutual-capacitance (MUTUAL_CAP):
 *   Touch REDUCES capacitance (finger diverts field from TX to ground)
 *   delta = baseline - raw  → positive = touch
 *
 * Absolute mode: same as self-cap for consistency.
 */
int32_t cap_compute_delta(uint32_t raw, uint32_t baseline, cap_sense_mode_t mode)
{
    switch (mode) {
    case CAP_SENSE_MUTUAL:
        return (int32_t)(baseline - raw);
    case CAP_SENSE_SELF:
    case CAP_SENSE_ABSOLUTE:
    default:
        return (int32_t)(raw - baseline);
    }
}

/**
 * cap_delta_count_to_farad
 *
 * Converts a delta count (integer from ADC/counter) to capacitance in Farads.
 *
 * The transfer function depends on the measurement method:
 *
 * For charge transfer with N-bit effective resolution:
 *   deltaC = delta_count * C_ref / (2^N * gain)
 *
 * For a system with C_ref=10pF, N=16, gain=1:
 *   LSB = 10e-12 / 65536 ≈ 0.153 fF
 *
 * For N=24 (sigma-delta CDC):
 *   LSB = 10e-12 / 16777216 ≈ 0.0006 fF (0.6 aF)
 */
double cap_delta_count_to_farad(int32_t delta_count, double c_ref_f,
                                uint8_t resolution_bits, double gain)
{
    if (resolution_bits == 0 || resolution_bits > 32) return 0.0;
    if (c_ref_f <= 0.0 || gain <= 0.0) return 0.0;

    double lsb = c_ref_f / ((double)(1ULL << resolution_bits));
    return (double)delta_count * lsb / gain;
}

/* ==========================================================================
 * L5: ADAPTIVE THRESHOLD AND NOISE ESTIMATION
 * ========================================================================== */

/**
 * cap_adaptive_threshold
 *
 * Computes touch detection threshold using the Neyman-Pearson criterion:
 *
 *   threshold = K * sigma_noise
 *
 * where K is chosen to achieve a desired constant false-alarm rate (CFAR).
 *
 * For Gaussian noise (valid when averaging >= 10 samples per measurement):
 *   P_false = 1 - Phi(K) ≈ exp(-K^2/2) / (K*sqrt(2*pi))  (for large K)
 *
 *   K=3 → P_false ≈ 1.35e-3 per sample
 *   K=4 → P_false ≈ 3.17e-5 per sample
 *   K=5 → P_false ≈ 2.87e-7 per sample
 *   K=6 → P_false ≈ 9.87e-10 per sample
 *
 * The threshold is clamped to [min_threshold, max_threshold] to prevent
 * pathological behavior when noise estimate is extreme.
 *
 * Ref: Neyman & Pearson (1933) "On the Problem of the Most Efficient Tests
 *      of Statistical Hypotheses" Phil Trans Roy Soc A, 231, 289-337
 */
double cap_adaptive_threshold(double noise_rms_f, double sigma_mult,
                              double min_thresh_f, double max_thresh_f)
{
    if (noise_rms_f < 0.0) return min_thresh_f;
    if (sigma_mult <= 0.0) sigma_mult = 5.0;

    double thresh = sigma_mult * noise_rms_f;
    if (thresh < min_thresh_f) return min_thresh_f;
    if (thresh > max_thresh_f) return max_thresh_f;
    return thresh;
}

/**
 * cap_update_noise_estimate
 *
 * Maintains running estimates of the signal mean and variance using
 * exponential windows:
 *
 *   mu[n]     = beta * x[n] + (1-beta) * mu[n-1]
 *   sigma2[n] = alpha * (x[n]-mu[n])^2 + (1-alpha) * sigma2[n-1]
 *
 * This is an IIR implementation of:
 *   mu = E[x] (exponentially weighted)
 *   sigma2 = E[(x-mu)^2] (exponentially weighted)
 *
 * alpha_var (typ 0.01-0.05): controls noise estimate responsiveness.
 *   Higher = faster response to changing noise, but noisier estimate.
 * alpha_mean (typ 0.001-0.01): should be slower than alpha_var to prevent
 *   the mean from tracking signal variations that should be counted as noise.
 *
 * This replaces the need for a large sample buffer (common in naive
 * implementations that store 100+ samples for variance computation).
 *
 * Complexity: O(1) per sample, constant memory.
 */
void cap_update_noise_estimate(double *current_noise_var, double *current_mean,
                               double new_sample, double alpha_var,
                               double alpha_mean)
{
    if (!current_noise_var || !current_mean) return;

    /* Update mean estimate */
    *current_mean = alpha_mean * new_sample + (1.0 - alpha_mean) * (*current_mean);

    /* Update variance estimate */
    double diff = new_sample - (*current_mean);
    double instant_var = diff * diff;
    *current_noise_var = alpha_var * instant_var + (1.0 - alpha_var) * (*current_noise_var);
}

/* ==========================================================================
 * L6: TOUCH STATE MACHINE
 * ========================================================================== */

/**
 * cap_touch_state_machine
 *
 * Implements the touch detection state machine with debounce and hysteresis.
 *
 * State transitions:
 *
 *   IDLE ──(deltaC > touch_thresh for N samples)──> DETECT
 *   DETECT ──(immediate if confirmed)──> ACTIVE
 *   DETECT ──(deltaC < touch_thresh before N)──> IDLE (rejected)
 *   ACTIVE ──(deltaC < release_thresh for M samples)──> RELEASE
 *   RELEASE ──> IDLE
 *   ACTIVE ──(duration > hold_time)──> HOLD
 *   HOLD ──(deltaC < release_thresh for M)──> RELEASE
 *
 * The proximity zone (PROX_ZONE) is handled outside this state machine
 * in the proximity module, but touch state responds to it by entering
 * APPROACH when transitioning from FAR to NEAR/CONTACT with deltaC rising.
 *
 * Debounce: A touch must persist for N consecutive scans above threshold
 * before being accepted. This eliminates false triggers from:
 * - ESD events (typically < 1 us)
 * - Switching noise spikes (typically < 1 ms)
 * - mains-coupled glitches (impulsive, not sustained)
 *
 * Release debounce: M consecutive scans below release_threshold confirms
 * release. This prevents flicker when the finger is being removed slowly
 * and deltaC oscillates around the threshold.
 *
 * @param chan            Channel state
 * @param delta_c_f       Current delta capacitance [F] (positive = touch direction)
 * @param config          Detection configuration
 * @param event           Output: populated on state change
 * @param current_time_ms System time [ms]
 * @return New touch state
 */
touch_state_t cap_touch_state_machine(cap_sensor_channel_t *chan,
                                      double delta_c_f,
                                      const cap_touch_detect_config_t *config,
                                      cap_touch_event_t *event,
                                      uint32_t current_time_ms)
{
    if (!chan || !config) return TOUCH_IDLE;

    touch_state_t prev_state = chan->state;
    touch_state_t new_state = prev_state;
    bool state_changed = false;
    uint8_t db_needed = config->debounce_samples;

    /* Use faster debounce if we already had a touch recently */
    if (prev_state == TOUCH_DETECT || prev_state == TOUCH_ACTIVE ||
        prev_state == TOUCH_HOLD) {
        db_needed = config->fast_debounce_samples;
    }

    switch (prev_state) {
    case TOUCH_IDLE:
        if (delta_c_f > config->touch_threshold_f) {
            chan->debounce_count++;
            if (chan->debounce_count >= db_needed) {
                new_state = TOUCH_DETECT;
                chan->debounce_count = 0;
                state_changed = true;
            }
        } else {
            chan->debounce_count = 0;
        }
        break;

    case TOUCH_DETECT:
        /* Confirm touch is still present before activating */
        if (delta_c_f > config->touch_threshold_f) {
            new_state = TOUCH_ACTIVE;
        } else {
            /* False detection - noise spike, return to idle */
            new_state = TOUCH_IDLE;
        }
        state_changed = true;
        break;

    case TOUCH_ACTIVE:
        if (delta_c_f < config->release_threshold_f) {
            chan->debounce_count++;
            if (chan->debounce_count >= config->release_debounce) {
                new_state = TOUCH_RELEASE;
                chan->debounce_count = 0;
                state_changed = true;
            }
        } else {
            chan->debounce_count = 0;
            /* Check for long-press -> hold transition */
            if ((current_time_ms - chan->state_entry_ms) > 500) {
                new_state = TOUCH_HOLD;
                if (new_state != prev_state) state_changed = true;
            }
        }
        break;

    case TOUCH_HOLD:
        if (delta_c_f < config->release_threshold_f) {
            chan->debounce_count++;
            if (chan->debounce_count >= config->release_debounce) {
                new_state = TOUCH_RELEASE;
                chan->debounce_count = 0;
                state_changed = true;
            }
        } else {
            chan->debounce_count = 0;
        }
        break;

    case TOUCH_RELEASE:
        new_state = TOUCH_IDLE;
        state_changed = true;
        break;

    case TOUCH_APPROACH:
        if (delta_c_f > config->touch_threshold_f) {
            new_state = TOUCH_DETECT;
            state_changed = true;
        } else if (delta_c_f < config->proximity_threshold_f) {
            new_state = TOUCH_IDLE;
            state_changed = true;
        }
        break;

    default:
        break;
    }

    /* Populate event on state change */
    if (state_changed && event) {
        event->channel_id = chan->channel_id;
        event->event_type = new_state;
        event->timestamp_ms = current_time_ms;
        event->snr_at_detect_db = chan->self_meas.snr_db;
        event->peak_delta_c_f = delta_c_f;
        event->duration_ms = 0.0;

        if (new_state == TOUCH_RELEASE) {
            event->duration_ms = (double)(current_time_ms - chan->state_entry_ms);
        }
        event->touch_pressure_est = 0.0;
    }

    /* Update state */
    if (state_changed) {
        chan->state = new_state;
        chan->state_entry_ms = current_time_ms;
    }

    return chan->state;
}

/* ==========================================================================
 * L6: TOUCH PRESSURE AND ENVIRONMENTAL DETECTION
 * ========================================================================== */

/**
 * cap_estimate_touch_pressure
 *
 * Estimates normalized touch pressure [0-1] from the delta capacitance.
 *
 * Model: As finger pressure increases, the contact area increases due to
 * flesh deformation in a saturating manner:
 *
 *   A(P) = A_max * (1 - exp(-P/P0))
 *   deltaC(P) = C_max * (1 - exp(-P/P0))
 *
 * Inverting: P/P0 = -ln(1 - deltaC/C_max)
 * Pressure_est = -ln(1 - deltaC/C_max) / -ln(1 - C_ref/C_max)
 *
 * Where C_ref is a reference deltaC for normalization (e.g., deltaC at
 * 100g force). This is unitless [0,1] representing relative pressure.
 *
 * For a typical smartphone touchscreen: 50g light touch → ~30% deltaC_max,
 * 300g firm press → ~90% deltaC_max.
 *
 * @param delta_c_f      Measured delta capacitance [F]
 * @param c_max_f        Saturation deltaC at very high pressure [F]
 * @param pressure_ref_f Reference deltaC for 1.0 pressure [F]
 * @return Estimated pressure [0, 1]
 */
double cap_estimate_touch_pressure(double delta_c_f, double c_max_f,
                                   double pressure_ref_f)
{
    if (c_max_f <= 0.0 || pressure_ref_f <= 0.0) return 0.0;
    if (delta_c_f <= 0.0) return 0.0;

    /* Prevent log of number >= 1 */
    double ratio = delta_c_f / c_max_f;
    if (ratio >= 0.999) ratio = 0.999;

    double raw_pressure = -log(1.0 - ratio);

    /* Normalize to reference */
    double ref_ratio = pressure_ref_f / c_max_f;
    if (ref_ratio >= 0.999) ref_ratio = 0.999;
    double ref_pressure = -log(1.0 - ref_ratio);

    if (ref_pressure <= 0.0) return 0.0;
    double normalized = raw_pressure / ref_pressure;
    if (normalized > 1.0) normalized = 1.0;
    return normalized;
}

/**
 * cap_detect_water_film
 *
 * Detects the presence of water or conductive film on the touch surface.
 *
 * Physical principle:
 *   Water (eps_r ≈ 80) has much higher permittivity than air (eps_r ≈ 1)
 *   or finger tissue (eps_r ≈ 40-50). A water film on the sensor:
 *   1. Increases mutual capacitance (C_mutual RISES because water couples
 *      TX to RX more strongly than air)
 *   2. Increases self-capacitance (C_self RISES because water is a
 *      conductor near the electrode)
 *
 *   A finger touch has the OPPOSITE effect on mutual capacitance:
 *   1. Decreases mutual capacitance (finger diverts field to ground)
 *   2. Increases self-capacitance (finger couples to earth)
 *
 * Decision logic:
 *   If deltaC_self > water_thresh_self AND deltaC_mutual > water_thresh_mut:
 *     → WATER (both increase, mutual increases = not a finger)
 *   If deltaC_self > water_thresh_self AND deltaC_mutual < -water_thresh_mut:
 *     → FINGER (self increases, mutual decreases = finger diversion)
 *
 * This discrimination enables "wet finger tracking" and "water rejection"
 * modes in modern touchscreen controllers.
 *
 * @return true if water/film is detected
 */
bool cap_detect_water_film(double delta_self, double delta_mutual,
                           double water_thresh_self, double water_thresh_mut)
{
    /* Both self and mutual increase = conductive film coupling */
    return (delta_self > water_thresh_self) && (delta_mutual > water_thresh_mut);
}

/**
 * cap_glove_touch_confidence
 *
 * Estimates confidence that a touch is through a glove.
 *
 * Glove touches have distinctive characteristics:
 * 1. Lower deltaC: glove adds extra dielectric layer, reducing
 *    finger-to-electrode coupling. Typical reduction: 50-90%.
 *    (Thick winter glove: 10-20% of bare finger deltaC)
 *    (Thin medical glove: 60-80% of bare finger)
 * 2. Slower rise time: glove material has lower conductivity and
 *    adds series impedance. RC time constant increases.
 *
 * The confidence is a heuristic combining both metrics:
 *
 *   conf_delta = clamp(1 - (ratio/expected_min), 0, 1)
 *   conf_rise  = clamp((rise_ratio - 1)/(max_rise - 1), 0, 1)
 *   confidence  = 0.6 * conf_delta + 0.4 * conf_rise
 *
 * where ratio = deltaC_measured / deltaC_bare
 *       rise_ratio = rise_time_measured / rise_time_bare
 *
 * @return Confidence [0, 1]: 0 = definitely bare, 1 = definitely gloved
 */
double cap_glove_touch_confidence(double delta_c_f, double expected_bare_f,
                                  double rise_time_ms, double bare_rise_ms)
{
    if (expected_bare_f <= 0.0 || bare_rise_ms <= 0.0) return 0.0;

    /* Amplitude ratio: lower ratio = more likely glove */
    double amp_ratio = delta_c_f / expected_bare_f;
    if (amp_ratio > 1.0) amp_ratio = 1.0;
    double conf_amp = 1.0 - amp_ratio;
    if (conf_amp < 0.0) conf_amp = 0.0;

    /* Rise time ratio: larger ratio = more likely glove */
    double rise_ratio = rise_time_ms / bare_rise_ms;
    if (rise_ratio < 1.0) rise_ratio = 1.0;
    double conf_rise = (rise_ratio - 1.0) / 3.0; /* 3x slower = fully gloved */
    if (conf_rise > 1.0) conf_rise = 1.0;
    if (conf_rise < 0.0) conf_rise = 0.0;

    return 0.6 * conf_amp + 0.4 * conf_rise;
}

/* ==========================================================================
 * L2: STATISTICS
 * ========================================================================== */

/**
 * cap_touch_stats_reset
 *
 * Resets all accumulated statistics to zero.
 * Use before starting a new test run or calibration cycle.
 */
void cap_touch_stats_reset(cap_touch_stats_t *stats)
{
    if (!stats) return;
    memset(stats, 0, sizeof(*stats));
}

/**
 * cap_touch_stats_accumulate
 *
 * Accumulates a single measurement into running statistics.
 * Maintains running mean (not requiring storage of all samples):
 *
 *   mean[n] = mean[n-1] + (x[n] - mean[n-1]) / n
 *
 * This is Welford's online algorithm, which is numerically stable
 * and requires O(1) memory regardless of sample count.
 *
 * For touch statistics, we maintain separate means for touch and idle
 * states, plus track extrema (max noise, min SNR).
 */
void cap_touch_stats_accumulate(cap_touch_stats_t *stats, double delta_c_f,
                                bool is_touch, bool is_valid)
{
    if (!stats) return;

    stats->total_scans++;

    if (is_touch) {
        stats->touch_events++;
        /* Welford's online mean update */
        double n = (double)stats->touch_events;
        stats->mean_delta_c_touch_f += (delta_c_f - stats->mean_delta_c_touch_f) / n;
    } else {
        double n_idle = (double)(stats->total_scans - stats->touch_events);
        if (n_idle > 0.0) {
            stats->mean_delta_c_idle_f +=
                (delta_c_f - stats->mean_delta_c_idle_f) / n_idle;
        }
    }

    /* Track noise extrema */
    double abs_delta = fabs(delta_c_f);
    if (abs_delta > stats->max_noise_f) {
        stats->max_noise_f = abs_delta;
    }

    /* Update running mean noise (using delta from baseline as noise proxy) */
    if (stats->total_scans == 1) {
        stats->mean_noise_f = abs_delta;
    } else {
        stats->mean_noise_f += (abs_delta - stats->mean_noise_f) / (double)stats->total_scans;
    }

    /* False positive: detected touch but not valid */
    if (is_touch && !is_valid) {
        stats->false_positives++;
    }
    /* False negative: not detected but is valid touch */
    if (!is_touch && is_valid) {
        stats->false_negatives++;
    }
}
