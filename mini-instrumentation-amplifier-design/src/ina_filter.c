/**
 * @file ina_filter.c
 * @brief Filtering Implementation for IA Signal Chains
 *
 * Implements anti-aliasing filter design, RFI/EMI filtering,
 * noise bandwidth analysis, Sallen-Key analog filter design,
 * and notch filter design.
 *
 * Reference:
 *   Zumbahlen, "Linear Circuit Design Handbook" (ADI, 2008)
 *   Kester, "Data Conversion Handbook" (ADI, 2005)
 *   Horowitz & Hill, "The Art of Electronics" (2015, 3rd Ed.)
 */
#include "ina_filter.h"
#include <math.h>
#include <string.h>

/*===========================================================================
 * Anti-Aliasing Filter Design
 *===========================================================================*/

void ina_design_antialias_filter(double signal_bw_hz,
                                  double sampling_freq_hz,
                                  int adc_bits,
                                  FilterSpec *spec)
{
    /**
     * Design anti-aliasing filter per Nyquist criterion.
     *
     * Per the Nyquist-Shannon sampling theorem (L4):
     *   All frequency components above fs/2 must be removed
     *   before sampling to prevent aliasing.
     *
     * Design steps (L5 Algorithm):
     *   1. Target attenuation at fs/2:
     *      Atten = 6.02 * N_bits + 1.76 dB (quantization noise floor)
     *      With margin: Atten = 6.02 * N + 10 dB
     *
     *   2. Required filter order:
     *      For a Butterworth lowpass, attenuation slope is 20n dB/decade.
     *      n = ceil(Atten / (20 * log10(fs/(2*fc))))
     *
     *   3. Set cutoff frequency:
     *      fc = signal_bw_hz (or slightly higher for passband flatness)
     *
     *   4. Choose approximation:
     *      Butterworth ? flat passband, moderate roll-off
     *      Bessel ? constant group delay (good for time-domain signals)
     *      Elliptic ? steepest roll-off, but passband ripple
     *
     * Reference: Kester, Ch. 2; Oppenheim & Schafer, Ch. 4
     */
    if (!spec) return;

    memset(spec, 0, sizeof(FilterSpec));

    /* Target attenuation: at least quantization noise floor + margin */
    double atten_target = 6.02 * adc_bits + 10.0;  /* dB */

    /* Order calculation */
    double fs_over_2fc = sampling_freq_hz / (2.0 * signal_bw_hz);
    if (fs_over_2fc <= 1.0) fs_over_2fc = 2.0;

    int order = (int)ceil(atten_target / (20.0 * log10(fs_over_2fc)));
    if (order < 1) order = 1;
    if (order > 8) order = 8;  /* Practical limit for active filters */

    spec->type = FILTER_LOWPASS;
    spec->approx = FILTER_APPROX_BUTTERWORTH;
    spec->cutoff_frequency_hz = signal_bw_hz;
    spec->stopband_atten_db = atten_target;
    spec->stopband_frequency_hz = sampling_freq_hz / 2.0;
    spec->sampling_frequency_hz = sampling_freq_hz;
    spec->order = order;
    spec->passband_ripple_db = 0.0;  /* Butterworth has no ripple */
}

int ina_antialias_filter_order(double signal_bw_hz,
                                double sampling_freq_hz,
                                double required_attenuation_db)
{
    /**
     * Compute required filter order.
     *
     * For an n-th order Butterworth filter:
     *   |H(f)| = 1 / sqrt(1 + (f/fc)^(2n))
     *
     * At f = fs/2:
     *   Attenuation = 10*log10(1 + (fs/(2*fc))^(2n))
     *              ? 20*n*log10(fs/(2*fc))  (for large f/fc)
     *
     * Solving for n:
     *   n = ceil(Atten_dB / (20*log10(fs/(2*fc))))
     */
    if (signal_bw_hz <= 0.0 || sampling_freq_hz <= 0.0) return 1;

    double ratio = sampling_freq_hz / (2.0 * signal_bw_hz);
    if (ratio <= 1.0) ratio = 1.1;

    double n_exact = required_attenuation_db / (20.0 * log10(ratio));
    int n = (int)ceil(n_exact);
    if (n < 1) n = 1;
    return n;
}

double ina_aliased_frequency(double input_freq_hz, double sampling_freq_hz)
{
    /**
     * Compute the aliased frequency after sampling.
     *
     * The Nyquist theorem states that sampling at fs maps all
     * frequencies to the range [0, fs/2].
     *
     * f_alias = |f_in - n*fs|
     * where n = round(f_in / fs)
     *
     * Examples:
     *   f_in = 700 Hz, fs = 1000 Hz
     *     n = round(700/1000) = 1
     *     f_alias = |700 - 1000| = 300 Hz
     *
     *   f_in = 1200 Hz, fs = 1000 Hz
     *     n = round(1.2) = 1
     *     f_alias = |1200 - 1000| = 200 Hz
     *
     * This is why a 1200 Hz input appears as 200 Hz after
     * sampling at 1000 Hz ? aliasing!
     */
    if (sampling_freq_hz <= 0.0) return input_freq_hz;

    double n = round(input_freq_hz / sampling_freq_hz);
    double f_alias = fabs(input_freq_hz - n * sampling_freq_hz);

    if (f_alias > sampling_freq_hz / 2.0) {
        f_alias = sampling_freq_hz - f_alias;
    }

    return f_alias;
}

/*===========================================================================
 * RFI/EMI Filter Design
 *===========================================================================*/

void ina_design_rfi_filter(double signal_bw_hz,
                            double ia_bandwidth_hz,
                            double ib_max_na,
                            double allowed_dc_error_uv,
                            RfiFilter *filter)
{
    /**
     * Design input RFI/EMI filter for IA.
     *
     * RFI (Radio Frequency Interference) can cause DC offset errors
     * in IAs through input stage rectification. An input filter
     * attenuates RF before it reaches the sensitive input stage.
     *
     * Design algorithm (L5):
     *
     * 1. Select series resistors:
     *    R must be low enough that Ib * R < allowed DC error
     *    R_max = allowed_error / Ib
     *
     * 2. Select differential capacitor:
     *    C_diff = 1 / (4*pi*R*fc_diff)
     *    where fc_diff = 10 to 100 times signal BW
     *
     * 3. Select common-mode capacitors:
     *    C_cm = 1 / (2*pi*R*fc_cm)
     *    where fc_cm is typically much higher than fc_diff
     *
     * 4. Verify CMRR:
     *    Capacitor mismatch degrades CMRR.
     *    Use matched capacitors (1% or better).
     *
     * The 3-dB differential bandwidth:
     *   fc_diff = 1 / (2*pi * 2R * C_diff)
     *           = 1 / (4*pi * R * C_diff)
     *
     * The 3-dB common-mode bandwidth:
     *   fc_cm = 1 / (2*pi * R * C_cm)
     *
     * Reference: Rich, "Understanding Interference-Type Noise"
     *   Analog Dialogue 16-3 (1982)
     */
    if (!filter) return;

    memset(filter, 0, sizeof(RfiFilter));

    /* Max series resistor from Ib constraint */
    double ib = ib_max_na * 1e-9;
    double r_max = (ib > 0.0) ? (allowed_dc_error_uv * 1e-6) / ib : 100000.0;

    /* Choose reasonable value (typically 1k-10k) */
    double r = 4700.0;  /* 4.7k? is common */
    if (r > r_max) r = r_max;
    if (r < 100.0) r = 100.0;

    filter->r_series[0] = r;
    filter->r_series[1] = r;

    /* Differential cutoff: 100x signal BW */
    double fc_diff = signal_bw_hz * 100.0;
    if (fc_diff > ia_bandwidth_hz) fc_diff = ia_bandwidth_hz;

    /* C_diff = 1 / (4*pi*R*fc_diff) */
    filter->c_diff = 1.0 / (4.0 * M_PI * r * fc_diff);
    filter->differential_cutoff_hz = fc_diff;

    /* Common-mode cutoff: 10x above differential cutoff */
    double fc_cm = fc_diff * 10.0;
    filter->c_cm[0] = 1.0 / (2.0 * M_PI * r * fc_cm);
    filter->c_cm[1] = filter->c_cm[0];
    filter->common_mode_cutoff_hz = fc_cm;

    /* CMRR degradation from capacitor mismatch (1% typical) */
    double cap_mismatch = 0.01;
    filter->cmrr_degradation_db = -20.0 * log10(cap_mismatch);
}

double ina_rfi_rectification_offset(double rfi_amplitude_v,
                                     double rfi_frequency_hz,
                                     const RfiFilter *filter)
{
    /**
     * Estimate DC offset from RFI rectification in IA input stage.
     *
     * The input differential pair of an IA acts as a rectifier
     * for out-of-band RF signals. The rectified DC offset is:
     *
     * V_offset ? Vrf^2 / (2 * Vt) * |H(f_rf)|^2 / (1 + (f_rf/fc)^2)
     *
     * where:
     *   Vrf = RF amplitude at IA input
     *   Vt = thermal voltage (~26 mV at 25?C)
     *   |H(f_rf)|^2 = filter attenuation at RF frequency
     *   fc = IA input stage -3dB frequency
     *
     * This is a second-order effect: offset ? Vrf^2.
     *
     * Example: 1 Vrms RF at 100 MHz with 100 kHz diff filter,
     *   no filter: V_offset ? 1^2/(2*0.026) ? 19 V (saturates!)
     *   with filter: 1 MHz fc gives ~40 dB attenuation at 100 MHz,
     *     V_offset ? 0.25 mV (still significant)
     *
     * This is why RFI filtering is critical for IA applications
     * in electrically noisy environments.
     */
    if (!filter) return 0.0;

    const double vt = 0.026;  /* thermal voltage at 25?C */

    /* Filter attenuation at RF frequency */
    double fc = filter->differential_cutoff_hz;
    if (fc <= 0.0) fc = 1.0;
    double attenuation = 1.0 / sqrt(1.0 + pow(rfi_frequency_hz / fc, 2.0));

    double vrf_at_ia = rfi_amplitude_v * attenuation;
    double v_offset = (vrf_at_ia * vrf_at_ia) / (2.0 * vt);

    return v_offset;
}

/*===========================================================================
 * Noise Bandwidth Analysis
 *===========================================================================*/

double ina_noise_bandwidth(double cutoff_frequency_hz,
                            FilterApproximation approx,
                            int order)
{
    /**
     * Compute noise equivalent bandwidth (NEB).
     *
     * The noise bandwidth is the bandwidth of an ideal brick-wall
     * filter that passes the same total noise power as the actual filter.
     *
     * NBW = ?|H(f)|^2 df   (integrated over all frequencies)
     *
     * For an n-th order Butterworth:
     *   NBW = fc * ?/(2n) / sin(?/(2n))
     *
     * For common orders:
     *   n=1 (1st order):   NBW = fc * 1.571  (?/2)
     *   n=2 (2nd order):   NBW = fc * 1.111
     *   n=3 (3rd order):   NBW = fc * 1.047
     *   n=4 (4th order):   NBW = fc * 1.026
     *   n=8 (8th order):   NBW = fc * 1.006
     *
     * For Bessel filters, NBW is slightly wider due to
     * slower initial roll-off.
     *
     * Reference: Motchenbacher & Connelly, Ch. 11
     */
    if (order <= 0 || cutoff_frequency_hz <= 0.0) return 0.0;

    double factor;
    switch (approx) {
        case FILTER_APPROX_BUTTERWORTH:
            factor = (M_PI / (2.0 * order)) / sin(M_PI / (2.0 * order));
            break;
        case FILTER_APPROX_BESSEL:
            /* Bessel NBW is approximately 1.1 for order ? 2 */
            factor = (order == 1) ? 1.571 : 1.1;
            break;
        case FILTER_APPROX_CHEBYSHEV:
        case FILTER_APPROX_ELLIPTIC:
            /* Slightly wider due to passband ripple */
            factor = (M_PI / (2.0 * order)) / sin(M_PI / (2.0 * order));
            factor *= 1.05;  /* ~5% wider for ripple */
            break;
        default:
            factor = 1.571;  /* 1st-order fallback */
    }

    return cutoff_frequency_hz * factor;
}

double ina_noise_brickwall_factor(FilterApproximation approx, int order)
{
    /**
     * Compute brick-wall correction factor K_n.
     *
     * K_n = NBW / fc
     *
     * Total RMS noise:
     *   Vn_rms = en * sqrt(NBW) = en * sqrt(K_n * fc)
     *
     * Using K_n instead of fc directly improves noise
     * estimation accuracy, especially for low-order filters
     * where NBW >> fc.
     *
     * For 1st-order: K_1 = 1.571 (57% more noise than naive fc estimate)
     * For 2nd-order: K_2 = 1.111 (11% more)
     */
    if (order <= 0) return 1.571;
    return ina_noise_bandwidth(1.0, approx, order);  /* fc=1 gives K_n directly */
}

double ina_optimal_filter_cutoff(double signal_bandwidth_hz,
                                  double noise_corner_hz,
                                  double signal_atten_tolerance_db)
{
    /**
     * Determine optimal lowpass cutoff frequency.
     *
     * This is a trade-off optimization (L5):
     *   - Higher fc: more signal energy preserved, less distortion
     *                but more noise admitted
     *   - Lower fc: less noise, but signal may be attenuated
     *               in the passband
     *
     * For white noise dominated systems:
     *   fc_opt = signal_bw  (no benefit from narrower filtering)
     *
     * For 1/f noise dominated systems:
     *   fc_opt may be lower than signal_bw because the noise
     *   power grows as ln(f_high/f_low) rather than linearly.
     *
     * This function finds fc that maximizes SNR:
     *   SNR(fc) = Signal_at_fc / Noise_total(fc)
     *
     * Simplified: if noise corner << signal BW, use signal BW.
     * If noise corner > signal BW, reduce fc.
     */
    if (signal_bandwidth_hz <= 0.0) return 0.0;

    if (noise_corner_hz <= 0.0 || noise_corner_hz < signal_bandwidth_hz * 0.1) {
        /* White noise dominated: fc = signal BW */
        return signal_bandwidth_hz;
    }

    /* 1/f noise dominated: reduce fc */
    double attenuation_linear = pow(10.0, -signal_atten_tolerance_db / 20.0);
    /* For 1/f noise, optimal is roughly fc = signal_bw * attenuation_factor */
    double fc_opt = signal_bandwidth_hz * sqrt(attenuation_linear);

    return fc_opt;
}

/*===========================================================================
 * Sallen-Key Filter Design
 *===========================================================================*/

SallenKeyFilter ina_design_sallen_key_lowpass(double cutoff_hz,
                                                double q_factor,
                                                double c_value)
{
    /**
     * Design a Sallen-Key lowpass filter.
     *
     * The Sallen-Key topology uses a single op-amp with positive
     * feedback to create a 2nd-order lowpass response.
     *
     * Transfer function:
     *   H(s) = K / (s^2/?0^2 + s/(Q*?0) + 1)
     *
     * where:
     *   ?0 = 1/sqrt(R1*R2*C1*C2)
     *   Q = sqrt(R1*R2*C1*C2) / ((R1+R2)*C2)
     *   K = 1 + R4/R3 (DC gain, K ? 3 for stability)
     *
     * Equal-C design (C1 = C2 = C):
     *   R1 = R2 = R
     *   fc = 1/(2*pi*R*C)
     *   Q = 1/(3 - K)  ?  K = 3 - 1/Q
     *
     * For Butterworth (Q = 0.707):
     *   K = 3 - 1/0.707 = 3 - 1.414 = 1.586
     *
     * Component selection:
     *   1. Choose convenient C (e.g., 100 nF for audio, 1 nF for RF)
     *   2. Compute R: R = 1/(2*pi*fc*C)
     *   3. Set K with R3, R4: K = 1 + R4/R3
     *      For Butterworth: R4/R3 = 0.586, e.g., R3 = 10k, R4 = 5.86k
     *
     * Reference: Horowitz & Hill, ?5.4; Zumbahlen, Ch. 8
     */
    SallenKeyFilter sk;
    memset(&sk, 0, sizeof(sk));

    if (c_value <= 0.0) c_value = 1e-9;  /* default 1 nF */
    if (cutoff_hz <= 0.0) cutoff_hz = 1000.0;

    sk.c1 = c_value;
    sk.c2 = c_value;

    /* R = 1/(2*pi*fc*C) */
    double r = 1.0 / (2.0 * M_PI * cutoff_hz * c_value);
    sk.r1 = r;
    sk.r2 = r;

    /* Gain for desired Q */
    if (q_factor <= 0.0) q_factor = 0.7071;  /* Butterworth default */
    sk.gain = 3.0 - 1.0 / q_factor;
    if (sk.gain > 3.0) sk.gain = 3.0;  /* stability limit */
    if (sk.gain < 1.0) sk.gain = 1.0;

    sk.q_factor = q_factor;
    sk.f0 = cutoff_hz;

    return sk;
}

SallenKeyFilter ina_design_sallen_key_highpass(double cutoff_hz,
                                                 double q_factor,
                                                 double c_value)
{
    /**
     * Design a Sallen-Key highpass filter.
     *
     * The highpass is obtained by swapping R and C positions
     * in the lowpass Sallen-Key topology.
     *
     * Transfer function:
     *   H(s) = K * s^2 / (s^2/?0^2 + s/(Q*?0) + 1)
     *
     * With equal-R, equal-C:
     *   fc = 1/(2*pi*R*C)  (same as lowpass)
     *   Q = 1/(3 - K)      (same relationship)
     *
     * The component values are identical to the lowpass case;
     * only the topology is different.
     */
    /* Same formulas as lowpass for equal-R equal-C design */
    SallenKeyFilter sk = ina_design_sallen_key_lowpass(cutoff_hz,
                                                        q_factor, c_value);
    return sk;  /* R, C values are the same for equal-R equal-C */
}

void ina_sallen_key_response(const SallenKeyFilter *filter,
                              double frequency,
                              double *magnitude,
                              double *phase_rad)
{
    /**
     * Compute Sallen-Key frequency response at a given frequency.
     *
     * The magnitude response of a 2nd-order Sallen-Key:
     *   |H(f)| = K / sqrt( (1 - (f/fc)^2)^2 + ((f/fc)/Q)^2 )
     *
     * Phase response:
     *   ?(f) = -atan2( (f/fc)/Q, 1 - (f/fc)^2 )
     *
     * Key features:
     *   - At f = fc: |H(fc)| = K * Q (for Q > 0.707, there's peaking)
     *   - At f << fc: |H| ? K (flat passband)
     *   - At f >> fc: |H| ? K / (f/fc)^2   (40 dB/decade roll-off)
     *   - At f = fc: ? = -90? (regardless of Q)
     */
    if (!filter) {
        if (magnitude) *magnitude = 0.0;
        if (phase_rad) *phase_rad = 0.0;
        return;
    }

    if (filter->f0 <= 0.0) {
        if (magnitude) *magnitude = filter->gain;
        if (phase_rad) *phase_rad = 0.0;
        return;
    }

    double omega_ratio = frequency / filter->f0;
    double omega_ratio_sq = omega_ratio * omega_ratio;

    double denom = sqrt((1.0 - omega_ratio_sq) * (1.0 - omega_ratio_sq)
                        + (omega_ratio / filter->q_factor)
                          * (omega_ratio / filter->q_factor));

    if (magnitude) {
        *magnitude = filter->gain / denom;
    }

    if (phase_rad) {
        if (denom > 0.0) {
            *phase_rad = -atan2(omega_ratio / filter->q_factor,
                                1.0 - omega_ratio_sq);
        } else {
            *phase_rad = 0.0;
        }
    }
}

double ina_cascaded_filter_response(const SallenKeyFilter *stages,
                                     int num_stages,
                                     double frequency)
{
    /**
     * Compute overall response of cascaded Sallen-Key stages.
     *
     * For n-th order filter built from n/2 2nd-order stages:
     *   H_total(f) = ? H_i(f)
     *   |H_total(f)| = ? |H_i(f)|
     *   ?_total(f) = ? ?_i(f)
     *
     * Each stage has different Q and fc to realize the
     * desired overall response (Butterworth, Bessel, etc.).
     *
     * Butterworth pole locations for cascaded stages:
     *   4th-order: Q1 = 0.541, Q2 = 1.307
     *   6th-order: Q1 = 0.518, Q2 = 0.707, Q3 = 1.932
     *   8th-order: Q1 = 0.510, Q2 = 0.601, Q3 = 0.900, Q4 = 2.563
     */
    if (!stages || num_stages <= 0) return 0.0;

    double mag_total = 1.0;
    for (int i = 0; i < num_stages; i++) {
        double mag;
        ina_sallen_key_response(&stages[i], frequency, &mag, NULL);
        mag_total *= mag;
    }

    return mag_total;
}

/*===========================================================================
 * 50/60 Hz Notch Filter (Twin-T)
 *===========================================================================*/

TwinTNotchFilter ina_design_notch_filter(double notch_frequency_hz,
                                           double q_factor,
                                           double c_value)
{
    /**
     * Design a Twin-T notch filter for power line rejection.
     *
     * The Twin-T is a passive notch filter using two T-networks:
     *   - Series R-R with shunt 2C (first T)
     *   - Series C-C with shunt R/2 (second T)
     *
     * Notch frequency: f_notch = 1/(2*pi*R*C)
     *
     * Without bootstrapping, Q is limited to ~0.25 (broad notch).
     * With active feedback, Q can be increased to 10 or more
     * (sharp notch at the expense of component sensitivity).
     *
     * For 50/60 Hz rejection:
     *   f_notch = 50 Hz ? choose C = 100 nF ? R ? 31.83 k?
     *   (use nearest standard: 31.6k or 32.4k)
     *
     * Reference: Horowitz & Hill, ?5.6.6
     */
    TwinTNotchFilter notch;
    memset(&notch, 0, sizeof(notch));

    if (c_value <= 0.0) c_value = 0.1e-6;  /* 100 nF for audio */
    if (notch_frequency_hz <= 0.0) notch_frequency_hz = 50.0;

    notch.notch_frequency = notch_frequency_hz;
    notch.c_value = c_value;

    /* R = 1/(2*pi*f_notch*C) */
    notch.r_value = 1.0 / (2.0 * M_PI * notch_frequency_hz * c_value);

    if (q_factor <= 0.0) q_factor = 5.0;  /* Moderate Q for practical */
    notch.q_factor = q_factor;

    /* Notch depth: ideally infinite at f_notch, limited by component matching */
    notch.notch_depth_db = 60.0;  /* Typical 0.1% matching ? 60 dB */

    return notch;
}

double ina_notch_filter_response(const TwinTNotchFilter *notch,
                                  double frequency)
{
    /**
     * Compute Twin-T notch filter frequency response.
     *
     * Transfer function of ideal Twin-T:
     *   H(s) = (s^2 + ?0^2) / (s^2 + s*?0/Q + ?0^2)
     *
     * Magnitude at frequency f:
     *   |H(f)| = |1 - (f/f0)^2| / sqrt( (1 - (f/f0)^2)^2 + ((f/f0)/Q)^2 )
     *
     * At f = f0 (notch): |H(f0)| = 0 (ideal, limited by matching in practice)
     * At f << f0: |H| ? 1
     * At f >> f0: |H| ? 1
     *
     * The width of the notch is f0/Q.
     * For f0 = 50 Hz, Q = 5: notch width = 10 Hz
     */
    if (!notch || notch->notch_frequency <= 0.0) return 1.0;

    double f_ratio = frequency / notch->notch_frequency;
    double f_ratio_sq = f_ratio * f_ratio;

    double numer = fabs(1.0 - f_ratio_sq);
    double denom = sqrt((1.0 - f_ratio_sq) * (1.0 - f_ratio_sq)
                        + (f_ratio / notch->q_factor)
                          * (f_ratio / notch->q_factor));

    if (denom <= 0.0) return 0.0;

    double mag = numer / denom;

    /* Limit notch depth by practical matching */
    double min_mag = pow(10.0, -notch->notch_depth_db / 20.0);
    if (mag < min_mag) mag = min_mag;

    return mag;
}

/*===========================================================================
 * L7: Complete Signal Chain Filter Design
 *===========================================================================*/

void ina_design_signal_chain_filters(double signal_bw_hz,
                                      double sampling_freq_hz,
                                      int adc_bits,
                                      double ib_max_na,
                                      int enable_notch,
                                      InaSignalChainFilters *chain)
{
    /**
     * Design the complete filter chain for an IA-based sensor system.
     *
     * This L7 application integrates:
     *   1. Input RFI/EMI filter (protects IA from radiated interference)
     *   2. Anti-aliasing filter (prevents ADC aliasing)
     *   3. Optional notch filter (rejects 50/60 Hz power line)
     *
     * The filters must be coordinated:
     *   - RFI filter fc is much higher (10-100x) than signal BW
     *   - Anti-alias filter fc ? signal BW
     *   - Notch at 50 or 60 Hz (remove before anti-alias if possible)
     *
     * Total noise bandwidth = cascade of all filter responses.
     */
    if (!chain) return;

    memset(chain, 0, sizeof(InaSignalChainFilters));

    /* 1. RFI filter: cutoff ~100x signal BW */
    ina_design_rfi_filter(signal_bw_hz,
                          signal_bw_hz * 100.0,
                          ib_max_na,
                          10.0,  /* 10 ?V max DC error */
                          &chain->input_rfi);

    /* 2. Anti-aliasing filter */
    ina_design_antialias_filter(signal_bw_hz, sampling_freq_hz, adc_bits,
                                &chain->antialias);

    /* 3. Optional notch filter */
    if (enable_notch) {
        /* Use 50 Hz default, could be parameterized */
        chain->notch = ina_design_notch_filter(50.0, 5.0, 0.1e-6);
    }

    /* Total noise bandwidth: approximated by anti-alias filter */
    chain->total_noise_bw_hz = ina_noise_bandwidth(
        chain->antialias.cutoff_frequency_hz,
        chain->antialias.approx,
        chain->antialias.order);

    /* Total latency: group delay through all filters */
    /* RFI: ~1/(2*pi*fc_rfi) = negligible at RF frequencies */
    /* Anti-aliasing: ~order/fc (rough estimate) */
    chain->total_latency_us = (chain->antialias.order
                               / chain->antialias.cutoff_frequency_hz)
                              * 1e6;  /* Convert to ?s */
}
