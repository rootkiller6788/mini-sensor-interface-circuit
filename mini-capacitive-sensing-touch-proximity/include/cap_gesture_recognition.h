/**
 * @file cap_gesture_recognition.h
 * @brief Gesture Recognition from Capacitive Touch Trajectories
 *
 * Processes touch position/velocity/acceleration trajectories to recognize
 * gestures: tap, double-tap, swipe (4 directions), long-press, pinch,
 * rotate, and custom multi-finger gestures.
 *
 * Knowledge Coverage:
 *   L1: gesture vocabulary, velocity, acceleration, trajectory
 *   L2: finite state machines for gesture sequences, time windows
 *   L3: trajectory curvature, velocity profiles, feature extraction
 *   L4: dynamic time warping optimal alignment, minimum distance classification
 *   L5: 1-dollar recognizer, template matching, velocity-threshold segmentation
 *   L6: swipe detection on touchscreen, tap vs double-tap disambiguation
 *
 * Ref: Wobbrock et al. (2007) "$1 Recognizer" UIST
 *      Anthony & Wobbrock (2010) "$N Recognizer" CHI
 *      Rubine (1991) "Specifying Gestures by Example" SIGGRAPH
 */

#ifndef CAP_GESTURE_RECOGNITION_H
#define CAP_GESTURE_RECOGNITION_H

#include "cap_sense_core.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * L1: GESTURE VOCABULARY
 * ========================================================================== */

/** Recognized gesture types.
 *
 *  Tap:           Brief touch (< 200 ms), single contact, no movement.
 *  Double-tap:    Two taps within 500 ms on same location.
 *  Long-press:    Touch > 500 ms, no movement.
 *  Swipe-left:    Touch-move-release, horizontal motion leftward.
 *  Swipe-right:   Horizontal rightward swipe.
 *  Swipe-up:      Vertical upward swipe.
 *  Swipe-down:    Vertical downward swipe.
 *  Pinch-in:      Two fingers moving closer (multi-touch).
 *  Pinch-out:     Two fingers moving apart (multi-touch).
 *  Rotate-cw:     Two-finger clockwise rotation.
 *  Rotate-ccw:    Two-finger counter-clockwise rotation.
 *  Hold-swipe:    Hold then swipe (drag).
 */
typedef enum {
    GESTURE_NONE,
    GESTURE_TAP,
    GESTURE_DOUBLE_TAP,
    GESTURE_LONG_PRESS,
    GESTURE_SWIPE_LEFT,
    GESTURE_SWIPE_RIGHT,
    GESTURE_SWIPE_UP,
    GESTURE_SWIPE_DOWN,
    GESTURE_PINCH_IN,
    GESTURE_PINCH_OUT,
    GESTURE_ROTATE_CW,
    GESTURE_ROTATE_CCW,
    GESTURE_HOLD_SWIPE,
    GESTURE_COUNT
} cap_gesture_type_t;

/** Touch point on a 2D surface (for touchscreen/slider/wheel).
 *
 *  For 1D sensors (single slider), only x is used.
 *  For 2D touchscreens, both x and y track finger position.
 */
typedef struct {
    double   x_mm;               /**< X coordinate [mm] */
    double   y_mm;               /**< Y coordinate [mm] */
    double   pressure;           /**< Touch pressure [0-1] */
    double   area_mm2;           /**< Contact area [mm2] */
    uint32_t timestamp_ms;       /**< Sample timestamp [ms] */
} cap_touch_point_t;

/** Gesture trajectory: sequence of touch points.
 *
 *  A gesture is a time-ordered sequence of touch points with a
 *  beginning (touch-down) and end (touch-up).
 */
typedef struct {
    cap_touch_point_t points[128]; /**< Point buffer (max 128 samples) */
    uint16_t num_points;          /**< Number of recorded points */
    uint32_t start_time_ms;       /**< Gesture start timestamp [ms] */
    uint32_t end_time_ms;         /**< Gesture end timestamp [ms] */
    uint8_t  num_fingers;         /**< 1 or 2 fingers */
    double   total_path_length_mm; /**< Total travel distance [mm] */
    double   max_speed_mm_s;      /**< Peak speed [mm/s] */
    double   avg_speed_mm_s;      /**< Average speed [mm/s] */
    double   direction_deg;       /**< Principal direction of motion [deg] */
    double   bounding_box_w_mm;   /**< Trajectory extent X [mm] */
    double   bounding_box_h_mm;   /**< Trajectory extent Y [mm] */
} cap_gesture_trajectory_t;

/** Gesture recognition result.
 *
 *  Contains the recognized gesture type, confidence score, and
 *  extracted parameters (e.g., direction, magnitude).
 */
typedef struct {
    cap_gesture_type_t gesture;   /**< Recognized gesture */
    double   confidence;          /**< Recognition confidence [0-1] */
    double   param1;              /**< Gesture parameter 1 (direction deg, scale, etc.) */
    double   param2;              /**< Gesture parameter 2 */
    uint32_t timestamp_ms;        /**< Recognition timestamp [ms] */
} cap_gesture_result_t;

/* ==========================================================================
 * L5: TEMPLATE MATCHING
 * ========================================================================== */

/** Gesture template for $1 recognizer.
 *
 *  Each template is a normalized point sequence representing one
 *  gesture class. Matching uses Euclidean distance after resampling
 *  to N points, rotation to 0-degree indicative angle, scaling to
 *  reference size, and translation to origin.
 */
typedef struct {
    cap_gesture_type_t type;      /**< Gesture class */
    char    name[32];             /**< Human-readable name */
    double  points_x[64];         /**< Normalized X coordinates */
    double  points_y[64];         /**< Normalized Y coordinates */
    uint8_t num_points;           /**< Number of points in template */
    double  indicative_angle_deg; /**< Indicative angle for rotation */
} cap_gesture_template_t;

/** Gesture recognizer state.
 *
 *  Maintains the template library and ongoing gesture tracking
 *  for one or two concurrent fingers.
 */
typedef struct {
    cap_gesture_template_t templates[16]; /**< Template library */
    uint8_t  num_templates;        /**< Number of registered templates */
    cap_gesture_trajectory_t active; /**< Currently tracked trajectory */
    cap_gesture_trajectory_t prev;   /**< Previous trajectory (for double-tap) */
    bool     gesture_in_progress;    /**< Touch-down detected, tracking */
    uint32_t last_touch_up_ms;       /**< Time of last touch-up [ms] */
    double   tap_max_interval_ms;    /**< Max interval for double-tap [ms] */
    double   tap_max_displacement_mm; /**< Max displacement for tap [mm] */
    double   long_press_min_ms;      /**< Min duration for long-press [ms] */
    double   swipe_min_speed_mm_s;   /**< Min speed for swipe [mm/s] */
    double   swipe_min_distance_mm;  /**< Min distance for swipe [mm] */
} cap_gesture_recognizer_t;

/* ==========================================================================
 * L5-L6: API
 * ========================================================================== */

/** Initialize gesture recognizer with default templates.
 *
 *  Registers templates for: tap, swipe-left/right/up/down, circle.
 *  Uses $1 recognizer normalized templates.
 *
 *  @param recognizer  Recognizer to initialize
 */
void cap_gesture_recognizer_init(cap_gesture_recognizer_t *recognizer);

/** Add a custom gesture template.
 *
 *  @param recognizer Recognizer
 *  @param type       Gesture type
 *  @param name       Human-readable name
 *  @param points_x   X coordinates (raw, will be normalized)
 *  @param points_y   Y coordinates
 *  @param num_points Number of points
 *  @return Template index, or -1 if library full
 */
int cap_gesture_add_template(cap_gesture_recognizer_t *recognizer,
                             cap_gesture_type_t type, const char *name,
                             const double *points_x, const double *points_y,
                             uint8_t num_points);

/** Feed a touch point into the gesture recognizer.
 *
 *  Touch-down (finger==1 when previous was 0) starts a trajectory.
 *  Touch-move adds points.
 *  Touch-up (finger==0 when previous was 1) ends trajectory and
 *  attempts recognition.
 *
 *  @param recognizer   Recognizer state
 *  @param point        Current touch point (or NULL for no touch)
 *  @param now_ms       Current time [ms]
 *  @param result       Output: populated if gesture recognized
 *  @return true if gesture was recognized on this update
 */
bool cap_gesture_feed_point(cap_gesture_recognizer_t *recognizer,
                            const cap_touch_point_t *point,
                            uint32_t now_ms,
                            cap_gesture_result_t *result);

/** Recognize a complete gesture trajectory by template matching.
 *
 *  Implements the $1 recognizer algorithm:
 *  1. Resample to N evenly-spaced points
 *  2. Rotate to 0-degree indicative angle
 *  3. Scale to reference bounding box
 *  4. Translate centroid to origin
 *  5. Match against all templates using Euclidean distance
 *  6. Return best match with confidence score
 *
 *  @param recognizer  Recognizer with template library
 *  @param trajectory  Complete gesture to recognize
 *  @param result      Output: best match
 */
void cap_gesture_recognize_trajectory(const cap_gesture_recognizer_t *recognizer,
                                      const cap_gesture_trajectory_t *trajectory,
                                      cap_gesture_result_t *result);

/** Resample a trajectory to N evenly-spaced points.
 *
 *  Computes total path length L, then samples at intervals L/(N-1)
 *  using linear interpolation between original points.
 *
 *  @param trajectory  Trajectory to resample (modified in-place)
 *  @param n           Target number of points
 */
void cap_gesture_resample(cap_gesture_trajectory_t *trajectory, uint8_t n);

/** Compute trajectory path length.
 *
 *  @param trajectory  Trajectory
 *  @return Total path length [mm]
 */
double cap_gesture_path_length(const cap_gesture_trajectory_t *trajectory);

/** Compute centroid of trajectory points.
 *
 *  @param trajectory Trajectory
 *  @param cx         Output: centroid X [mm]
 *  @param cy         Output: centroid Y [mm]
 */
void cap_gesture_centroid(const cap_gesture_trajectory_t *trajectory,
                          double *cx, double *cy);

/** Compute indicative angle for trajectory rotation normalization.
 *
 *  Angle from centroid to first point, used by $1 recognizer to
 *  rotationally align the gesture.
 *
 *  @param trajectory  Trajectory
 *  @param cx          Centroid X
 *  @param cy          Centroid Y
 *  @return Indicative angle [deg]
 */
double cap_gesture_indicative_angle(const cap_gesture_trajectory_t *trajectory,
                                    double cx, double cy);

/** Detect tap from touch-up timing and displacement.
 *
 *  @param trajectory    Trajectory
 *  @param max_time_ms   Maximum touch duration for tap [ms]
 *  @param max_dist_mm   Maximum displacement for tap [mm]
 *  @return Confidence [0-1], 0 = not a tap
 */
double cap_gesture_detect_tap(const cap_gesture_trajectory_t *trajectory,
                              double max_time_ms, double max_dist_mm);

/** Detect swipe direction and speed.
 *
 *  Analyzes trajectory displacement and timing to classify swipe
 *  into one of 4 directions (or none if too small/slow).
 *
 *  @param trajectory        Trajectory
 *  @param min_distance_mm   Minimum displacement for swipe [mm]
 *  @param min_speed_mm_s    Minimum speed for swipe [mm/s]
 *  @param direction_deg     Output: swipe direction [deg] (0=right, 90=up, etc.)
 *  @param speed_mm_s        Output: swipe speed [mm/s]
 *  @return Confidence [0-1]
 */
double cap_gesture_detect_swipe(const cap_gesture_trajectory_t *trajectory,
                                double min_distance_mm, double min_speed_mm_s,
                                double *direction_deg, double *speed_mm_s);

/** Detect single-finger gestures (tap, long-press, swipe) from trajectory.
 *
 *  @param trajectory     Complete touch-up trajectory
 *  @param now_ms         Current time [ms]
 *  @param last_tap_ms    Time of previous tap (for double-tap) [ms]
 *  @param config_timeout Double-tap max interval [ms]
 *  @return Detected gesture
 */
cap_gesture_result_t cap_gesture_detect_single_finger(
    const cap_gesture_trajectory_t *trajectory,
    uint32_t now_ms, uint32_t last_tap_ms, double config_timeout);

/** Compute Euclidean distance between two normalized point sequences.
 *
 *  Used for $1 recognizer template matching.
 *
 *  @param pts1_x, pts1_y  First sequence
 *  @param pts2_x, pts2_y  Second sequence
 *  @param n               Number of points
 *  @param angle_deg       Rotation to apply to pts2 before comparison
 *  @return Euclidean distance (lower = better match)
 */
double cap_gesture_point_distance(const double *pts1_x, const double *pts1_y,
                                  const double *pts2_x, const double *pts2_y,
                                  uint8_t n, double angle_deg);

/** Reset gesture recognizer state.
 *
 *  @param recognizer  Recognizer to reset
 */
void cap_gesture_recognizer_reset(cap_gesture_recognizer_t *recognizer);

/** Get human-readable name for a gesture type.
 *
 *  @param gesture  Gesture type
 *  @return String name (static, do not free)
 */
const char *cap_gesture_type_name(cap_gesture_type_t gesture);

#endif /* CAP_GESTURE_RECOGNITION_H */
