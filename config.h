#ifndef CONFIG_H
#define CONFIG_H

#include <QFont>
#include <set>

// Height and width of the virtual viewport moving along the trajectory, in pixels
#define CAMERA_HEIGHT 600
#define CAMERA_WIDTH 800

// Distance of anchor to lower POI border, in percent of camera height
#define ANCHOR_DIST_PERCENT 0.1

// Distance (in meters) that the trajectory is being sampled at. At each
// INTERPOLATION_STEP meters, conflicting intervals will be re-computed.
#define INTERPOLATION_STEP 1.0

// Determines which OpenStreetMap POIs are used for labels. See config.cpp for details.
extern std::map<std::string, std::set<std::string>> POI_DEFAULT;

// Font to be used for the labels, and therefore for bounding box computation
// set in config.cpp
extern const QFont LABEL_FONT;

// Number of seconds that the "car" should (at the current assumed speed) need to
// cross the whole display, i.e., before the point that is currently at the upper
// end of the screen is moved out of the screen, if the car moved straight ahead.
// This determines the desired level of zoom for a certain (assumed) driving speed
#define TIME_VISIBLE_SECONDS 60.0

// The speed at which zooming in or out happens when the zoom level changes
#define ZOOM_PER_METER 0.008

// Maximum number of vertices / edges in the conflict graph before the graph gets
// 'too large' and graph-based heuristics are not run
#define MAX_VERTICES 100000000
#define MAX_EDGES 100000000

// Set to false to skip computing very large ILPs
#define COMPUTE_LARGE_ILPS true
// ILP fine-tuning
#define PRESOLVE_ITERATIONS 15

// Maximum time (in seconds) that the ILP should run on every instance.
// configured in config.cpp
extern int TIME_LIMIT;

/**********************************************************
 *
 *     No changes should be necessary below this line
 *
 **********************************************************/

// Only used for debugging purposes
#define CHECK_STEP 5.0
#undef ENABLE_DEBUG
#undef CONSISTENCY_CHECKS
#undef OMGCHECK
// Salmen
#define DBG_POI_1 96039171
// Aposto
#define DBG_POI_2 746786773

#define HEU_ILP_AM1_TAG 0
#define HEU_GREEDY_AM1_TAG 1
#define HEU_GREEDY_AM1_COMPOUND_TAG 9
#define HEU_IG_AM1_TAG 2

#define HEU_ILP_AM2_TAG 3
#define HEU_GREEDY_AM2_TAG 4
#define HEU_IG_AM2_TAG 5

#define HEU_ILP_AM3_TAG 6
#define HEU_GREEDY_AM3_TAG 7
#define HEU_IG_AM3_TAG 8



#define OVERSCAN_FACTOR 2.5

// Determings which OpenStreetMap POIs are used for labels
// configuration variables
extern int num_threads;

// double-comparison delta
#define DELTA 0.0000001

// Options controlling minimum selection length
#define MINIMUM_SELECTION_LENGTH 33.0

/* Options controlling map drawing */
#include <QColor>
#define ROAD_STYLE_COUNT 4
typedef struct st_road_style {
  double outer_width;
  double inner_width;
  QColor outer_color;
  QColor inner_color;
} road_style;
extern double roadstyle_speeds[];
extern road_style roadstyle_styles[];
extern QColor poi_box_background;
extern QColor poi_box_outline;
extern double poi_box_dx;
extern double poi_box_dy;
extern double poi_box_dw;

#endif // CONFIG_H
