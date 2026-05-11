#pragma once

#include <vector>
#include "rplidar.h"

struct Obstacle {
    float angle_start_deg;
    float angle_end_deg;
    float dist_avg_mm;
    int   point_count;
};

// Regroupe les points du scan en obstacles par clustering angulaire.
// dist_tolerance_mm : écart max entre deux points voisins pour appartenir au même cluster
// min_points        : nombre minimum de points pour valider un obstacle
std::vector<Obstacle> detectObstacles(
    rplidar_response_measurement_node_hq_t* nodes,
    size_t count,
    float dist_tolerance_mm = 200.0f,
    int   min_points        = 5,
    float max_dist_mm       = 3000.0f
);
