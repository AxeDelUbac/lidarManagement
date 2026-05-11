#include "obstacleDetection.h"
#include <cmath>

static float nodeAngle(const rplidar_response_measurement_node_hq_t& n)
{
    return n.angle_z_q14 * 90.f / 16384.f;
}

static float nodeDist(const rplidar_response_measurement_node_hq_t& n)
{
    return n.dist_mm_q2 / 4.f;
}

bool simpleObstacleDetection(int mmDistance){
    int immDistanceTreshold = 20;
    if (mmDistance < immDistanceTreshold){
        return true;
    }
    return false;
}

std::vector<Obstacle> detectObstacles(
    rplidar_response_measurement_node_hq_t* nodes,
    size_t count,
    float dist_tolerance_mm,
    int   min_points,
    float max_dist_mm)
{
    std::vector<Obstacle> obstacles;

    // --- collecte les points valides (qualité > 0, distance > 0) ---
    struct Point { float angle; float dist; };
    std::vector<Point> pts;
    pts.reserve(count);

    for (size_t i = 0; i < count; i++) {
        if (nodes[i].quality == 0) continue;
        float d = nodeDist(nodes[i]);
        if (d <= 0.f || d > max_dist_mm) continue;
        pts.push_back({ nodeAngle(nodes[i]), d });
    }

    if (pts.empty()) return obstacles;

    // --- clustering : parcours linéaire des points triés par angle ---
    // Un nouveau cluster démarre si l'écart de distance avec le point
    // précédent dépasse dist_tolerance_mm.
    int  cluster_start = 0;
    float dist_sum     = pts[0].dist;

    auto closeCluster = [&](int end_idx) {
        int n = end_idx - cluster_start + 1;
        if (n >= min_points) {
            obstacles.push_back({
                pts[cluster_start].angle,
                pts[end_idx].angle,
                dist_sum / n,
                n
            });
        }
    };

    for (int i = 1; i < (int)pts.size(); i++) {
        float gap = std::fabs(pts[i].dist - pts[i - 1].dist);
        if (gap > dist_tolerance_mm) {
            closeCluster(i - 1);
            cluster_start = i;
            dist_sum      = 0.f;
        }
        dist_sum += pts[i].dist;
    }
    closeCluster((int)pts.size() - 1);

    // --- cas jointure 359° → 0° ---
    // Si le premier et le dernier cluster sont à distance proche,
    // on les fusionne en un seul obstacle.
    if (obstacles.size() >= 2) {
        Obstacle& first = obstacles.front();
        Obstacle& last  = obstacles.back();
        float gap = std::fabs(first.dist_avg_mm - last.dist_avg_mm);
        if (gap < dist_tolerance_mm) {
            int total = first.point_count + last.point_count;
            first.angle_start_deg = last.angle_start_deg;  // commence avant 0°
            first.dist_avg_mm = (first.dist_avg_mm * first.point_count +
                                 last.dist_avg_mm  * last.point_count) / total;
            first.point_count = total;
            obstacles.pop_back();
        }
    }

    return obstacles;
}
