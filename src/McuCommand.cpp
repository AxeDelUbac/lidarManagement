#include "McuCommand.hpp"

static bool angleInZone(float angle, float zoneStart, float zoneEnd)
{
    if (zoneStart <= zoneEnd)
        return angle >= zoneStart && angle <= zoneEnd;
    // zone qui croise 0° (ex: 315°–45°)
    return angle >= zoneStart || angle <= zoneEnd;
}

static bool obstacleInZone(const std::vector<Obstacle>& obstacles,
                           float zoneStart, float zoneEnd)
{
    for (const auto& o : obstacles)
        if (angleInZone(o.angle_start_deg, zoneStart, zoneEnd) ||
            angleInZone(o.angle_end_deg,   zoneStart, zoneEnd))
            return true;
    return false;
}

McuCmd computeCommand(const std::vector<Obstacle>& obstacles)
{
    if (obstacles.empty())
        return McuCmd::CLEAR;

    bool front = obstacleInZone(obstacles, 315.0f, 45.0f);
    bool left  = obstacleInZone(obstacles,  45.0f, 135.0f);
    bool right = obstacleInZone(obstacles, 225.0f, 315.0f);

    if (front && left && right) return McuCmd::OBSTACLE_ALL;
    if (front)                  return McuCmd::OBSTACLE_FRONT;
    if (left)                   return McuCmd::OBSTACLE_LEFT;
    if (right)                  return McuCmd::OBSTACLE_RIGHT;

    return McuCmd::CLEAR;
}
