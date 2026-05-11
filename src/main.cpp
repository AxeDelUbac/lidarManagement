#include <iostream>
#include "rplidar.h"
#include "obstacleDetection.h"

using namespace rp::standalone::rplidar;

int main(int argc, char* argv[])
{
    RPlidarDriver* lidarDriver = RPlidarDriver::CreateDriver();
    if (!lidarDriver) {
        std::cerr << "Impossible de créer le driver\n";
        return 1;
    }

    rplidar_response_device_info_t   info;
    rplidar_response_device_health_t health;

    lidarDriver->connect("/dev/ttyUSB1", 115200);
    lidarDriver->getDeviceInfo(info);
    lidarDriver->getHealth(health);

    printf("Modèle      : %d\n",   info.model);
    printf("Firmware    : %d.%d\n", info.firmware_version >> 8,
                                    info.firmware_version & 0xFF);
    printf("Hardware    : %d\n",   info.hardware_version);
    printf("Numéro série: ");
    for (int i = 0; i < 16; i++)
        printf("%02X", info.serialnum[i]);
    printf("\n");

    const char* statusStr = (health.status == 0) ? "OK" :
                            (health.status == 1) ? "Warning" : "Error";
    printf("Santé : %s (code erreur: %d)\n\n", statusStr, health.error_code);

    lidarDriver->startScan(false, true);

    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    while (true) {
        if (!IS_OK(lidarDriver->grabScanDataHq(nodes, count)))
            continue;

        lidarDriver->ascendScanData(nodes, count);

        auto obstacles = detectObstacles(nodes, count, 200.0f, 5, 500.0f);

        printf("\033[2J\033[H");  // efface le terminal
        printf("%zu obstacle(s) détecté(s) :\n\n", obstacles.size());
        for (int i = 0; i < (int)obstacles.size(); i++) {
            const auto& o = obstacles[i];
            printf("  [%d] angle %.1f° → %.1f°  |  dist moy %.0f mm  (%d points)\n",
                   i + 1, o.angle_start_deg, o.angle_end_deg,
                   o.dist_avg_mm, o.point_count);
        }
    }

    return 0;
}