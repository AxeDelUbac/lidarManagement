#pragma once

#include <cstdint>
#include <vector>
#include "obstacleDetection.h"

// Protocole : frame 3 octets  [0xAA][CMD][0x55]
static constexpr uint8_t FRAME_START = 0xAA;
static constexpr uint8_t FRAME_END   = 0x55;

enum class McuCmd : uint8_t {
    CLEAR          = 0x00,  // aucun obstacle
    OBSTACLE_FRONT = 0x01,  // obstacle devant
    OBSTACLE_LEFT  = 0x02,  // obstacle à gauche
    OBSTACLE_RIGHT = 0x03,  // obstacle à droite
    OBSTACLE_ALL   = 0x04,  // cerné de tous côtés
};

// Déduit la commande à envoyer depuis la liste d'obstacles détectés.
// Zones angulaires (ajustables selon montage du LiDAR) :
//   Front  : 315°–360° et 0°–45°
//   Left   : 45°–135°
//   Right  : 225°–315°
McuCmd computeCommand(const std::vector<Obstacle>& obstacles);
