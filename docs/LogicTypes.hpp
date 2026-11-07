#ifndef LOGIC_TYPES_HPP
#define LOGIC_TYPES_HPP

#include <cstdint>

constexpr float STEER_CENTER = 0.0f;
constexpr float STEER_MIN    = -20.0f;
constexpr float STEER_MAX    = 20.0f;

constexpr float VEHICLE_MASS_KG = 4.0f;

constexpr float MAX_DRIVE_FORCE_N = 4.0f;
constexpr float MAX_BRAKE_FORCE_N = 3.0f;

constexpr float ROLLING_RESISTANCE_COEFF = 0.02f;
constexpr float AIR_DRAG_COEFF = 0.8f;

constexpr float MAX_FORWARD_SPEED = 1.3f;   
constexpr float MAX_REVERSE_SPEED = 0.8f;   

constexpr float SPEED_DEADZONE = 0.15f;

struct Input {
    long encoder = 0;
    float ga = 0.0f;
    float lui = 0.0f;
    float phanh = 0.0f;
    int driveDir = 0;
    float speed = 0.0f;
    float distance = -1.0f;
    int reset = 0;
    int score = 0;
};

struct Output {
    float steerCmd = STEER_CENTER;
    float speedCmd = 0.0f;
    int driveDirCmd = 0;
};
    
#endif