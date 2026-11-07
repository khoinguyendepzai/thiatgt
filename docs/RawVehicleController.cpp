#include "RawVehicleController.hpp"

#include <algorithm>
#include <cmath>

namespace {
    float clampf(float x, float lo, float hi) {
        return std::max(lo, std::min(hi, x));
    }
}

RawVehicleController::RawVehicleController()
    : speed_(0.0f),
      dtSec_(0.02f)
{
}

void RawVehicleController::reset() {
    speed_ = 0.0f;
}

void RawVehicleController::update(const Input& in, Output& out) {
    updateSteering(in, out);
    updateVehicleDynamics(in, out);
}

void RawVehicleController::updateSteering(const Input& in, Output& out) {
    long encoder = std::max(-180L, std::min(180L, in.encoder));
    out.steerCmd = STEER_MIN + static_cast<float>(encoder + 180L) * (STEER_MAX - STEER_MIN) / 360.0f;
}

void RawVehicleController::updateVehicleDynamics(const Input& in, Output& out) {
    float ga    = clampf(std::max(0.0f, in.ga),    0.0f, 100.0f);
    float lui   = clampf(std::max(0.0f, in.lui),   0.0f, 100.0f);
    float phanh = clampf(std::max(0.0f, in.phanh), 0.0f, 100.0f);

    if (ga < 5.0f) ga = 0.0f;
    if (lui < 5.0f) lui = 0.0f;
    if (phanh < 5.0f) phanh = 0.0f;

    float throttlePercent = 0.0f;
    float maxSpeed = 0.0f;

    if (in.driveDir == 1){
        throttlePercent = ga / 100.0f;
        maxSpeed = MAX_FORWARD_SPEED;
    }else if (in.driveDir == -1){
        throttlePercent = lui / 100.0f;
        maxSpeed = MAX_REVERSE_SPEED;
    }else{
        throttlePercent = 0.0f;
        maxSpeed = speed_;
    }

    float driveForce = MAX_DRIVE_FORCE_N * throttlePercent;
    float brakeForce = MAX_BRAKE_FORCE_N * (phanh / 100.0f);
    
    float rollingForce = 0.0f;
    if(speed_ > SPEED_DEADZONE){
        rollingForce = ROLLING_RESISTANCE_COEFF * VEHICLE_MASS_KG * 9.81f;
    }

    float airDragForce = AIR_DRAG_COEFF * speed_ * speed_;

    float totalForce = driveForce - brakeForce - rollingForce - airDragForce;
    float acceleration = totalForce / VEHICLE_MASS_KG;

    speed_ += acceleration * dtSec_;

    speed_ = clampf(speed_,0.0f,maxSpeed);

    out.speedCmd = speed_;
    out.driveDirCmd = in.driveDir;
}