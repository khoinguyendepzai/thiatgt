#ifndef RAW_VEHICLE_CONTROLLER_HPP
#define RAW_VEHICLE_CONTROLLER_HPP

#include "LogicTypes.hpp"

class RawVehicleController {
public:
    RawVehicleController();

    void update(const Input& in, Output& out);
    void reset();

private:
    void updateSteering(const Input& in, Output& out);
    void updateVehicleDynamics(const Input& in, Output& out);

    float speed_;
    float dtSec_;
};

#endif