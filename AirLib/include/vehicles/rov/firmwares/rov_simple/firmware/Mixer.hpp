#pragma once

#include <vector>
#include <algorithm>
#include "Params.hpp"
#include "interfaces/CommonStructs.hpp"

namespace Rov_simple
{

class Mixer
{
public:
    Mixer(const Params* params)
        : params_(params)
    {
    }

    void getMotorOutput(const Axis4r& controls, std::vector<float>& motor_outputs) const
    {
        if (motor_outputs.size() < static_cast<size_t>(kMotorCount)) {
            motor_outputs.resize(kMotorCount, 0.0f);
        }

        if (controls.throttle() < params_->actuator.min_angling_throttle) {
            motor_outputs.assign(params_->actuator.actuator_count, controls.throttle());
            return;
        }

        for (int motor_index = 0; motor_index < kMotorCount; ++motor_index) {
            motor_outputs[motor_index] =
                controls.throttle() * mixerRov8[motor_index].throttle + controls.pitch() * mixerRov8[motor_index].pitch + controls.roll() * mixerRov8[motor_index].roll + controls.yaw() * mixerRov8[motor_index].yaw;
        }

        float min_motor = *std::min_element(motor_outputs.begin(), motor_outputs.begin() + kMotorCount);
        if (min_motor < params_->actuator.min_actuator_output) {
            float undershoot = params_->actuator.min_actuator_output - min_motor;
            for (int motor_index = 0; motor_index < kMotorCount; ++motor_index)
                motor_outputs[motor_index] += undershoot;
        }

        float max_motor = *std::max_element(motor_outputs.begin(), motor_outputs.begin() + kMotorCount);
        float scale = max_motor / params_->actuator.max_actuator_output;
        if (scale > params_->actuator.max_actuator_output) {
            for (int motor_index = 0; motor_index < kMotorCount; ++motor_index)
                motor_outputs[motor_index] /= scale;
        }

        for (int motor_index = 0; motor_index < kMotorCount; ++motor_index)
            motor_outputs[motor_index] = std::max(params_->actuator.min_actuator_output,
                                                  std::min(motor_outputs[motor_index], params_->actuator.max_actuator_output));
    }

private:
    static const int kMotorCount = 8;

    const Params* params_;

    // Custom mixer data per motor
    typedef struct motorMixer_t
    {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    } motorMixer_t;

    // BlueROV2 Heavy 8-thruster configuration
    // r1-r4: Vertical thrusters (heave/throttle, roll, pitch)
    // r5-r8: Horizontal vectored thrusters (surge, sway, yaw)
    const motorMixer_t mixerRov8[8] = {
        // Vertical thrusters (r1-r4)
        { 1.0f, -1.0f, 1.0f, 0.0f }, // r1: FRONT_R VERTICAL
        { 1.0f, 1.0f, 1.0f, 0.0f }, // r2: FRONT_L VERTICAL
        { 1.0f, 1.0f, -1.0f, 0.0f }, // r3: REAR_L VERTICAL
        { 1.0f, -1.0f, -1.0f, 0.0f }, // r4: REAR_R VERTICAL
        // Horizontal vectored thrusters (r5-r8)
        { 0.0f, 0.0f, 0.0f, 1.0f }, // r5: FRONT_R HORIZONTAL
        { 0.0f, 0.0f, 0.0f, -1.0f }, // r6: FRONT_L HORIZONTAL
        { 0.0f, 0.0f, 0.0f, 1.0f }, // r7: REAR_L HORIZONTAL
        { 0.0f, 0.0f, 0.0f, -1.0f }, // r8: REAR_R HORIZONTAL
    };
};

} //namespace
