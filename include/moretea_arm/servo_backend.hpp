#ifndef MORETEA_ARM_SERVO_BACKEND_HPP
#define MORETEA_ARM_SERVO_BACKEND_HPP

#include <string>
#include <functional>
#include "SCServo_Linux/SMS_STS.h"

std::string auto_detect_serial();

// Original trajectory logic, now with a mode string to handle "arm" vs "all"
bool play_trajectory_backend(
    SMS_STS& sm_st,
    const std::string& yaml_file,
    const std::string& mode, 
    std::function<void(float)> progress_cb,
    std::function<bool()> cancel_cb
);

// Gripper specific logic
bool play_gripper_backend(
    SMS_STS& sm_st,
    const std::string& yaml_file,
    int load_limit,
    std::function<bool()> cancel_cb
);

#endif