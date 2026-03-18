#include "moretea_arm/servo_backend.hpp"
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include <vector>
#include <glob.h>

std::string auto_detect_serial()
{
    // Check common serial port patterns
    std::vector<std::string> patterns = {
        "/dev/wave_share_board"
    };

    for (const auto& pattern : patterns) {
        glob_t glob_result;
        glob(pattern.c_str(), GLOB_TILDE, nullptr, &glob_result);
        
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            std::string port = glob_result.gl_pathv[i];
            // Try to verify it's accessible
            if (std::filesystem::exists(port)) {
                globfree(&glob_result);
                std::cout << "Auto-detected serial port: " << port << std::endl;
                return port;
            }
        }
        globfree(&glob_result);
    }

    // Default fallback
    std::cerr << "Warning: No serial port auto-detected, using /dev/ttyUSB0" << std::endl;
    return "/dev/ttyUSB0";
}

bool play_trajectory_backend(
    SMS_STS& sm_st,
    const std::string& yaml_file,
    const std::string& mode,
    std::function<void(float)> progress_cb,
    std::function<bool()> cancel_cb
) {
    const int GRIPPER_ID = 6;
    bool ignore_gripper = (mode == "arm");

    try {
        YAML::Node doc = YAML::LoadFile(yaml_file);
        
        // Speed settings from MENU.cpp for consistency
        const uint16_t ALIGN_SPEED = 150;
        const uint8_t  ALIGN_ACC   = 200;
        const uint16_t PLAY_SPEED  = 290;
        const uint8_t  PLAY_ACC    = 200;

        /* ---------- ALIGNMENT ---------- */
        const YAML::Node& first_pose = doc["trajectory"][0]["servos"];
        for (const auto& kv : first_pose) {
            int id = kv.first.as<int>();
            if (ignore_gripper && id == GRIPPER_ID) continue;

            sm_st.EnableTorque(id, 1);
            sm_st.WritePosEx(id, kv.second.as<int>(), ALIGN_SPEED, ALIGN_ACC);
        }

        /* ---------- PLAYBACK ---------- */
        int step_idx = 0;
        int total_steps = doc["trajectory"].size();

        for (const auto& step : doc["trajectory"]) {
            if (cancel_cb()) return false;

            const YAML::Node& servos = step["servos"];
            for (const auto& kv : servos) {
                int id = kv.first.as<int>();
                if (ignore_gripper && id == GRIPPER_ID) continue;

                sm_st.WritePosEx(id, kv.second.as<int>(), PLAY_SPEED, PLAY_ACC);
            }
            step_idx++;
            progress_cb(float(step_idx) / total_steps);
        }
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error playing trajectory: " << e.what() << std::endl;
        return false;
    }
}

// Separate gripper-only logic inspired by record_gripper/play_gripper
bool play_gripper_backend(
    SMS_STS& sm_st,
    const std::string& yaml_file,
    int load_limit,
    std::function<bool()> cancel_cb
) {
    const int GRIPPER_ID = 6;
    try {
        YAML::Node doc = YAML::LoadFile(yaml_file);
        int target = doc["trajectory"][0]["servos"][GRIPPER_ID].as<int>();
        
        sm_st.EnableTorque(GRIPPER_ID, 1);
        sm_st.WritePosEx(GRIPPER_ID, target, 200, 200);

        // Monitoring loop for load or position
        for (int i = 0; i < 400; ++i) { // 4 second timeout
            if (cancel_cb()) return false;
            
            int load = sm_st.ReadLoad(GRIPPER_ID);
            if (std::abs(load) >= load_limit) return true;
            
            int pos = sm_st.ReadPos(GRIPPER_ID);
            if (std::abs(pos - target) <= 8) return true;
            
            usleep(10000);
        }
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error playing trajectory: " << e.what() << std::endl;
        return false;
    }
}