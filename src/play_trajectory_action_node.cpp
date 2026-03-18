#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <filesystem>
#include <thread>

#include "moretea_arm/action/play_trajectory.hpp"
#include "moretea_arm/servo_backend.hpp"
#include "SCServo_Linux/SMS_STS.h"

using PlayTrajectory = moretea_arm::action::PlayTrajectory;

class PlayTrajectoryServer : public rclcpp::Node
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<PlayTrajectory>;

    PlayTrajectoryServer()
    : Node("play_trajectory_server")
    {
        action_server_ = rclcpp_action::create_server<PlayTrajectory>(
            this,
            "play_trajectory",
            std::bind(&PlayTrajectoryServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PlayTrajectoryServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PlayTrajectoryServer::handle_accepted, this, std::placeholders::_1)
        );

        std::string serial = auto_detect_serial();
        sm_st_.begin(1000000, serial.c_str());

        RCLCPP_INFO(get_logger(), "PlayTrajectory Action Server started");
    }

    ~PlayTrajectoryServer()
    {
        sm_st_.end();
    }

private:
    SMS_STS sm_st_;
    rclcpp_action::Server<PlayTrajectory>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const PlayTrajectory::Goal> goal)
    {
        if (!std::filesystem::exists(goal->yaml_file)) {
            RCLCPP_WARN(get_logger(), "YAML file does not exist");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread(&PlayTrajectoryServer::execute, this, goal_handle).detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        // 1. Declare the result and feedback objects
        auto result = std::make_shared<PlayTrajectory::Result>();
        auto feedback = std::make_shared<PlayTrajectory::Feedback>();
        
        bool ok = false;

        if (goal->mode == "all" || goal->mode == "arm") {
            ok = play_trajectory_backend(
                sm_st_, 
                goal->yaml_file, 
                goal->mode,
                // 2. Fix unused parameter by actually publishing feedback
                [&](float p) { 
                    feedback->progress = p;
                    goal_handle->publish_feedback(feedback);
                },
                [&]() { return goal_handle->is_canceling(); }
            );
        } 
        else if (goal->mode == "gripper") {
            ok = play_gripper_backend(
                sm_st_, 
                goal->yaml_file, 
                goal->load_limit,
                [&]() { return goal_handle->is_canceling(); }
            );
        }

        // 3. Set the result values before completing the goal
        if (ok) {
            result->success = true;
            result->message = "Task completed successfully";
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->message = "Task failed or was cancelled";
            goal_handle->canceled(result);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlayTrajectoryServer>());
    rclcpp::shutdown();
    return 0;
}