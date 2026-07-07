/**
 * @file job_runner_node.cpp
 * @brief YAML 포즈 파일을 읽어 JointTrajectoryController에 순차 전달하는 노드
 *
 * 사용법:
 *   ros2 run my_robot_ros2_control job_runner --ros-args -p job_file:=<path/to/poses.yaml>
 *
 * 서비스:
 *   /job_pause   (std_srvs/Trigger) - 다음 웨이포인트 이동 전 일시정지
 *   /job_resume  (std_srvs/Trigger) - 일시정지 해제
 *   /job_abort   (std_srvs/Trigger) - 작업 중단 후 home으로 복귀
 */

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <atomic>
#include <chrono>

static const std::vector<std::string> JOINT_NAMES = {
    "link1_1_joint", "link2_1_joint", "link3_1_joint",
    "link4_1_joint", "link5_1_joint", "link6_1_joint"
};
static const std::string CONTROLLER_TOPIC = "/joint_trajectory_controller/joint_trajectory";

struct Waypoint {
    std::vector<double> positions;  // radians, 6 joints
    std::string name;
};

class JobRunnerNode : public rclcpp::Node
{
public:
    JobRunnerNode() : Node("job_runner")
    {
        // 파라미터
        declare_parameter<std::string>("job_file", "");
        declare_parameter<double>("velocity_rpm", 2.0);
        declare_parameter<double>("acceleration_rpm_s", 5.0);
        declare_parameter<double>("waypoint_duration_sec", 3.0);  // 웨이포인트 간 이동 시간

        trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            CONTROLLER_TOPIC, 10);

        pause_srv_ = create_service<std_srvs::srv::Trigger>(
            "/job_pause",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                if (!running_) {
                    res->success = false;
                    res->message = "No job running";
                    return;
                }
                paused_ = true;
                res->success = true;
                res->message = "Job paused at waypoint " + std::to_string(current_wp_ + 1) +
                               "/" + std::to_string(total_wp_);
                RCLCPP_INFO(get_logger(), "⏸  %s", res->message.c_str());
            });

        resume_srv_ = create_service<std_srvs::srv::Trigger>(
            "/job_resume",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                if (!paused_) {
                    res->success = false;
                    res->message = "Job is not paused";
                    return;
                }
                paused_ = false;
                res->success = true;
                res->message = "Job resumed";
                RCLCPP_INFO(get_logger(), "▶  Job resumed");
            });

        abort_srv_ = create_service<std_srvs::srv::Trigger>(
            "/job_abort",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                aborted_ = true;
                paused_ = false;
                res->success = true;
                res->message = "Job aborted";
                RCLCPP_WARN(get_logger(), "🛑 Job aborted by user");
            });

        // 1초 후 실행 시작 (publisher가 준비될 시간)
        start_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                start_timer_->cancel();
                run_job();
            });
    }

private:
    bool load_job(const std::string & file_path)
    {
        if (file_path.empty()) {
            RCLCPP_ERROR(get_logger(), "No job_file parameter specified");
            return false;
        }
        try {
            YAML::Node root = YAML::LoadFile(file_path);

            // safe_poses.yaml 포맷: safe_poses -> <name> -> joint_positions
            YAML::Node poses_node;
            if (root["safe_poses"]) {
                poses_node = root["safe_poses"];
            } else if (root["printing_job"] && root["printing_job"]["waypoints"]) {
                poses_node = root["printing_job"]["waypoints"];
            } else {
                RCLCPP_ERROR(get_logger(), "Unknown YAML format in: %s", file_path.c_str());
                return false;
            }

            for (auto it = poses_node.begin(); it != poses_node.end(); ++it) {
                Waypoint wp;
                // Map 형태 (safe_poses.yaml): key=name, value=node
                if (it->second.IsMap() && it->second["joint_positions"]) {
                    wp.name = it->first.as<std::string>();
                    for (auto val : it->second["joint_positions"]) {
                        wp.positions.push_back(val.as<double>());
                    }
                }
                // Sequence 형태 (printing_job.yaml)
                else if (it->IsMap() && (*it)["joint_positions"]) {
                    wp.name = "wp_" + std::to_string(waypoints_.size());
                    for (auto val : (*it)["joint_positions"]) {
                        wp.positions.push_back(val.as<double>());
                    }
                } else {
                    continue;
                }

                if (wp.positions.size() != 6) {
                    RCLCPP_WARN(get_logger(), "Waypoint '%s' has %zu positions (expected 6), skipping",
                                wp.name.c_str(), wp.positions.size());
                    continue;
                }
                waypoints_.push_back(wp);
            }
        } catch (const YAML::Exception & e) {
            RCLCPP_ERROR(get_logger(), "YAML parse error: %s", e.what());
            return false;
        }

        if (waypoints_.empty()) {
            RCLCPP_ERROR(get_logger(), "No valid waypoints found in: %s", file_path.c_str());
            return false;
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from: %s",
                    waypoints_.size(), file_path.c_str());
        return true;
    }

    void send_waypoint(const Waypoint & wp, double duration_sec)
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = now();
        traj.joint_names = JOINT_NAMES;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = wp.positions;
        point.velocities.resize(6, 0.0);
        point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
        traj.points.push_back(point);

        trajectory_pub_->publish(traj);
        RCLCPP_INFO(get_logger(), "  → Sent waypoint '%s': [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    wp.name.c_str(),
                    wp.positions[0], wp.positions[1], wp.positions[2],
                    wp.positions[3], wp.positions[4], wp.positions[5]);
    }

    void run_job()
    {
        const std::string job_file = get_parameter("job_file").as_string();
        const double duration_sec  = get_parameter("waypoint_duration_sec").as_double();

        if (!load_job(job_file)) {
            return;
        }

        total_wp_ = static_cast<int>(waypoints_.size());
        running_ = true;

        RCLCPP_INFO(get_logger(), "▶  Job started: %d waypoints, %.1fs per waypoint",
                    total_wp_, duration_sec);

        for (int i = 0; i < total_wp_; ++i) {
            if (aborted_) {
                RCLCPP_WARN(get_logger(), "Job aborted at waypoint %d/%d", i + 1, total_wp_);
                break;
            }

            // 일시정지 대기
            while (paused_ && !aborted_) {
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
            if (aborted_) break;

            current_wp_ = i;
            RCLCPP_INFO(get_logger(), "[%d/%d] Moving to '%s'...",
                        i + 1, total_wp_, waypoints_[i].name.c_str());

            send_waypoint(waypoints_[i], duration_sec);

            // 이동 완료 대기 (duration + 0.5s 여유)
            rclcpp::sleep_for(
                std::chrono::milliseconds(static_cast<int>((duration_sec + 0.5) * 1000)));
        }

        running_ = false;

        if (!aborted_) {
            RCLCPP_INFO(get_logger(), "✅ Job completed: all %d waypoints executed", total_wp_);
        }
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
    rclcpp::TimerBase::SharedPtr start_timer_;

    std::vector<Waypoint> waypoints_;
    std::atomic<bool> paused_{false};
    std::atomic<bool> aborted_{false};
    std::atomic<bool> running_{false};
    std::atomic<int>  current_wp_{0};
    int total_wp_{0};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JobRunnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
