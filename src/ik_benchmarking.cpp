#include "ik_benchmarking/ik_benchmarking.hpp"

#include <fmt/core.h>
#include <fmt/ranges.h>

#include <chrono>
#include <numeric>

using namespace std::chrono_literals;

void IKBenchmarking::initialize() {
    const auto seed = static_cast<unsigned int>(node_->get_parameter("random_seed").as_int());
    random_numbers::RandomNumberGenerator generator_{seed};

    planning_group_name_ = node_->get_parameter("planning_group").as_string();
    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);

    // Load the tip link name (not the end effector)
    auto const &link_names = joint_model_group_->getLinkModelNames();
    std::string tmpstr; 
    for (int i=0;i<link_names.size();i++)
	    tmpstr+=(link_names[i]+", ");
    RCLCPP_INFO(logger_, "planning group links %s\n" , tmpstr.c_str());
    if (!link_names.empty()) {
        tip_link_name_ = link_names.back();
	RCLCPP_INFO(logger_, "planning group tip link %s\n" , tip_link_name_.c_str());
    } else {
        RCLCPP_ERROR(logger_, "ERROR: The move group is corrupted. Links count is zero.\n");
        rclcpp::shutdown();
    }

    robot_state_->setToDefaultValues();
}
//trial,found_ik,solve_time,position_error,orientation_error,t1, t2,t3,t4,t5,t6,t7, p1, p2, p3, p4, p5, p6 , p7

void IKBenchmarking::gather_data() {
    // Collect IK solving data
    sample_size_ = static_cast<size_t>(node_->get_parameter("sample_size").as_int());
    ik_timeout_ = node_->get_parameter("ik_timeout").as_double();
    ik_iteration_display_step_ =
        static_cast<size_t>(node_->get_parameter("ik_iteration_display_step").as_int());

    for (size_t i = 0; i < sample_size_; ++i) {
        if ((i + 1) % ik_iteration_display_step_ == 0) {
            RCLCPP_INFO(logger_, "Solved sample %ld/%ld ...", i + 1, sample_size_);
        }

        // Generate a random state and solve Forward Kinematics (FK)
        robot_state_->setToRandomPositions(joint_model_group_, generator_);
        robot_state_->updateLinkTransforms();
        const Eigen::Isometry3d tip_link_pose =
            robot_state_->getGlobalLinkTransform(tip_link_name_);

        //  Log the sampled random joint values for debugging
        std::vector<double> random_joint_values;
        robot_state_->copyJointGroupPositions(joint_model_group_, random_joint_values);
        std::stringstream ss;
        ss << "";
        for (size_t i = 0; i < random_joint_values.size(); ++i) {
            ss << random_joint_values[i];
            if (i != random_joint_values.size() - 1) {
                ss << ", ";
            }
        }
        ss << "";
        RCLCPP_INFO(logger_, "The sampled random joint values are:\n%s\n", ss.str().c_str());

        // Randomize the initial seed state of the robot before solving IK
        robot_state_->setToRandomPositions(joint_model_group_, generator_);
        robot_state_->updateLinkTransforms();

        // Solve Inverse kinematics (IK)
        const auto start_time = std::chrono::high_resolution_clock::now();
        const bool found_ik =
            robot_state_->setFromIK(joint_model_group_, tip_link_pose, ik_timeout_);
        const auto end_time = std::chrono::high_resolution_clock::now();

        std::stringstream ik_sol;
        if (found_ik) {
            success_count_++;
            robot_state_->copyJointGroupPositions(joint_model_group_, random_joint_values);
            ik_sol << "";
            for (size_t i = 0; i < random_joint_values.size(); ++i) {
              ik_sol << random_joint_values[i];
              if (i != random_joint_values.size() - 1) {
                ik_sol << ", ";
              }
            }
            ik_sol << "";
        }

        const auto solve_time =
            std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        solve_times_.push_back(solve_time.count());

        // Calculate position error
        Eigen::Isometry3d ik_tip_link_pose = robot_state_->getGlobalLinkTransform(tip_link_name_);
        Eigen::Vector3d position_diff =
            ik_tip_link_pose.translation() - tip_link_pose.translation();
        double position_error = position_diff.norm();
	Eigen::Vector3d tipxyz = ik_tip_link_pose.translation();

        // Calculate orientation error (angle between two quaternions)
        Eigen::Quaterniond orientation(tip_link_pose.rotation());
        Eigen::Quaterniond ik_orientation(ik_tip_link_pose.rotation());
        double orientation_error = orientation.angularDistance(ik_orientation);

        data_file_ << std::boolalpha << i + 1 << "," << found_ik << "," << solve_time.count() << ","
                   << position_error << "," << orientation_error << "," 
		   << tipxyz(0) << "," << tipxyz(1) << "," << tipxyz(2) <<","
		   << ss.str().c_str() << "," <<ik_sol.str().c_str()<<  "\n";
    }

    // Average IK solving time and success rate
    average_solve_time_ =
        std::accumulate(solve_times_.begin(), solve_times_.end(), 0.0) / solve_times_.size();
    success_rate_ = success_count_ / sample_size_;

    RCLCPP_INFO(logger_, "Success rate = %f and average IK solving time is %f microseconds\n",
                success_rate_, average_solve_time_);

    calculation_done_ = true;
}

void IKBenchmarking::run() {
    this->initialize();
    this->gather_data();

    this->data_file_.close();
}

double IKBenchmarking::get_success_rate() const { return success_rate_; }

double IKBenchmarking::get_average_solve_time() const { return average_solve_time_; }

bool IKBenchmarking::calculation_done() const { return calculation_done_; }
