#ifndef __JOINT_ADMITTANCE_CONTROLLER_HPP__
#define __JOINT_ADMITTANCE_CONTROLLER_HPP__

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <realtime_tools/realtime_publisher.h>

#include <cartesian_interface/cartesian_state_handle.h>
#include <cartesian_interface/cartesian_command_interface.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

#include <ur_robot_driver/robot_state_interface.h>


namespace ur_controllers {

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Matrix6d;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3d;

/**
 * Admittance Controller
 * 
 * @brief A simple class implementing an admittance controller for UR5 robot. A control loop contains:
 * 1) Read the robot state 
 * 2) Compute the desired twist (cartesian velocity) and joint velocity of robot arm
 * 3) Send the desired joint velocity to robot
 */
class JointAdmittanceController : public controller_interface::MultiInterfaceController<
                                                ur_driver::RobotStateInterface,
                                                hardware_interface::VelocityJointInterface> {

public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

private:

    hardware_interface::VelocityJointInterface* velocity_joint_interface_; // velocity-based control interface
    std::vector<hardware_interface::JointHandle> velocity_joint_handles_;  // velocity-based control handles
    ur_driver::RobotStateInterface* robot_state_interface_;                // robot state interface
    std::unique_ptr<ur_driver::RobotStateHandle> robot_state_handle_;      // robot state handle

    std::vector<std::string> joint_names_;

    ros::Subscriber external_wrench_subscriber_;
    std::string wrench_topic_name_;
    ros::Subscriber velocity_command_subscriber_;
    std::string command_topic_name_;
    typedef realtime_tools::RealtimePublisher<sensor_msgs::JointState> RobotStatePublisher;
    typedef std::unique_ptr<RobotStatePublisher> RobotStatePublisherPtr;
    RobotStatePublisherPtr robot_state_publisher_;
    std::string state_topic_name_;

    ros::Time last_state_publish_time_;
    ros::Duration state_publish_period_;

    // Admittance parameters
    Matrix6d M_d_;  // Desired mass matrix
    Matrix6d D_d_;  // Desired damping matrix
    Matrix6d K_d_;  // Desired stiffness matrix

    // Arm limit in Cartesian space and joint space
    double max_twist_;  // Desired max cartesian velocity
    Vector6d max_dq_;   // Desired max joint velocity
    Vector6d min_dq_;   // Desired min joint velocity in case of oscillation
    Vector6d max_ddq_;  // Desired max joint accelaration

    // FT dead zone threshold
    Vector6d wrench_dead_zone_thres_;

    // State variables
    // Cartesian state
    geometry_msgs::Pose cart_pose_;
    geometry_msgs::Twist cart_twist_;
    // Joint state
    Vector6d q_;
    Vector6d dq_;
    Vector6d dq_filtered_;
    // External F/T
    Vector6d wrench_;
    bool collision_{false};

    // Control variables
    Vector6d dq_d_;     // Desired velocity
    Vector6d dq_c_;     // Commanded velocity
    Vector6d pose_d_;   // Desired cartesian pose
    Vector6d twist_d_;  // Desired cartesian velocity
    // Error variables
    Vector6d pose_e_;   // Pose error  e
    Vector6d twist_e_;  // Twist error  de
    Vector6d accel_e_;  // Accelaration error  dde

    // Kinematic pinocchio lib parser variables
    std::string urdf_name_;
    pinocchio::Model robot_model_;

    double gravity_;       // gravity value z
    Vector6d wrench_bias_; 
    double gripper_mass_;  // gripper mass
    Vector3d gripper_com_; // gripper center of mass position (relative to FT sensor frame)

    // Publish current robot controller state which high-level planner will use. 
    // This method is realtime-safe and is meant to be called from update().
    void publishRobotState(const ros::Time& time);

    // Utility functions
    Vector6d getVector6dParam(const std::string& param_name, ros::NodeHandle& nh);
    Matrix6d getMatrix6dParam(const std::string& param_name, ros::NodeHandle& nh);
    Vector3d getVector3dParam(const std::string& param_name, ros::NodeHandle& nh);

    // Callback functions
    void externalWrenchCallback(const geometry_msgs::WrenchStampedConstPtr msg);
    void desiredCommandCallback(const sensor_msgs::JointStateConstPtr msg);
};

}

#endif
