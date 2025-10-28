#include <ur_controllers/joint_admittance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace ur_controllers {

bool JointAdmittanceController::init(hardware_interface::RobotHW* robot_hw, 
                                     ros::NodeHandle& node_handle) {
    // parameters
    if (!node_handle.getParam("joints", joint_names_)) {
        ROS_ERROR("JointAdmittanceController: Could not parse joint names.");
    }
    if (joint_names_.size() != 6) {
        ROS_ERROR_STREAM(
            "JointAdmittanceController: Wrong number of joint names, got " << joint_names_.size() 
                         << "instead of 6 names."
        );
        return false;
    }

    // robot state publish rate
    double state_publish_rate = 200.0;
    max_twist_ = 1.0;
    gravity_ = 9.80665;

    try {
        M_d_ = getMatrix6dParam("mass", node_handle);
        D_d_ = getMatrix6dParam("damping", node_handle);
        K_d_ = getMatrix6dParam("stiffness", node_handle);

        node_handle.getParam("max_twist", max_twist_);
        max_dq_ = getVector6dParam("max_dq", node_handle);
        max_ddq_ = getVector6dParam("max_ddq", node_handle);
        min_dq_ = getVector6dParam("min_dq", node_handle);
        wrench_dead_zone_thres_ = getVector6dParam("wrench_dead_zone_thres", node_handle);
        
        node_handle.getParam("state_publish_rate", state_publish_rate);

        node_handle.getParam("wrench_topic_name", wrench_topic_name_);
        node_handle.getParam("command_topic_name", command_topic_name_);
        node_handle.getParam("state_topic_name", state_topic_name_);
        node_handle.getParam("robot_urdf_name", urdf_name_);

        node_handle.getParam("gripper_mass", gripper_mass_);
        gripper_com_ = getVector3dParam("gripper_com", node_handle);
        wrench_bias_ = getVector6dParam("wrench_bias", node_handle);
        node_handle.getParam("gravity", gravity_);
    } catch (const std::invalid_argument& e) {
        ROS_ERROR_STREAM(
            "JointAdmittanceController: " << e.what()
        );
        return false;
    }

    wrench_.setZero();
    dq_filtered_.setZero();

    pose_e_.setZero();
    twist_e_.setZero();
    accel_e_.setZero();

    // Init subscribers
    external_wrench_subscriber_ = node_handle.subscribe(wrench_topic_name_, 1, 
                                                        &JointAdmittanceController::externalWrenchCallback, 
                                                        this,
                                                        ros::TransportHints().reliable().tcpNoDelay());
    
    velocity_command_subscriber_ = node_handle.subscribe(command_topic_name_, 1,
                                                         &JointAdmittanceController::desiredCommandCallback,
                                                         this,
                                                         ros::TransportHints().reliable().tcpNoDelay());

    // Init publisher
    state_publish_period_ = ros::Duration(1.0 / state_publish_rate);
    robot_state_publisher_.reset(new RobotStatePublisher(node_handle, state_topic_name_, 1));

    // Init interfaces and handles
    velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr) {
        ROS_ERROR(
            "JointAdmittanceController: Error getting velocity joint interface from hardware."
        );
        return false;
    }

    velocity_joint_handles_.resize(6);
    for (std::size_t i = 0; i < 6; ++i) {
        try {
            velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names_[i]);
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                "JointAdmittanceController: Exception getting joint handles: " << e.what()
            );
            return false;
        }
    }


    robot_state_interface_ = robot_hw->get<ur_driver::RobotStateInterface>();
    if (robot_state_interface_ == nullptr) {
        ROS_ERROR(
            "JointAdmittanceController: Could not get robot state interface from hardware."
        );
        return false;
    }

    try {
        robot_state_handle_ = std::make_unique<ur_driver::RobotStateHandle>(
            robot_state_interface_->getHandle("ur_robot")
        );
    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
            "JointAdmittanceController: Exception getting robot state handle: " << e.what()
        );
        return false;
    }

    
    // Load urdf model
    pinocchio::urdf::buildModel(urdf_name_, robot_model_);
    ROS_INFO_STREAM("Load and build model" << robot_model_.name << " successfully.");

    collision_ = false;

    return true;
}


void JointAdmittanceController::starting(const ros::Time& time) {
    // Initialize robot state
    ur_driver::RobotState robot_state = robot_state_handle_->getRobotState();
    
    Eigen::Map<Vector6d> q_(robot_state.q.data());
    Eigen::Map<Vector6d> dq_(robot_state.dq.data());
    Eigen::Map<Vector6d> dq_filtered_(robot_state.dq.data());

    Eigen::Map<Vector6d> cart_pose_(robot_state.pose_EE.data());
    Eigen::Map<Vector6d> cart_twist_(robot_state.twist_EE.data());

    // Initialize desired state with the current state on startup
    dq_d_ = dq_;
    dq_c_ = dq_;
    twist_d_ = cart_twist_;
    pose_d_ = cart_pose_;

    publishRobotState(time);

    // Initialize last state update time
    last_state_publish_time_ = time;
}
    

void JointAdmittanceController::update(const ros::Time& time, const ros::Duration& period) {
    // Update robot joint state
    ur_driver::RobotState robot_state = robot_state_handle_->getRobotState();

    double alpha = 0.99;
    Eigen::Map<Vector6d> q_(robot_state.q.data());
    Eigen::Map<Vector6d> dq_(robot_state.dq.data());
    dq_filtered_ = (1 - alpha) * dq_filtered_ + alpha * dq_;

    pinocchio::Data robot_data(robot_model_);
    pinocchio::computeJointJacobians(robot_model_, robot_data, q_);
    Matrix6d Jacobian = robot_data.J;

    twist_d_ = Jacobian * dq_c_;

    pinocchio::forwardKinematics(robot_model_, robot_data, q_);
    Matrix3d cart_rot = robot_data.oMi[robot_model_.njoints - 1].rotation();
    
    /** Gravity compensation for F/T sensor (static) */
    Vector3d gravity = gravity_ * cart_rot.inverse() * Eigen::Vector3d(0, 0, 1); // on the earth
    Vector6d ft_compensation;
    ft_compensation.topRows(3) = - gripper_mass_ * gravity;
    ft_compensation.bottomRows(3) = - gripper_mass_ * gripper_com_.cross(gravity);
    Vector6d wrench_compensation = wrench_;
    wrench_compensation -= ft_compensation;
    wrench_compensation -= wrench_bias_;
    ROS_INFO_STREAM("Ft compensation: " << ft_compensation[0] << " " << ft_compensation[1] << " " << ft_compensation[2] << " "
                                        << ft_compensation[3] << " " << ft_compensation[4] << " " << ft_compensation[5]);
    ROS_INFO_STREAM("Ft: " << wrench_compensation[0] << " " << wrench_compensation[1] << " " << wrench_compensation[2] << " "
                           << wrench_compensation[3] << " " << wrench_compensation[4] << " " << wrench_compensation[5]);

    for (std::size_t i = 0; i < 6; ++i) {
        if (fabs(wrench_compensation[i]) < wrench_dead_zone_thres_[i]) {
            wrench_compensation[i] = 0;
        } else {
            collision_ = true;
        }
    }

    // ft transform matrix
    Matrix6d ft_transform = Matrix6d::Zero();
    ft_transform.topLeftCorner(3, 3) = cart_rot;
    ft_transform.bottomRightCorner(3, 3) = cart_rot;

    Vector6d wrench_e = ft_transform * wrench_compensation;

    /** Compute admittance control law in Cartesian frame */
    Vector6d accel_e = M_d_.inverse() * (- D_d_ * twist_e_ - K_d_ * pose_e_ + wrench_e);
    Vector6d twist_e = twist_e_ + accel_e_ * period.toSec();
    Vector6d pose_e = pose_e_ + twist_e_ * period.toSec();

    // Update desired twist (Integrate for velocity based interface)
    twist_d_ += twist_e;
    ROS_INFO_STREAM("Eorror twist: " << twist_e[0] << " " << twist_e[1] << " " << twist_e[2] << " "
                                     << twist_e[3] << " " << twist_e[4] << " " << twist_e[5]);
    ROS_INFO_STREAM("Desired twist: " << twist_d_[0] << " " << twist_d_[1] << " " << twist_d_[2] << " "
                                      << twist_d_[3] << " " << twist_d_[4] << " " << twist_d_[5]);

    // Limit velocity for better stability and safety
    double twist_norm = (twist_d_.segment(0, 3)).norm();
    if (twist_norm > max_twist_) {
        ROS_WARN_STREAM(
            "JointAdmittanceController: Admittance generates high arm velocity. Norm: " << twist_norm
        );
        twist_d_.segment(0, 3) *= (max_twist_ / twist_norm);
    }

    // Inverse kinematics solve
    dq_d_ = Jacobian.inverse() * twist_d_;

    // Limit and Output joint velocity dq_d_
    for (std::size_t i = 0; i < 6; ++i) {
        if (fabs(dq_d_[i]) > max_dq_[i]) {
            dq_d_[i] *= (max_dq_[i] / fabs(dq_d_[i]));
        } else if (fabs(dq_d_[i]) < min_dq_[i]) {
            dq_d_[i] = 0.0;
        }
        velocity_joint_handles_[i].setCommand(dq_d_[i]);
    }
    ROS_INFO_STREAM("Desired dq: " << dq_d_[0] << " " << dq_d_[1] << " " << dq_d_[2] << " "
                                   << dq_d_[3] << " " << dq_d_[4] << " " << dq_d_[5]);

    publishRobotState(time);

    // Set no collision if collision
    if (collision_) {
        collision_ = false;
    }
    // Update error to last error (global)
    pose_e_ = pose_e;
    twist_e_ = twist_e;
    accel_e_ = accel_e;
}


Vector6d JointAdmittanceController::getVector6dParam(const std::string& param_name,
                                                     ros::NodeHandle& nh) {
    std::vector<double> vec;
    if (!nh.getParam(param_name, vec) || vec.size() != 6) {
        ROS_ERROR_STREAM(
            "JointAdmittanceController: Could not parse " << param_name << "."
        );
    }
    return Vector6d(Eigen::Map<Vector6d>(vec.data()));
}

Matrix6d JointAdmittanceController::getMatrix6dParam(const std::string& param_name,
                                                     ros::NodeHandle& nh) {
    std::vector<double> vec;
    if (!nh.getParam(param_name, vec) || vec.size() != 36) {
        ROS_ERROR_STREAM(
            "JointAdmittanceController: Could not parse " << param_name << "."
        );
    }
    return Matrix6d(Eigen::Map<Matrix6d>(vec.data(), 6, 6));
}

Vector3d JointAdmittanceController::getVector3dParam(const std::string& param_name, 
                                                     ros::NodeHandle& nh) {
    std::vector<double> vec;
    if (!nh.getParam(param_name, vec) || vec.size() != 3) {
        ROS_ERROR_STREAM(
            "JointAdmittanceController: Could not parse " << param_name << "."
        );
    }
    return Vector3d(Eigen::Map<Vector3d>(vec.data()));
}

void JointAdmittanceController::publishRobotState(const ros::Time& time) {
    // Check if it is time to publish
    if (!state_publish_period_.isZero() && last_state_publish_time_ + state_publish_period_ < time) {
        if (robot_state_publisher_ && robot_state_publisher_->trylock()) {

            last_state_publish_time_ += state_publish_period_;

            robot_state_publisher_->msg_.header.stamp = time;
            robot_state_publisher_->msg_.name.resize(6);
            robot_state_publisher_->msg_.header.seq = collision_; // collision flag
            robot_state_publisher_->msg_.position.resize(6);
            robot_state_publisher_->msg_.velocity.resize(6);
            for (std::size_t i = 0; i < 6; ++i) {
                robot_state_publisher_->msg_.position[i] = q_[i];
                robot_state_publisher_->msg_.velocity[i] = dq_filtered_[i];
            }

            robot_state_publisher_->unlockAndPublish();
        }
    }
}

void JointAdmittanceController::externalWrenchCallback(const geometry_msgs::WrenchStampedConstPtr msg) {
    // Read F/T sensor in its own frame (ftsensor_frame)
    Vector6d wrench_ft;
    wrench_ft << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                 msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;

    ROS_INFO_STREAM("F/T in sensor frame: " << wrench_ft[0] << " " << wrench_ft[1] << " " << wrench_ft[2] << " "
                                            << wrench_ft[3] << " " << wrench_ft[4] << " " << wrench_ft[5]);

    wrench_ = wrench_ft;
}

void JointAdmittanceController::desiredCommandCallback(const sensor_msgs::JointStateConstPtr msg) {
    // Subscribe desired velocity command
    Vector6d dq_d;
    if (msg->velocity.size() != 6) {
        ROS_ERROR_STREAM(
            "JointAdmittanceController: desiredVelocityCallback the size of velocity is " << msg->velocity.size() << "."
        );
    }

    dq_d << msg->velocity[0], msg->velocity[1], msg->velocity[2],
            msg->velocity[3], msg->velocity[4], msg->velocity[5];

    ROS_INFO_STREAM("Received dq_d: " << dq_d[0] << " " << dq_d[1] << " " << dq_d[2] << " "
                                      << dq_d[3] << " " << dq_d[4] << " " << dq_d[5]);

    dq_c_ = dq_d;
}

}

PLUGINLIB_EXPORT_CLASS(ur_controllers::JointAdmittanceController,
                       controller_interface::ControllerBase)
