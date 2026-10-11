#include <jointspace_control/jointspace_control.h>

JointSpaceControl::JointSpaceControl() : Node("jointspace_control")
{
    // setup subscriber for robot description
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description", 
        rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
        std::bind(&JointSpaceControl::robotDescriptionCallback, this, std::placeholders::_1));

    // frame names
    frame_names_[0] = "target_frame_1";
    frame_names_[1] = "target_frame_2";

    // setup transform buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool JointSpaceControl::moveToNextPose()
{
    // return false if done
    if (pose_index_ >= frame_names_.size())
        return false;
    
    KDL::Frame frame;
    if (lookupFrame(frame_names_[pose_index_], frame))
        solveIK(frame);
    
    // return true if not finished
    return true;
}

void JointSpaceControl::initializeMoveGroup()
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "meca500_arm");
}

void JointSpaceControl::moveJoints()
{
    // set movegroup to target joint angles
    move_group_->setJointValueTarget(goal_angles_);

    // if move plan succeeded, move robot
    if (move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        move_group_->execute(plan_);
    }
    else // print error otherwise
    {
        RCLCPP_INFO(this->get_logger(), "Plan failed");
    }

}

void JointSpaceControl::robotDescriptionCallback(const std_msgs::msg::String& msg)
{
    // Construct KDL tree from URDF
    const std::string urdf = msg.data;
    kdl_parser::treeFromString(urdf, tree_);

    // Get kinematic chain of the robot
    tree_.getChain("meca_base_link", "meca_axis_6_link", chain_);

    q_ = KDL::JntArray(chain_.getNrOfJoints());

    // Create IK solver
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);

    goal_angles_.resize(chain_.getNrOfJoints());
}

void JointSpaceControl::solveIK(const KDL::Frame & frame)
{
    if (ik_solver_ != nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Trying pose number %i", pose_index_);

        // initial guess of IK solution
        KDL::JntArray q_init(chain_.getNrOfJoints());
        q_init(0) = 0.0; q_init(1) = 0.0; q_init(2) = 0.0;  q_init(3) = 0.0; q_init(4) = 0.0; q_init(5) = 0.0;

        // solve IK 
        int error = ik_solver_->CartToJnt(q_init, frame, q_);

        if (error == KDL::ChainIkSolverPos_LMA::E_NOERROR) // IK succeeded
        {
            RCLCPP_INFO(this->get_logger(), "IK succeeded");
            
            // copy the joint angles 
            std::copy(q_.data.begin(), q_.data.end(), goal_angles_.begin());

            moveJoints();

        }
        else // IK failed
        {
            RCLCPP_ERROR(this->get_logger(), "Inverse kinematics failed!");
        }

        // increase the index (only if it worked)
        ++pose_index_;
    }
}

bool JointSpaceControl::lookupFrame(std::string & frame_name, KDL::Frame & output_frame)
{
    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("world", frame_name, tf2::TimePointZero);
        
        // set translation of output frame
        output_frame.p.x(transform.transform.translation.x); 
        output_frame.p.y(transform.transform.translation.y); 
        output_frame.p.z(transform.transform.translation.z);

        // set translation of output frame
        output_frame.M = KDL::Rotation::Quaternion(
            transform.transform.rotation.x, 
            transform.transform.rotation.y, 
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );

        RCLCPP_INFO(this->get_logger(), "Found frame: %s", frame_name.data());
        
        return true;
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_ERROR(this->get_logger(), "lookup failed! Reason: %s", ex.what());

        return false;
    }
}

int main(int argc, char** argv)
{
    // initialize the node
    rclcpp::init(argc, argv);
    
    // create instance of class
    auto node = std::make_shared<JointSpaceControl>();

    // Set loop rate
    rclcpp::Rate rate = rclcpp::Rate(1); // Hz

    node->initializeMoveGroup();
    
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node); // allow the /robot_description callback to fire first
        
        rate.sleep();

        // break if there are not any new poses to solve
        if (!node->moveToNextPose())
            break;
    }
    rclcpp::shutdown();
    return 0;
}