
#ifndef JOINT_SPACE_CONTROL
#define JOINT_SPACE_CONTROL

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit/move_group_interface/move_group_interface.h>

/**
 * Solution for take-home assignment 8 
 * Applied Robotics
 */
class JointSpaceControl : public rclcpp::Node
{
public:
    /**
     * Constructor
     */
    JointSpaceControl();

    /**
     * Destructor
     */
    ~JointSpaceControl() = default;

    /**
     * Solve the inverse kinematics of the next pose in the pose list and move robot to pose
     * @return returns true if there is another pose to solve 
     */
    bool moveToNextPose();

    /**
     * Sets up the move group. Must call before moveToNextPose()
     */
    void initializeMoveGroup();
    
private:
    /**
     * Solves the IK for a given frame
     * @param frame The ee frame to try and solve the inverse kinematics for
     */
    void solveIK(const KDL::Frame & frame);

    /**
     * Wrapper function for transformLookup(). Will lookup transform between world and the inputted frame name
     * @param frame_name the name of the frame to look up
     * @param output_frame If the lookup is successful, this will get set to the looked up frame
     * @return True if successful, false otherwise
     */
    bool lookupFrame(std::string & frame_name, KDL::Frame & output_frame);

    /**
     * Moves joints to to goal_angles_
     */
    void moveJoints();
    
    /**
     * Callback for the /robot_description topic. Initializes the IK solver
     * @param msg the message on the /robot_description topic
     */
    void robotDescriptionCallback(const std_msgs::msg::String& msg);

    // subscriber for the robot description
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;

    // KDL
    KDL::Tree tree_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    KDL::JntArray q_; 

    // Frames
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    uint8_t pose_index_ = 0;
    std::array<std::string, 2> frame_names_;

    // MoveIt
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::vector<double> goal_angles_;
};

#endif // JOINT_SPACE_CONTROL