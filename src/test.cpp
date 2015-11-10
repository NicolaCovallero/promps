#include <ros/ros.h>
#include <iri_common_drivers_msgs/QueryJointsMovement.h>
#include <iri_common_drivers_msgs/QueryInverseKinematics.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

#include "ac_promp.h"
#include "iri_wamik.h"
#include <iri_base_algorithm/iri_base_algorithm.h>

/*   

    useful commands:
    rossrv show iri_common_drivers_msgs/QueryInverseKinematics 

*/


typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm
{
    void get_joints(geometry_msgs::PoseStamped cartesian_pose)
    {
        std::cout << "########################################" << std::endl;
        std::cout << "Requesting Robot's Inverse Kinematics   " << std::endl;



        return;
    }    
};

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle n;


    //----- EXAMPLE to get the IK of a single pose 
    ros::ServiceClient client = n.serviceClient<iri_common_drivers_msgs::QueryInverseKinematics>("/iri_wam/iri_wam_tcp_ik/get_wam_ik");

    iri_common_drivers_msgs::QueryInverseKinematics srv;
    
    // create 
    geometry_msgs::PoseStamped cartesian_pose;
    cartesian_pose.header.frame_id    = "/iri_wam_link_base";
    cartesian_pose.header.seq = 1;
    cartesian_pose.header.stamp = ros::Time::now();
    cartesian_pose.pose.position.x    = 0.5;
    cartesian_pose.pose.position.y    = 0.5;
    cartesian_pose.pose.position.z    = 0.5;
    cartesian_pose.pose.orientation.x = 0.0;
    cartesian_pose.pose.orientation.y = 0.0;
    cartesian_pose.pose.orientation.z = 0.0;
    cartesian_pose.pose.orientation.w = 1.0;

    srv.request.pose = cartesian_pose;

    sensor_msgs::JointState joints;
    if (client.call(srv))
    {
      ROS_INFO("Inverse Kinematics done");
      joints = srv.response.joints;

      for (int i = 0; i < joints.position.size(); ++i)
      {
          std::cout << "angle joint " << i <<" : " << joints.position[i] << std::endl;
      }

    }
    else
    {
      ROS_ERROR("Failed to call service iri_wam_tcp_ik/get_wam_ik");
      return 1;
    }
    //-------------------------------------------------

    // TO DO:
    // 1)test that the IK was correct sending the joints pose and see if it is correct
    // 2)function ik (in a class)
    // 3)build trajectory 
    // 4)move 

    // 1) move to a point it is not for trajectories
/*    ros::ServiceClient client_joint = n.serviceClient<iri_common_drivers_msgs::QueryJointsMovement>("/iri_wam/iri_wam_controller/joints_move");

    iri_common_drivers_msgs::QueryJointsMovement srv2;
    srv2.request.positions.resize(7);
    srv2.request.positions[0] = joints.position[0];
    srv2.request.positions[1] = joints.position[1];
    srv2.request.positions[2] = joints.position[2];
    srv2.request.positions[3] = joints.position[3];
    srv2.request.positions[4] = joints.position[4];
    srv2.request.positions[5] = joints.position[5];
    srv2.request.positions[6] = joints.position[6];
    srv2.request.velocity = 0.5;
    srv2.request.acceleration = 0.5;

    if (!client_joint.call(srv2))
    {
        ROS_ERROR("Failed to call service joint move");
        return false;
    }
*/

//this stuff is for a trajectory
    TrajClient* traj_client_; // this should be a member of the class
    traj_client_ = new TrajClient("/iri_wam/iri_wam_controller/follow_joint_trajectory", true);
    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the joint_trajectory_action server");
    }

    ROS_INFO("Creating goal msg");
    control_msgs::FollowJointTrajectoryGoal goal;
    // frame_id
    goal.trajectory.header.frame_id = "/iri_wam_link_footprint";
    // joint names
    goal.trajectory.joint_names.resize(7);
    goal.trajectory.joint_names[0] = "iri_wam_joint_1";
    goal.trajectory.joint_names[1] = "iri_wam_joint_2";
    goal.trajectory.joint_names[2] = "iri_wam_joint_3";
    goal.trajectory.joint_names[3] = "iri_wam_joint_4";
    goal.trajectory.joint_names[4] = "iri_wam_joint_5";
    goal.trajectory.joint_names[5] = "iri_wam_joint_6";
    goal.trajectory.joint_names[6] = "iri_wam_joint_7";

    std::cout << "size of joints: " << joints.position.size() << std::endl;
    ROS_INFO("filling first point");
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].positions[0] = 0.7;
    ROS_INFO("filled first angle FIX");
    goal.trajectory.points[0].positions[0] = joints.position[0];
    ROS_INFO("filled first angle");
    goal.trajectory.points[0].positions[1] = joints.position[1];
    goal.trajectory.points[0].positions[2] = joints.position[2];
    goal.trajectory.points[0].positions[3] = joints.position[3];
    goal.trajectory.points[0].positions[4] = joints.position[4];
    goal.trajectory.points[0].positions[5] = joints.position[5];
    goal.trajectory.points[0].positions[6] = joints.position[6];
    goal.trajectory.points[0].velocities.resize(7);
    goal.trajectory.points[0].velocities[0] = 0.2f;
    goal.trajectory.points[0].velocities[1] = 0.2f;
    goal.trajectory.points[0].velocities[2] = 0.2f;
    goal.trajectory.points[0].velocities[3] = 0.2f;
    goal.trajectory.points[0].velocities[4] = 0.2f;
    goal.trajectory.points[0].velocities[5] = 0.2f;
    goal.trajectory.points[0].velocities[6] = 0.2f;
    goal.trajectory.points[0].accelerations.resize(7);
    goal.trajectory.points[0].accelerations[0] = 0.2f;
    goal.trajectory.points[0].accelerations[1] = 0.2f;
    goal.trajectory.points[0].accelerations[2] = 0.2f;
    goal.trajectory.points[0].accelerations[3] = 0.2f;
    goal.trajectory.points[0].accelerations[4] = 0.2f;
    goal.trajectory.points[0].accelerations[5] = 0.2f;
    goal.trajectory.points[0].accelerations[6] = 0.2f;
    goal.trajectory.points[0].time_from_start = ros::Duration(1*0.3f); // in a trajectory with mor epoints, each point should have a different time stamp


    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    ROS_INFO("sending goal");
    traj_client_->sendGoal(goal);
     if (!traj_client_->waitForResult(ros::Duration(10.0)))
    { 
        traj_client_->cancelGoal();
        ROS_INFO("Action did not finish before the time out.\n"); 
    }

    // planning
    //WAMIK WAM_kinematics;
    
    return 0;
}