#include <ros/ros.h>
#include <iri_common_drivers_msgs/QueryJointsMovement.h>
#include <iri_common_drivers_msgs/QueryInverseKinematics.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>

    //#include "ac_promp.h"
    //#include "iri_wamik.h"
#include <iri_base_algorithm/iri_base_algorithm.h>

#include <visualization_msgs/Marker.h>

#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

/*   

    useful commands:
    rossrv show iri_common_drivers_msgs/QueryInverseKinematics 

*/
typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;


class cartesian2joint
{
    std::string file_name;
    std::string srv_name_;

    ros::NodeHandle n;

    ros::Publisher marker_pub;

    std::vector<geometry_msgs::PoseStamped> cartesian_trajectory;
    std::vector<sensor_msgs::JointState> joints_trajectory;
    ros::ServiceClient client;
    iri_common_drivers_msgs::QueryInverseKinematics srv;
    TrajClient* traj_client_;

    public:

        cartesian2joint():srv_name_("/iri_wam/iri_wam_tcp_ik/get_wam_ik") 
        {
           client = n.serviceClient<iri_common_drivers_msgs::QueryInverseKinematics>(this->srv_name_.c_str());
           traj_client_ = new TrajClient("/iri_wam/iri_wam_controller/follow_joint_trajectory", true);

        }

        void readTrainingSample()
        {
            std::fstream myfile("cartesian_trajectory.txt", std::ios_base::in);

            std::cout << "Loading the info from training_data_info\n";
            int i =0;
            if(myfile.is_open())
            {
                while (!myfile.eof())
                {
                    double x,y,z;
                    myfile >> x >> y >> z;
                    geometry_msgs::PoseStamped cartesian_pose;
                    cartesian_pose.header.frame_id = "/iri_wam_link_base";
                    cartesian_pose.header.seq = i;
                    cartesian_pose.header.stamp = ros::Time::now();     
                    //try
                    //{
                        cartesian_pose.pose.position.x = x;
                        cartesian_pose.pose.position.y = y;
                        cartesian_pose.pose.position.z = z;
                    //}
                    //catch
                    //{
                    //    ROS_ERROR("File format wrong");
                    //    std::exit(0);
                    //}
                    cartesian_pose.pose.orientation.x = 0.0;
                    cartesian_pose.pose.orientation.y = 0.0;
                    cartesian_pose.pose.orientation.z = 0.0;
                    cartesian_pose.pose.orientation.w = 1.0;

                    cartesian_trajectory.push_back(cartesian_pose);
                    i++;    

                }
            }
            else
            {
                ROS_ERROR("Failed to open file cartesian_trajectory.txt ");
            }
            myfile.close();

            for (int i = 0; i < cartesian_trajectory.size(); ++i)
            {
                std::cout << "x: " << cartesian_trajectory[i].pose.position.x << "  y: " << 
                                      cartesian_trajectory[i].pose.position.y << "  z: " << 
                                      cartesian_trajectory[i].pose.position.z << std::endl;
            }

            for (int i = 0; i < cartesian_trajectory.size(); ++i)
            {   
                srv.request.pose = cartesian_trajectory[i];
            
                if (client.call(srv))
                {
                  ROS_INFO("Inverse Kinematics done");
                  joints_trajectory.push_back(srv.response.joints);       
                  std::cout << "joint 1: " << joints_trajectory[i].position[0] <<
                      "joint 2: " << joints_trajectory[i].position[1] <<
                      "joint 3: " << joints_trajectory[i].position[2] <<
                      "joint 4: " << joints_trajectory[i].position[3] <<
                      "joint 5: " << joints_trajectory[i].position[4] <<
                      "joint 6: " << joints_trajectory[i].position[5] <<
                      "joint 7: " << joints_trajectory[i].position[6] << std::endl;
                }
                else
                {
                  ROS_ERROR("Failed to call service iri_wam_tcp_ik/get_wam_ik");
                  return;
                }

            }
            return;
        }

        void trajectoryMarker(float& f)
        {

            marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
            visualization_msgs::Marker points, line_strip;

            points.header.frame_id = line_strip.header.frame_id = "/iri_wam_link_footprint";
            points.header.stamp = line_strip.header.stamp = ros::Time::now();
            points.ns = line_strip.ns = "points_and_lines";
            points.action = line_strip.action =  visualization_msgs::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w =  1.0;

            points.id = 0;
            line_strip.id = 1;

            points.type = visualization_msgs::Marker::POINTS;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;

                    // POINTS markers use x and y scale for width/height respectively
            points.scale.x = 0.002;
            points.scale.y = 0.002;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_strip.scale.x = 0.001;



            // Points are green
            points.color.g = 1.0f;
            points.color.a = 1.0;

            // Line strip is blue
            line_strip.color.b = 1.0;
            line_strip.color.a = 1.0;


            // Create the vertices for the points and lines
            for (uint32_t i = 0; i < cartesian_trajectory.size(); ++i)
            {
              
              geometry_msgs::Point p;
              p.x = cartesian_trajectory[i].pose.position.x;
              p.y = cartesian_trajectory[i].pose.position.y;
              p.z = cartesian_trajectory[i].pose.position.z;

              points.points.push_back(p);
              line_strip.points.push_back(p);
            }

            marker_pub.publish(points);
            marker_pub.publish(line_strip);
        }

        void writeJointSpaceTrajectory() // writes only positions
        {
            std::string file_name = "joints_trajectory.txt";
            
            std::ofstream myfile (file_name.c_str());

            if (myfile.is_open())
            {
              for (int i = 0; i < joints_trajectory.size(); ++i)
              {
                myfile <<   joints_trajectory[i].position[0] << " " <<
                            joints_trajectory[i].position[1] << " " <<
                            joints_trajectory[i].position[2] << " " <<
                            joints_trajectory[i].position[3] << " " <<
                            joints_trajectory[i].position[4] << " " <<
                            joints_trajectory[i].position[5] << " " <<
                            joints_trajectory[i].position[6] << "\n";
              }
              
              myfile.close();
              std::cout << "\n\n     File saved  :) \n\n";
            }
            else 
            {
                std::cout << "Unable to open the file.\n";
                std::exit(0);
            }
            
            return;
        }

        void goHome()
        {
            while(!traj_client_->waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the joint_trajectory_action server");
            }

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

            goal.trajectory.points.resize(1);
            goal.trajectory.points[0].positions.resize(7);
            goal.trajectory.points[0].positions[0] = 0.0;
            goal.trajectory.points[0].positions[1] = 0.0;
            goal.trajectory.points[0].positions[2] = 0.0;
            goal.trajectory.points[0].positions[3] = 0.0;
            goal.trajectory.points[0].positions[4] = 0.0;
            goal.trajectory.points[0].positions[5] = 0.0;
            goal.trajectory.points[0].positions[6] = 0.0;
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
            goal.trajectory.points[0].time_from_start = ros::Duration(0.2); // in a trajectory with mor epoints, each point should have a different time stamp

            

            goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
            ROS_INFO("Going home");
            traj_client_->sendGoal(goal);
            if (!traj_client_->waitForResult(ros::Duration(10.0)))
            { 
                traj_client_->cancelGoal();
                ROS_INFO("Action did not finish before the time out.\n"); 
            }

            sleep(3.0);

            return;
        }

        void performTrajectory()
        {
            while(!traj_client_->waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the joint_trajectory_action server");
            }

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

            for (int i = 0; i < joints_trajectory.size(); ++i)
            {
                goal.trajectory.points.resize(i+1);
                goal.trajectory.points[i].positions.resize(7);
                goal.trajectory.points[i].positions[0] = joints_trajectory[i].position[0];
                goal.trajectory.points[i].positions[1] = joints_trajectory[i].position[1];
                goal.trajectory.points[i].positions[2] = joints_trajectory[i].position[2];
                goal.trajectory.points[i].positions[3] = joints_trajectory[i].position[3];
                goal.trajectory.points[i].positions[4] = joints_trajectory[i].position[4];
                goal.trajectory.points[i].positions[5] = joints_trajectory[i].position[5];
                goal.trajectory.points[i].positions[6] = joints_trajectory[i].position[6];
                goal.trajectory.points[i].velocities.resize(7);
                goal.trajectory.points[i].velocities[0] = 0.2f;
                goal.trajectory.points[i].velocities[1] = 0.2f;
                goal.trajectory.points[i].velocities[2] = 0.2f;
                goal.trajectory.points[i].velocities[3] = 0.2f;
                goal.trajectory.points[i].velocities[4] = 0.2f;
                goal.trajectory.points[i].velocities[5] = 0.2f;
                goal.trajectory.points[i].velocities[6] = 0.2f;
                goal.trajectory.points[i].accelerations.resize(7);
                goal.trajectory.points[i].accelerations[0] = 0.2f;
                goal.trajectory.points[i].accelerations[1] = 0.2f;
                goal.trajectory.points[i].accelerations[2] = 0.2f;
                goal.trajectory.points[i].accelerations[3] = 0.2f;
                goal.trajectory.points[i].accelerations[4] = 0.2f;
                goal.trajectory.points[i].accelerations[5] = 0.2f;
                goal.trajectory.points[i].accelerations[6] = 0.2f;
                goal.trajectory.points[i].time_from_start = ros::Duration(i*0.3f); // in a trajectory with mor epoints, each point should have a different time stamp

            }

            goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
            ROS_INFO("sending trajectory");
            traj_client_->sendGoal(goal);
             if (!traj_client_->waitForResult(ros::Duration(10.0)))
            { 
                traj_client_->cancelGoal();
                ROS_INFO("Action did not finish before the time out.\n"); 
            }

            return;
        }

};

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "robot_driver");


    cartesian2joint c2j;
    c2j.readTrainingSample();//read the trajectory from the file cartesian_trajectory.txt
    c2j.writeJointSpaceTrajectory();//write tje joints value in joints_trajectory.txt
    
    // move the robot to show the trajectory
    c2j.goHome(); 
    c2j.performTrajectory();

    float f = 0.0;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        c2j.trajectoryMarker(f);
        loop_rate.sleep();  
        ros::spinOnce();
    }

    return 0;
}