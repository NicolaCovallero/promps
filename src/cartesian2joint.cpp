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
#include <visualization_msgs/MarkerArray.h>


#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

#include <tf/transform_datatypes.h>


/*   
    useful commands:
    rossrv show iri_common_drivers_msgs/QueryInverseKinematics 

*/
typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

std::string trajectory_frame = "/iri_wam_link_base";

inline bool isInteger(const std::string & s)
{
   if(s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false ;

   char * p ;
   strtol(s.c_str(), &p, 10) ;

   return (*p == 0) ;
}

tf::Quaternion quaternionFromVector(tf::Vector3 v2)
{
  // all the normalizations are important! except the alst one (with this method!)
  tf::Vector3 v1 (0,0,1); // this to make it works with ros
  v2.normalize();

  tf::Vector3 cross_vector = v1.cross(v2);
  cross_vector.normalize();
  
  double angle = acos(v1.dot(v2))/2;
  //check for nans values
  // this occures only when v2 has the same direction of v1 (v2.normalize()=v1.normalize())
  if(cross_vector.x() != cross_vector.x()) //this will be true only for nans value
  {
    cross_vector.setX(0);
    cross_vector.setY(0);
    cross_vector.setZ(0);
    // check if the sense of v2 is positive or negative
    if(v2.z()<0)
    { // this means that the sense has to eb negative,
      // and it is equal to a rotation in the y axis of M_PI
      angle = angle + M_PI;
      cross_vector.setY(1);
    }
    
  }

  // debug:
  //std::cout << "angle(degree): " << angle*180/M_PI << std::endl;
  //std::cout << "sin(angle): " << sin(angle) << std::endl;

  // Build quaternion
  tf::Quaternion quatern; 
  quatern.setX(cross_vector.x() * sin(angle));
  quatern.setY(cross_vector.y() * sin(angle));
  quatern.setZ(cross_vector.z() * sin(angle));
  quatern.setW(cos(angle));  
  quatern.normalize(); 

  return quatern;
}

struct via_point_
{
    double x,y,z;

};

class cartesian2joint
{
    char file_name[50];
    char file_name_jt[50];
    std::string srv_name_;

    ros::NodeHandle n;

    

    double time_trajectory; //time for the trajectoiry execution (total time)
    std::vector<geometry_msgs::PoseStamped> cartesian_trajectory;
    std::vector<sensor_msgs::JointState> joints_trajectory;
    ros::ServiceClient client;
    iri_common_drivers_msgs::QueryInverseKinematics srv;
    TrajClient* traj_client_;
    ros::Publisher marker_pub,joint_state_pub,demos_pub,mean_pub,samples_pub;
    ros::Publisher demos_points_pub,mean_points_pub,samples_points_pub,via_points_pub;

    //demo trajectories
    std::vector<std::vector<geometry_msgs::PoseStamped> > demoTraj;
    std::vector<std::vector<geometry_msgs::PoseStamped> > sampleTraj;
    std::vector<geometry_msgs::PoseStamped> mean; //mean of the distribution
    via_point_ viaPoint_coord;


    public:

        cartesian2joint():srv_name_("/iri_wam/iri_wam_tcp_ik/get_wam_ik") 
        {
           client = n.serviceClient<iri_common_drivers_msgs::QueryInverseKinematics>(this->srv_name_.c_str());
           traj_client_ = new TrajClient("/iri_wam/iri_wam_controller/follow_joint_trajectory", true);
           marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
           demos_pub = n.advertise<visualization_msgs::MarkerArray>("demos_marker", 10);
           mean_pub = n.advertise<visualization_msgs::MarkerArray>("mean_marker", 10);
           samples_pub = n.advertise<visualization_msgs::MarkerArray>("samples_marker", 10);

           demos_points_pub = n.advertise<visualization_msgs::MarkerArray>("demos_points_marker", 10);
           mean_points_pub = n.advertise<visualization_msgs::MarkerArray>("mean_points_marker", 10);
           samples_points_pub = n.advertise<visualization_msgs::MarkerArray>("samples_points_marker", 10);
           via_points_pub = n.advertise<visualization_msgs::MarkerArray>("via_points_marker", 10);

           joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1); 
        }

        void readTrainingSample() //reads a cartesian trajectory
        {

            std::cout << "Input file for cartesian_trajectory:\n";
            std::cin >> file_name;

            std::fstream myfile(file_name, std::ios_base::in);

            std::cout << "Loading the info from training_data_info\n";
            int i =0;
            
            //poiting down
            tf::Vector3 vec;
            vec.setX(0);vec.setY(0);vec.setZ(-1);
            tf::Quaternion quat = quaternionFromVector(vec);
            std::cout << "quat  x: " << quat.x() << " y: " << quat.y() << " z: " << quat.z() << " w: " << quat.w() << "\n";
            sleep(1);

            if(myfile.is_open())
            {
                if(cartesian_trajectory.size() >0 )
                    cartesian_trajectory.resize(0);

                myfile >> time_trajectory;
                while (!myfile.eof())
                {
                    double x,y,z;
                    myfile >> x >> y >> z;
                    geometry_msgs::PoseStamped cartesian_pose;
                    cartesian_pose.header.frame_id = trajectory_frame;
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
                    cartesian_pose.pose.orientation.x = quat.x();
                    cartesian_pose.pose.orientation.y = quat.y();
                    cartesian_pose.pose.orientation.z = quat.z();
                    cartesian_pose.pose.orientation.w = quat.z();

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

            return;
        }

        void cartesianToJointSpace()
        {
            if(cartesian_trajectory.size()>0 && joints_trajectory.size()>0)
                joints_trajectory.resize(0);

            for (int i = 0; i < cartesian_trajectory.size(); ++i)
            {   
                srv.request.pose = cartesian_trajectory[i];
                std::cout << "Requesting IK of x: " << cartesian_trajectory[i].pose.position.x << " y: " <<
                                            cartesian_trajectory[i].pose.position.y << " z: " <<
                                             cartesian_trajectory[i].pose.position.z << std::endl;
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

        void trajectoryMarker()
        {
            if(cartesian_trajectory.size()>0)
            {
                visualization_msgs::Marker points, line_strip;

                points.header.frame_id = line_strip.header.frame_id = trajectory_frame;
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
                line_strip.scale.x = 0.005;



                // Points are green
                points.color.g = 1.0f;
                points.color.a = 1.0;

                // Line strip is blue
                line_strip.color.b = 1.0;
                line_strip.color.a = 1.0;


                // Create the vertices for the points and lines
                std::cout << "publishign trajectory. Size cartesian_trajectory: " << cartesian_trajectory.size() << std::endl;
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
            else
                ROS_ERROR("No trajectory in cartesian space to show");
        }

        void loadJointTrajectory()
        {
            std::cout << "Input file for joints_trajectory:\n";
            std::cin >> file_name_jt;
            std::fstream myfile(file_name_jt, std::ios_base::in);

            if(myfile.is_open())
            {
                myfile >> time_trajectory;

                if(joints_trajectory.size()>0)
                    joints_trajectory.resize(0);

                while (!myfile.eof())
                {
                    sensor_msgs::JointState jointState;
                    jointState.position.resize(7);
                    myfile  >>    jointState.position[0] 
                            >>    jointState.position[1] 
                            >>    jointState.position[2] 
                            >>    jointState.position[3] 
                            >>    jointState.position[4] 
                            >>    jointState.position[5] 
                            >>    jointState.position[6];

                    std::cout << jointState.position[0] << " " <<
                               jointState.position[1]  << " " <<
                                jointState.position[2] << " " <<
                                jointState.position[3] << " " <<
                                jointState.position[4] << " " <<
                                jointState.position[5] << " "<<
                                jointState.position[6]<< "\n";

                    joints_trajectory.push_back(jointState);
                }
                std::cout << "Trajectory readed. " << joints_trajectory.size() << " points\n";

            }
            else
                std::cout << "Impossible to open the file: " << file_name_jt << std::endl;
        }

        void testJointState()
        {

            sensor_msgs::JointState jointState;
            jointState.position.resize(7);
            jointState.position[0] =0.4;
            jointState.position[1] =0.4;
            jointState.position[2] =0.4;
            jointState.position[3] =0.4;
            jointState.position[4] =0.4;
            jointState.position[5]=0.4; 
            jointState.position[6]=0.4;
            jointState.header.stamp = ros::Time::now();
            jointState.name.resize(7);
            jointState.name[0] = "iri_wam_joint_1";
            jointState.name[1] = "iri_wam_joint_2";
            jointState.name[2] = "iri_wam_joint_3";
            jointState.name[3] = "iri_wam_joint_4";
            jointState.name[4] = "iri_wam_joint_5";
            jointState.name[5] = "iri_wam_joint_6";
            jointState.name[6] = "iri_wam_joint_7";

            joint_state_pub.publish(jointState);
            return;
        }



        void writeJointSpaceTrajectory() // writes only positions
        {
            if(joints_trajectory.size()>0)
            {
                std::string file_name = "joints_trajectory.txt";
                
                std::ofstream myfile (file_name.c_str());

                if (myfile.is_open())
                {
                  myfile << time_trajectory <<"\n";
                  for (int i = 0; i < joints_trajectory.size()-1; ++i)
                  {
                    myfile <<   joints_trajectory[i].position[0] << " " <<
                                joints_trajectory[i].position[1] << " " <<
                                joints_trajectory[i].position[2] << " " <<
                                joints_trajectory[i].position[3] << " " <<
                                joints_trajectory[i].position[4] << " " <<
                                joints_trajectory[i].position[5] << " " <<
                                joints_trajectory[i].position[6] << "\n";
                  }

                  if(joints_trajectory.size()>0)
                  {
                    int i = joints_trajectory.size()-1;
                    myfile <<   joints_trajectory[i].position[0] << " " <<
                                joints_trajectory[i].position[1] << " " <<
                                joints_trajectory[i].position[2] << " " <<
                                joints_trajectory[i].position[3] << " " <<
                                joints_trajectory[i].position[4] << " " <<
                                joints_trajectory[i].position[5] << " " <<
                                joints_trajectory[i].position[6];
                  }
                  
                  myfile.close();
                  std::cout << "\n\n     File saved  :) \n\n";
                }
                else 
                {
                    std::cout << "Unable to open the file.\n";
                    std::exit(0);
                }
            }
            else
                ROS_ERROR("No joints_trajectory");
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
            goal.trajectory.points[0].velocities[0] = 0.0f;
            goal.trajectory.points[0].velocities[1] = 0.0f;
            goal.trajectory.points[0].velocities[2] = 0.0f;
            goal.trajectory.points[0].velocities[3] = 0.0f;
            goal.trajectory.points[0].velocities[4] = 0.0f;
            goal.trajectory.points[0].velocities[5] = 0.0f;
            goal.trajectory.points[0].velocities[6] = 0.0f;
            goal.trajectory.points[0].accelerations.resize(7);
            goal.trajectory.points[0].accelerations[0] = 0.0f;
            goal.trajectory.points[0].accelerations[1] = 0.0f;
            goal.trajectory.points[0].accelerations[2] = 0.0f;
            goal.trajectory.points[0].accelerations[3] = 0.0f;
            goal.trajectory.points[0].accelerations[4] = 0.0f;
            goal.trajectory.points[0].accelerations[5] = 0.0f;
            goal.trajectory.points[0].accelerations[6] = 0.0f;
            goal.trajectory.points[0].time_from_start = ros::Duration(2); // in a trajectory with mor epoints, each point should have a different time stamp

            

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

        void performTrajectory() //with GAZEBO
        {
            //trajectoryMarker();
            if(joints_trajectory.size()>0)
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
                    goal.trajectory.points[i].velocities[0] = 0.00f;
                    goal.trajectory.points[i].velocities[1] = 0.00f;
                    goal.trajectory.points[i].velocities[2] = 0.00f;
                    goal.trajectory.points[i].velocities[3] = 0.00f;
                    goal.trajectory.points[i].velocities[4] = 0.00f;
                    goal.trajectory.points[i].velocities[5] = 0.00f;
                    goal.trajectory.points[i].velocities[6] = 0.00f;
                    goal.trajectory.points[i].accelerations.resize(7);
                    goal.trajectory.points[i].accelerations[0] = 0.0f;
                    goal.trajectory.points[i].accelerations[1] = 0.0f;
                    goal.trajectory.points[i].accelerations[2] = 0.0f;
                    goal.trajectory.points[i].accelerations[3] = 0.0f;
                    goal.trajectory.points[i].accelerations[4] = 0.0f;
                    goal.trajectory.points[i].accelerations[5] = 0.0f;
                    goal.trajectory.points[i].accelerations[6] = 0.0f;
                    goal.trajectory.points[i].time_from_start = ros::Duration(i*time_trajectory/joints_trajectory.size()); // in a trajectory with mor epoints, each point should have a different time stamp

                }

                goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
                ROS_INFO("sending trajectory");
                traj_client_->sendGoal(goal);
                 if (!traj_client_->waitForResult(ros::Duration(time_trajectory*2)))
                { 
                    traj_client_->cancelGoal();
                    ROS_INFO("Action did not finish before the time out.\n"); 
                }
            }
            else
                ROS_ERROR("No trajectory in joint space loaded");
            return;
        }

        void performTrajectoryRVIZ()
        {
            if(joints_trajectory.size()>0)
            {
                ros::Rate loopRate(1/(time_trajectory/joints_trajectory.size()));
                for (int i = 0; i < joints_trajectory.size(); ++i)
                {
                    sensor_msgs::JointState jointState;
                    jointState.position.resize(7);
                    jointState.position[0] = joints_trajectory[i].position[0];
                    jointState.position[1] = joints_trajectory[i].position[1];
                    jointState.position[2] = joints_trajectory[i].position[2];
                    jointState.position[3] = joints_trajectory[i].position[3];
                    jointState.position[4] = joints_trajectory[i].position[4];
                    jointState.position[5] = joints_trajectory[i].position[5];
                    jointState.position[6] = joints_trajectory[i].position[6];
                    jointState.header.stamp = ros::Time::now();
                    jointState.name.resize(7);
                    jointState.name[0] = "iri_wam_joint_1";
                    jointState.name[1] = "iri_wam_joint_2";
                    jointState.name[2] = "iri_wam_joint_3";
                    jointState.name[3] = "iri_wam_joint_4";
                    jointState.name[4] = "iri_wam_joint_5";
                    jointState.name[5] = "iri_wam_joint_6";
                    jointState.name[6] = "iri_wam_joint_7";

                    joint_state_pub.publish(jointState);

                    loopRate.sleep();

                }

            }
            else
                ROS_ERROR("No trajectory in joint space loaded");
            return;
        }

        void loadDemoTrajectories()
        {

            std::cout << "Use load all the files with demoTraj#.txt inside the current folder.\n";
            char str[30];

            demoTraj.resize(0);

            //poiting down
            tf::Vector3 vec;
            vec.setX(0);vec.setY(0);vec.setZ(-1);
            tf::Quaternion quat = quaternionFromVector(vec);
            std::cout << "quat  x: " << quat.x() << " y: " << quat.y() << " z: " << quat.z() << " w: " << quat.w() << "\n";
            sleep(1);

            int n=0;
            while(true)
            {

                sprintf(str,"demoTraj%d.txt",n);
                std::fstream myfile(str, std::ios_base::in);

                std::cout << "Loading the info from " << str << "\n";
                int i =0;
                
                std::vector<geometry_msgs::PoseStamped> sample;

                if(myfile.is_open())
                {
                    double nothing; // this variable is to store the time, the time is useless for plotting trajectories!
                    myfile >> nothing;  
                    while (!myfile.eof())
                    {
                        double x,y,z;
                        myfile >> x >> y >> z;
                        geometry_msgs::PoseStamped cartesian_pose;
                        cartesian_pose.header.frame_id = trajectory_frame;
                        cartesian_pose.header.seq = i;
                        cartesian_pose.header.stamp = ros::Time::now();     
                        cartesian_pose.pose.position.x = x;
                        cartesian_pose.pose.position.y = y;
                        cartesian_pose.pose.position.z = z;
                        cartesian_pose.pose.orientation.x = quat.x();
                        cartesian_pose.pose.orientation.y = quat.y();
                        cartesian_pose.pose.orientation.z = quat.z();
                        cartesian_pose.pose.orientation.w = quat.z();

                        sample.push_back(cartesian_pose);
                        i++;    

                    }
                }
                else
                {
                    std::cout << "failed to read: " << str << " file not existing\n";
                    break;
                }
                myfile.close();

                for (int i = 0; i < cartesian_trajectory.size(); ++i)
                {
                    std::cout << "x: " << cartesian_trajectory[i].pose.position.x << "  y: " << 
                                          cartesian_trajectory[i].pose.position.y << "  z: " << 
                                          cartesian_trajectory[i].pose.position.z << std::endl;
                }

                demoTraj.push_back(sample);
                n++;
            }

            return;
        }

        void loadSampleTrajectories()
        {

            std::cout << "Use load all the files with sampleTraj#.txt inside the current folder.\n";
            char str[30];

            sampleTraj.resize(0);

            //poiting down
            tf::Vector3 vec;
            vec.setX(0);vec.setY(0);vec.setZ(-1);
            tf::Quaternion quat = quaternionFromVector(vec);
            std::cout << "quat  x: " << quat.x() << " y: " << quat.y() << " z: " << quat.z() << " w: " << quat.w() << "\n";
            sleep(1);

            int n=0;
            while(true)
            {

                sprintf(str,"sampleTraj%d.txt",n);
                std::fstream myfile(str, std::ios_base::in);

                std::cout << "Loading the info from " << str << "\n";
                int i =0;
                
                std::vector<geometry_msgs::PoseStamped> sample;

                if(myfile.is_open())
                {
                    double nothing; // this variable is to store the time, the time is useless for plotting trajectories!
                    myfile >> nothing;  
                    while (!myfile.eof())
                    {
                        double x,y,z;
                        myfile >> x >> y >> z;
                        geometry_msgs::PoseStamped cartesian_pose;
                        cartesian_pose.header.frame_id = trajectory_frame;
                        cartesian_pose.header.seq = i;
                        cartesian_pose.header.stamp = ros::Time::now();     
                        cartesian_pose.pose.position.x = x;
                        cartesian_pose.pose.position.y = y;
                        cartesian_pose.pose.position.z = z;
                        cartesian_pose.pose.orientation.x = quat.x();
                        cartesian_pose.pose.orientation.y = quat.y();
                        cartesian_pose.pose.orientation.z = quat.z();
                        cartesian_pose.pose.orientation.w = quat.z();

                        sample.push_back(cartesian_pose);
                        i++;    

                    }
                }
                else
                {
                    std::cout << "failed to read: " << str << " file not existing\n";
                    break;
                }
                myfile.close();

                for (int i = 0; i < cartesian_trajectory.size(); ++i)
                {
                    std::cout << "x: " << cartesian_trajectory[i].pose.position.x << "  y: " << 
                                          cartesian_trajectory[i].pose.position.y << "  z: " << 
                                          cartesian_trajectory[i].pose.position.z << std::endl;
                }

                sampleTraj.push_back(sample);
                n++;
            }

            return;
        }

        void loadSampleMean()
        {

            std::cout << "Waht mean to load?\n";
            char str[20];
            std::cin >> str;

            mean.resize(0);

            //poiting down
            tf::Vector3 vec;
            vec.setX(0);vec.setY(0);vec.setZ(-1);
            tf::Quaternion quat = quaternionFromVector(vec);
            std::cout << "quat  x: " << quat.x() << " y: " << quat.y() << " z: " << quat.z() << " w: " << quat.w() << "\n";
            sleep(1);

            std::fstream myfile(str, std::ios_base::in);

            std::cout << "Loading the info from " << str << "\n";
            int i =0;
            
            std::vector<geometry_msgs::PoseStamped> sample;

            if(myfile.is_open())
            {
                double nothing; // this variable is to store the time, the time is useless for plotting trajectories!
                myfile >> nothing;  
                while (!myfile.eof())
                {
                    double x,y,z;
                    myfile >> x >> y >> z;
                    geometry_msgs::PoseStamped cartesian_pose;
                    cartesian_pose.header.frame_id = trajectory_frame;
                    cartesian_pose.header.seq = i;
                    cartesian_pose.header.stamp = ros::Time::now();     
                    cartesian_pose.pose.position.x = x;
                    cartesian_pose.pose.position.y = y;
                    cartesian_pose.pose.position.z = z;
                    cartesian_pose.pose.orientation.x = quat.x();
                    cartesian_pose.pose.orientation.y = quat.y();
                    cartesian_pose.pose.orientation.z = quat.z();
                    cartesian_pose.pose.orientation.w = quat.z();

                    sample.push_back(cartesian_pose);
                    i++;    

                }
            }
            else
            {
                std::cout << "failed to read: " << str << " file not existing\n";
            }
            myfile.close();

            for (int i = 0; i < cartesian_trajectory.size(); ++i)
            {
                std::cout << "x: " << cartesian_trajectory[i].pose.position.x << "  y: " << 
                                      cartesian_trajectory[i].pose.position.y << "  z: " << 
                                      cartesian_trajectory[i].pose.position.z << std::endl;
            }

            mean = sample;

            return;
        }

        void plotTrajectory(  std::vector<geometry_msgs::PoseStamped> & traj,
                                double  r,double  g,double  b,
                                visualization_msgs::MarkerArray & traj_mark,
                                visualization_msgs::MarkerArray & traj_points_mark,
                                int n
                                )
        {
            
            if(traj.size()>0)
            {
                visualization_msgs::Marker  points,line_strip;

                points.header.frame_id = line_strip.header.frame_id = trajectory_frame;
                points.header.stamp = line_strip.header.stamp = ros::Time::now();
                points.ns = line_strip.ns = "points_and_lines";
                points.action = line_strip.action =  visualization_msgs::Marker::ADD;
                points.pose.orientation.w =line_strip.pose.orientation.w =  1.0;
                points.id = line_strip.id = n;
                points.type = visualization_msgs::Marker::POINTS;
                line_strip.type = visualization_msgs::Marker::LINE_STRIP;

                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                line_strip.scale.x = 0.003;
                line_strip.scale.y = 0.003;
                line_strip.scale.z = 0.003;

                points.scale.x = 0.003;
                points.scale.y = 0.003;
                points.scale.z = 0.003;

                line_strip.color.r = r;
                line_strip.color.g = g;
                line_strip.color.b = b;
                line_strip.color.a = 1.0;

                points.color.r = r;
                points.color.g = g;
                points.color.b = b;
                points.color.a = 1.0;

                // Create the vertices for the points and lines
                std::cout << "publishign trajectory. Size cartesian_trajectory: " << traj.size() << std::endl;
                for (uint32_t i = 0; i < traj.size(); ++i)
                {
                  
                  geometry_msgs::Point p;
                  p.x = traj[i].pose.position.x;
                  p.y = traj[i].pose.position.y;
                  p.z = traj[i].pose.position.z;

                  line_strip.points.push_back(p);
                  points.points.push_back(p);
                }
                traj_mark.markers.push_back(line_strip);
                traj_points_mark.markers.push_back(points);

            }
            else
                ROS_ERROR("No trajectory in cartesian space to show");
        }

        void plotDemoTrajectories()
        {
            std::cout << "What demos do you want to plot?\n"<<"a integer to plot the relative trajectory (1=0) or \"a\" for all\n"; 
            char opt[10];
            std::cin >> opt;

            int n;

            if (strcmp(opt,"a")==0)
            {
                visualization_msgs::MarkerArray traj_mark;
                visualization_msgs::MarkerArray traj_points_mark;
                for (int i = 0; i < demoTraj.size(); ++i)
                {
                    plotTrajectory(demoTraj[i],0,0,1,traj_mark,traj_points_mark,i);
                }
                demos_pub.publish(traj_mark);
                demos_points_pub.publish(traj_points_mark);

            }
            else if (isInteger(opt))
            {
                n = atof(opt);
                visualization_msgs::MarkerArray traj_mark;
                visualization_msgs::MarkerArray traj_points_mark;
                plotTrajectory(demoTraj[n],0,0,1,traj_mark,traj_points_mark,n);
                demos_pub.publish(traj_mark);
                demos_points_pub.publish(traj_points_mark);

            }
            else
                std::cout << opt <<" no valid input\n";

            return;
        }

        void plotSampleTrajectories()
        {
            std::cout << "What samples do you want to plot?\n"<<"a integer to plot the relative trajectory (1=0) or \"a\" for all\n"; 
            char opt[10];
            std::cin >> opt;

            int n;

            if (strcmp(opt,"a")==0)
            {
                visualization_msgs::MarkerArray traj_mark;
                visualization_msgs::MarkerArray traj_points_mark;

                for (int i = 0; i < sampleTraj.size(); ++i)
                {
                    plotTrajectory(sampleTraj[i],0,1,0,traj_mark ,traj_points_mark  ,i );
                }
                samples_pub.publish(traj_mark);
                samples_points_pub.publish(traj_points_mark);
            }
            else if (isInteger(opt))
            {
                n = atof(opt);
                visualization_msgs::MarkerArray traj_mark;
                visualization_msgs::MarkerArray traj_points_mark;

                plotTrajectory(sampleTraj[n],0,1,0,traj_mark ,traj_points_mark ,n  );
                samples_pub.publish(traj_mark);
                samples_points_pub.publish(traj_points_mark);
            }
            else
                std::cout << opt <<" no valid input\n";

            return;
        }

        void plotMeanTrajectory()
        {
            visualization_msgs::MarkerArray traj_mark;
            visualization_msgs::MarkerArray traj_points_mark;

            plotTrajectory(mean,1,0,0,traj_mark,traj_points_mark,1);
            mean_pub.publish(traj_mark);
            mean_points_pub.publish(traj_points_mark);
            return;
        }

        void position() //put the robot to the 3D position - With Gazebo Controller
        {
            //call IK
            int i,p;
            std::cout << "Sample trajectorty number:\n";
            std::cin >> i;
            std::cout << "The sample trajectory has "<< sampleTraj[i].size() << " points. Select a point:\n";
            std::cin >> p;
            
            if(p >= sampleTraj[i].size())
            {
                std::cout << "invalid point. Out of size\n";
                return;
            }

            geometry_msgs::PoseStamped point;
            point = sampleTraj[i][p];

            srv.request.pose = point;
            std::cout << "Requesting IK of x: " << point.pose.position.x << " y: " <<
                                        point.pose.position.y << " z: " <<
                                         point.pose.position.z << std::endl;

            sensor_msgs::JointState jointState;
            if (client.call(srv))
            {
              ROS_INFO("Inverse Kinematics done");
              jointState = srv.response.joints;       
              std::cout << "joint 1: " << jointState.position[0] <<
                  "joint 2: " << jointState.position[1] <<
                  "joint 3: " << jointState.position[2] <<
                  "joint 4: " << jointState.position[3] <<
                  "joint 5: " << jointState.position[4] <<
                  "joint 6: " << jointState.position[5] <<
                  "joint 7: " << jointState.position[6] << std::endl;
            }
            else
            {
              ROS_ERROR("Failed to call service iri_wam_tcp_ik/get_wam_ik");
              return;
            }

            //go to that position //with gazebo
            while(!traj_client_->waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the joint_trajectory_action server");
            }

            control_msgs::FollowJointTrajectoryGoal goal;
            // frame_id
            goal.trajectory.header.frame_id = trajectory_frame;
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
            goal.trajectory.points[0].positions[0] = jointState.position[0];
            goal.trajectory.points[0].positions[1] = jointState.position[1];
            goal.trajectory.points[0].positions[2] = jointState.position[2];
            goal.trajectory.points[0].positions[3] = jointState.position[3];
            goal.trajectory.points[0].positions[4] = jointState.position[4];
            goal.trajectory.points[0].positions[5] = jointState.position[5];
            goal.trajectory.points[0].positions[6] = jointState.position[6];
            goal.trajectory.points[0].velocities.resize(7);
            goal.trajectory.points[0].velocities[0] = 0.0f;
            goal.trajectory.points[0].velocities[1] = 0.0f;
            goal.trajectory.points[0].velocities[2] = 0.0f;
            goal.trajectory.points[0].velocities[3] = 0.0f;
            goal.trajectory.points[0].velocities[4] = 0.0f;
            goal.trajectory.points[0].velocities[5] = 0.0f;
            goal.trajectory.points[0].velocities[6] = 0.0f;
            goal.trajectory.points[0].accelerations.resize(7);
            goal.trajectory.points[0].accelerations[0] = 0.0f;
            goal.trajectory.points[0].accelerations[1] = 0.0f;
            goal.trajectory.points[0].accelerations[2] = 0.0f;
            goal.trajectory.points[0].accelerations[3] = 0.0f;
            goal.trajectory.points[0].accelerations[4] = 0.0f;
            goal.trajectory.points[0].accelerations[5] = 0.0f;
            goal.trajectory.points[0].accelerations[6] = 0.0f;
            goal.trajectory.points[0].time_from_start = ros::Duration(2); // in a trajectory with mor epoints, each point should have a different time stamp           

            goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
            ROS_INFO("Going to point");
            traj_client_->sendGoal(goal);
            if (!traj_client_->waitForResult(ros::Duration(10.0)))
            { 
                traj_client_->cancelGoal();
                ROS_INFO("Action did not finish before the time out.\n"); 
            }

            sleep(3.0);

            return;
            

        }

        void positionViaPoint() //put the robot to the 3D position - With Gazebo Controller
        {
            //call IK
            
            geometry_msgs::PoseStamped point;
            
            //poiting down
            tf::Vector3 vec;
            vec.setX(0);vec.setY(0);vec.setZ(-1);
            tf::Quaternion quat = quaternionFromVector(vec);

            point.header.frame_id = trajectory_frame;
            point.header.seq = 1;
            point.header.stamp = ros::Time::now();     
            //try
            //{
                point.pose.position.x = viaPoint_coord.x;
                point.pose.position.y = viaPoint_coord.y;
                point.pose.position.z = viaPoint_coord.z;
            //}
            //catch
            //{
            //    ROS_ERROR("File format wrong");
            //    std::exit(0);
            //}
            point.pose.orientation.x = quat.x();
            point.pose.orientation.y = quat.y();
            point.pose.orientation.z = quat.z();
            point.pose.orientation.w = quat.z();


            srv.request.pose = point;
            std::cout << "Requesting IK of x: " << point.pose.position.x << " y: " <<
                                        point.pose.position.y << " z: " <<
                                         point.pose.position.z << std::endl;

            sensor_msgs::JointState jointState;
            if (client.call(srv))
            {
              ROS_INFO("Inverse Kinematics done");
              jointState = srv.response.joints;       
              std::cout << "joint 1: " << jointState.position[0] <<
                  "joint 2: " << jointState.position[1] <<
                  "joint 3: " << jointState.position[2] <<
                  "joint 4: " << jointState.position[3] <<
                  "joint 5: " << jointState.position[4] <<
                  "joint 6: " << jointState.position[5] <<
                  "joint 7: " << jointState.position[6] << std::endl;
            }
            else
            {
              ROS_ERROR("Failed to call service iri_wam_tcp_ik/get_wam_ik");
              return;
            }

            //go to that position //with gazebo
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
            goal.trajectory.points[0].positions[0] = jointState.position[0];
            goal.trajectory.points[0].positions[1] = jointState.position[1];
            goal.trajectory.points[0].positions[2] = jointState.position[2];
            goal.trajectory.points[0].positions[3] = jointState.position[3];
            goal.trajectory.points[0].positions[4] = jointState.position[4];
            goal.trajectory.points[0].positions[5] = jointState.position[5];
            goal.trajectory.points[0].positions[6] = jointState.position[6];
            goal.trajectory.points[0].velocities.resize(7);
            goal.trajectory.points[0].velocities[0] = 0.0f;
            goal.trajectory.points[0].velocities[1] = 0.0f;
            goal.trajectory.points[0].velocities[2] = 0.0f;
            goal.trajectory.points[0].velocities[3] = 0.0f;
            goal.trajectory.points[0].velocities[4] = 0.0f;
            goal.trajectory.points[0].velocities[5] = 0.0f;
            goal.trajectory.points[0].velocities[6] = 0.0f;
            goal.trajectory.points[0].accelerations.resize(7);
            goal.trajectory.points[0].accelerations[0] = 0.0f;
            goal.trajectory.points[0].accelerations[1] = 0.0f;
            goal.trajectory.points[0].accelerations[2] = 0.0f;
            goal.trajectory.points[0].accelerations[3] = 0.0f;
            goal.trajectory.points[0].accelerations[4] = 0.0f;
            goal.trajectory.points[0].accelerations[5] = 0.0f;
            goal.trajectory.points[0].accelerations[6] = 0.0f;
            goal.trajectory.points[0].time_from_start = ros::Duration(2); // in a trajectory with mor epoints, each point should have a different time stamp           

            goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
            ROS_INFO("Go to via point");
            traj_client_->sendGoal(goal);
            if (!traj_client_->waitForResult(ros::Duration(10.0)))
            { 
                traj_client_->cancelGoal();
                ROS_INFO("Action did not finish before the time out.\n"); 
            }

            sleep(3.0);

            return;
            

        }

        void viaPoint()
        {

            double x,y,z;
            std::cout << "X coordinate of the via point:\n";
            std::cin >> x;
            std::cout << "Y coordinate of the via point:\n";
            std::cin >> y;
            std::cout << "Z coordinate of the via point:\n";
            std::cin >> z;
            viaPoint_coord.x=x;
            viaPoint_coord.y=y;
            viaPoint_coord.z=z;


            visualization_msgs::MarkerArray  viaPoints;
            visualization_msgs::Marker  points;           

            points.header.frame_id  = trajectory_frame;
            points.header.stamp = ros::Time::now();
            points.ns =  "via_point";
            points.action =   visualization_msgs::Marker::ADD;
            points.id =  1;
            points.type = visualization_msgs::Marker::SPHERE;

            points.scale.x = 0.05;
            points.scale.y = 0.05;
            points.scale.z = 0.05;

            points.pose.orientation.w = 1;

            points.color.r = 1;
            points.color.a = 0.5;


            points.pose.position.x = x;
            points.pose.position.y = y;
            points.pose.position.z = z;

            //points.points.push_back(p);

            viaPoints.markers.push_back(points);

            via_points_pub.publish(viaPoints);

            return;
        }

        void help()
        {
            std::cout << "-rc read cartesian trajectory in joint space \n"
                      << "-w write joints_trajectory\n"
                      << "-rj read joints_trajectory\n"
                      << "-gh go home\n"
                      << "-h help\n"
                      << "-pt performTrajectory\n"
                      << "-st show trayectory\n"
                      << "-c2j cartesian to joint space conversion IK \n"
                      << "-ptr perform trajectory in RVIZ (no Gazebo - No Physic)\n"
                      << "-ldt load demo trajectories\n"
                      << "-lst load sample trajectories\n"
                      << "-lm load mean trajectory\n"
                      << "-plot_d plot demos (training samples - blue)\n"
                      << "-plot_s plot samples(green)\n"    
                      << "-plot_m plot mean (red)\n"     
                      << "-plot_v plot via point\n"  
                      << "-gtp go to specific point of a trajectory\n" 
                      << "-gtvp go to via point PREVIOUSLY specified\n"                                                                 
                      << "-exit exit\n" ;
        }



        void askForOption()
        {
            std::cout << "What to do now? [-h to see the available options]\n"; 
            std::string opt;
            std::cin >> opt;

            if(opt=="h")
                help();
            else if(opt=="rc")
                readTrainingSample();
            else if(opt=="w")  
                writeJointSpaceTrajectory();
            else if(opt=="gh")
                goHome();          
            else if(opt=="pt")
                performTrajectory();
            else if(opt=="st")
                trajectoryMarker();    
            else if(opt=="rj")
                loadJointTrajectory();
            else if(opt=="tjs")
                testJointState();
            else if(opt=="c2j")
                cartesianToJointSpace();
            else if(opt=="ptr")
                performTrajectoryRVIZ();
            else if(opt=="ldt")
                loadDemoTrajectories();        
            else if(opt=="lst")
                loadSampleTrajectories();
            else if(opt=="lm")
                loadSampleMean();
            else if(opt=="plot_d")
                plotDemoTrajectories();
            else if(opt=="plot_s")
                plotSampleTrajectories();
            else if(opt=="plot_m")
                plotMeanTrajectory();
            else if(opt=="plot_v")
                viaPoint();
            else if(opt=="gtp")
                position();
            else if(opt=="gtvp")
                positionViaPoint();            
            else if(opt=="exit")
                std::exit(0);
            else
            {
                help();
                ROS_ERROR("No valid option");
            }

        }

};

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "joint_state_publisher");


    cartesian2joint c2j;

    float f = 0.0;
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        //c2j.trajectoryMarker();
        c2j.askForOption();
        loop_rate.sleep();  
        ros::spinOnce();
    }

    return 0;
}