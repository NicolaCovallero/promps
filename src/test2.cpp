#include <ros/ros.h>
#include <iri_common_drivers_msgs/QueryJointsMovement.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include "ac_promp.h"

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm
{
    private:

        ros::NodeHandle n;
        ros::ServiceClient client;
        std::string srv_name_;
        // Action client for the joint trajectory action 
        // used to trigger the arm movement action
        TrajClient* traj_client_;

    public:

        //! Initialize the action client and wait for action server to come up
        RobotArm():srv_name_("/iri_wam/iri_wam_controller/joints_move") {
            client = n.serviceClient<iri_common_drivers_msgs::QueryJointsMovement>(this->srv_name_.c_str());
            // tell the action client that we want to spin a thread by default
            traj_client_ = new TrajClient("/iri_wam/iri_wam_controller/follow_joint_trajectory", true);

            // wait for action server to come up
            while(!traj_client_->waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the joint_trajectory_action server");
            }
        }

        //! Clean up the action client
        ~RobotArm() {
            delete traj_client_;
        }

        //! Sends the command to start a given joints position
        bool startJointsMove(iri_common_drivers_msgs::QueryJointsMovement srv) {
            if (!this->client.call(srv))
            {
                ROS_ERROR("Failed to call service %s", this->srv_name_.c_str());
                return false;
            }
            return true;
        }

        //! Generates a joints_move service message from a
        iri_common_drivers_msgs::QueryJointsMovement getJointsMovementSrvFromPoint(trajectory_msgs::JointTrajectoryPoint point)
        {
            //our Joints move position variable
            iri_common_drivers_msgs::QueryJointsMovement srv;
            srv.request.positions.resize(7);
            srv.request.positions[0] = point.positions[0];
            srv.request.positions[1] = point.positions[1];
            srv.request.positions[2] = point.positions[2];
            srv.request.positions[3] = point.positions[3];
            srv.request.positions[4] = point.positions[4];
            srv.request.positions[5] = point.positions[5];
            srv.request.positions[6] = point.positions[6];
            srv.request.velocity = 0.5;
            srv.request.acceleration = 0.5;

            return srv;
        }

        //! Generates a joints_move service message to Home Position
        //  (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        iri_common_drivers_msgs::QueryJointsMovement armAtHomePosition()
        {
            //our Joints move position variable
            iri_common_drivers_msgs::QueryJointsMovement srv;
            srv.request.positions.resize(7);
            srv.request.positions[0] = 0.0;
            srv.request.positions[1] = 0.0;
            srv.request.positions[2] = 0.0;
            srv.request.positions[3] = 0.0;
            srv.request.positions[4] = 0.0;
            srv.request.positions[5] = 0.0;
            srv.request.positions[6] = 0.0;
            srv.request.velocity = 0.5;
            srv.request.acceleration = 0.5;

            return srv;
        }

        //! Sends the command to start a given trajectory
        void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
        {
            // When to start the trajectory: 1s from now
            goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
            traj_client_->sendGoal(goal);
            if (!traj_client_->waitForResult(ros::Duration(10.0)))
            { 
                traj_client_->cancelGoal();
                ROS_INFO("Action did not finish before the time out.\n"); 
            }
        }

        //! Generates a simple trajectory with two waypoints, used as an example
        /*! Note that this trajectory contains two waypoints, joined together
          as a single trajectory. Alternatively, each of these waypoints could
          be in its own trajectory - a trajectory can have one or more waypoints
          depending on the desired application.
          */
        control_msgs::FollowJointTrajectoryGoal getProMpTrajectory(const std::string& traj_filename)
        {
            int b_explore(false);
            bool b_isCartesian(true);
            bool b_isCondition(false);
            std::vector<Eigen::VectorXd> eigen_trajectory_points;
            std::vector<double> local_times;
            float time_of_change(0.4f); // [0-1] 
            std::vector<float> pose;
            pose.resize(7);
            // Original position was (0.6368, 0.1252, -0.2237)
            pose[0] = 0.5168;
            pose[1] = 0.1252; 
            pose[2] = -0.2137; 

            float cov_ratio(0.3f);

            ac_math::compute_promp(b_explore, b_isCartesian, b_isCondition, traj_filename, pose, time_of_change, cov_ratio, eigen_trajectory_points, local_times);

            //our goal variable
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

            // Waypoints in this goal trajectory
            goal.trajectory.points.resize(eigen_trajectory_points.size());

            // First trajectory point
            // Positions
            for (size_t ind = 0; ind < eigen_trajectory_points.size(); ind++) {
                goal.trajectory.points[ind].positions.resize(7);
                goal.trajectory.points[ind].positions[0] = eigen_trajectory_points[ind][0];
                goal.trajectory.points[ind].positions[1] = eigen_trajectory_points[ind][1];
                goal.trajectory.points[ind].positions[2] = eigen_trajectory_points[ind][2];
                goal.trajectory.points[ind].positions[3] = eigen_trajectory_points[ind][3];
                goal.trajectory.points[ind].positions[4] = eigen_trajectory_points[ind][4];
                goal.trajectory.points[ind].positions[5] = eigen_trajectory_points[ind][5];
                goal.trajectory.points[ind].positions[6] = eigen_trajectory_points[ind][6];
                goal.trajectory.points[ind].velocities.resize(7);
                goal.trajectory.points[ind].velocities[0] = 0.2f;
                goal.trajectory.points[ind].velocities[1] = 0.2f;
                goal.trajectory.points[ind].velocities[2] = 0.2f;
                goal.trajectory.points[ind].velocities[3] = 0.2f;
                goal.trajectory.points[ind].velocities[4] = 0.2f;
                goal.trajectory.points[ind].velocities[5] = 0.2f;
                goal.trajectory.points[ind].velocities[6] = 0.2f;
                goal.trajectory.points[ind].accelerations.resize(7);
                goal.trajectory.points[ind].accelerations[0] = 0.2f;
                goal.trajectory.points[ind].accelerations[1] = 0.2f;
                goal.trajectory.points[ind].accelerations[2] = 0.2f;
                goal.trajectory.points[ind].accelerations[3] = 0.2f;
                goal.trajectory.points[ind].accelerations[4] = 0.2f;
                goal.trajectory.points[ind].accelerations[5] = 0.2f;
                goal.trajectory.points[ind].accelerations[6] = 0.2f;
                goal.trajectory.points[ind].time_from_start = ros::Duration(ind*0.3f);
                //goal.trajectory.points[ind].time_from_start = ros::Duration(local_times[ind]);
            }
//SF: Test
//            for (size_t ind = 0; ind < eigen_trajectory_points.size(); ind++) {
//                if (ind==0){
//                    goal.trajectory.points[ind].positions.resize(7);
//                    goal.trajectory.points[ind].velocities.resize(7);
//                    goal.trajectory.points[ind].accelerations.resize(7);
//                    goal.trajectory.points[ind].time_from_start = ros::Duration(ind*0.3f);
//                }else{
//                    goal.trajectory.points[ind].positions.resize(7);
//                    goal.trajectory.points[ind].positions[0] = goal.trajectory.points[ind-1].positions[0]+0.05f;
//                    goal.trajectory.points[ind].positions[1] = goal.trajectory.points[ind-1].positions[1]+0.05f;
//                    goal.trajectory.points[ind].positions[2] = goal.trajectory.points[ind-1].positions[2]+0.05f;
//                    goal.trajectory.points[ind].positions[3] = goal.trajectory.points[ind-1].positions[3]+0.05f;
//                    goal.trajectory.points[ind].positions[4] = goal.trajectory.points[ind-1].positions[4]+0.05f;
//                    goal.trajectory.points[ind].positions[5] = goal.trajectory.points[ind-1].positions[5]+0.05f;
//                    goal.trajectory.points[ind].positions[6] = goal.trajectory.points[ind-1].positions[6]+0.05f;
//                    goal.trajectory.points[ind].velocities.resize(7);
//                    goal.trajectory.points[ind].velocities[0] = 0.2f;
//                    goal.trajectory.points[ind].velocities[1] = 0.2f;
//                    goal.trajectory.points[ind].velocities[2] = 0.2f;
//                    goal.trajectory.points[ind].velocities[3] = 0.2f;
//                    goal.trajectory.points[ind].velocities[4] = 0.2f;
//                    goal.trajectory.points[ind].velocities[5] = 0.2f;
//                    goal.trajectory.points[ind].velocities[6] = 0.2f;
//                    goal.trajectory.points[ind].accelerations.resize(7);
//                    goal.trajectory.points[ind].accelerations[0] = 0.2f;
//                    goal.trajectory.points[ind].accelerations[1] = 0.2f;
//                    goal.trajectory.points[ind].accelerations[2] = 0.2f;
//                    goal.trajectory.points[ind].accelerations[3] = 0.2f;
//                    goal.trajectory.points[ind].accelerations[4] = 0.2f;
//                    goal.trajectory.points[ind].accelerations[5] = 0.2f;
//                    goal.trajectory.points[ind].accelerations[6] = 0.2f;
//                    goal.trajectory.points[ind].time_from_start = ros::Duration(ind*0.3f);
//                    //goal.trajectory.points[ind].time_from_start = ros::Duration(local_times[ind]);
//                }
//            }
//SF: Test
            return goal;
        }

        //! Returns the current state of the action
        actionlib::SimpleClientGoalState getState()
        {
            return traj_client_->getState();
        }

};

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "robot_driver");

    if (argc == 2) /// If the program was started with a trajectory parameter to set in an initial position the robo
    {
        std::string traj_filename(argv[1]);
        //std::string traj_filename("/home/robot/code/iri/libbarrett/bin/test");
        RobotArm arm;

        // IMPORTANT: The robot must be at the start trajectory position
        //arm.startJointsMove(arm.armAtHomePosition());

        // Start the trajectory
        control_msgs::FollowJointTrajectoryGoal trajectory_goal = arm.getProMpTrajectory(traj_filename);
        arm.startJointsMove(arm.getJointsMovementSrvFromPoint(trajectory_goal.trajectory.points[0]));
        arm.startTrajectory(trajectory_goal);
        ROS_INFO("Action finished: %s\n", arm.getState().toString().c_str()); 

        // Wait for trajectory completion
        while(!arm.getState().isDone() && ros::ok())
        {
            usleep(50000);
        }
    }else{
        std::cerr << "You must pass the path to the stored trajectory as an argument" << std::endl;
        std::cerr << "Example rosrun iri_wam_controller test_simple_trajectory_promp /home/user/path/to/file" << std::endl;
        return 0;
    }


}