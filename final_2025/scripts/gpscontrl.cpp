#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include "getch.h"
#include <cmath>
#include <geometry_msgs/Point.h>


geometry_msgs::PoseStamped pose_current;
geometry_msgs::TwistStamped vel_current;

void bound_yaw(double* yaw){
        if(*yaw>M_PI)
            *yaw = *yaw - 2*M_PI;
        else if(*yaw<-M_PI)
            *yaw = *yaw + 2*M_PI;
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose_current = *msg;
}
geometry_msgs::PoseStamped getPose()
{
    return pose_current;
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    vel_current = *msg;
}
geometry_msgs::TwistStamped getVel()
{
    return vel_current;
}
int main(int argc, char **argv)
{
    //  ROS_initialize  //
    ros::init(argc, argv, "keyboard_ctrl");
    ros::NodeHandle nh;
    //subscribe
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,&pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/twist",10,&vel_cb);
    // publisher
    ros::Publisher desired_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);


    ros::Rate rate(100);
    ros::Time last_request = ros::Time::now();
    
    geometry_msgs::PoseStamped desired_pose;
    desired_pose.pose.position.x = 0;
    desired_pose.pose.position.y = 0;
    desired_pose.pose.position.z = 0;

    geometry_msgs::TwistStamped desired_vel;
    desired_vel.twist.linear.x = 0;
    desired_vel.twist.linear.y = 0;
    desired_vel.twist.linear.z = 0;    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        desired_pose_pub.publish(desired_pose);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Vehicle is ready to start");
    
    double yaw_vel;
    double move_step = 0.1;
    double desired_yaw = 0;
    bool trajectory = false;
    bool revise_trajectory = false;
    int trajectory_step = 0;
    double current_x = 0;
    double current_y = 0;
    double current_z = 0;
    std::vector<std::vector<double>> waypoint{  {0 , 0},
                                                {5 , 0},
                                                {10.5, 0},
                                                {10.5 , 3 },
                                                {5 , 3},
                                                {1 , 3},
                                                {1, 6},
                                                {6, 6},
                                                {11.5,6}};
    while (ros::ok()) 
    {
        current_x = getPose().pose.position.x;
        current_y = getPose().pose.position.y;
        current_z = getPose().pose.position.z;
        // keyboard control
        int c = getch();
        // ROS_INFO("C: %d",c);
        // update desired pose
        if (c != 0) 
        {
            switch (c) {
                case 65:    // key up
                    desired_pose.pose.position.z += move_step;
                    break;
                case 66:    // key down
                    desired_pose.pose.position.z += -move_step;
                    break;
                case 67:    // key CW(->)
                    desired_yaw -= 0.03;
                    bound_yaw(&desired_yaw); 
                    break;
                case 68:    // key CCW(<-)
                    desired_yaw += 0.03;
                    bound_yaw(&desired_yaw); 
                    break;
                case 119:    // key foward(w)
                    desired_pose.pose.position.y += move_step;
                    break;
                case 115:    // key back(s)
                    desired_pose.pose.position.y -= move_step;
                    break;
                case 97:    // key left(a)
                    desired_pose.pose.position.x -= move_step;
                    break;
                case 100:    // key right(d)
                    desired_pose.pose.position.x += move_step;
                    break;
                case 108:    // key land(l)
                    desired_pose.pose.position.z = 0.5;
                    break;
                case 101:    // key trajectory_CCW(e)
                    trajectory = true;
                    break;
                case 112:   // key (p)
                    trajectory = false;
                    revise_trajectory = true;
                    break;
                case 107:   // key kill(k)
                    return 0;
            }
            ROS_INFO("setpoint: %.2f, %.2f, %.2f", desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z);
        }

        if(trajectory && trajectory<5)
        {   


            desired_pose.pose.position.x = waypoint[trajectory_step][0];
            desired_pose.pose.position.y = waypoint[trajectory_step][1];
            desired_pose.pose.position.z = 2;

            desired_vel.twist.linear.x = 0.5*(desired_pose.pose.position.x - current_x);
            desired_vel.twist.linear.y = 0.5*(desired_pose.pose.position.y - current_y);     
            desired_vel.twist.linear.z = (desired_pose.pose.position.z - current_z); 
            if ((pow(current_x-waypoint[trajectory_step][0],2)+pow(current_y-waypoint[trajectory_step][1],2))<0.3)
                trajectory_step++;
            if(trajectory_step==waypoint.size())
                trajectory_step = waypoint.size()-1;

        //     // vel_cmd_pub.publish(desired_vel);
        }
        else if (revise_trajectory)
        {
            desired_pose.pose.position.x = waypoint[trajectory_step][0];
            desired_pose.pose.position.y = waypoint[trajectory_step][1];
            // vel_msg.twist.angular.z = yaw_vel;

            desired_vel.twist.linear.x = 0.5*(desired_pose.pose.position.x - current_x);
            desired_vel.twist.linear.y = 0.5*(desired_pose.pose.position.y - current_y);     
            desired_vel.twist.linear.z = (desired_pose.pose.position.z - current_z); 
            if ((pow(current_x-waypoint[trajectory_step][0],2)+pow(current_y-waypoint[trajectory_step][1],2))<0.3 && trajectory_step !=0)
                trajectory_step--;


        }
        else
        {
            desired_pose.pose.position.x = 0;
            desired_pose.pose.position.y = 0;
            desired_pose.pose.position.z = 2;

            desired_vel.twist.linear.x = 0.5*(desired_pose.pose.position.x - current_x);
            desired_vel.twist.linear.y = 0.5*(desired_pose.pose.position.y - current_y);     
            desired_vel.twist.linear.z = (desired_pose.pose.position.z - current_z); 
        }
        // else
        //     trajectory = false;
        // else
        // {
        //     desired_pose_pub.publish(desired_pose);
        // }

        desired_pose_pub.publish(desired_pose);
        // vel_cmd_pub.publish(desired_vel);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}