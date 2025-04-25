#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include "apriltag_ros/single_image_detector.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include "final/test.h"

#include <Eigen/Dense>
#include <queue>
#include <unsupported/Eigen/MatrixFunctions>

double gravity = 9.75;
geometry_msgs::PoseStamped pose_current;
geometry_msgs::TwistStamped vel_current;
sensor_msgs::Imu imu_current;

// apriltag_ros::AprilTagDetectionArray tag;
struct TagInfo {
    int id;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};
int tagid;
Eigen::Vector3d tagposition;
Eigen::Quaterniond tagorientation;

std::vector<Eigen::VectorXd> setup_ukf(int n);
void dynamic(std::vector<Eigen::VectorXd> *state);
std::vector<Eigen::VectorXd> Sprinkle(Eigen::VectorXd u, Eigen::MatrixXd p, int n,std::vector<Eigen::VectorXd> data);
Eigen::Vector3d statepre(Eigen::Vector3d x,int n ,std::vector<Eigen::VectorXd> data,std::vector<Eigen::VectorXd> state);
Eigen::Matrix3d covpre(Eigen::VectorXd u,Eigen::MatrixXd p,int n ,std::vector<Eigen::VectorXd> data,std::vector<Eigen::VectorXd> state);
std::vector<Eigen::Vector3d> measurement_model(std::vector<Eigen::VectorXd> state,std::vector<Eigen::Vector3d> tag, int ID);
Eigen::Vector3d measurepre(std::vector<Eigen::Vector3d> h, Eigen::VectorXd wm);
Eigen::Matrix3d measureMatrix(std::vector<Eigen::Vector3d> h, Eigen::Vector3d z,Eigen::VectorXd wc);
Eigen::Matrix3d sigma_get(std::vector<Eigen::VectorXd> state, Eigen::Vector3d x_est, std::vector<Eigen::Vector3d> h, Eigen::Vector3d z,Eigen::VectorXd wc);
void setPose(geometry_msgs::Pose Pose);
void setTwist(geometry_msgs::Twist Twist);
void tag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);

void groundTruth_cb(const gazebo_msgs::ModelStates::ConstPtr& msg);
geometry_msgs::PoseStamped getpose();
geometry_msgs::TwistStamped getTwist();
sensor_msgs::Imu getImu();
Eigen::Vector3d gettag();

// std::vector<TagInfo> gettag();
void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) ;
std::vector<TagInfo> gettag(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
std::vector<TagInfo> alltag;
void test(){
    std::cout<<alltag[0].position(1);
}
int main(int argc, char **argv)
{   
    //  ROS_initialize  //
    ros::init(argc, argv, "ukf");
    ros::NodeHandle nh;
    ros::Subscriber groundTruth_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 30, &groundTruth_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 30, &imu_cb);

    ros::Subscriber apriltag_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/camera_down/tag_detections", 1000, &apriltagCallback);

    ros::Publisher ukf_pub = nh.advertise<final::test>("/ukf/ukf_pose",30);

    ros::Rate rate(20);

/*///////////////////  Define ////////////////////////////////*/

    Eigen::Vector3d pre_z;
    int nx = 3;
    int ID=0;
    int pre_id;
    // ros::Time last_time;
    Eigen::Vector3d x_est;
    Eigen::Matrix3d p_est;
    std::vector<Eigen::VectorXd> state;
    std::vector<Eigen::Vector3d> tag;
    std::vector<Eigen::Vector3d> tag_loca;
    std::vector<Eigen::VectorXd> data;
    std::vector<Eigen::Vector3d> h ;
    Eigen::Vector3d z_hat;
    Eigen::Matrix3d S;  
    Eigen::Vector3d z;
    Eigen::Matrix3d K;
    Eigen::Matrix3d Rb2w;
    std::vector<std::vector<double>> waypoint{  {0 , 0},
                                                {5 , 0},
                                                {10.5, 0},
                                                {10.5 , 3 },
                                                {5 , 3},
                                                {1 , 3},
                                                {1, 6},
                                                {6, 6},
                                                {12,6}};
    Eigen::Vector3d temp;

    final::test plot_data;

    double dt = 0.01;
    Eigen::Vector3d acc;
/*///////////////////  setInitial ////////////////////////////////*/

    for(int i=0;i<waypoint.size();i++)
    {   
        temp.setZero(3);
        temp << waypoint[i][0] ,waypoint[i][1] , 0;
        tag_loca.push_back(temp);
        // std::cout<<tag_loca[i]<<"\n\n";

    }
    
    p_est = 0.02*p_est.setIdentity(3,3);
    x_est << 2,2,2;
    
    temp.setZero(3);
    temp << 0 , 0, 5;
    data = setup_ukf(nx);
/*/////////////////wait for ros::ok ///////////////////////*/
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        std::cout<<" wait for imu topic"<<"\n";

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Vehicle is ready to start");


    while(ros::ok())
    {
        // dt = (double)last_time -(double)ros::Time::now().toSec();
        ID =tagid;
        if (ID == 10)
            ID =8;
        tag.clear();
        tag.push_back(tagposition);
    /*///////////////////  prediction ////////////////////////////////*/

        state =Sprinkle(x_est,p_est,nx,data);
        dynamic(&state);
        x_est = statepre(x_est,nx,data,state);
        p_est = covpre(x_est,p_est,nx,data,state);
        
        // std::cout<<"pre p \n"<<p_est<<"\n";

        // Rb2w = Eigen::Quaterniond(
        //     imu_current.orientation.w,
        //     imu_current.orientation.x,
        //     imu_current.orientation.y,
        //     imu_current.orientation.z        
        // ).toRotationMatrix();
        // acc.setZero(3);
        // acc <<  imu_current.linear_acceleration.x ,
        //         imu_current.linear_acceleration.y,
        //         imu_current.linear_acceleration.z - gravity;
        // x_est += dt*dt * Rb2w*(-acc);
      


    /*///////////////////  correction ////////////////////////////////*/
        if (pre_z != tag[0])
        {

            state =Sprinkle(x_est,p_est,nx,data);

            h = measurement_model(state,tag_loca,ID);
            z_hat = measurepre(h,data[0]);
            S = measureMatrix(h,z_hat,data[1]);
            p_est = sigma_get(state, x_est, h , z_hat,data[1]);

            K = p_est * S.inverse();
            z = tag[0] + tag_loca[ID] ;
            x_est = x_est + K*(z- z_hat);
            p_est = p_est - K*S*K.transpose();

        }

            std::cout<<"x_est : \n"<<p_est<<"\n\n";

        pre_z = tag[0];
        pre_id = ID;

/*//////////////////////    publisher   ///////////////////////////////////*/
        plot_data.header.stamp = ros::Time::now();
        // plot_data.header.frame_id = "map";
        plot_data.ukf_pose.position.x = x_est(0);
        plot_data.ukf_pose.position.y = x_est(1);
        plot_data.ukf_pose.position.z = x_est(2);

        plot_data.gt_pose = getpose().pose;
        ukf_pub.publish(plot_data);
        // last_time = ros::Time::now().toSec();
        ros::spinOnce();
        rate.sleep();
    }

}
Eigen::Vector3d gettag()
{   
    Eigen::Vector3d temp;
    // std::vector<Eigen::Vector3d> tag;
    // for (int i =0; i <alltag.size(); i++)
    // {
        temp << alltag[0].position(0),
                alltag[0].position(1),
                alltag[0].position(2);
    //     tag.push_back(temp);
    // }
    return temp;
}
sensor_msgs::Imu getImu(){return imu_current;}

geometry_msgs::PoseStamped getpose(){return pose_current;};
geometry_msgs::TwistStamped getTwist(){return vel_current;};
void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    std::vector<TagInfo> tags;

    for (const auto& detection : msg->detections) {
        TagInfo tag;
        tag.id = detection.id[0];
        tagid = detection.id[0];
        tag.position = Eigen::Vector3d(
            detection.pose.pose.pose.position.x,
            detection.pose.pose.pose.position.y,
            detection.pose.pose.pose.position.z
        );
        tagposition = Eigen::Vector3d(
            detection.pose.pose.pose.position.x,
            detection.pose.pose.pose.position.y,
            detection.pose.pose.pose.position.z
        );
        tag.orientation = Eigen::Quaterniond(
            detection.pose.pose.pose.orientation.w,
            detection.pose.pose.pose.orientation.x,
            detection.pose.pose.pose.orientation.y,
            detection.pose.pose.pose.orientation.z
        );
        tags.push_back(tag);
    }

    alltag = tags;
}
void setPose(geometry_msgs::Pose Pose)
{
    // pose_current.header.stamp = ros::Time::now();
    pose_current.pose = Pose;
}
void setTwist(geometry_msgs::Twist Twist)
{
    // vel_current.header.stamp = ros::Time::now();
    vel_current.twist = Twist;
}
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_current = *msg;
    pose_current.pose.orientation = imu_current.orientation;
    vel_current.twist.angular = imu_current.angular_velocity;
    // acc_current = imu_current.linear_acceleration;
}
void groundTruth_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	////////////////////////// get groundTruth model states and arrange their ID////////////////////
	std::vector<std::string> name = msg->name;
    std::string prefix =  std::string("iris") ;
    for (int i = 0; i<name.size(); i++)
    {

            if(prefix == name[i])
            {
                setPose(msg->pose[i]);
                setTwist(msg->twist[i]);
                // std::cout << name[i]<<"\n"<<id <<"\n";

            }
        
    }
}
std::vector<Eigen::VectorXd> setup_ukf(int n)
{       
    double alpha = 0.5;
    double beta = 2;
    double kappa = 3-n;
    double lamb;

    std::vector <Eigen::VectorXd> data;
    // data.resize(3);
    Eigen::VectorXd wm,wc,gamma;
    wm.resize(2*n+1);
    wc.resize(2*n+1);
    gamma.resize(1);
    lamb = pow(alpha,2) * (n+kappa) - n;
    wm(0) = ( lamb/(lamb+n));
    wc(0) = ( wm(0) + 1 - pow(alpha,2) + beta);

    for (int i =1; i <= 2*n; i++)
    {
        wm(i)=( 1/(2 * (n +lamb)));
        wc(i)=( 1/(2 * (n +lamb)));
    }
    gamma(0) = sqrt(n +lamb);
    data.push_back(wm);
    data.push_back(wc);
    data.push_back(gamma);

    return data;
}

void dynamic(std::vector<Eigen::VectorXd> *state)
{   
    double dt = 0.01;
    Eigen::Vector3d acc;
        acc.setZero(3);
        acc <<  imu_current.linear_acceleration.x ,
                imu_current.linear_acceleration.y,
                imu_current.linear_acceleration.z - gravity;
     for(int i=0; i<state->size(); i++){
         state->at(i) +=  0.5*dt*dt*(-acc);

    // std::cout<<"state \n"<<state->at(i)<<"\n\n";
     }
}
 std::vector<Eigen::VectorXd> Sprinkle(Eigen::VectorXd u, Eigen::MatrixXd p, int n ,std::vector<Eigen::VectorXd> data)
{
    
    std::vector<Eigen::VectorXd> state;
    Eigen::MatrixXd p_sqrt;
    p_sqrt = p.llt().matrixL();
    state.push_back (u);
    for(int i=1; i <= 2*n; i++)
    {
        if(i<=n)
            state.push_back( u + data[2](0)* p_sqrt.col(i-1));       
        else
            state.push_back( u - data[2](0)* p_sqrt.col(i-n-1));
        // std::cout<< i<<"\n"<<p_sqrt<<"\n\n";
    }
    
    return state;
}
Eigen::Vector3d statepre(Eigen::Vector3d u,int n ,std::vector<Eigen::VectorXd> data,std::vector<Eigen::VectorXd> state)
{
    Eigen::Vector3d u_temp;
    u_temp.setZero(3);
    u = data[0](0) * u;
    for(int i =1; i<=n*2; i++)
        u += data[0](i) * state[i];
    
    return u;
}

Eigen::Matrix3d covpre(Eigen::VectorXd u,Eigen::MatrixXd p,int n ,std::vector<Eigen::VectorXd> data,std::vector<Eigen::VectorXd> state)
{
    Eigen::Matrix3d R;
    R.diagonal()<< 3e-5, 3e-5 , 1e-5;
    p = data[1][0] * p;
    for(int i=1; i <= 2*n; i++)
        p += data[1][i] * (state[i]-u)*((state[i]-u).transpose());
    p += R;
    return p;
}
std::vector<Eigen::Vector3d> measurement_model(std::vector<Eigen::VectorXd> state,std::vector<Eigen::Vector3d> tag, int ID)
{   
    std::vector<Eigen::Vector3d> h;
    for (int i=0; i<state.size(); i++)
        h.push_back(state[i] );
    
    return h;
}
Eigen::Vector3d measurepre(std::vector<Eigen::Vector3d> h, Eigen::VectorXd wm)
{   
    Eigen::Vector3d z;
    z = z.setZero();
    for(int i=0; i<h.size(); i++)
        z += wm(i) * h[i];

    return z;
}
Eigen::Matrix3d measureMatrix(std::vector<Eigen::Vector3d> h, Eigen::Vector3d z,Eigen::VectorXd wc)
{   
    Eigen::Matrix3d S;
    Eigen::Matrix3d Q;
    S.setZero(3,3);
    Q.diagonal()<< 3e-5, 3e-5 , 1e-6;
    for(int i=0; i<h.size(); i++)
       S += wc(i) *(h[i]- z)*((h[i]-z).transpose())+Q;
    //    std::cout << S <<"\n\n";
    return S;
}
Eigen::Matrix3d sigma_get(std::vector<Eigen::VectorXd> state, Eigen::Vector3d x_est, std::vector<Eigen::Vector3d> h, Eigen::Vector3d z,Eigen::VectorXd wc)
{   
    Eigen::Matrix3d sigma;
    sigma.setZero(3,3);
    for(int i=0; i<state.size(); i++)
        sigma += wc(i)*(state[i]-x_est)*((h[i]-z).transpose());

    return sigma;
}
