#include "Sim_path_tracking.h"



/*
# Copyright 2018 HyphaROS Workshop.
# Latest Modifier: HaoChih, LIN (hypha.ros@gmail.com)
# Original Author: ChanYuan KUO & YoRu LU
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <morai_msgs/CtrlCmd.h>
#include<math.h>
# define M_PI       3.14159265358979323846
# define M_RAD      57.29577951
/********************/
/* CLASS DEFINITION */
/********************/
class PurePursuit
{
    public:
        PurePursuit();
        void initMarker();
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        double getYawFromPose(const geometry_msgs::Pose& carPose);
        double getEta(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getSteering(double eta);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
         void test(const geometry_msgs::Pose& carPose);

    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub, amcl_sub, state_sub,vel_sub,joy_sub,parking_path_sub,parking_state_sub;
        ros::Publisher cmdvel_pub, marker_pub,moraiVel_pub,cameraLane_pub;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;

        visualization_msgs::Marker points, line_strip, goal_circle;
        geometry_msgs::Point odom_goal_pos, goal_pos;
        geometry_msgs::Twist cmd_vel;
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path, odom_path,parking_path;
        std_msgs::Int16 parking_state;
        std_msgs::String state;
        std_msgs::Int16 current_vel;
        sensor_msgs::Joy cmd_joy;
        morai_msgs::CtrlCmd morai_vel;

        double L, Lfw, Vcmd, lfw, steering, velocity, brake;
        std::string simORreal;
        double steering_gain, velocity_gain,base_angle, goal_radius, speed_incremental,speed_decrease;
        int controller_freq;
        double go_vel_from, stop_vel_from, cross_road_vel_from,slow_down_for_traffic_light_vel_from,static_obs_vel_from,outbreak_obs_vel_from,parking_search_vel_from,parking_in_vel_from,parking_wait_vel_from,parking_out_vel_from,finish_vel_from;
        double go_vel_to, stop_vel_to, cross_road_vel_to,slow_down_for_traffic_light_vel_to,static_obs_vel_to,outbreak_obs_vel_to,parking_search_vel_to,parking_in_vel_to,parking_wait_vel_to,parking_out_vel_to,finish_vel_to;
        bool foundForwardPt, goal_received, goal_reached, cmd_vel_mode, debug_mode, smooth_accel;
        
        double angle_avg=0;
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
        void controlLoopCB(const ros::TimerEvent&);
        void stateCB(const std_msgs::String stateMsg);
        void velCB(const std_msgs::Int16 velMsg);
        void joyCB(const sensor_msgs::Joy joyMsg);
        void parkingpathCB(const nav_msgs::Path::ConstPtr& parkingpathMsg);
        void parkingstateCB(const std_msgs::Int16 parkingstateMsg);

}; // end of class


PurePursuit::PurePursuit()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter
    pn.param("L", L, 2.0); // length of car
    pn.param("Vcmd", Vcmd, 0.0);// reference speed (m/s)
    // pn.param("Lfw", Lfw, 0.5); // forward look ahead distance (m)
    pn.param("lfw", lfw, 0.13); // distance between front the center of car
    pn.param("velocity_gain",velocity_gain,1.0);


/*****************************************************************************************/

   go_vel_from=100.0*velocity_gain;
   go_vel_to=200.0*velocity_gain;

   stop_vel_from=0.0*velocity_gain;
   stop_vel_to=0.0*velocity_gain;


   cross_road_vel_from=100.0*velocity_gain;
   cross_road_vel_to=200.0*velocity_gain;


   slow_down_for_traffic_light_vel_from=50.0;
   slow_down_for_traffic_light_vel_to=50.0;


   static_obs_vel_from=100.0*velocity_gain;
   static_obs_vel_to =150.0*velocity_gain;


   outbreak_obs_vel_from=100.0*velocity_gain;
   outbreak_obs_vel_to=150.0*velocity_gain;

   parking_search_vel_from=50.0;
   parking_search_vel_to=50.0;


   parking_in_vel_from=50.0;
   parking_in_vel_to=50.0;


   parking_wait_vel_from=0.0*velocity_gain;
   parking_wait_vel_to=0.0*velocity_gain;


   parking_out_vel_from=-50.0;
   parking_out_vel_to=-50.0;

   finish_vel_from=0.0*velocity_gain;
   finish_vel_to=0.0*velocity_gain;


   pn.param("base_angle", base_angle, 0.0);
/****************************************************************************************/
    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    // pn.param("steering_gain", steering_gain, 1.0);
    pn.param("goal_radius", goal_radius, 0.5); // goal radius (m)
  //  pn.param("base_angle", base_angle, -0.08726); // neutral point of servo (rad)
    pn.param("cmd_vel_mode", cmd_vel_mode, true); // whether or not publishing cmd_vel
    pn.param("debug_mode", debug_mode, true); // debug mode
    pn.param("smooth_accel", smooth_accel, false); // smooth the acceleration of car
    pn.param("speed_incremental", speed_incremental, 0.5); // speed incremental value (discrete acceleraton), unit: m/s
    pn.param("speed_decrease", speed_decrease, 0.04); // decrease percent per frame

    pn.param<std::string>("simORreal",simORreal,"real");
    //Publishers and Subscribers
    if(simORreal == "real"){
        odom_sub = n_.subscribe("/morai_odom", 1, &PurePursuit::odomCB, this);
        steering_gain = 1.0;
    }
    else{
        odom_sub = n_.subscribe("/morai_odom", 1, &PurePursuit::odomCB, this);
        steering_gain = M_RAD;
    }
    path_sub = n_.subscribe("/waypoint", 1, &PurePursuit::pathCB, this);
    goal_sub = n_.subscribe("/goal", 1, &PurePursuit::goalCB, this);
    amcl_sub = n_.subscribe("/pose", 5, &PurePursuit::amclCB, this);
    marker_pub = n_.advertise<visualization_msgs::Marker>("/pure_pursuit/path_marker", 10);
    state_sub = n_.subscribe("/state", 1, &PurePursuit::stateCB, this);
    vel_sub =n_.subscribe("/MSG_CON/Rx_Vel", 1, &PurePursuit::velCB, this);
    joy_sub = n_.subscribe("/joy", 1, &PurePursuit::joyCB, this);
    parking_path_sub =n_.subscribe("/parking_path", 1, &PurePursuit::parkingpathCB, this);
    parking_state_sub = n_.subscribe("/parking_state", 1, &PurePursuit::parkingstateCB, this);
    if(cmd_vel_mode) cmdvel_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    moraiVel_pub = n_.advertise<morai_msgs::CtrlCmd>("ctrl_cmd", 1);
    cameraLane_pub = n_.advertise<nav_msgs::Path>("lane_path", 1);


    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &PurePursuit::controlLoopCB, this); // Duration(0.05) -> 20Hz


    //Init variables
    foundForwardPt = false;
    goal_received = true;
    goal_reached = false;
    velocity = 200.0;
    
    steering = base_angle;
    // brake = 0;

    //Show info
    ROS_INFO("[param] base_angle: %f", base_angle);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings
    initMarker();

    cmd_vel = geometry_msgs::Twist();
}



void PurePursuit::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "map";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goal_radius;
    goal_circle.scale.y = goal_radius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}


void PurePursuit::parkingpathCB(const nav_msgs::Path::ConstPtr& parkingpathMsg)
{
    if(this->state.data=="parking")
    this->map_path = *parkingpathMsg;
    this->map_path.header.frame_id = "/novatel";


}
void PurePursuit::parkingstateCB(const std_msgs::Int16 parkingstateMsg)
{

    parking_state=parkingstateMsg;

}









void PurePursuit::joyCB(const sensor_msgs::Joy joyMsg)
{


    if(joyMsg.buttons.at(0)==1)
        state.data="stop";
    if(joyMsg.buttons.at(1)==1)
        state.data="go";
    if(joyMsg.buttons.at(2)==1)
        state.data="cross_road";

}

void PurePursuit::velCB(const std_msgs::Int16 velMsg)
{
    current_vel.data=velMsg.data;


}


void PurePursuit::stateCB(const std_msgs::String stateMsg)
{

    this->state.data=stateMsg.data;

}



void PurePursuit::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    this->odom = *odomMsg;
}


void PurePursuit::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
   if(this->state.data!="parking"){
      this->map_path = *pathMsg;
      this->map_path.header.frame_id = "/novatel";
    
   }
}

void PurePursuit::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{


    this->goal_pos = goalMsg->pose.position;
    odom_goal_pos = goalMsg->pose.position;

  //  try
   // {
      //  geometry_msgs::PoseStamped odom_goal;
     //   tf_listener.transformPose("novatel", ros::Time(0) , *goalMsg, "map" ,odom_goal);
    //    odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;

        //Draw Goal on RVIZ
        goal_circle.pose =goalMsg->pose;
        marker_pub.publish(goal_circle);
 //   }
  //  catch(tf::TransformException &ex)
  //  {
  //      ROS_ERROR("%s",ex.what());
  //      ros::Duration(1.0).sleep();
   // }

}

double PurePursuit::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;

    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}

bool PurePursuit::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x;
    float car2wayPt_y = wayPt.y;

    double car_theta;

    if(this->parking_state.data==2)
        car_theta = -getYawFromPose(carPose);
    else
        car_theta = getYawFromPose(carPose);
    ROS_INFO("x:%lf, y:%lf",wayPt.x, wayPt.y);



    if(this->parking_state.data==2)
    {


        if(car2wayPt_y <0) /*is Forward WayPt*/
        {

            return true;

        }
        else
        {

            return false;

        }




    }
    else
    {
        if(car2wayPt_x >2) /*is Forward WayPt*/
        {

            return true;

        }
        else
            return false;

    }




}


bool PurePursuit::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x;
    double dy = wayPt.y;
    double dist = sqrt(dx*dx + dy*dy);


    std::cout<<"dist"<< dist<<std::endl;

        this->Lfw= this->current_vel.data *0.03;
        if(this->state.data=="static_obs"){
            if(this->Lfw <4)
            this->Lfw=4;
        }
        else{
            if(this->Lfw <5)
                this->Lfw=5;

            if(this->velocity - this->current_vel.data >100 )
                this->Lfw =7.0;
        }

  //

    ROS_INFO("dist %.2f| Lfw %.2f",dist,this->Lfw);

    if(dist < Lfw){
        return false;
    }
    else if(dist >= Lfw)
        return true;



}

geometry_msgs::Point PurePursuit::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;
    nav_msgs::Path localPathToFollow;
    localPathToFollow.header.frame_id = "/map";
    localPathToFollow.header.stamp = odom.header.stamp;
    if(!goal_reached){

        for(int i =0; i< map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped odom_path_pose= map_path.poses[i];
            // geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
            // tf::Quaternion q(
            //                 carPose.orientation.x,
            //                 carPose.orientation.y,
            //                 carPose.orientation.z,
            //                 carPose.orientation.w);
            // tf::Matrix3x3 m(q);
            // double roll,pitch,yaw;
            // m.getRPY(roll, pitch, yaw);
            // double current_th = yaw;
            // double cos_th = cos(yaw);
            // double sin_th = sin(yaw);
            // double rot_x = cos_th*(odom_path_pose.pose.position.x) - sin_th*(-odom_path_pose.pose.position.y);
            // double rot_y = sin_th*(odom_path_pose.pose.position.x) + cos_th*(-odom_path_pose.pose.position.y);
            
            // odom_path_pose.pose.position.x = rot_x+carPose.position.x;
            // odom_path_pose.pose.position.y = rot_y+carPose.position.y;
            odom_path_pose.pose.position.y *=-1;
            localPathToFollow.poses.push_back(odom_path_pose);
        }
        for(int i =0; i< map_path.poses.size(); i++)
        {

              geometry_msgs::PoseStamped odom_path_pose= map_path.poses[i];

                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);
                if(_isForwardWayPt)
                {
                    
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                    if(_isWayPtAwayFromLfwDist)
                    {
                        // double tempX =odom_path_wayPt.x;
                        // odom_path_wayPt.y += 1.5\
                        // odom_path_wayPt.x = -odom_path_wayPt.y;
                        // odom_path_wayPt.y = -tempX;

                        // odom_path_wayPt.x += carPose.position.x;
                        // odom_path_wayPt.y += carPose.position.y;
                        tf::Quaternion q(
                                        carPose.orientation.x,
                                        carPose.orientation.y,
                                        carPose.orientation.z,
                                        carPose.orientation.w);
                        tf::Matrix3x3 m(q);
                        double roll,pitch,yaw;
                        m.getRPY(roll, pitch, yaw);
                        double current_th = yaw;
                        double cos_th = cos(yaw);
                        double sin_th = sin(yaw);
                        double rot_x = cos_th*(odom_path_wayPt.x) - sin_th*(-odom_path_wayPt.y);
                        double rot_y = sin_th*(odom_path_wayPt.x) + cos_th*(-odom_path_wayPt.y);
                        
                        odom_path_wayPt.x = rot_x;
                        odom_path_wayPt.y = rot_y;
                        odom_path_wayPt.x += carPose.position.x;
                        odom_path_wayPt.y += carPose.position.y;
                        forwardPt = odom_path_wayPt;

                        foundForwardPt = true;
                        break;
                    }
                    else{
                    ROS_INFO("no wayptawtyadfeqwfgsdfdist");
                    }
                }
                else{
                    ROS_INFO("no forward");
                }

        }



    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        ROS_INFO("goal REACHED!");
    }
    this->cameraLane_pub.publish(localPathToFollow);
    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();

    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}

void PurePursuit::test(const geometry_msgs::Pose& carPose)
{

    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;


    int point_num=0;
    double angle_sum=0;
     angle_avg=0;

    for(int i =0; i< map_path.poses.size(); i++)
    {


        geometry_msgs::PoseStamped odom_path_pose= map_path.poses[i];
        geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;

          bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);

          if(_isForwardWayPt)
          {
              bool distance_check ;

              double dx = odom_path_wayPt.x- carPose_pos.x;
              double dy = odom_path_wayPt.y - carPose_pos.y;
              double dist = sqrt(dx*dx + dy*dy);

              if(dist>Lfw && dist<Lfw+5)
              {
                  forwardPt = odom_path_wayPt;
                  odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
                  odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
                  double th=atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
                  double steer_angle= atan2((this->L*sin(th)),(this->Lfw/2 + this->lfw*cos(th)));


                  steer_angle=steer_angle*180/M_PI;

                  angle_sum+=steer_angle;
                  point_num++;
                 // ROS_INFO("%d  distance : %.2f  , angle : %.2f",i,dist,steer_angle);





              }
          }



    }

   if(point_num>0)
       angle_avg=angle_sum/point_num;





  //if(angle_avg<0)
  //    angle_avg=-angle_avg;

  ROS_INFO("Point num : %d,   Avg: %.2f " ,point_num,angle_avg);


}





double PurePursuit::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
    return atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
}


double PurePursuit::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = this->odom.pose.pose.position;
    double car2goal_x = this->odom_goal_pos.x - car_pose.x;
    double car2goal_y = this->odom_goal_pos.y - car_pose.y;

    return sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
}


double PurePursuit::getSteering(double eta)
{
    return atan2((this->L*sin(eta)),(this->Lfw/2 + this->lfw*cos(eta)));
}


void PurePursuit::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{
/*
    if(this->goal_received)
    {
        double car2goal_x = this->goal_pos.x - amclMsg->pose.pose.position.x;
        double car2goal_y = this->goal_pos.y - amclMsg->pose.pose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        if(dist2goal < this->goal_radius)
        {
            this->goal_reached = true;
            this->goal_received = false;
            ROS_INFO("Goal Reached !");
        }
    }
    */
}


void PurePursuit::controlLoopCB(const ros::TimerEvent&)
{

    geometry_msgs::Pose carPose = this->odom.pose.pose;
    geometry_msgs::Twist carVel = this->odom.twist.twist;


/*
    tf::Quaternion q(carPose.orientation.x,carPose.orientation.y,carPose.orientation.z,carPose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    ROS_INFO("th : %f",yaw);

    yaw= yaw-M_PI;
     geometry_msgs::Quaternion backside_q = tf::createQuaternionMsgFromYaw(yaw);


     carPose.orientation=backside_q;


*/


    if(this->goal_received)
    {
        //Estimate Steering Angle
        test(carPose);


        double eta = getEta(carPose);
        if(foundForwardPt)
        {
            this->steering = this->base_angle - getSteering(eta)*this->steering_gain;



            //Estimate Gas Input
            if(!this->goal_reached)
            {


                    int vel_penalty=abs(angle_avg*3);

                    


                    if(this->state.data=="go")
                    {
                        this->velocity = go_vel_to-vel_penalty;
                        if(this->velocity <go_vel_from)
                            this->velocity=go_vel_from;

                    }
                    else if(this->state.data=="stop")
                    {
                        // if(this->velocity > stop_vel_from)
                        //     this->velocity -= 200.0*speed_decrease;
                        // if(this->velocity <stop_vel_from)
                            this->velocity=stop_vel_from;
    
                    }
                    else if(this->state.data=="cross_road")
                    {
                         this->velocity = cross_road_vel_to-vel_penalty;
                         if(this->velocity <cross_road_vel_from)
                            this->velocity=cross_road_vel_from;
                    }
                    else if(this->state.data=="slow_down_for_traffic_light")
                    {
                         this->velocity = slow_down_for_traffic_light_vel_to-vel_penalty;
                         if(this->velocity <slow_down_for_traffic_light_vel_from)
                           this->velocity=slow_down_for_traffic_light_vel_from;
                    }
                    else if(this->state.data=="static_obs")
                    {

                         this->velocity = static_obs_vel_to-vel_penalty;
                         if(this->velocity <static_obs_vel_from)
                            this->velocity=static_obs_vel_from;
                    }
                    else if(this->state.data=="outbreak_obs")
                    {
                         this->velocity = outbreak_obs_vel_to-vel_penalty;
                         if(this->velocity <outbreak_obs_vel_from)
                           this->velocity=outbreak_obs_vel_from;
                    }
                    else if(this->state.data=="parking_search")
                    {
                         this->velocity = parking_search_vel_to-vel_penalty;
                         if(this->velocity <parking_search_vel_from)
                            this->velocity=parking_search_vel_from;

                    }
                    else if(this->state.data=="parking")
                    {

                        if(parking_state.data==0)
                            this->velocity = parking_in_vel_to;
                        else if(parking_state.data==1){
                            this->velocity=parking_wait_vel_to;
                        }
                        else if(parking_state.data==2)
                            this->velocity = parking_out_vel_to;
                        else if(parking_state.data==3){
                             if(this->velocity > parking_wait_vel_to)
                            this->velocity -= 200.0*speed_decrease;
                             if(this->velocity <parking_wait_vel_to)
                                this->velocity=parking_wait_vel_to;
                        }

                    }
                    

                    else if(this->state.data=="finish")
                    {
                          if(this->velocity > finish_vel_to)
                            this->velocity -= 200.0*speed_decrease;
                            if(this->velocity <finish_vel_to)
                            this->velocity=finish_vel_to;

                    }




                if(debug_mode) ROS_INFO("TARGET VEL : %.2f, Velocity = %d, Lfw : %.2f  ,  Steering = %.2f  ", this->velocity ,current_vel.data,Lfw, this->steering*180/M_PI);
            }
        }
        else
        {
            if(debug_mode) ROS_INFO("NO FoundRorwardPt");

            if(state.data=="parking" && parking_state.data == 1)
            {
                this->velocity=parking_wait_vel_to;
            }
            if(state.data=="parking" && parking_state.data == 3)
            {
                 if(this->velocity > parking_wait_vel_to)
                    this->velocity -= 200.0*speed_decrease;
                if(this->velocity <parking_wait_vel_to)
                    this->velocity=parking_wait_vel_to;
            }
            

        }
    }
    if(this->goal_reached)
    {
        this->velocity = 0.0;
        this->steering = this->base_angle;
    }



    if(this->cmd_vel_mode)
    {
        this->cmd_vel.linear.x = this->velocity;
        this->cmd_vel.angular.z = this->steering;
        this->cmdvel_pub.publish(this->cmd_vel);
        // this->morai_vel.velocity = this->velocity/10;
        this->morai_vel.accel = this->velocity/200.0;

        if(this->velocity ==0){
            this->morai_vel.brake = 1.0;
        }
        else{ 
            this->morai_vel.brake = 0;
        }
        this->morai_vel.steering = this->steering;
        this->morai_vel.longlCmdType = 0; //when value = 0, brake&accel, value = 1, velocity control.

        this->moraiVel_pub.publish(this->morai_vel);
    }






}

