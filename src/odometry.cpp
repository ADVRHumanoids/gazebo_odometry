#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Header.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetModelStateRequest.h"
#include "gazebo_msgs/GetModelStateResponse.h"
#include "gazebo_msgs/ModelStates.h"


#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include "gazebo_odometry/LinkRequested.h"
#include "gazebo_odometry/ModelRequested.h"

using namespace std;

std::string model_name="",link_requested="",cartesian_interface_link="";

bool pub_odom_link(gazebo_odometry::LinkRequested::Request  &req, gazebo_odometry::LinkRequested::Response& res) {

 link_requested=req.link_name;
 return true;
}

bool pub_odom_static_link(gazebo_odometry::LinkRequested::Request  &req, gazebo_odometry::LinkRequested::Response& res) {

 cartesian_interface_link=req.link_name;
 return true;
}

bool set_model_name(gazebo_odometry::ModelRequested::Request  &req, gazebo_odometry::ModelRequested::Response& res) {

 model_name=req.model_name;
 return true;
}

void getmodelname(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  model_name= msg->name[1];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_pub");
  ros::NodeHandle n;
  ros::Publisher odom_pub                     = n.advertise<nav_msgs::Odometry>("/gazebo_odom", 1000); 
  ros::Subscriber sub_get_modelName           = n.subscribe("/gazebo/model_states", 1000, getmodelname);
  ros::ServiceServer service_set_model        = n.advertiseService("gazebo_odom/set_model_name", set_model_name);
  
  ros::ServiceClient get_model_srv            = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::ServiceServer service_set_link         = n.advertiseService("gazebo_odom/pub_odom_link", pub_odom_link);
  ros::ServiceServer service_setstatic_link   = n.advertiseService("gazebo_odom/pub_odom_static_link", pub_odom_static_link);
  
  nav_msgs::Odometry odom;
  std_msgs::Header header;
  header.frame_id="/odom";
  
  gazebo_msgs::GetModelStateRequest model;
  gazebo_msgs::GetModelStateResponse result;
 
  ros::Rate loop_rate(10);
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  
  bool one_time=true;
  
  while (ros::ok())
  {
    if(model_name!="")
    {    
        model.model_name=model_name;
     
        get_model_srv.call(model,result);   
                                
        transform.setOrigin( tf::Vector3(result.pose.position.x,result.pose.position.y,result.pose.position.z));
        
        q.setX(result.pose.orientation.x);
        q.setY(result.pose.orientation.y);
        q.setZ(result.pose.orientation.z);
        q.setW(result.pose.orientation.w);

        transform.setRotation(q);
        
        if(link_requested!="")
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"gazebo/world",link_requested));
        
        if(cartesian_interface_link!="")
        {
            if(one_time)
            {
                one_time=false;
                
                static_transformStamped.header.stamp = ros::Time::now();
                static_transformStamped.header.frame_id = "gazebo/world";
                static_transformStamped.child_frame_id = cartesian_interface_link;
                
                static_transformStamped.transform.translation.x = result.pose.position.x;
                static_transformStamped.transform.translation.y = result.pose.position.y;
                static_transformStamped.transform.translation.z = result.pose.position.z;
                
               
                static_transformStamped.transform.rotation.x = result.pose.orientation.x;
                static_transformStamped.transform.rotation.y = result.pose.orientation.y;
                static_transformStamped.transform.rotation.z = result.pose.orientation.z;
                static_transformStamped.transform.rotation.w = result.pose.orientation.w;
                
                static_broadcaster.sendTransform(static_transformStamped);
            }
        }
     
        
        odom.pose.pose= result.pose;
        odom.twist.twist = result.twist;
        
        header.stamp = ros::Time::now();
        odom.header = header;
        
        odom_pub.publish(odom);
        
    }

    ros::spinOnce();
  }
  
  return 0;
}
