#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <cartesian_interface/ResetWorld.h>

using namespace std;
static bool reset_request=false;
static tf::StampedTransform w1_T_r1, w1_T_r2,trasf_reset;


bool service_reset_world(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

 cout << " x1: " << w1_T_r1.getOrigin().getX() << " y1: " << w1_T_r1.getOrigin().getY() << " z1: " << w1_T_r1.getOrigin().getZ() << endl;
 cout << " x2: " << w1_T_r2.getOrigin().getX() << " y2: " << w1_T_r2.getOrigin().getY() << " z2: " << w1_T_r2.getOrigin().getZ() << endl;
 //trasf_reset.setOrigin(tf::Vector3(w1_T_r1.getOrigin().getX()-w1_T_r2.getOrigin().getX(),w1_T_r1.getOrigin().getY()-w1_T_r2.getOrigin().getY(),w1_T_r1.getOrigin().getZ()-w1_T_r2.getOrigin().getZ()));
 trasf_reset.setData(w1_T_r2.inverse()*w1_T_r1);
 cout << " xe: " << trasf_reset.getOrigin().getX() << " ye: " << trasf_reset.getOrigin().getY() << " ze: " << trasf_reset.getOrigin().getZ() << endl;
 reset_request=true;
 return true;
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "reset_world_odometry");
    

    std::string world_1="gazebo/world",root_1="pelvis", root_2="ci/pelvis";
    
    ros::NodeHandle n;
    ros::ServiceClient client               = n.serviceClient<cartesian_interface::ResetWorld>("cartesian/reset_world");
    ros::ServiceServer service_reset        = n.advertiseService("reset_world_odometry/reset", service_reset_world);
    cartesian_interface::ResetWorld rst_world;
    
    tf::TransformListener listener;
    
    tf::TransformBroadcaster br;

    ros::Rate rate(10);

    while(ros::ok())
    {
        try{

            listener.waitForTransform(world_1, root_2, ros::Time(0), ros::Duration(1.0) );
            listener.lookupTransform(world_1, root_2,ros::Time(0), w1_T_r2);
            
            listener.waitForTransform(world_1, root_1, ros::Time(0), ros::Duration(1.0) );
            listener.lookupTransform(world_1, root_1, ros::Time(0), w1_T_r1);

        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        if(reset_request)
        {
            reset_request=false;
            
            rst_world.request.new_world.position.x=trasf_reset.getOrigin().getX();
            rst_world.request.new_world.position.y=0.0-trasf_reset.getOrigin().getY();
            rst_world.request.new_world.position.z=0.0-trasf_reset.getOrigin().getZ();

        
        rst_world.request.new_world.orientation.w =1;
        rst_world.request.new_world.orientation.x =0;
        rst_world.request.new_world.orientation.y =0;
        rst_world.request.new_world.orientation.z =0;
        
        client.call(rst_world);    
        }


        ros::spinOnce();
    }
    
    return true;
    
}
