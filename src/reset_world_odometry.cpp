#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <cartesian_interface/ResetWorld.h>

using namespace std;
static bool reset_request=false;
static tf::StampedTransform w1_T_r1, w1_T_r2,trasf_reset;


bool service_reset_world(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

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
        
            cout << " xp1: " << w1_T_r1.getOrigin().getX() << " yp1: " << w1_T_r1.getOrigin().getY() << " zp1: " << w1_T_r1.getOrigin().getZ() << endl;
            cout << " xo1: " << w1_T_r1.getRotation().getX() << " yo1: " << w1_T_r1.getRotation().getY() << " zo1: " << w1_T_r1.getRotation().getZ() << " w1: " << w1_T_r1.getRotation().getW() << endl;
            cout << " xp2: " << w1_T_r2.getOrigin().getX() << " yp2: " << w1_T_r2.getOrigin().getY() << " zp2: " << w1_T_r2.getOrigin().getZ() << endl;
            cout << " xo2: " << w1_T_r2.getRotation().getX() << " yo2: " << w1_T_r2.getRotation().getY() << " zo2: " << w1_T_r2.getRotation().getZ() << " w2: " << w1_T_r2.getRotation().getW() << endl;

            trasf_reset.setData(w1_T_r1.inverse()*w1_T_r2);
 
            cout << " xpe: " << trasf_reset.getOrigin().getX() << " ype: " << trasf_reset.getOrigin().getY() << " zpe: " << trasf_reset.getOrigin().getZ() << endl;
            cout << " xeo: " << trasf_reset.getRotation().getX() << " yeo: " << trasf_reset.getRotation().getY() << " zoe: " << trasf_reset.getRotation().getZ() << " we: " << trasf_reset.getRotation().getW() << endl;        
            /*
            if((abs(trasf_reset.getOrigin().getX()) <= 0.001)&&(abs(trasf_reset.getOrigin().getY()) <= 0.001)&&(abs(trasf_reset.getOrigin().getZ()) <= 0.001))
                if((abs(trasf_reset.getRotation().getX()) <= 0.001)&&(abs(trasf_reset.getRotation().getY()) <= 0.001)&&(abs(trasf_reset.getRotation().getZ()) <= 0.001))
                    if((trasf_reset.getRotation().getW() >= 0.95)&&(trasf_reset.getRotation().getW() <=1))*/
                        //reset_request=false;
            
            if(reset_request)
            {
            rst_world.request.new_world.position.x=trasf_reset.getOrigin().getX();
            rst_world.request.new_world.position.y=trasf_reset.getOrigin().getY();
            rst_world.request.new_world.position.z=trasf_reset.getOrigin().getZ();
            rst_world.request.new_world.orientation.w =trasf_reset.getRotation().getW();
            rst_world.request.new_world.orientation.x =trasf_reset.getRotation().getX();
            rst_world.request.new_world.orientation.y =trasf_reset.getRotation().getY();
            rst_world.request.new_world.orientation.z =trasf_reset.getRotation().getZ();
            client.call(rst_world);
            }
            reset_request=false;
            
        }


        ros::spinOnce();
    }
    
    return true;
    
}
