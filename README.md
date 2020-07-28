# gazebo_odometry

This code publishes the odometry of the robot into a topic /gazebo_odom and into a tf specifying the link, using /gazebo/get_model_state service.

It's possible to setup the model name using a service or /gazebo/model_states topic which gives back the name of the robot.

Useful link: 

https://answers.ros.org/question/222033/how-do-i-publish-gazebo-position-of-a-robot-model-on-odometry-topic/

## How to run

1) Run gazebo and the robot.
2) set the name if the /gazebo/model_states is not present using:
   i.e rosservice call /gazebo_odom/set_model_name "model_name: 'cogimon'"
3) set the name of the link of the odometry tf:
   i.e rosservice call /gazebo_odom/pub_odom_link "link_name: 'world'" 
4) set the name of the link of the static odometry tf:
   i.e rosservice call /gazebo_odom/pub_odom_static_link "link_name: 'ci/world'"
5) Check the result on Rviz or using the topic:
   rostopic echo /gazebo_odom

  
