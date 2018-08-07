#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <sstream>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "pkm_coordinates");

    ros::NodeHandle n;

    ros::Rate loop_rate(10);
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("arm_demo", 1);



    while (ros::ok())
    {
        //205 MM BETWEEN THE TWO CARRIGES


        /**FIRST POSITION**/
        std_msgs::Float64MultiArray msg;
        msg.data.resize(3);
        msg.data[0]= -100; //x
        msg.data[1]= 0; //y
        msg.data[2]= 350; //z

        chatter_pub.publish(msg);

        ros::Duration(1).sleep();

        /**SECOND POSITION**/
        msg.data.resize(3);
        msg.data[0]= -100; //x
        msg.data[1]= -340; //y
        msg.data[2]= 10; //z


        chatter_pub.publish(msg);
        ros::Duration(0.5).sleep();


        /**SECOND POSITION**/
        msg.data.resize(3);
        msg.data[0]= -100; //x
        msg.data[1]= -100; //y
        msg.data[2]= 250; //z


        chatter_pub.publish(msg);
        ros::Duration(0.5).sleep();
        /**THIRD POSITION**/
        msg.data.resize(3);
        msg.data[0]= 0; //x
        msg.data[1]= -250; //y
        msg.data[2]= 250; //z

        chatter_pub.publish(msg);

        ros::Duration(0.5).sleep();

        /**THIRD POSITION==STARTING POSITION! **/
        msg.data.resize(3);
        msg.data[0]= 0; //x
        msg.data[1]= 0; //y
        msg.data[2]= 250; //z
        chatter_pub.publish(msg);
        ros::Duration(0.5).sleep();

        /**THIRD POSITION**/
        msg.data.resize(3);
        msg.data[0]= 0; //x
        msg.data[1]= 0; //y
        msg.data[2]= 340; //z

        chatter_pub.publish(msg);

        ros::Duration(0.5).sleep();
        ros::spinOnce();
        loop_rate.sleep();

    }








    return 0;
}
