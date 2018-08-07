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
        msg.data[0]= 0; //x
        msg.data[1]= 0; //y
        msg.data[2]= 350; //z

        chatter_pub.publish(msg);

        ros::Duration(1).sleep();


        /**FIRST POSITION**/
        msg.data.resize(3);
        msg.data[0]= 300; //x
        msg.data[1]= 0; //y
        msg.data[2]= 350; //z

        chatter_pub.publish(msg);

        ros::Duration(10).sleep();



        //circle trajectory
/*
        double x_0 = 50, y_0 = -200;
        double R = 125;
        for (double t = 0; t < 2 * M_PI; t += 1) {

            msg.data.resize(3);
            msg.data[0] = (R * cos(t) + x_0); //x
            msg.data[1] = (R * sin(t) + y_0); //y
            msg.data[2] = 250; //z

            chatter_pub.publish(msg);

            ros::Duration(3).sleep();

        }
        //ros::Duration(1).sleep();


*/
        ros::spinOnce();
        loop_rate.sleep();

    }








    return 0;
}
