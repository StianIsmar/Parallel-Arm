#include "ros/ros.h"
#include "std_msgs/String.h"
#include <thorvald_base/CANFrame.h>
#include <array>
#include <sstream>
#include <math.h>
#include "std_msgs/Float64MultiArray.h"




class endPos{
private:

    double x_,y_,z_;  /** The position we want the endbase in.**/
    double q1_,q2_,q3_; /** The values that are sent to the corresponding CAN-nodes**/
    double dxe_, dye_,dze_;     /** the dimensions of the endbase.**/
    double la_; /** Length of each arm**/
    double S_;
    double n_1; //gear ratio for motor 1 and 2.
    double n_3; //gear rato for motor 3.
    double diameter_;
    int counter;
    int count_;
    int can_id_q1_, can_id_q2_, can_id_q3_; /** Node IDs for the CAN-nodes (motorcontrollers) **/
    double dx23_, dy23_,dz23_;
    std::vector<double> pos_arr;
    ros::Publisher pub_;
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    bool new_cmd_;
public:

    /** Constructor for the endPos-class **/
    endPos()
    {
        pos_arr.resize(3);

        pub_ = n_.advertise<thorvald_base::CANFrame>("/can_frames_device_t", 1);
        sub_=n_.subscribe("arm_demo",1000,&endPos::armCallback,this);



        count_ = 0;
        can_id_q1_ = 1;
        can_id_q2_ = 2;
        can_id_q3_ = 3;
        setEndbaseDims(83.6137054,40.0,30.0); //Has to be measured in the CAD!
        la_= 360; //Has to be measured in the CAD!

        dx23_ = 45.58335036; // mm /**CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE**/
        dy23_ = 15.00000657; // mm /**CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE**/
        dz23_ = 111.48838609; // mm /**CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE**/
        S_ = 131.6; //85.26852949; //mm
        diameter_= 30; /**Spur gear**/
        n_1 = 4.3; /** Gear ratio motor 1 CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE CHANGE**/
        n_3 = 66;
        new_cmd_ = false;
    }
    /** Destructor for the endPos-class **/

    ~endPos() {}

    void setEndPos(double x,double y,double z);
    void sendCanFrame(int CANID, int dpos);
    double inverseKinematics();
    void setEndbaseDims(double dxe,double dye,double dze);
    void armCallback(const std_msgs::Float64MultiArray::ConstPtr& msg2);

    /**Convert meters and rad to counts**/
    double mmToCounts(double input);
    double radToCounts(double input);

    /** Getters for private variables:**/
    double getQ1(){return q1_;};
    double getQ2(){return q2_;};
    double getQ3(){return q3_;};
    bool getNewCmd(){return new_cmd_;};

    int getCanId1(){return can_id_q1_;};
    int getCanId2(){return can_id_q2_;};
    int getCanId3(){return can_id_q3_;};

    double getGearRatio1(){return n_1;};
    double getGearRatio3(){return n_3;};

};
/** Defining endposition x_, y_, z_**/
void endPos::setEndPos(double x, double y, double z)
{
    x_ = x;
    y_ = y;
    z_ = z;
}

/**Method for converting mm to motorCounts. Argument is either q1 or q2**/
double endPos::mmToCounts(double input)
{
    double result = input * (1024 * 4 * n_1 )/(diameter_ * M_PI);
    return result;
}

/**Method for converting rad to motorcounts. Argument is q3 **/
double endPos::radToCounts(double input)
{
    double  result = input * (1024 * 4 * n_3 )/(2 * M_PI);
    return result;
}


/** Method for setting up endbase dims**/
void endPos::setEndbaseDims(double dxe,double dye,double dze)
{
    dxe_= dxe;
    dye_ = dye;
    dze_ = dze;
}

/**
* Send messages in the CANFrame form. Argument dpos equals motorcounts. */
void endPos::sendCanFrame(int CANID, int dpos)
{

    thorvald_base::CANFrame msg;

    msg.id = 0x200 + CANID; //hexatodecimal + node id. Node ID is for example CANNOde = can_id_q3_;

    /**Bit shifting**/
    msg.length = 8;
    msg.data.resize(8);
    msg.data[0] = dpos;
    msg.data[0] = dpos;
    msg.data[1] = dpos >> 8;
    msg.data[2] = dpos >> 16;
    msg.data[3] = dpos >> 24;

    int arr[] {1,0,0,0,0,0,0,0};


    ROS_INFO("%d, %d", msg.id,msg.length);
    //ROS_INFO("%d, %d, %d",arr[0],arr[1],arr[2]);
    pub_.publish(msg);
    ++count_;

}
/**Method for defining q1_, q2_, q3_**/
double endPos::inverseKinematics()
{
    double a1 = 1;
    double b1 = -2*x_ + dxe_;
    double c1 = pow(x_,2) - x_*dxe_ + pow(dxe_,2)/4 + pow(y_,2) + pow(z_,2)-pow(378.7725892,2);

    q1_ = (-b1 - sqrt(pow(b1,2) - 4*a1*c1)) / (2*a1); //q1_ er satt //m
    q2_ = 2*x_ - q1_; //q2_ er satt //m

    double A3 = q2_-x_- dx23_;
    double B3 = y_ + dye_ + dy23_;
    double C3 = z_- dze_ + dz23_;
    double D3 = pow(A3,2) + pow(B3,2) + pow(C3,2) + pow(S_,2) - pow(378.7725892,2);

    double a3 = pow(2*B3*S_,2) + pow(2*C3*S_,2);
    double b3 = 4*D3*B3*S_;
    double c3 = pow(D3,2) - pow(2*C3*S_,2);
    double sinq3 =  (-b3 + sqrt(pow(b3,2)-4*a3*c3)) / (2*a3);
    q3_ = asin(sinq3); //<- SVINGARM //rad


}
void endPos::armCallback(const std_msgs::Float64MultiArray::ConstPtr& msg2) {

    new_cmd_ = false;
    ROS_ERROR("fsafassf");
    std::cout << "pos_arr[0]: "<< pos_arr[0] <<std::endl;
    std::cout << "msg2-data[0]: " << msg2->data[0]<<std::endl;

    new_cmd_ = new_cmd_ | pos_arr[0] != msg2->data[0];
    new_cmd_ = new_cmd_ | pos_arr[1] != msg2->data[1];
    new_cmd_ = new_cmd_ | pos_arr[2] != msg2->data[2];

    setEndPos(msg2->data[0], msg2->data[1], msg2->data[2]);


}


int main(int argc, char **argv) //Legg inn en subscriber også på motorhastighet. Finn ut av hvor langt et klikk er.
{

    /** Setting up the node and publisher **/
    ros::init(argc, argv, "talker");
    endPos pos; //kaller på konstuktøren som setter statiske avstander og node-IDer.

   // pos.setEndPos(705.50885687,99.35443528,276.24289268); //x_,y_,z_ defined. From Lars' code.
    //(705.50885687,99.35443528,276.24289268)

    std::cout <<"DETTE ER q1_:  "<<pos.getQ1()<<std::endl;
    std::cout <<"DETTE ER q2_:  "<<pos.getQ2()<<std::endl;
    std::cout <<"DETTE ER q3_:  "<<pos.getQ3()<<std::endl;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std::cout << pos.getNewCmd()<<std::endl;
        if (pos.getNewCmd())
        {
            /** Find q1_, q2_ and q3_ for the inout (x_,y_,z_)**/
            pos.inverseKinematics(); //now q1_,q2_ and q3_ are defined.

            double mc1 = pos.mmToCounts(pos.getQ1()); //The RIGHT cart
            double mc2 = pos.mmToCounts(pos.getQ2()- 205); //The LEFT cart. Need to subtract 205, since this
                  //is the distance between the two carts.
            double mc3 = pos.radToCounts(pos.getQ3()); //Swing arm.

            std::cout <<"DETTE ER mc1_(right:  "<< mc1<<std::endl;
            std::cout <<"DETTE ER left:  "<<mc2<<std::endl;
            std::cout <<"DETTE ER mc3_:arm  "<<mc3<<std::endl;


            /** Publish one message for each motor node. Positive x-axis is defined to the left. Hence the negative signs
             * for left and right motor. **/
            pos.sendCanFrame(pos.getCanId1(), -mc1);
            pos.sendCanFrame(pos.getCanId2(), -mc2);
            pos.sendCanFrame(pos.getCanId3(), mc3);

        }
        ros::spinOnce();

        loop_rate.sleep();


    }


    return 0;
} 
