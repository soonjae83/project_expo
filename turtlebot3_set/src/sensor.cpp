#include <string>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <wiringPi.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#define Trig 23
#define Echo 32

using namespace std;

class Sensor{
public:
    Sensor(){
        pubCmdvel = nh.advertise<geometry_msgs::Twist("/cmd_vel", 10);
        pubUltraEnd = nh.advertise<std_msgs::String>("/ultra", 1);

        sub_turn_ultra = nh.subscribe("/ultra", 1, &Sensor::subUltra, this);
    }

    // 초음파 작동 //
    void ultra_move(string direction){

        clock_t start_time, end_time, distance;

        while (ros::ok())
        {
            digitalWrite(Trig, LOW);
            delay(100);
            digitalWrite(Trig, HIGH);
            delayMicroseconds(10);
            digitalWrite(Trig, LOW);

            while (GPIO::input(Echo) == 0);
            start_time = clock();

            while (GPIO::input(Echo) == 1);
            end_time = clock();

            distance = (end_time - start_time)/29./2.;

            cout << "[INFO] Distance : " << distance << "cm\r" << end1;

            if (direction == "forward"){
                if (distance > 8){
                    cmd_vel.linear.x = 0.06;
                    pubCmdvel.publish(cmd_vel);
                }
                else {
                    cmd_vel.linear.x = 0.00;
                    pubCmdvel.publish(cmd_vel);
                    ROS_INFO("STOP");
                    break;
                }
            }
        }
    }

    // 90도 회전 //
    void turn90deg(string direction){
        if (direction == "left"){
            cmd_vel.angular.z = 0.8;
        }

        if (direction == "right"{
            cmd_vel.angular.z = -0.8;
        })

        ROS_INFO("Turn 90deg start");

        for (int i = 0; i < 21; i++){
            pubCmdvel.publish(cmd_vel);
            delay(100);
        }

        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn 90deg end");
    }
    // 180도 회전 //
    void turn180deg(string direction){
        if (direction == "left"){
            cmd_vel.angular.z = 0.785;
        }
        
        if (direction == "right"){
            cmd_vel.angular.z = -0.785;
        }

        ROS_INFO("Turn 180deg start");

        for (int i = 0; i < 42; i++){
            pubCmdvel.publish(cmd_vel);
            delay(100);
        }

        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn 180deg end");
    }
    
private:
    ros::NodeHandle nh;
    
    ros::Publisher pubCmdvel;
    ros::Publisher pubUltraEnd;

    ros::Subscriber sub_turn_ultra;

    geometry_msgs::Twist cmd_vel;
    std_msgs::String end;

    inline void delay(int ms) { this_thread::sleep_for(chrono::milliseconds(ms)); }
    inline void delayMicrosecond(int us) { this_thread::sleep_for(chrono::microseconds(us)); }
};

int main(int argc, char**argv){
    ros::init(argc, argv, "main");

    GPIO::setmode(GPIO::BOARD);
    GPIO::setwarnings(false);
    GPIO::setup(Trig, GPIO::OUT);
    GPIO::setup(Echo, GPIO::IN);
    
    ROS_INFO("SET UP ULTRA SENSOR");

    Sensor sensor;

    ros::spin();

    return 0;
}
