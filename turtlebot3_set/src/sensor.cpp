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

#define Trig 15 // BCM 14 (WiringPi 15)
#define Echo 26 // BCM 12 (WiringPi 26)

using namespace std;

class Sensor{
public:
    Sensor(){
        pubCmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        pubUltraEnd = nh.advertise<std_msgs::String>("/ultra", 1);

        sub_turn_ultra = nh.subscribe("/ultra", 1, &Sensor::subUltra, this);
    }

    // 초음파 작동 //
    void ultra_move(string direction){

        int start_time, end_time;
        float distance;

        while (ros::ok())
        {
            digitalWrite(Trig, LOW);
            delay(1000);
            digitalWrite(Trig, HIGH);
            delayMicroseconds(10);
            digitalWrite(Trig, LOW);

            while (digitalRead(Echo) == 0);
            start_time = micros();

            while (digitalRead(Echo) == 1);
            end_time = micros();

            distance = (end_time - start_time)/29./2.;

            cout << "[INFO] Distance : " << distance << "cm\r" << endl;

            if (direction == "forward"){
                if (distance > 12.5){
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
            if (direction == "ex_ev"){
                if (distancd > 150){
                    cmd_vel.linear.x = 0.06;
                    pubCmdvel.publish(cmd_vel);
                }
                else {
                    cmd_vel.linear.x == 0.00;
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

        if (direction == "right"){
            cmd_vel.angular.z = -0.8;
        }

        ROS_INFO("Turn 90deg start");

        for (int i = 0; i < 206; i++){
            pubCmdvel.publish(cmd_vel);
            delay(10);
        }

        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn 90deg end");
    }
    // 180도 회전 //
    void turn180deg(string direction){
        if (direction == "left"){
            cmd_vel.angular.z = 0.8;
        }
        
        if (direction == "right"){
            cmd_vel.angular.z = -0.8;
        }

        ROS_INFO("Turn 180deg start");

        for (int i = 0; i < 403; i++){
            pubCmdvel.publish(cmd_vel);
            delay(10);
        }

        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn 180deg end");
    }

    void subUltra(const std_msgs::String ultra) {
        // 90도 회전
        if (strcmp(ultra.data.c_str(), "90") == 0 ) {
            ROS_INFO("90turn");
            turn90deg("left");
        }

        // 180도 회전
        if (strcmp(ultra.data.c_str(), "180") == 0 ) {
            ROS_INFO("180turn");
            turn180deg("left");
        }

        // 초음파 전진
        if (strcmp(ultra.data.c_str(), "ex_ev") == 0 {
            ROS_INFO("Exit_EV");
            ultra_move("ex_ev");
        }

        // 초음파 전진 후 180도 회전
        if (strcmp(ultra.data.c_str(), "EVin") == 0) {
            ROS_INFO("in to EV");
            ultra_move("forward");
            delay(500);
            ROS_INFO("180turn");
            turn180deg("left");
            delay(500);
            end.data= "end";
            pubUltraEnd.publish(end);
        }
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
    ros::init(argc, argv, "Sensor");


    if (wiringPiSetup() == -1) {
        ROS_ERROR("Failed to initialize WiringPi.");
        return 1;
    }

    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    
    ROS_INFO("SET UP ULTRA SENSOR");

    Sensor sensor;

    ros::spin();

    return 0;
}
