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
#include <sensor_msgs/Imu.h>

#define Trig 23
#define Echo 32

using namespace std;

class Ultra{
public:
    Ultra(){
        pubCmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel_ultra", 10);
        pubUltraEnd = nh.advertise<std_msgs::String>("/ultra", 1);
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

private:
    ros::NodeHandle nh;
    
    ros::Publisher pubCmdvel;
    ros::Publisher pubUltraEnd;

    geometry_msgs::Twist cmd_vel;
    std_msgs::String end;

    inline void delay(int ms) { this_thread::sleep_for(chrono::milliseconds(ms)); }
    inline void delayMicrosecond(int us) { this_thread::sleep_for(chrono::microseconds(us)); }
};

class Imu {
public:
    Imu() {
        pubCmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel_imu", 10);

        sub_turn_imu = nh.subscribe("/imu", 1, &Imu::subImu, this);

        target_yaw = 0.0;
        is_turning = false;
        rotation_direction = 0.0;

        cmd_vel.angular.z = rotation_direction;
    }

    void turn90deg(string direction) {
        if (!is_turning) {
            if(direction == "left") {
                target_yaw += 1.5708;  // 90도 회전 (라디안 단위)
                rotation_direction = 0.8;
            }

            if (direction == "right") {
                target_yaw -= 1.5708;  
                rotation_direction = -0.8;
            }

            is_turning(true);
            ROS_INFO("Turn 90deg start");
        }    
    }

    void turn180deg(string direction) {
        if (!is_turning) {
            if (direction == "left") {
                target_yaw += 3.1416;  // 180도 회전 (라디안 단위)
                rotation_direction = 0.8;
            } 

            if (direction == "right") {
                target_yaw -= 3.1416;  // 180도 회전 (라디안 단위)
                rotation_direction = -0.8;
            }

            is_turning = true;
            ROS_INFO("Turn 180deg start");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_subscriber_;
    ros::Publisher cmd_vel_publisher_;
    double target_yaw;
    bool is_turning;
    double rotation_direction;
};

int main(int argc, char**argv){
    ros::init(argc, argv, "main");


    if (wiringPiSetup() == -1) {
        ROS_ERROR("Failed to initialize WiringPi.");
        return 1;
    }

    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    
    ROS_INFO("SET UP ULTRA SENSOR");

    Ultra ultra;
    Imu imu;

    ros::spin();

    return 0;
}
