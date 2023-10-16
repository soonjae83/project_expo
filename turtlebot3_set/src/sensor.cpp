#include <string>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <wiringPi.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#define Trig1 0  // BCM 17 (WiringPi 0)
#define Echo1 2  // BCM 27 (WiringPi 2)

#define Trig2 15 // BCM 14 (WiringPi 15)
#define Echo2 26 // BCM 12 (WiringPi 26)

using namespace std;

class Sensor{
public:
    Sensor(){
        pubCmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        pubUltraEnd = nh.advertise<std_msgs::String>("/ultra", 1);

        sub_turn_ultra = nh.subscribe("/ultra", 1, &Sensor::subUltra, this);
    }

    // 초음파 작동 //
    void ultra_move(string direction) {
        while (ros::ok()) {
            int start_time1, end_time1, start_time2, end_time2;
            float distance1, distance2;
            float target_distance = 12.5; // 목표 거리
            float distance_difference_threshold = 0.01; // 거리 차이 허용 범위 (1cm)

            digitalWrite(Trig1, LOW);
            delayMicroseconds(2);
            digitalWrite(Trig1, HIGH);
            delayMicroseconds(10);
            digitalWrite(Trig1, LOW);

            // 초음파 센서 1 읽기
            while (digitalRead(Echo1) == 0);
            start_time1 = micros();
            while (digitalRead(Echo1) == 1);
            end_time1 = micros();

            // 초음파 센서 2 읽기
            digitalWrite(Trig2, LOW);
            delayMicroseconds(2);
            digitalWrite(Trig2, HIGH);
            delayMicroseconds(10);
            digitalWrite(Trig2, LOW);

            while (digitalRead(Echo2) == 0);
            start_time2 = micros();
            while (digitalRead(Echo2) == 1);
            end_time2 = micros();

            distance1 = round((end_time1 - start_time1) / 29.0 / 2.0 * 100) / 100.0; // 반올림하여 소수점 두 자리까지 표시
            distance2 = round((end_time2 - start_time2) / 29.0 / 2.0 * 100) / 100.0; // 반올림하여 소수점 두 자리까지 표시

            ROS_INFO("[INFO] Distance1: %.2f cm", distance1);
            ROS_INFO("[INFO] Distance2: %.2f cm", distance2);

            if (direction == "forward") {
                // 두 초음파 센서의 거리 차이 계산
                float distance_difference = distance1 - distance2;

                // 거리 오차를 보정하여 로봇의 자세를 조절
                if (distance_difference > distance_difference_threshold) {
                    ROS_INFO("right_position");
                    cmd_vel.angular.z = 0.1; // 오른쪽으로 약간 회전
                    pubCmdvel.publish(cmd_vel);
                } else if (distance_difference < -distance_difference_threshold) {
                    ROS_INFO("left_position");
                    cmd_vel.angular.z = -0.1; // 왼쪽으로 약간 회전
                    pubCmdvel.publish(cmd_vel);
                } else {
                    ROS_INFO("alright");
                    cmd_vel.angular.z = 0.0; // 오차가 허용 범위 안에 있으면 회전 중지
                    pubCmdvel.publish(cmd_vel);
                }

                // 만약 두 distance 값이 모두 12.5cm보다 작으면 루프를 빠져나감
                if (distance1 < target_distance && distance2 < target_distance) {
                    cmd_vel.linear.x = 0.0;
                    pubCmdvel.publish(cmd_vel);
                    ROS_INFO("STOP");
                    break;
                }

                // 전진 속도 설정
                ROS_INFO("go_straight");
                cmd_vel.linear.x = 0.08;
                pubCmdvel.publish(cmd_vel);
            }

            ros::Duration(0.1).sleep(); // 반복 주기를 조절할 수 있습니다.
        }
    }

    void exit_ev(string direction) {
        if (direction == "ex_ev") {
            cmd_vel.linear.x = 0.10;
        }

        ROS_INFO("Exit_Elevator");

        for (int i = 0; i < 1010; i++) {
            pubCmdvel.publish(cmd_vel);
            delay(10);
        }

        cmd_vel.linear.x = 0.00;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Exit_end");
    }
    // 90도 회전 //
    void turn90deg(string direction) {
        if (direction == "left") {
            cmd_vel.angular.z = 0.8;
        }

        if (direction == "right") {
            cmd_vel.angular.z = -0.8;
        }

        ROS_INFO("Turn 90deg start");

        for (int i = 0; i < 206; i++) {
            pubCmdvel.publish(cmd_vel);
            delay(10);
        }

        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn 90deg end");
    }
    // 180도 회전 //
    void turn180deg(string direction) {
        if (direction == "left") {
            cmd_vel.angular.z = 0.8;
        }
        
        if (direction == "right") {
            cmd_vel.angular.z = -0.8;
        }

        ROS_INFO("Turn 180deg start");

        for (int i = 0; i < 403; i++) {
            pubCmdvel.publish(cmd_vel);
            delay(10);
        }

        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn 180deg end");
    }

    void subUltra(const std_msgs::String ultra) {
        // 초음파 전진
        if (strcmp(ultra.data.c_str(), "forward") == 0 ) {
        ROS_INFO("Forward");
        ultra_move("forward");
        }
        // 90도 회전
        if (strcmp(ultra.data.c_str(), "right90") == 0 ) {
            ROS_INFO("90turn");
            turn90deg("right");
        }

        // 180도 회전
        if (strcmp(ultra.data.c_str(), "right180") == 0 ) {
            ROS_INFO("180turn");
            turn180deg("right");
        }

        // 엘베 탈출
        if (strcmp(ultra.data.c_str(), "exit") == 0 ) {
            ROS_INFO("Exit_EV");
            exit_ev("ex_ev");
            delay(500);
            ROS_INFO("eixt_complete");
            end.data= "complete";
            pubUltraEnd.publish(end);
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

    pinMode(Trig1, OUTPUT);
    pinMode(Trig2, OUTPUT);
    pinMode(Echo1, INPUT);
    pinMode(Echo2, INPUT);
    
    ROS_INFO("SET UP ULTRA SENSOR");

    Sensor sensor;

    ros::spin();

    return 0;
}
