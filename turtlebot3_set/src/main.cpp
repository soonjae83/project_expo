#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h" 

using namespace std;

class Main {
public:
Main() {
    fnInitParam();

    pubPoseStamped = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    pubUltra = nh.advertise<std_msgs::String>("/ultra", 1);
    pubFirebase = nh.advertise<std_msgs::String>("/ros_to_firebase", 1000);
    pubElevator = nh.advertise<std_msgs::String>("/elevator", 1);
    pubRobot = nh.advertise<std_msgs::String>("/Check", 1);

    sub_arrival_status = nh.subscribe("/move_base/result", 1, &Main::CheckArrival, this);
    sub_ultra_status = nh.subscribe("/ultra", 1, &Main::CheckUltra, this);
    sub_firebase = nh.subscribe("firebase_to_ros", 1000, &Main::CheckFirebase, this);
    sub_elevator = nh.subscribe("/elevator", 1, &Main::CheckElevator, this);
    sub_module_status = nh.subscribe("/state", 1, &Main::CheckModule, this);
};

enum RobotStatus {
    WAIT,
    FRONT_1F,
    FRONT_2F,
    EV_IN,
    EV_EXIT,
    ARR,
    MOVING,
    MOVING_EV,
    ARR_1fRoom,
    ARR_2fRoom,
    RETURN_LOBBY
};

enum RobotLocate {
    FIRST_FLOOR,
    SECOND_FLOOR
};

// 초기 좌표 설정
void fnInitParam() {
    // 101호 / 201호 //
    nh.getParam("go_to_01/position", target_pose_position);
    nh.getParam("go_to_01/orientation", target_pose_orientation);

    poseStamped[0].header.frame_id = "map";
    poseStamped[0].header.stamp = ros::Time::now();

    poseStamped[0].pose.position.x = target_pose_position[0];
    poseStamped[0].pose.position.y = target_pose_position[1];
    poseStamped[0].pose.position.z = target_pose_position[2];

    poseStamped[0].pose.orientation.x = target_pose_orientation[0];
    poseStamped[0].pose.orientation.y = target_pose_orientation[1];
    poseStamped[0].pose.orientation.z = target_pose_orientation[2];
    poseStamped[0].pose.orientation.w = target_pose_orientation[3];

    // 201호 / 202호 //
    nh.getParam("go_to_02/position", target_pose_position);
    nh.getParam("go_to_02/orientation", target_pose_orientation);

    poseStamped[1].header.frame_id = "map";
    poseStamped[1].header.stamp = ros::Time::now();

    poseStamped[1].pose.position.x = target_pose_position[0];
    poseStamped[1].pose.position.y = target_pose_position[1];
    poseStamped[1].pose.position.z = target_pose_position[2];

    poseStamped[1].pose.orientation.x = target_pose_orientation[0];
    poseStamped[1].pose.orientation.y = target_pose_orientation[1];
    poseStamped[1].pose.orientation.z = target_pose_orientation[2];
    poseStamped[1].pose.orientation.w = target_pose_orientation[3];

    // 로비 //
    nh.getParam("return_to_lobby/position", target_pose_position);
    nh.getParam("return_to_lobby/orientation", target_pose_orientation);

    poseStamped[2].header.frame_id = "map";
    poseStamped[2].header.stamp = ros::Time::now();

    poseStamped[2].pose.position.x = target_pose_position[0];
    poseStamped[2].pose.position.y = target_pose_position[1];
    poseStamped[2].pose.position.z = target_pose_position[2];

    poseStamped[2].pose.orientation.x = target_pose_orientation[0];
    poseStamped[2].pose.orientation.y = target_pose_orientation[1];
    poseStamped[2].pose.orientation.z = target_pose_orientation[2];
    poseStamped[2].pose.orientation.w = target_pose_orientation[3];

    // 엘리베이터 나가기 //
    nh.getParam("out_of_ev/position", target_pose_position);
    nh.getParam("out_of_ev/orientation", target_pose_orientation);

    poseStamped[3].header.frame_id = "map";
    poseStamped[3].header.stamp = ros::Time::now();

    poseStamped[3].pose.position.x = target_pose_position[0];
    poseStamped[3].pose.position.y = target_pose_position[1];
    poseStamped[3].pose.position.z = target_pose_position[2];

    poseStamped[3].pose.orientation.x = target_pose_orientation[0];
    poseStamped[3].pose.orientation.y = target_pose_orientation[1];
    poseStamped[3].pose.orientation.z = target_pose_orientation[2];
    poseStamped[3].pose.orientation.w = target_pose_orientation[3];

    // 엘리베이터 진입 전 문 앞 (1층/2층) //
    nh.getParam("front_of_ev/position", target_pose_position);
    nh.getParam("front_of_ev/orientation", target_pose_orientation);

    poseStamped[4].header.frame_id = "map";
    poseStamped[4].header.stamp = ros::Time::now();

    poseStamped[4].pose.position.x = target_pose_position[0];
    poseStamped[4].pose.position.y = target_pose_position[1];
    poseStamped[4].pose.position.z = target_pose_position[2];

    poseStamped[4].pose.orientation.x = target_pose_orientation[0];
    poseStamped[4].pose.orientation.y = target_pose_orientation[1];
    poseStamped[4].pose.orientation.z = target_pose_orientation[2];
    poseStamped[4].pose.orientation.w = target_pose_orientation[3];
}

// 모듈 장착 여부 확인
void CheckModule(const std_msgs::String module) {
    if (strcmp(module.data.c_str(), "Hotel_Mode") == 0) {
        hotel = true;
        _ROBOT_FLAG = WAIT;
        _ROBOT_LOCATE = FIRST_FLOOR;
        ROS_INFO("Hotel Module Activated.");
        sleep(0.5);
    }
}

// 파이어베이스 확인
void CheckFirebase(const std_msgs::String firebase) {
    if (_ROBOT_LOCATE == FIRST_FLOOR) {    
        if (_ROBOT_FLAG == WAIT || _ROBOT_FLAG == ARR_1fRoom || _ROBOT_FLAG == ARR_2fRoom || _ROBOT_FLAG == MOVING) {
            if (strcmp(firebase.data.c_str(), "101_go") == 0) {
                ROS_INFO("move_to_101");
                pubPoseStamped.publish(poseStamped[0]);
                sleep(5);
                fb.data = "101_arrive";
                pubFirebase.publish(fb);
                ROS_INFO("arrive_to_101");
                _ROBOT_FLAG = ARR_1fRoom;
            }      
            if (strcmp(firebase.data.c_str(), "102_go") == 0) {
                ROS_INFO("move_to_102");
                pubPoseStamped.publish(poseStamped[1]);
                sleep(5);
                fb.data = "102_arrive";
                pubFirebase.publish(fb);
                ROS_INFO("arrive_to_102");
                _ROBOT_FLAG = ARR_1fRoom;  
            }
            if (strcmp(firebase.data.c_str(), "201_go") == 0) {
                ROS_INFO("move_to_EV");
                pubPoseStamped.publish(poseStamped[4]);
                sleep(5);
                _ROBOT_FLAG = MOVING_EV;
                if (_ROBOT_LOCATE == SECOND_FLOOR) {
                    ROS_INFO("move_to_201");
                    pubPoseStamped.publish(poseStamped[0]);
                    sleep(5);
                    fb.data = "201_arrive";
                    pubFirebase.publish(fb);
                    ROS_INFO("arrive_to_201");
                    _ROBOT_FLAG = ARR_2fRoom;
                }
            }
            if (strcmp(firebase.data.c_str(), "202_go") == 0) {       
                ROS_INFO("move_to_EV");
                pubPoseStamped.publish(poseStamped[4]);
                sleep(5);
                _ROBOT_FLAG = MOVING_EV;
                if (_ROBOT_LOCATE == SECOND_FLOOR) {
                    ROS_INFO("move_to_202");
                    pubPoseStamped.publish(poseStamped[1]);
                    sleep(5);
                    fb.data = "202_arrive";
                    pubFirebase.publish(fb);
                    ROS_INFO("arrive_to_202");
                    _ROBOT_FLAG = ARR_2fRoom;
                }
            }
            if (strcmp(firebase.data.c_str(), "home") == 0) {
                ROS_INFO("return_to_lobby");
                pubPoseStamped.publish(poseStamped[2]);
                sleep(5);
                ROS_INFO("arrive_to_Lobby");
                _ROBOT_FLAG = RETURN_LOBBY;
            }   
        }
    }

    if (_ROBOT_LOCATE == SECOND_FLOOR) {    
        if (_ROBOT_FLAG == WAIT || _ROBOT_FLAG == ARR_1fRoom || _ROBOT_FLAG == ARR_2fRoom || _ROBOT_FLAG == MOVING) {
            if (strcmp(firebase.data.c_str(), "101_go") == 0) {
                ROS_INFO("move_to_EV");
                pubPoseStamped.publish(poseStamped[4]);
                sleep(5);
                _ROBOT_FLAG = MOVING_EV;
                if (_ROBOT_LOCATE == FIRST_FLOOR) {
                    ROS_INFO("move_to_101");
                    pubPoseStamped.publish(poseStamped[0]);
                    sleep(5);
                    fb.data = "101_arrive";
                    pubFirebase.publish(fb);
                    ROS_INFO("arrive_to_101");
                    _ROBOT_FLAG = ARR_1fRoom;
                }
            }
            if (strcmp(firebase.data.c_str(), "102_go") == 0) {       
                ROS_INFO("move_to_EV");
                pubPoseStamped.publish(poseStamped[4]);
                sleep(5);
                _ROBOT_FLAG = MOVING_EV;
                if (_ROBOT_LOCATE == FIRST_FLOOR) {
                    ROS_INFO("move_to_102");
                    pubPoseStamped.publish(poseStamped[1]);
                    sleep(5);
                    fb.data = "102_arrive";
                    pubFirebase.publish(fb);
                    ROS_INFO("arrive_to_102");
                    _ROBOT_FLAG = ARR_1fRoom;
                }
            }
            if (strcmp(firebase.data.c_str(), "201_go") == 0) {
                ROS_INFO("move_to_201");
                pubPoseStamped.publish(poseStamped[0]);
                sleep(5);
                fb.data = "201_arrive";
                pubFirebase.publish(fb);
                ROS_INFO("arrive_to_201");
                _ROBOT_FLAG = ARR_2fRoom;
            }      
            if (strcmp(firebase.data.c_str(), "202_go") == 0) {
                ROS_INFO("move_to_202");
                pubPoseStamped.publish(poseStamped[1]);
                sleep(5);
                fb.data = "202_arrive";
                pubFirebase.publish(fb);
                ROS_INFO("arrive_to_202");
                _ROBOT_FLAG = ARR_2fRoom;  
            }
            if (strcmp(firebase.data.c_str(), "home") == 0) {
                ROS_INFO("move_to_ev");
                pubPoseStamped.publish(poseStamped[4]);
                sleep(5);
                _ROBOT_FLAG = MOVING_EV;    
                if (_ROBOT_LOCATE == FIRST_FLOOR) {
                    ROS_INFO("return_to_lobby");
                    pubPoseStamped.publish(poseStamped[2]);
                    sleep(5);
                    ROS_INFO("arrive_to_Lobby");
                    _ROBOT_FLAG = RETURN_LOBBY;               
                }
            }   
        }
    }
}

void CheckArrival(const move_base_msgs::MoveBaseActionResult arrival) {
    if (arrival.status.status == 3) {
        if (_ROBOT_LOCATE == FIRST_FLOOR && _ROBOT_FLAG == MOVING_EV) {
            ROS_INFO("front_of_1f");
            _ROBOT_FLAG = FRONT_1F;
        }
        if (_ROBOT_LOCATE == SECOND_FLOOR && _ROBOT_FLAG == MOVING_EV) {
            ROS_INFO("front_of_2f");
            _ROBOT_FLAG = FRONT_2F;
        }
        if (_ROBOT_FLAG == RETURN_LOBBY) {
            ROS_INFO("Lobby");
            sleep(2);
            _ROBOT_FLAG = WAIT;
        }
        else {
            ROS_INFO("turn180deg");
            sleep(5);
            ultra.data = "right180";
            pubUltra.publish(ultra);
            sleep(5);
            ROS_INFO("Next_Target");
        }
    }
}
void CheckUltra(const std_msgs::String ultra) {
    // 엘레베이터 1층에서 탈출 후 // 
    if (_ROBOT_LOCATE == FIRST_FLOOR) {
        if (_ROBOT_FLAG == EV_EXIT && (strcmp(ultra.data.c_str(), "end") == 0)) { 
            ROS_INFO("Arrive_1F");
            sleep(5);
            direction.data = "right";
            pubUltra.publish(direction);
            _ROBOT_FLAG = MOVING;
        }
    }
    // 엘레베이터 2층에서 탈출 후 //
    if (_ROBOT_LOCATE == SECOND_FLOOR) {
        if (_ROBOT_FLAG == EV_EXIT && (strcmp(ultra.data.c_str(), "end") == 0)) {
            ROS_INFO("Arrive_2F");
            sleep(5);
            direction.data = "right";
            pubUltra.publish(direction);
            _ROBOT_FLAG = MOVING;
        }
    }
}


// 1층 -> 2층 //
void CheckElevator(const std_msgs::String elevator) {
    // 1층 -> 2층 // 
    if (_ROBOT_FLAG == FRONT_1F) {
        ev.data = "1F_open";
        pubElevator.publish(ev);
        sleep(5);
        if (strcmp(elevator.data.c_str(), "first_open_in") == 0) {
            ROS_INFO("1F_door_open");
            ultra.data = "EVin";
            pubUltra.publish(ultra);
            sleep(5);
            ev.data = "1F_in";
            pubElevator.publish(ev);
            _ROBOT_FLAG = EV_IN;
        }
    }
    // 2층 -> 1층 //
    if (_ROBOT_FLAG == FRONT_2F) {
        ev.data = "2F_open";
        pubElevator.publish(ev);
        sleep(5);
        if (strcmp(elevator.data.c_str(), "second_open_in") == 0) {
            ROS_INFO("2F_door_open");
            ultra.data = "EVin";
            pubUltra.publish(ultra);
            sleep(5);
            ev.data = "2F_in";
            pubElevator.publish(ev);
            _ROBOT_FLAG = EV_IN;
        }
    }

    if (_ROBOT_FLAG == EV_IN) {
    // 2층 도착//
        if (strcmp(elevator.data.c_str(), "second_open_out") == 0) {
            _ROBOT_LOCATE = SECOND_FLOOR;
            ROS_INFO("2F_ARRIVE");
            sleep(2);
            ultra.data = "exit";
            pubUltra.publish(ultra);
            sleep(2);
            _ROBOT_FLAG = EV_EXIT;
        }
    // 1층 도착 //
        if (strcmp(elevator.data.c_str(), "first_open_out") == 0) {
            _ROBOT_LOCATE = FIRST_FLOOR;
            ROS_INFO("1F_ARRIVE");
            sleep(2);
            ultra.data = "exit";
            pubUltra.publish(ultra);
            sleep(2);
            _ROBOT_FLAG = EV_EXIT;
        }
    }
}

private:
ros::NodeHandle nh;

ros::Publisher pubPoseStamped;
ros::Publisher pubUltra;
ros::Publisher pubElevator;
ros::Publisher pubFirebase;
ros::Publisher pubRobot;

ros::Subscriber sub_arrival_status;
ros::Subscriber sub_ultra_status;
ros::Subscriber sub_firebase;
ros::Subscriber sub_elevator;
ros::Subscriber sub_module_status;

geometry_msgs::PoseStamped poseStamped[5];

vector<double> target_pose_position;
vector<double> target_pose_orientation;

int _ROBOT_FLAG, _ROBOT_LOCATE;

std_msgs::String ev, ultra, fb, direction;

bool hotel = false;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Main");

    ROS_INFO("[SYSTEM ON]");

    Main main;

    ros::spin();

    return 0; 
}
