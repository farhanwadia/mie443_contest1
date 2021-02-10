#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <stdio.h>
#include <cmath>

#include <chrono>
#include <tf/transform_datatypes.h>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)

float angular = 0.0;
float linear = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0, yawStart = 0.0;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5; 

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	//fill with your code
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));

}

void update(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed,
            const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){    
    // Sets linear and angular velocities, updates main loop timer
    (*pVel).angular.z = angular;
    (*pVel).linear.x = linear;
    (*pVel_pub).publish(*pVel);

    // The last thing to do is to update the timer.
    *pSecondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    (*pLoop_rate).sleep();
    ros::spinOnce();
}

void rotateThruAngle(float angleRAD, float yawStart, float laserDistStart, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, 
                    uint64_t* pSecondsElapsed, const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    //Rotates turtlebot angleRAD rad CW or CCW in place depending on angleRAD's sign at pi/6 rad/s
    while (abs(yaw - yawStart) <= angleRAD){
        //ros::spinOnce();
        ROS_INFO("ROTATING %f \n Start yaw: %f \n Current yaw: %f \n minLaserDistance %f \n", angleRAD, yawStart, yaw, minLaserDist);
        angular = copysign(M_PI/6, angleRAD); //turn pi/6 rad/s in direction of angleRAD
        linear = 0;
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular

        //Break if laser distance not changing or larger than 0.4
        if(minLaserDist > 0.42 && minLaserDist < 7){
            ROS_INFO("BREAKING OUT OF ROTATION - CLEAR PATH");
            linear = 0.05;
            angular = 0;
            update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular
            break;
        }
/*         if(abs(minLaserDist - laserDistStart) < 0.01){
            ROS_INFO("BREAKING OUT OF ROTATION - STUCK");
            linear = -0.05;
            angular = 0;
            update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular
            break;
        } */

    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel; 
    
    ros::Rate loop_rate(10);

    float prevLaserDist = 0;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    ros::spinOnce();
    while(ros::ok() && secondsElapsed <= 900) {
        //fill with your code
        //ros::spinOnce();
        ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);

        // Check if any of the bumpers were pressed.
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }

        if (!any_bumper_pressed){
            if (minLaserDist > 0.75 && minLaserDist < 7){
                ROS_INFO("Clear path");
                linear = 0.25;
                angular = 0;  
            }
            else if (minLaserDist > 0.6 && minLaserDist <= 0.75){
                ROS_INFO("Slowing down");
                linear = 0.1;
                angular = 0;
            }
            else if (minLaserDist > 0.5 && minLaserDist <= 0.6){
                ROS_INFO("Path small. Rotate 2pi");
                rotateThruAngle(-M_PI + (2*M_PI)*(rand()/RAND_MAX), yaw, minLaserDist, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
            else {
                //Rotate  degrees
                ROS_INFO("Small Distance. Rotating");
                rotateThruAngle(DEG2RAD(90), yaw, minLaserDist, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
        }
        else{
            ROS_INFO("Bumper pressed or laser inf");
            linear = -0.1;
            //rotateThruAngle(M_PI/2, yaw, minLaserDist, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
        }

         // Control logic after bumpers are being pressed. (CHANGE this later!)
/*         if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
            angular = 0.0;
            linear = 0.2;
        }
        else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed) {
            angular = M_PI / 6;
            linear = 0.0;
        }
        else if (minLaserDist > 1. && !any_bumper_pressed) {
            linear = 0.1;
            if (yaw < 17 / 36 * M_PI || posX > 0.6) {
                angular = M_PI / 12.;
            }
            else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
                angular = -M_PI / 12.;
            }
            else {
                angular = 0;
            }
        }
        else {
            linear = 0.0;
            ROS_INFO("BUMPER HIT! Changing angular velocity");
            
            rotateThruAngle(M_PI/2, yaw, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
        }  */

        prevLaserDist = minLaserDist;
        update(&vel, &vel_pub, &secondsElapsed, start, &loop_rate);
    }

    return 0;
}
