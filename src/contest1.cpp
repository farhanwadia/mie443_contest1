#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <tf/transform_datatypes.h>
#include <vector>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)

float angular = 0.0;
float linear = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0, yaw_imu = 0.0, yawStart = 0.0, vel_odom = 0.0, omega = 0.0, accX = 0.0, accY = 0.0;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
const uint8_t LEFT = 0, CENTER = 1, RIGHT = 2;
bool LSLaserClearer = true;

float minLaserDist = std::numeric_limits<float>::infinity(), minLSLaserDist = std::numeric_limits<float>::infinity(), minRSLaserDist = std::numeric_limits<float>::infinity();
float LSLaserSum = 0, RSLaserSum = 0;
int32_t nLasers=0, desiredNLasers=0, desiredAngle=22.5; 

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	// Calculates minLaserDist overall, per side as minLSLaserDist and minRSLaserDist, 
    // and the sum of laser measurements on each side LSLaserSum and RSLaserSum
    
    minLaserDist = std::numeric_limits<float>::infinity();
    minLSLaserDist = std::numeric_limits<float>::infinity();
    minRSLaserDist = std::numeric_limits<float>::infinity();
    LSLaserSum = 0, RSLaserSum = 0;
    float maxLaserThreshold = 7;
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;

    int start = 0, end = nLasers;
    if (DEG2RAD(desiredAngle) < msg->angle_max && DEG2RAD(-desiredAngle) > msg->angle_min){
        start = nLasers / 2 - desiredNLasers;
        end = nLasers / 2 + desiredNLasers;
    }
    for (uint32_t laser_idx = start; laser_idx < end; ++laser_idx){
        minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        if (laser_idx <= nLasers / 2){
            minRSLaserDist = std::min(minRSLaserDist, msg->ranges[laser_idx]);
            if (msg->ranges[laser_idx] < maxLaserThreshold){
                RSLaserSum += msg->ranges[laser_idx];
            } 
        }
        else{
            minLSLaserDist = std::min(minLSLaserDist, msg->ranges[laser_idx]);
            if (msg->ranges[laser_idx] < maxLaserThreshold){
                LSLaserSum += msg->ranges[laser_idx];
            } 
        }
    }
    LSLaserClearer = LSLaserSum > RSLaserSum;
    ROS_INFO("Min Laser Distance: %f \n Left: %f \n Right: %f \n LSum: %f \n RSum: %f", minLaserDist, minLSLaserDist, minRSLaserDist, LSLaserSum, RSLaserSum);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    vel_odom = msg->twist.twist.linear.x;
    ROS_INFO("Position: (%f, %f) \n Orientation: %f deg. \n Velocity: %f", posX, posY, RAD2DEG(yaw), vel_odom);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    omega = msg->angular_velocity.z;
    accX = msg->linear_acceleration.x;
    accY = msg->linear_acceleration.y;
    yaw_imu = tf::getYaw(msg->orientation);
    ROS_INFO("Acceleration: (%f, %f) \n IMU Yaw: %f deg Omega %f", accX, accY, RAD2DEG(yaw_imu), omega);
}

template <class T>
T randBetween(T a, T b){
    // Returns a random number between a and b inclusive. Assumes a<b.
    if (std::is_same<T, int>::value){
        float x = float(rand())/float(RAND_MAX);
        return int(round(float(b-a)*x + float(a)));
    }
    else{
        T x = T(float(rand())/float(RAND_MAX));
        return (b-a)*x + a;
    }
}

void update(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed,
            const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){    
    // Sets linear and angular velocities, updates main loop timer
    (*pVel).angular.z = angular;
    (*pVel).linear.x = linear;
    (*pVel_pub).publish(*pVel);

    // Update the timer.
    *pSecondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    (*pLoop_rate).sleep();
}

bool anyBumperPressed(){
    // Returns true if any bumper is pressed, false otherwise
    bool any_bumper_pressed = false;
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
    }
    return any_bumper_pressed;
}

void moveThruDistance(float desired_dist, float move_speed, float startX, float startY, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub,
                    uint64_t* pSecondsElapsed, const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    // Moves turtlebot desired_dist at move_speed. Negative desired_dist moves backwards. Only magnitude of move_speed is used.
    int i = 0;
    float current_dist = sqrt(pow(posX-startX, 2) + pow(posY-startY, 2));
    while (current_dist < fabs(desired_dist) && i < 100 && *pSecondsElapsed < 900){
        ros::spinOnce();
        angular = 0;
        linear = copysign(fabs(move_speed), desired_dist); //move move_speed m/s in direction of desired_dist
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular
        current_dist = sqrt(pow(posX-startX, 2) + pow(posY-startY, 2));
        
        if (anyBumperPressed()){
            break;
        }
        i+=1;
    }
}

void rotateThruAngle(float angleRAD, float yawStart, float laserDistStart, float set_linear, bool breakEarly, geometry_msgs::Twist* pVel, 
                     ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    // Rotates turtlebot angleRAD rad CW(-) or CCW(+) depending on angleRAD's sign at pi/8 rad/s. 
    // Make sure angleRAD is between +/- pi
    // Use set_linear = 0 to rotate in place
    int i = 0;
    float clearPathThreshold = 0.75, maxLaserThreshold = 7;
    ROS_INFO("In rotating thru. \n Start yaw: %f \n Current yaw: %f \n minLaserDistance %f \n Desired angle: %f", yawStart, yaw, minLaserDist, angleRAD);
    ROS_INFO("Condition check: %i", fabs(yaw - yawStart) <= fabs(angleRAD));

    while (fabs(yaw - yawStart) <= fabs(angleRAD) && i < 250 && *pSecondsElapsed < 900){
        ros::spinOnce();
        ROS_INFO("ROTATING %f \n Start yaw: %f \n Current yaw: %f \n minLaserDistance %f \n IMU: %f \n Omega: %f \n Iter: %i", angleRAD, yawStart, yaw, minLaserDist, yaw_imu, omega, i);
        ROS_INFO("Condition check %i \n LS: %f \n RS %f \n", fabs(yaw - yawStart) <= fabs(angleRAD), fabs(yaw - yawStart), fabs(angleRAD));
        angular = copysign(M_PI/8, angleRAD); //turn pi/8 rad/s in direction of angleRAD
        linear = set_linear;
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular

        if (anyBumperPressed()){
            ROS_INFO("Breaking out of rotate due to bumper press \n Left: %d \n Center: %d \n Right: %d \n", bumper[LEFT], bumper[CENTER], bumper[RIGHT]);
            break;
        }
        if(fabs(omega) < 0.035 && i > 50){
            ROS_INFO("Moving forward and braking out. Likely stuck.");
            moveThruDistance(0.1, 0.1, posX, posY, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
            break;
        }

        if (minLaserDist > clearPathThreshold && minLaserDist < maxLaserThreshold && breakEarly){
            ROS_INFO("Breaking out of rotate due to large distance");
            break;
        }
        i +=1;
    }
}

float chooseAngular(float laserSideSumThreshold, float probSpinToLarger){
    // Chooses the angular velocity and direction
    // Can also call this in second argument of copysign() to only extract a direction (e.g. for a rotation)
    float prob = randBetween(0.0, 1.0), angular_vel = M_PI/8, maxLaserThreshold = 7;
    ros::spinOnce();
    if(fabs(fabs(LSLaserSum) - fabs(RSLaserSum)) > laserSideSumThreshold){
        //If one side's laser distance > other side by more than laserSideSumThreshold, go to that side
        if(fabs(LSLaserSum) - fabs(RSLaserSum) > laserSideSumThreshold){
            ROS_INFO("LS >> RS. Spin CCW");
            angular_vel = randBetween(M_PI/16, M_PI/8); //improves gmapping resolution compared to always using constant value
        }
        else{
            ROS_INFO("RS >> LS. Spin CW");
            angular_vel = -randBetween(M_PI/16, M_PI/8);
        }
    }
    else{
        // Laser distances approx. equal. Go to larger at probSpinToLarger probability
        ROS_INFO("LS ~ RS. Spinning to larger at %.2f chance", probSpinToLarger);
        if ((fabs(LSLaserSum) > fabs(RSLaserSum) || fabs(minLSLaserDist) > fabs(minRSLaserDist)) && prob < probSpinToLarger){
            angular_vel = randBetween(M_PI/16, M_PI/8); 
        }
        else{
            angular_vel = -randBetween(M_PI/16, M_PI/8);
        }
    }  
    return angular_vel;
}

void bumperPressedAction(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
                         const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    bool any_bumper_pressed = true;
    any_bumper_pressed = anyBumperPressed();
    
    if (any_bumper_pressed && *pSecondsElapsed < 900){
        ROS_INFO("Bumper pressed \n Left: %d \n Center: %d \n Right: %d \n", bumper[LEFT], bumper[CENTER], bumper[RIGHT]);
       
        if (bumper[LEFT]){
            ROS_INFO("Left hit. Move back and spin 90 CW");
            moveThruDistance(-0.25, 0.2, posX, posY, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
            rotateThruAngle(DEG2RAD(-90), yaw, minLaserDist, 0, true, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
        }
        else if (bumper[RIGHT]){
            ROS_INFO("Right hit. Move back and spin 90 CCW");
            moveThruDistance(-0.25, 0.2, posX, posY, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
            rotateThruAngle(DEG2RAD(90), yaw, minLaserDist, 0, true, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
        }
        else if (bumper[CENTER]){
            ROS_INFO("Center hit. Move back and spin");
            moveThruDistance(-0.25, 0.2, posX, posY, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
            rotateThruAngle(copysign(M_PI/2, chooseAngular(150, 0.75)), yaw, minLaserDist, 0, false, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
        }
    }
    update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
}

void moveAfterStuck(float startX, float startY, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
                    const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    // Handles trying to break the turtlebot out of being stuck
    int direction = 1;
    ROS_INFO("Moving out of stuck area");
    if (minLaserDist < 0.45 || minLaserDist > 7){
        direction = -1;
    }
    moveThruDistance(direction*0.15, 0.1, posX, posY, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
    rotateThruAngle(copysign(M_PI, chooseAngular(50, 0.9)), yaw, minLaserDist, 0, true, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);
    ros::Subscriber imu_sub = nh.subscribe("mobile_base/sensors/imu_data", 1, &imuCallback);
    
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel; 
    
    ros::Rate loop_rate(10);
    float maxLaserThreshold = 7, clearPathThreshold = 0.75, slowThreshold = 0.6, stopThreshold = 0;
    float bestAngle = 0, prob = 1, stuckStartX = posX, stuckStartY = posY;
    int clearPathIters = 0, checkStuckIters = 0;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    while(ros::ok() && secondsElapsed <= 900) {
        ros::spinOnce();
        ROS_INFO("Position: (%f, %f) Yaw: %f deg IMU %f minLaserDist: %f", posX, posY, RAD2DEG(yaw), RAD2DEG(yaw_imu), minLaserDist);

        bool any_bumper_pressed = anyBumperPressed();

        prob = randBetween(0.0, 1.0);

        if (!any_bumper_pressed && minLaserDist < maxLaserThreshold){
            if (minLaserDist > clearPathThreshold){
                ROS_INFO("Clear path. Iter: %d", clearPathIters);
                linear = 0.2;
                angular = 0;
                clearPathIters ++;
                if (clearPathIters > 20){
                    rotateThruAngle(copysign(randBetween(0.0, M_PI/6), chooseAngular(200, 0.55)), yaw, minLaserDist, 0.2, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                    clearPathIters = 0;
                }
            }
            else if(minLaserDist > slowThreshold && minLaserDist <= clearPathThreshold){
                ROS_INFO("Slowing and following clearer path");
                linear = 0.15;
                rotateThruAngle(copysign(randBetween(0.0, M_PI/8), chooseAngular(100, 0.75)), yaw, minLaserDist, 0.15, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
            else if (minLaserDist > stopThreshold && minLaserDist <= slowThreshold){
                ROS_INFO("Slowing down");
                linear = 0.1;
                rotateThruAngle(copysign(M_PI, chooseAngular(25, 0.9)), yaw, minLaserDist, 0.1, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate); 
                clearPathIters = 0;  
            }
        }
        else if (minLaserDist > maxLaserThreshold && !any_bumper_pressed){
            ROS_INFO("Laser inf, bumper free. Moving back");
            moveThruDistance(-0.7, 0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            clearPathIters = 0;
        }
        else if (any_bumper_pressed){
            ROS_INFO("Bumper pressed");
            bumperPressedAction(&vel, &vel_pub, &secondsElapsed, start, &loop_rate);   
            clearPathIters = 0;
        }

        checkStuckIters ++;
        if(checkStuckIters == 40 && clearPathIters == 0 && (sqrt(pow(posX - stuckStartX, 2) + pow(posY - stuckStartY, 2)) < 0.1 || fabs(omega) < 0.05)){
            ROS_INFO("STUCK");
            moveAfterStuck(posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            checkStuckIters = 0;
        }
        if(checkStuckIters == 40){
            checkStuckIters = 0;
        }

        update(&vel, &vel_pub, &secondsElapsed, start, &loop_rate);
    }
    ROS_INFO("15 minutes elapsed");
    return 0;
}