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
const uint8_t LEFT = 0, CENTER = 1, RIGHT = 2;

float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=20; 

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	//fill with your code
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    ROS_INFO("Size of laser scan array: %i and size of offset: %i. Laser Distance %f", nLasers, desiredNLasers, minLaserDist);

    if (DEG2RAD(desiredAngle) < msg->angle_max && DEG2RAD(-desiredAngle) > msg->angle_min) {
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

template <class T>
T randBetween(T a, T b){
    // Returns a random number between a and b inclusive. Assumes a<b.
    return (b-a)*(rand()/RAND_MAX) + a;
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
}

bool anyBumperPressed(){
    bool any_bumper_pressed = false;
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
    }
    return any_bumper_pressed;
}

void moveThruDistance(float desired_dist, float startX, float startY, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub,
                    uint64_t* pSecondsElapsed, const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    int i = 0;
    float current_dist = sqrt(pow(posX-startX, 2) + pow(posY-startY, 2));
    while (current_dist < fabs(desired_dist) && i < 250){
        ros::spinOnce();
        angular = 0;
        linear = copysign(0.15, desired_dist); //move 0.15 m/s in direction of desired_dist
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular
        current_dist = sqrt(pow(posX-startX, 2) + pow(posY-startY, 2));
        i+=1;
    }
}

void rotateThruAngle(float angleRAD, float yawStart, float laserDistStart, bool breakEarly, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub,
                    uint64_t* pSecondsElapsed, const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    //Rotates turtlebot angleRAD rad CW or CCW in place depending on angleRAD's sign at pi/6 rad/s
    int i = 0;
    ROS_INFO("In rotating thru. \n Start yaw: %f \n Current yaw: %f \n minLaserDistance %f \n Desired angle: %f", yawStart, yaw, minLaserDist, angleRAD);
    ROS_INFO("Condition check: %i", fabs(yaw - yawStart) <= fabs(angleRAD));

    while (fabs(yaw - yawStart) <= fabs(angleRAD) && i < 250){
        ros::spinOnce();
        ROS_INFO("ROTATING %f \n Start yaw: %f \n Current yaw: %f \n minLaserDistance %f \n Iter: %i", angleRAD, yawStart, yaw, minLaserDist, i);
        ROS_INFO("Condition check %i \n LS: %f \n RS %f \n", fabs(yaw - yawStart) <= fabs(angleRAD), fabs(yaw - yawStart), fabs(angleRAD));
        angular = copysign(M_PI/6, angleRAD); //turn pi/6 rad/s in direction of angleRAD
        linear = 0;
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular

        if (anyBumperPressed()){
            ROS_INFO("Breaking out of rotate due to bumper press \n Left: %d \n Center: %d \n Right: %d \n", bumper[LEFT], bumper[CENTER], bumper[RIGHT]);
            break;
        }
        if (minLaserDist >0.75 && minLaserDist <8 && breakEarly){
            ROS_INFO("Breaking out of rotate due to large distance");
            break;
        }
        i +=1;
    }
}

void wall_barrier(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
        const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    while(true){
        if(minLaserDist < 0.6){
            linear = 0;
            angular = 0;
            update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
            //edge();
        }
        if (randBetween(0,1) < 0.3){
            //depart();
        }
        if (minLaserDist < 0.6){
            //corner();
        }
        if (randBetween(0,1) < 0.4){
            //wall_turnaround();
        }
        
        angular = randBetween(-M_PI/12, M_PI/12);
        linear = 0.25* (M_PI/6 - abs(angular))/(M_PI/6);
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
    }
}

void edge(int leftAway, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
        const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    while(true){
        linear = 0.25;
        angular = -leftAway*0.3/minLaserDist;
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);

        if (randBetween(0, 1) < 0.4){
            linear = 0;
            update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
            //away();
        }
        if (minLaserDist < 0.6){
            angular = 0;
            update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
            //wall_barrier();
        }
        if (minLaserDist > 0.75 && minLaserDist < 7){
            angular = 0;
            linear = 0.25;
            update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
            //straight();
        } 
    }
}

void away(int leftAway, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
        const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    int i = 0;
    while(true){
        float targetAngle = leftAway*DEG2RAD(26) + yaw;
        angular = fmin(M_PI/6, 3*targetAngle);
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);

        if ((minLaserDist > 0.75 && minLaserDist < 7) || (minLaserDist < 0.75 && abs(yaw - targetAngle) < RAD2DEG(5.7)) || i>= 15){
            //straight();
        }
        if (minLaserDist < 0.6){
            //wall_barrier();
        }
        i += 1;
    }
}


void wall_turnaround(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
                    const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
        // Rotates turtlebot when going along a wall depending on which bumper is hit
        angular = M_PI*2;
        linear = 0;
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
}

void straight(geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
        const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    while(true){
        
        bool any_bumper_pressed = false;
        any_bumper_pressed = anyBumperPressed();

        if(minLaserDist > 0.7 && !any_bumper_pressed){
            linear = 0.25;
            angular = 0;
            update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
            //go straight();
        }
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

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    
    while(ros::ok() && secondsElapsed <= 900) {
        //fill with your code
        ros::spinOnce();
        ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);

        // Check if any of the bumpers were pressed.
        bool any_bumper_pressed = false;
        any_bumper_pressed = anyBumperPressed();

        if (!any_bumper_pressed && minLaserDist < 7){
            if (minLaserDist > 0.75){
                ROS_INFO("Clear path");
/*                 //Scan +/- 30 degrees if on clear path every 25s, then continue on path.
                if (secondsElapsed % 25 == 0){
                    ROS_INFO("STARTING SCAN");
                    rotateThruAngle(M_PI/6, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                    rotateThruAngle(-2*M_PI/6, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                    rotateThruAngle(M_PI/6, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                } */
                linear = 0.25;
                angular = 0;  
            }
            else if (minLaserDist > 0.6 && minLaserDist <= 0.75){
                ROS_INFO("Slowing down");
                linear = 0.1;
                angular = randBetween(-M_PI/6, M_PI/6);
            }
            else{
                //Rotate pi degrees
                ROS_INFO("Small Distance. Rotating pi");
                rotateThruAngle(M_PI, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
        }
        else{
            ROS_INFO("Bumper pressed or laser inf \n Left: %d \n Center: %d \n Right: %d \n", bumper[LEFT], bumper[CENTER], bumper[RIGHT]);
            if (minLaserDist > 0.6){
                linear = -0.1;
                //moveThruDistance(-0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                rotateThruAngle(M_PI, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
            if (bumper[CENTER]){
                linear = -0.1;
                //moveThruDistance(-0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                rotateThruAngle(DEG2RAD(180), yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
            else if (bumper[LEFT]){
                linear = -0.1;
                //moveThruDistance(-0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                rotateThruAngle(DEG2RAD(-90), yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
            else if (bumper[RIGHT]){
                linear = -0.1;
                //moveThruDistance(-0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                rotateThruAngle(DEG2RAD(90), yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
            else{
                ROS_INFO("Entering random rotation with laser inf");
                rotateThruAngle(randBetween(-M_PI, M_PI), yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
        }

        update(&vel, &vel_pub, &secondsElapsed, start, &loop_rate);
    }

    return 0;
}
