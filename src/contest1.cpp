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
float posX = 0.0, posY = 0.0, yaw = 0.0, yaw_imu = 0.0, yawStart = 0.0, vel_odom = 0.0, accX = 0.0, accY = 0.0;
int minLaserDist_idx = -1;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
const uint8_t LEFT = 0, CENTER = 1, RIGHT = 2;

float minLaserDist = std::numeric_limits<float>::infinity(), minLSLaserDist = std::numeric_limits<float>::infinity(), minRSLaserDist = std::numeric_limits<float>::infinity();
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

    int start = 0, end = nLasers;
    if (DEG2RAD(desiredAngle) < msg->angle_max && DEG2RAD(-desiredAngle) > msg->angle_min){
        start = nLasers / 2 - desiredNLasers;
        end = nLasers / 2 + desiredNLasers;
    }
    for (uint32_t laser_idx = start; laser_idx < end; ++laser_idx){
        minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        if (minLaserDist == msg->ranges[laser_idx]){
            minLaserDist_idx = laser_idx;
        }
        if (laser_idx <= nLasers / 2){
            minLSLaserDist = std::min(minLSLaserDist, msg->ranges[laser_idx]);
        }
        else{
            minRSLaserDist = std::min(minRSLaserDist, msg->ranges[laser_idx]);
        }
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //tf::getYaw(msg->pose.pose.orientation);
    vel_odom = msg->twist.twist.linear.x;
    ROS_INFO("Position: (%f, %f) \n Orientation: %f rad or %f degrees. \n Velocity: %f", posX, posY, yaw, RAD2DEG(yaw), vel_odom);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    yaw_imu = msg->angular_velocity.z;
    accX = msg->linear_acceleration.x;
    accY = msg->linear_acceleration.y;
    ROS_INFO("Acceleration: (%f, %f) \n IMU Yaw: %f rad or %f degrees.", accX, accY, yaw_imu, RAD2DEG(yaw_imu));
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
    while (current_dist < fabs(desired_dist) && i < 250 && *pSecondsElapsed < 900){
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
    //Rotates turtlebot angleRAD rad CW or CCW in place depending on angleRAD's sign at pi/6 rad/s. Make sure angleRAD is between +/- pi
    int i = 0;
    float clearPathThreshold = 0.75, maxLaserThreshold = 7;
    ROS_INFO("In rotating thru. \n Start yaw: %f \n Current yaw: %f \n minLaserDistance %f \n Desired angle: %f", yawStart, yaw, minLaserDist, angleRAD);
    ROS_INFO("Condition check: %i", fabs(yaw - yawStart) <= fabs(angleRAD));

    while (fabs(yaw - yawStart) <= fabs(angleRAD) && i < 250 && *pSecondsElapsed < 900){
        ros::spinOnce();
        ROS_INFO("ROTATING %f \n Start yaw: %f \n Current yaw: %f \n minLaserDistance %f \n Iter: %i", angleRAD, yawStart, yaw, minLaserDist, i);
        ROS_INFO("Condition check %i \n LS: %f \n RS %f \n", fabs(yaw - yawStart) <= fabs(angleRAD), fabs(yaw - yawStart), fabs(angleRAD));
        angular = copysign(M_PI/8, angleRAD); //turn pi/6 rad/s in direction of angleRAD
        linear = 0;
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular

        if (anyBumperPressed()){
            ROS_INFO("Breaking out of rotate due to bumper press \n Left: %d \n Center: %d \n Right: %d \n", bumper[LEFT], bumper[CENTER], bumper[RIGHT]);
            break;
        }
        if (minLaserDist > clearPathThreshold && minLaserDist < maxLaserThreshold && breakEarly){
            ROS_INFO("Breaking out of rotate due to large distance");
            break;
        }
        i +=1;
    }
}

float shiftPos(float angle){
    //Takes in an angle in radians, and if it is negative, adds 2pi to it
    // i.e. angles from -pi to 0 will convert to be between pi and 2pi
    if(angle < 0){
        angle = 2*M_PI + angle;
    }
    return angle;
}

float shiftNeg(float angle){
    //Takes in an angle in radians, and if it is > pi, subtracts 2pi from it
    // i.e. angles from pi to 2pi will convert to be between -pi and 0
    if(angle > M_PI){
        angle = angle - 2*M_PI;
    }
    return angle;
}

float findBestAngle(float yawStart, bool breakEarly, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
                    const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    //Spins 2pi and returns the angle to the centre of the space that had the most laser distances over threshold
    int i = 0, startIndex = 0, bestIndex = 0, largestGroup = 0, direction = 1;
    float threshold = 0.7, maxLaserThreshold = 7, bestAngle = 0;
    std::vector<float> yaws(1);
    std::vector<bool> overThreshold(1);

    if (minRSLaserDist > minLSLaserDist){
        direction = -1;
    }

    yawStart = shiftPos(yawStart);
    while (shiftPos(shiftPos(yaw) - yawStart) < fabs(2*M_PI-0.1) && i < 500 && *pSecondsElapsed < 900){
        ros::spinOnce();
        ROS_INFO("Finding best angle. \n Start yaw: %f \n Current yaw: %f \n", yawStart, shiftPos(yaw));
        ROS_INFO("Condition check %i \n LS: %f \n RS %f \n", shiftPos(shiftPos(yaw) - yawStart) < fabs(2*M_PI-0.1), shiftPos(shiftPos(yaw) - yawStart), fabs(2*M_PI-0.1));
        angular = direction*M_PI/6;
        linear = 0;
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular
    
        //Save angles relative to yawStart and if laser distances were over the threshold
        yaws.push_back(shiftPos(shiftPos(yaw) - yawStart));
        overThreshold.push_back(minLaserDist > threshold && minLaserDist < maxLaserThreshold);

        //Return early @ current heading if breakEarly is set True
        if (breakEarly && minLaserDist > threshold && minLaserDist < maxLaserThreshold){
            return bestAngle;
        }
        
        i += 1;
    }

    //Find the start index of the largest group of True's in overThreshold, and the group size
    i = 0;
    while(i < yaws.size()){
        if (overThreshold[i] == false){
            i++;
        }
        else{
            startIndex = i;
            while(overThreshold[i] == true){
                i++;
            }
            if (i-startIndex > largestGroup){
                largestGroup = i - startIndex;
                bestIndex = startIndex;
            }
        }
    }

    //Get desired angle at the midpoint of the largest group over threshold
    bestAngle = yaws[bestIndex + int((largestGroup-1)/2)];
    bestAngle = shiftNeg(bestAngle); //Convert coordinates back to (-pi, pi) rather than (0, 2pi)

    ROS_INFO("Best angle is: %f", bestAngle);
    return bestAngle;
}

float findBestAngleByMaxDist(float yawStart, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
        const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    
    int i = 0, bestLaserDist_idx = -1;
    float bestLaserDist = -1, yawAtBestDist = 0, bestAngle = 0, angle_increment = 0, laser_offset = 0, maxLaserThreshold = 7;

    yawStart = shiftPos(yawStart);
    while (shiftPos(shiftPos(yaw) - yawStart) < fabs(2*M_PI-0.1) && i < 500 && *pSecondsElapsed < 900){
        ros::spinOnce();
        ROS_INFO("Finding best angle. \n Start yaw: %f \n Current yaw: %f \n", yawStart, shiftPos(yaw));
        ROS_INFO("Condition check %i \n LS: %f \n RS %f \n", shiftPos(shiftPos(yaw) - yawStart) < fabs(2*M_PI-0.1), shiftPos(shiftPos(yaw) - yawStart), fabs(2*M_PI-0.1));
        angular = M_PI/6;
        linear = 0;
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular

        if (minLaserDist < maxLaserThreshold){
            bestLaserDist = std::max(bestLaserDist, minLaserDist);
            if (bestLaserDist == minLaserDist){
                bestLaserDist_idx = minLaserDist_idx;
                bestAngle = shiftPos(shiftPos(yaw) - yawStart);
            }
        }
        i+=1;
    }
   
    ROS_INFO("Best angle is: %f", bestAngle);
    //Calculate the angle to the best laser dist at the particular yaw
    /* angle_increment = DEG2RAD(desiredAngle)/desiredNLasers;
    laser_offset = (bestLaserDist_idx - nLasers/2)*angle_increment;
    if (laser_offset > 0){
        laser_offset = laser_offset - 0.5*angle_increment;
    }
    else{
        laser_offset = laser_offset + 0.5*angle_increment;
    }
    ROS_INFO("Laser offset is: %f", laser_offset); */

    //Calculate the best angle
    bestAngle = bestAngle - laser_offset; // - sign since positive laser_offset is to the right in CW direction
    if (bestAngle > M_PI){
        bestAngle = bestAngle - 2*M_PI; //Convert coordinates back to (-pi, pi) rather than (0, 2pi)
    }
    ROS_INFO("Best angle is: %f", bestAngle);
    return bestAngle;
}

void if_stuck(float posXStart, geometry_msgs::Twist* pVel, ros::Publisher* pVel_pub, uint64_t* pSecondsElapsed, 
        const std::chrono::time_point<std::chrono::system_clock> start, ros::Rate* pLoop_rate){
    int i = 0;
    ROS_INFO("Checking if I'm stuck. \n Start x pos: %f \n Current x pos: %f", posXStart, posX);
    
    while (fabs(posX - posXStart) <= 0.05 && i < 100 && *pSecondsElapsed < 900){
        ros::spinOnce();
        ROS_INFO("Likely stuck. Start x pos: %f \n Current x pos: %f \n Iter: %i", posXStart, posX, i);
        update(pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate); // publish linear and angular

        if(!anyBumperPressed()){
            ROS_INFO("No bumper pressed anymore. Breaking out of stuck check"); //Should I tell it to do smthg like reverse? What if it loops back to this point?
            break;
        }
        if(i == 100){
            ROS_INFO("Reversing out of stuck area");
            moveThruDistance(-0.15, posX, posY, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
            rotateThruAngle(M_PI, yaw, minLaserDist, true, pVel, pVel_pub, pSecondsElapsed, start, pLoop_rate);
        }
        i +=1;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);
    ros::Subscriber imu = nh.subscribe("imu_data", 1, &imuCallback);
    
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel; 
    
    ros::Rate loop_rate(10);
    float bestAngle = 0, maxLaserThreshold = 7, clearPathThreshold = 0.75, stopThreshold = 0.6, prob = 1;

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

        if (secondsElapsed % 20 == 0){
            stopThreshold = randBetween(0.55, 0.65); //Adds randomness in determining when to stop and turn
        }

        if (!any_bumper_pressed && minLaserDist < maxLaserThreshold){
            if (minLaserDist > clearPathThreshold){
                ROS_INFO("Clear path");
/*                  //Scan +/- 30 degrees if on clear path every 5s, 50% of the time, then continue on path.
                 if (secondsElapsed % 45 == 0 && randBetween(0.0, 1.0) < 0.5){
                    ROS_INFO("DOING CLEAR PATH SCAN");
                    rotateThruAngle(DEG2RAD(20), yaw, minLaserDist, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                    rotateThruAngle(DEG2RAD(-40), yaw, minLaserDist, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                    rotateThruAngle(DEG2RAD(20), yaw, minLaserDist, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                }  */
                linear = 0.2;
                angular = 0; 
            }
            else if (minLaserDist > stopThreshold && minLaserDist <= clearPathThreshold){
                ROS_INFO("Slowing down");
                linear = 0.1;
                if (minLSLaserDist > minRSLaserDist && randBetween(0.0, 1.0) < 0.75){
                    angular = randBetween(0.0, M_PI/8);
                }
                else{
                    angular = randBetween(-M_PI/8, 0.0);
                }
                //angular = randBetween(-M_PI/8, M_PI/8);
            }
            else{
                /* //Rotate to best angle
                ROS_INFO("Finding best angle");
                bestAngle = findBestAngle(yaw, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                ROS_INFO("Best angle: %f. Rotating to it.", bestAngle);
                rotateThruAngle(bestAngle, yaw, minLaserDist, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                linear = 0.1; */
                ROS_INFO("Small distance. Rotating in best laser direction at 0.35 chance");
                prob = randBetween(0.0, 1.0);
                if (prob < 0.35){
                    ROS_INFO("Chose by laser direction");
                    if (minLSLaserDist > minRSLaserDist){
                        ROS_INFO("Chose CCW");
                        rotateThruAngle(M_PI, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                    }
                    else{
                        ROS_INFO("Chose CW");
                        rotateThruAngle(-M_PI, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                    }
                }
                else if (prob >= 0.35 && prob < 0.55){
                    ROS_INFO("Finding best angle - no early break out");
                    bestAngle = findBestAngle(yaw, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                    rotateThruAngle(bestAngle, yaw, minLaserDist, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                }
                else{
                    ROS_INFO("Chose randomly");
                    rotateThruAngle((2*randBetween(0, 1)-1)*M_PI, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                }

                //rotateThruAngle((2*randBetween(0, 1) - 1)*2*M_PI, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
/*                 moveThruDistance(-0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                if (minLSLaserDist > minRSLaserDist && randBetween(0.0, 1.0) < 0.25){
                    rotateThruAngle(M_PI, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                }
                else{
                    rotateThruAngle(-M_PI, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                } */
            }
        }
        else if (minLaserDist > maxLaserThreshold && !any_bumper_pressed){
            ROS_INFO("Laser infinite. Finding best angle and not allowing early break out");
            bestAngle = findBestAngle(yaw, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            rotateThruAngle(bestAngle, yaw, minLaserDist, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            linear = 0.1;
            //rotateThruAngle(M_PI, yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
        }
        else if (any_bumper_pressed){
            ROS_INFO("Bumper pressed \n Left: %d \n Center: %d \n Right: %d \n", bumper[LEFT], bumper[CENTER], bumper[RIGHT]);
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
/* HENRY START
                //Rotate pi degrees
                ROS_INFO("Small Distance. Rotating pi");
                rotateThruAngle(DEG2RAD((randBetween(0,1)*2-1)*M_PI), yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
        }
        else if (any_bumper_pressed) {
            ROS_INFO("Bumper pressed or laser inf \n Left: %d \n Center: %d \n Right: %d \n", bumper[LEFT], bumper[CENTER], bumper[RIGHT]);
            if (minLaserDist > 0.6){
                linear = -0.1;
                //moveThruDistance(-0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                rotateThruAngle(DEG2RAD((randBetween(0,1)*2-1)*90), yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }            
            if (bumper[CENTER]){
                //linear = -0.1;
                moveThruDistance(-0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                rotateThruAngle(DEG2RAD((randBetween(0,1)*2-1)*90), yaw, minLaserDist, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                }
            else if (bumper[LEFT]){
                //linear = -0.1;
                rotateThruAngle(DEG2RAD(-90), yaw, minLaserDist, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);        
                moveThruDistance(0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                }
            else if (bumper[RIGHT]){
                //linear = -0.1;
                rotateThruAngle(DEG2RAD(90), yaw, minLaserDist, false, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                moveThruDistance(0.1, posX, posY, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
                }
HENRY END */
            else{
                ROS_INFO("Entering random rotation with laser inf");
                rotateThruAngle(randBetween(-M_PI, M_PI), yaw, minLaserDist, true, &vel, &vel_pub, &secondsElapsed, start, &loop_rate);
            }
        }
        update(&vel, &vel_pub, &secondsElapsed, start, &loop_rate);
    }
    ROS_INFO("15 minutes elapsed");
    return 0;
}