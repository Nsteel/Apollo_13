/*
 * car_handler.cpp
 *
 *      Authors: Sebastian Ehmes
 *               Nicolas Acero
 *               Huynh-Tan Truong
 *               Li Zhao
 *
 *      Co-Author: Eugen Lutz
 */

//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <car_handler/car_handler.h>
#include <sched.h>
#include <sys/mman.h>


#define NSEC_PER_SEC    (1000000000)

//define static members
us_data car_handler::us_distance_front;
us_data car_handler::us_distance_right;
us_data car_handler::us_distance_left;
battery_data car_handler::battery;
hall_data car_handler::hall_distance;
int car_handler::fd = 0;
char car_handler::write_buffer = 0;
char car_handler::line_detected = 0;
car_handler* car_handler::ch_instance = NULL;

int motorControlValue = 0;
int steeringControlValue = 0;
std_msgs::Int32 frontUsValue, rightUsValue, leftUsValue;

void motor(const std_msgs::Int32::ConstPtr& motor)
{
  motorControlValue = motor->data;
}

void steering(const std_msgs::Int32::ConstPtr& steering)
{
  steeringControlValue = steering->data;
}

int main(int argc, char **argv)
{

    ////////////////////RT HANDLING
   //lock memory
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            perror("mlockall failed");
            return -2;
    }

    //set rt scheduler and prio
    struct timespec t;
    struct sched_param param;
    int interval = 1000000;  //1ms
    param.sched_priority = 49; //50 is used by kernel tasklets and interrupts
    if(sched_setscheduler(0, SCHED_RR, &param) == -1) {
            perror("sched_setscheduler failed");
            return -1;
    }

    //start running in one second
    clock_gettime(CLOCK_MONOTONIC ,&t);
    t.tv_sec++;
    ////////////////////RT HANDLING

    ros::init(argc, argv, "car_handler");

    ros::NodeHandle nh;

    //create publishers for every sensor:
    ros::Publisher frontUs_pub = nh.advertise<std_msgs::Int32>("car_handler/front_us", 50);
    ros::Publisher rightUs_pub = nh.advertise<std_msgs::Int32>("car_handler/right_us", 50);
    ros::Publisher leftUs_pub = nh.advertise<std_msgs::Int32>("car_handler/left_us", 50);
    //create subscriber for steering signals:
    ros::Subscriber motorControl_sub = nh.subscribe<std_msgs::Int32>("car_handler/motor", 50, motor);
    ros::Subscriber steeringControl_sub = nh.subscribe<std_msgs::Int32>("car_handler/steering", 50, steering);

    car_handler* ch = car_handler::get_car_handler();
    ros::Time current_time;

    while(ros::ok()) {
	  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    // calculate next shot
    t.tv_nsec += interval;
    while (t.tv_nsec >= NSEC_PER_SEC) {
		t.tv_nsec -= NSEC_PER_SEC;
        t.tv_sec++;
    }

    // set subscribed steering signals:
    ch->set_motor_level(motorControlValue);
    ch->set_steering_level(steeringControlValue);

	  //publish sensor data:
    frontUsValue.data = ch->get_front_us();
    frontUs_pub.publish(frontUsValue);

	  rightUsValue.data = ch->get_right_us();
	  rightUs_pub.publish(rightUsValue);

	  leftUsValue.data = ch->get_left_us();
	  leftUs_pub.publish(leftUsValue);

    ros::spinOnce();

    }

ros::spin();

}
