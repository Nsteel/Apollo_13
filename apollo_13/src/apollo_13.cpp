/*
 * apollo_13.cpp
 *
 *      Authors: Sebastian Ehmes
 *         Nicolas Acero
 *         Huynh-Tan Truong
 *         Li Zhao
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <apollo_13/ScalableControl.h>
#include <sensor_msgs/Range.h>

// The following variables all receive data from the callbacks provided to fetch
// data published on topics.
int motorControlOSC = 0;
int steeringControlOSC = 0;
int autControlOSC = 0;
int arucoControlOSC = 0;
int killSwitch = 0;
int sbplMotorControl = 0;
int sbplSteeringControl = 0;
int frontUsValue = 0;
int leftUsValue = 0;
int rightUsValue = 0;

/* Callback for the motor control data send from the smartphone app
*/
void motorOSC(const touchosc_msgs::ScalableControl::ConstPtr& motorctrl)
{
  motorControlOSC = motorctrl->value;
}
/* Callback for the steering control data send from the smartphone app
*/
void steeringOSC(const touchosc_msgs::ScalableControl::ConstPtr& steeringctrl)
{
  steeringControlOSC = steeringctrl->value;
}
/* Callback for the autonomous control toggle data send from the smartphone app
*/
void autonomousOSC(const touchosc_msgs::ScalableControl::ConstPtr& autonomous)
{
  autControlOSC = autonomous->value;
}
/* Callback for the aruco control toggle data send from the smartphone app
*/
void arucoOSC(const touchosc_msgs::ScalableControl::ConstPtr& aruco)
{
  arucoControlOSC = aruco->value;
}
/* Callback for the killswitch toggle data send from the smartphone app
*/
void killOSC(const touchosc_msgs::ScalableControl::ConstPtr& kill)
{
  killSwitch = kill->value;
}
/* Callback for the motor control data send from the sbpl motion planner
*/
void sbplMotor(const std_msgs::Int32::ConstPtr& sbplMotor)
{
  sbplMotorControl = sbplMotor->data;
}
/* Callback for the steering control data send from the sbpl motion planner
*/
void sbplSteering(const std_msgs::Int32::ConstPtr& sbplSteering)
{
  sbplSteeringControl = sbplSteering->data;
}
/* Callback for the raw ultrasonic sensor data send from car_handler
*/
void frontUltraSonic(const std_msgs::Int32::ConstPtr& frontUS){
  frontUsValue = frontUS->data;
}
/* Callback for the raw ultrasonic sensor data send from car_handler
*/
void leftUltraSonic(const std_msgs::Int32::ConstPtr& leftUS){
  leftUsValue = leftUS->data;
}
/* Callback for the raw ultrasonic sensor data send from car_handler
*/
void rightUltraSonic(const std_msgs::Int32::ConstPtr& rightUS){
  rightUsValue = rightUS->data;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "apollo_13");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;

		/**
	   * The advertise() function is how you tell ROS that you want to
	   * publish on a given topic name. This invokes a call to the ROS
	   * master node, which keeps a registry of who is publishing and who
	   * is subscribing. After this advertise() call is made, the master
	   * node will notify anyone who is trying to subscribe to this topic name,
	   * and they will in turn negotiate a peer-to-peer connection with this
	   * node.  advertise() returns a Publisher object which allows you to
	   * publish messages on that topic through a call to publish().  Once
	   * all copies of the returned Publisher object are destroyed, the topic
	   * will be automatically unadvertised.
	   *
	   * The second parameter to advertise() is the size of the message queue
	   * used for publishing messages.  If messages are published more quickly
	   * than we can send them, the number here specifies how many messages to
	   * buffer up before throwing some away.
	   */

		//variables for publishing
    std_msgs::Int32 arucoConrolValue, motorValue, steeringValue;

    //subscriber for the smartphone app :
			//motor and steering commands
    ros::Subscriber motorOSC_sub = nh.subscribe<touchosc_msgs::ScalableControl>("touchosc/1/speed", 50, motorOSC);
    ros::Subscriber steeringOSC_sub = nh.subscribe<touchosc_msgs::ScalableControl>("touchosc/1/direction", 50, steeringOSC);
		  //autonomous steering and aruco on/off commands
    ros::Subscriber autonomousOSC_sub = nh.subscribe<touchosc_msgs::ScalableControl>("touchosc/modes/autonomous", 50, autonomousOSC);
    ros::Subscriber arucoOSC_sub = nh.subscribe<touchosc_msgs::ScalableControl>("touchosc/modes/follow_aruco", 50, arucoOSC);
			//emergency kill switch
		ros::Subscriber killOSC_sub = nh.subscribe<touchosc_msgs::ScalableControl>("touchosc/1/kill", 50, killOSC);
    //publishers for other ros nodes:
			//feedback messages for aruco goal planning algorithm
    ros::Publisher arucoActive = nh.advertise<std_msgs::Int32>("apollo_13/aruco_active", 50);
			//publish ultrasonic sensor values as range objects for the navigational stack
    ros::Publisher front_us_range = nh.advertise<sensor_msgs::Range>("front_us_range", 50);
    ros::Publisher left_us_range = nh.advertise<sensor_msgs::Range>("left_us_range", 50);
    ros::Publisher right_us_range = nh.advertise<sensor_msgs::Range>("right_us_range", 50);
    //subscriber for sbpl motion conrol values :
    ros::Subscriber sbplMotor_sub = nh.subscribe<std_msgs::Int32>("car_arduino/motor", 50, sbplMotor);
    ros::Subscriber sbplSteering_sub = nh.subscribe<std_msgs::Int32>("car_arduino/steering", 50, sbplSteering);
    //publisher for motor and steering values to car_handler :
    ros::Publisher motor = nh.advertise<std_msgs::Int32>("car_handler/motor", 50);
    ros::Publisher steering = nh.advertise<std_msgs::Int32>("car_handler/steering", 50);
    //subscriber for ultrasonic sensor values to car_handler :
    ros::Subscriber frontUsRaw = nh.subscribe<std_msgs::Int32>("car_handler/front_us", 50, frontUltraSonic);
    ros::Subscriber leftUsRaw = nh.subscribe<std_msgs::Int32>("car_handler/left_us", 50, leftUltraSonic);
    ros::Subscriber rightUsRaw = nh.subscribe<std_msgs::Int32>("car_handler/right_us", 50, rightUltraSonic);

    //Range Objects for the sonar sensors
		//-> navigational stack needs this to make use of ultrasonic range sensors
    sensor_msgs::Range front_range;
    front_range.header.frame_id = "front_sensor";
    front_range.radiation_type = 0;
    front_range.field_of_view = 0.76;
    front_range.min_range = 0.06;
    front_range.max_range = 3;

    sensor_msgs::Range left_range;
    left_range.header.frame_id = "left_sensor";
    left_range.radiation_type = 0;
    left_range.field_of_view = 0.76;
    left_range.min_range = 0.06;
    left_range.max_range = 3;

    sensor_msgs::Range right_range;
    right_range.header.frame_id = "right_sensor";
    right_range.radiation_type = 0;
    right_range.field_of_view = 0.76;
    right_range.min_range = 0.06;
    right_range.max_range = 3;

		//This node needs to publish at least at a rate of 20hz or navigational stack will complain
    ros::Rate loop_rate(20);
    while(ros::ok()) {

    /*	This Section checks which algorithm is in charge and is controlled
		 *	by the smartphone app.
		 *	When no control algorithm is selected only the smartphone app is in charge.
		 *	If autonomous control is selected, the sbpl motion planner takes over,
		 *  if aruco control is active, the algorithm will try to maneuver the car through
		 *  aruco marked goals.
		 */
    if (autControlOSC==0 && arucoControlOSC==0){
      motorValue.data = motorControlOSC;
      steeringValue.data = steeringControlOSC;
      arucoConrolValue.data = 0;

    }
    else if(autControlOSC!=0 && arucoControlOSC==0){
			motorValue.data = sbplMotorControl;
			steeringValue.data = sbplSteeringControl;
      arucoConrolValue.data = 0;
    }
    else if(autControlOSC==0 && arucoControlOSC!=0){
			motorValue.data = sbplMotorControl;
			steeringValue.data = sbplSteeringControl;
      arucoConrolValue.data = 1;
    }
		//publish feedback to the aruco algorithm
    arucoActive.publish(arucoConrolValue);

    //Emergency Switch
    if(killSwitch!=0){
			motorValue.data = 0;
			steeringValue.data = 0;
    }

		//publish control values to car_handler
    motor.publish(motorValue);
    steering.publish(steeringValue);

		//update the range objects and publish them
    ros::Time T = ros::Time::now();
    if(frontUsValue<=300){
      front_range.range = (float)frontUsValue/100.0;
      front_range.header.stamp = T;
      front_us_range.publish(front_range);

    }
    if(rightUsValue<=300){
      right_range.range = (float)rightUsValue/100.0;
      right_range.header.stamp = T;
      right_us_range.publish(right_range);
    }

    if(leftUsValue<=300){
      left_range.range = (float)leftUsValue/100.0;
      left_range.header.stamp = T;
      left_us_range.publish(left_range);
    }

    ros::spinOnce();
    loop_rate.sleep();
}

ros::spin();
}
