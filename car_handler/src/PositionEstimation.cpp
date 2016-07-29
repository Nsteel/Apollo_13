#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>

void imuFilter(const sensor_msgs::Imu::ConstPtr& imuMsg, sensor_msgs::Imu* imu){
	*imu = *imuMsg;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "position_estimation");

  ros::NodeHandle n;

  sensor_msgs::Imu imu;
  const float dt = 0.01;
  const float dt2 = 0.0001;
  cv::KalmanFilter* kalman = new cv::KalmanFilter( 9, 3, 0 );
  setIdentity(kalman->processNoiseCov, cv::Scalar::all(.05));
  cv::setIdentity(kalman->measurementMatrix);
  kalman->measurementMatrix = (cv::Mat_<float>(3, 9) << 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                                        0, 0, 0, 0, 0, 0, 0, 1, 0,
                                                        0, 0, 0, 0, 0, 0, 0, 0, 1);

  setIdentity(kalman->measurementNoiseCov, cv::Scalar::all(0.001));
  setIdentity(kalman->errorCovPost, cv::Scalar::all(.1));
  kalman->transitionMatrix = (cv::Mat_<float>(9, 9) << 1, 0, 0, dt, 0, 0, 0.5*dt2, 0, 0,
                                                         0, 1, 0, 0, dt, 0, 0, 0.5*dt2, 0,
                                                         0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt2,
                                                         0, 0, 0, 1, 0, 0, dt, 0, 0,
                                                         0, 0, 0, 0, 1, 0, 0, dt, 0,
                                                         0, 0, 0, 0, 0, 1, 0, 0, dt,
                                                         0, 0, 0, 0, 0, 0, 1, 0, 0,
                                                         0, 0, 0, 0, 0, 0, 0, 1, 0,
                                                         0, 0, 0, 0, 0, 0, 0, 0, 1);
  kalman->statePre.at<float>(0) = 0;
  kalman->statePre.at<float>(1) = 0;
  kalman->statePre.at<float>(2) = 0;
  kalman->statePre.at<float>(3) = 0;
  kalman->statePre.at<float>(4) = 0;
  kalman->statePre.at<float>(5) = 0;
  kalman->statePre.at<float>(6) = 0;
  kalman->statePre.at<float>(7) = 0;
  kalman->statePre.at<float>(8) = 0;


  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("imu/data", 10, boost::bind(imuFilter, _1, &imu));

  ros::spinOnce();
  ros::Duration d = ros::Duration(3, 0);
  d.sleep();
  ros::spinOnce();


	ros::Rate loop_rate(100);
	while(ros::ok()) {

    kalman->predict();

    double roll, pitch, yaw;
    tf::Quaternion q;

      tf::quaternionMsgToTF(imu.orientation, q);
      tf::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);

      if( isnan(roll) || isnan(pitch) || isnan(yaw)){
        roll = 0;
        pitch = 0;
        yaw = 0;
      }

    float g_x = -9.81*(std::sin(roll)*std::sin(yaw)+std::cos(roll)*std::sin(pitch)*std::cos(yaw));
    float g_y = -9.81*(-std::sin(roll)*std::cos(yaw)+std::cos(roll)*std::sin(pitch)*std::sin(yaw));
    float g_z = 9.81*(std::cos(roll)*std::cos(pitch));

    cv::Mat measurement = cv::Mat(3, 1, CV_32FC(1));
    //float ax_adj = ((int)((imu.linear_acceleration.x-g_x)*10.0))/10.0;
    //float ay_adj = ((int)((imu.linear_acceleration.y-g_y)*10.0))/10.0;
    //float az_adj = ((int)((imu.linear_acceleration.z-g_z)*10.0))/10.0;
    measurement.at<float>(0) = imu.linear_acceleration.x-g_x;
    measurement.at<float>(1) = imu.linear_acceleration.y-g_y;
    measurement.at<float>(2) = imu.linear_acceleration.z-g_z;

    cv::Mat estiMated = kalman->correct(measurement);
    //std::cout<<ax_adj<<" // "<<ay_adj<<" //"<<az_adj<<std::endl;
    std::cout<<estiMated<<std::endl;

		loop_rate.sleep();
		ros::spinOnce();
	}

  delete kalman;

	return 0;
}
