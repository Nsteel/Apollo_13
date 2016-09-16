#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>

bool newInfo = false;

void kinectCallback(const sensor_msgs::Image::ConstPtr& imgRaw, sensor_msgs::Image* raw){
    *raw=*imgRaw;
    newInfo = true;
}

void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info1, sensor_msgs::CameraInfo* info2){
    *info2=*info1;
    //std::cout<<"Encoding:"<<imgRaw->encoding<<" width: "<<imgRaw->width<<" height: "<<imgRaw->height<<" full Row: "<<imgRaw->step<<std::endl;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "kinect2_filter");
    ros::NodeHandle nh;

    sensor_msgs::Image proc;
    sensor_msgs::Image rawImg;

    sensor_msgs::CameraInfo info;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat imgIn;
    cv::Mat imgOut;

    ros::Subscriber kinectImg = nh.subscribe<sensor_msgs::Image>("/kinect2/sd/image_depth_rect", 10, boost::bind(kinectCallback, _1, &rawImg));
    ros::Subscriber kinectInfo = nh.subscribe<sensor_msgs::CameraInfo>("/kinect2/sd/camera_info", 10, boost::bind(infoCallback, _1, &info));
    ros::Publisher kinectProc = nh.advertise<sensor_msgs::Image>("/apollo_13/depth", 10);
    ros::Publisher kinectProcInfo = nh.advertise<sensor_msgs::CameraInfo>("/apollo_13/camera_info", 10);


    ros::Rate loop_rate(15);
    while(ros::ok()) {
    if(newInfo){

      try{
        cv_ptr = cv_bridge::toCvCopy(rawImg, rawImg.encoding);
      }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
      }
      imgIn = cv_ptr->image;
      cv::medianBlur(imgIn, imgOut, 3);

      cv_bridge::CvImage cvi;
      cvi.header = rawImg.header;
      cvi.encoding = rawImg.encoding;
      cvi.image = imgOut;
      cvi.toImageMsg(proc);
      rawImg.data = proc.data;
      /*
      proc.header = rawImg.header;
      proc.encoding = rawImg.encoding;
      proc.step = rawImg.step;
      proc.width = rawImg.width;
      proc.height = rawImg.height;
      */
      newInfo = false;
    }

    kinectProc.publish(rawImg);
    kinectProcInfo.publish(info);

    ros::spinOnce();
    loop_rate.sleep();
}

ros::spin();
}
