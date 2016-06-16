#include <ros/ros.h>
#include <FloatingEdges.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>

void mapMetaCallback(const nav_msgs::MapMetaData::ConstPtr& metaMsg, nav_msgs::MapMetaData *meta){
        *meta = *metaMsg;
}
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg, cv::Mat *map){
        int width = mapMsg->info.width;
        int height = mapMsg->info.height;
        *map = cv::Mat(width,height,CV_8UC1);
        if(mapMsg->data.size()>0) {
                for(int i = 0; i<height; i++) {
                        for(int j = 0; j<width; j++) {
                                int value = mapMsg->data[i*width+j];
                                int col = 0;
                                if(value==0) {
                                        col = 255;
                                }else if(value==100) {
                                        col = 0;
                                }else{
                                        col = 205;
                                }
                                map->at<uchar>(height-(i+1), j) = col;
                        }
                }
        }

}

void setGridPosition(geometry_msgs::Pose& laser, nav_msgs::MapMetaData& mapInfo, cv::Point * gridPose){
        unsigned int grid_x = (unsigned int)((laser.position.x - mapInfo.origin.position.x) / mapInfo.resolution);
        unsigned int grid_y = (unsigned int)((-laser.position.y - mapInfo.origin.position.y) / mapInfo.resolution);
        gridPose->x = grid_x;
        gridPose->y = grid_y;
}

void getPositionInfo(const std::string& base_frame, const std::string& target_frame, const tf::TransformListener& listener, geometry_msgs::Pose * position, std::vector<double> * rpy){
        bool transformReady = false;
        tf::StampedTransform stf;
        tf::Stamped<tf::Pose> tmp;
        geometry_msgs::PoseStamped tmp2;
        tf::Quaternion q;

        try{
                transformReady = listener.waitForTransform (base_frame, target_frame, ros::Time(0), ros::Duration(0.01));

        }
        catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());

        }

        if(transformReady) {
                try{
                        listener.lookupTransform(base_frame, target_frame, ros::Time(0), stf);

                }
                catch (tf::TransformException ex) {
                        ROS_ERROR("%s",ex.what());

                }
                tmp = tf::Stamped<tf::Pose>(stf, stf.stamp_, base_frame);
                tf::poseStampedTFToMsg(tmp,tmp2);
                *position = tmp2.pose;

                try{
                        tf::quaternionMsgToTF(tmp2.pose.orientation, q);
                        tf::Matrix3x3(q).getRPY((*rpy)[0], (*rpy)[1], (*rpy)[2]);
                }catch(std::exception ex) {
                        //ROS_ERROR("%s", ex.what());
                }

        }
}

int main(int argc, char **argv){

        ros::init(argc, argv, "automap");
        ros::NodeHandle nh;

        nav_msgs::MapMetaData mapMetaData;
        cv::Mat map;
        sensor_msgs::ImagePtr edgeImageMsg;

        FloatingEdges fe(0.4, 0.05);
        tf::TransformListener listener;
        geometry_msgs::Pose position;
        std::vector<double> rpy(3, 0.0);
        cv::Point gridPose;

        ros::Subscriber mapMetaSub = nh.subscribe<nav_msgs::MapMetaData>("map_metadata", 10, boost::bind(mapMetaCallback, _1, &mapMetaData));
        ros::Subscriber mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, boost::bind(mapCallback, _1, &map));
        image_transport::ImageTransport it(nh);
        image_transport::Publisher edgePub = it.advertise("floatingEdges", 1);

        // Loop starts here:
        ros::Rate loop_rate(20);
        while(ros::ok()) {



                getPositionInfo("map", "base_footprint", listener, &position, &rpy);
                setGridPosition(position, mapMetaData, &gridPose);
                //ROS_INFO("YAW: %lf", rpy[2]);

                // calculate the center of the sliding window
                edge windowBounds;
                int windowRadius = 200;
								int max_x = (gridPose.x+windowRadius<map.cols) ? gridPose.x+windowRadius : map.cols-1;
								int min_x = (gridPose.x-windowRadius>=0) ? gridPose.x-windowRadius : 0;
								int max_y = (gridPose.y+windowRadius<map.rows) ? gridPose.y+windowRadius : map.rows-1;
								int min_y = (gridPose.y-windowRadius>=0) ? gridPose.y-windowRadius : 0;

								windowBounds.push_back(cv::Point(min_x, gridPose.y));
								windowBounds.push_back(cv::Point(gridPose.x, max_y));
								windowBounds.push_back(cv::Point(gridPose.x, min_y));
								windowBounds.push_back(cv::Point(max_x, gridPose.y));

                // make sliding window a region of interest
								cv::Rect roi = cv::boundingRect(windowBounds);
                cv::Rect imROI(0, 0, map.rows, map.cols);
                //cv::Rect roi(1770, 1670, 800, 600);
                cv::Rect myROI = imROI & roi;

                // feed edge detecter with region of interest
                fe.getEdges(map(myROI), gridPose, rpy[2]/CV_PI*180.0, myROI);
                cv::Mat out = fe.drawEdges();
                edgeImageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out).toImageMsg();
                edgePub.publish(edgeImageMsg);


                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::spin();
}
