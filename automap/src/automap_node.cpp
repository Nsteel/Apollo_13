#include <ros/ros.h>
#include <FloatingEdges.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

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

int main(int argc, char **argv){

        ros::init(argc, argv, "automap");
        ros::NodeHandle nh;

        nav_msgs::MapMetaData mapMetaData;
        cv::Mat map;
        sensor_msgs::ImagePtr edgeImageMsg;

        FloatingEdges fe;

        ros::Subscriber mapMetaSub = nh.subscribe<nav_msgs::MapMetaData>("map_metadata", 10, boost::bind(mapMetaCallback, _1, &mapMetaData));
        ros::Subscriber mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, boost::bind(mapCallback, _1, &map));
        image_transport::ImageTransport it(nh);
        image_transport::Publisher edgePub = it.advertise("floatingEdges", 1);

        // Loop starts here:
        ros::Rate loop_rate(1);
        while(ros::ok()) {

                cv::Rect imROI(0, 0, map.rows, map.cols);
                cv::Rect ROI(1770, 1670, 800, 600);
                cv::Rect myROI = imROI & ROI;

                fe.getEdges(map(myROI));
                cv::Mat out = fe.drawEdges();
                edgeImageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out).toImageMsg();
                edgePub.publish(edgeImageMsg);


                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::spin();
}
