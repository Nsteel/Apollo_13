#include "FrontierDetector.h"

FrontierDetector::FrontierDetector(){
}

FrontierDetector::FrontierDetector(const FrontierDetector& other) :
        freeColor(other.freeColor), unknownColor(other.unknownColor), occupiedColor(other.occupiedColor), config(other.config),
        dilateRate(other.dilateRate), blurKernel(other.blurKernel){
        detectedFrontiers = contours();
}

FrontierDetector::FrontierDetector(const int freeColor, const int unknownColor, const int occupiedColor) :
        freeColor(freeColor), unknownColor(unknownColor), occupiedColor(occupiedColor){
        dilateRate = 0;
        blurKernel = 0;
        detectedFrontiers = contours();

}

void FrontierDetector::setConfig(automap::ExplorationConfig& config){
        this->config = config;
        this->dilateRate = config.frontier_dilation_rate;
        this->blurKernel = config.frontier_blur_kernel;

}

const contours& FrontierDetector::getFrontiers() const {
        return detectedFrontiers;
}

void FrontierDetector::processFrontiers(const cv::Mat& occupancyGrid){
        map = occupancyGrid.clone();
        if(map.rows>0 && map.cols>0) {
                imgProc();
                detectEdges();
                findFrontiers();
        }
}

void FrontierDetector::imgProc(){
        th_all_map = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
        th_wall_map = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
        all_edges = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
        wall_only_edges = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
        bw_mask = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
        floating_edges = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
        black = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));

        cv::threshold(map, th_all_map, unknownColor+3, freeColor, CV_THRESH_BINARY);
        cv::threshold(map, th_wall_map, unknownColor-1, freeColor, CV_THRESH_BINARY);
        // remove "micro map holes"
        cv::blur(th_all_map, th_all_map, cv::Size(blurKernel, blurKernel));

}

void FrontierDetector::detectEdges(){
        int lTh = unknownColor-1;
        // detect obstacles and floating edges
        cv::Canny(th_all_map, all_edges, lTh, 300, 3);
        // detect obstacles only
        cv::Canny(th_wall_map, wall_only_edges, lTh, 300, 3);

        // inflate detected obstacles to ensure clean removal
        cv::dilate(wall_only_edges, bw_mask, cv::Mat());
        for(int i = 0; i<dilateRate; i++) {
                cv::dilate(bw_mask, bw_mask, cv::Mat());
        }
        // create a mask where the obstacles are true and free space is false
        cv::bitwise_not(bw_mask, bw_mask);

        // remove obstacles so only floating edges remain
        cv::bitwise_or(all_edges, black, floating_edges, bw_mask);
}

void FrontierDetector::findFrontiers(){
        detectedFrontiers = contours();
        std::vector<cv::Vec4i> hierarchy = std::vector<cv::Vec4i>();

        //inflate edges to merge micro edges and small gaps
        if(config.frontier_inflate_frontiers) {
                cv::dilate(floating_edges, floating_edges, cv::Mat());
        }
        cv::findContours(floating_edges, detectedFrontiers, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

}
