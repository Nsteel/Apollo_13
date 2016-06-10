/*
 * FloatingEdges.cpp
 *
 *  Created on: Jun 6, 2016
 *      Author: sebastian
 */

#include "FloatingEdges.h"

FloatingEdges::FloatingEdges() {
	edges = contours();
	detectedEdges = contVector();
	hierarchy = std::vector<cv::Vec4i>();

}

void FloatingEdges::getEdges(const cv::Mat& map){
	if(map.rows>0 && map.cols>0){
		this->map = map;
		imgProc();
		detectEdges();
		findFloatingEdges();
	}
}

void FloatingEdges::imgProc(){
	cv::threshold(map, th_all_map, 208, 255, CV_THRESH_BINARY);
    cv::threshold(map, th_wall_map, 204, 255, CV_THRESH_BINARY);
    black = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
}

void FloatingEdges::detectEdges(){
	int lTh = 204;
	cv::Canny(th_all_map, all_edges, lTh, 300, 3);
	cv::Canny(th_wall_map, wall_only_edges, lTh, 300, 3);
	cv::dilate(wall_only_edges, bw_mask, cv::Mat());
	for(int i = 1; i<=4; i++){
		cv::dilate(bw_mask, bw_mask, cv::Mat());
	}
	cv::bitwise_not(bw_mask, bw_mask);
	cv::bitwise_or(all_edges, black, floating_edges, bw_mask);
}

bool FloatingEdges::findFloatingEdges(){
	detectedEdges = contVector();
	hierarchy = std::vector<cv::Vec4i>();
	findContours(floating_edges, detectedEdges, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	if(detectedEdges.size()==0){
		return false;
	}else{
		edges = contours();
		for(auto current : detectedEdges){
			edges.push_back(Contour(current));
		}

	}
	return true;
}

cv::Mat FloatingEdges::drawEdges(){
	cv::Mat out = map.clone();
	cv::cvtColor(out, out, CV_GRAY2RGB);;
	if(detectedEdges.size()==0){
		return out;
	}else{
		for(int idx = 0; idx >=0; idx = hierarchy[idx][0]){
			cv::drawContours(out, detectedEdges, idx, cv::Scalar(0, 0, 255), CV_FILLED, 8, hierarchy);
		}
		for(auto current : edges){
			cv::circle(out, current.getCenter(), 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		}
		return out;
	}

}
