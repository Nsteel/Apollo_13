/*
 * FloatingEdges.cpp
 *
 *  Created on: Jun 6, 2016
 *      Author: sebastian
 */

#include "FloatingEdges.h"

FloatingEdges::FloatingEdges(const double vehicleWidth, const double mapResolution) : vehicleWidth(vehicleWidth), mapResolution(mapResolution) {
								edges = contours();
								detectedEdges = contVector();
								hierarchy = std::vector<cv::Vec4i>();

}

void FloatingEdges::getEdges(const cv::Mat& map, const cv::Point& vehicle, const double& yaw, const cv::Rect& roi){
								if(map.rows>0 && map.cols>0) {
																this->map = map;
																imgProc();
																detectEdges();
																if(findFloatingEdges(vehicle, yaw, roi)){
																	sortByScore();
																	Contour best = edges[0];
																	edges = contours();
																	edges.push_back(best);
																	detectedEdges = contVector();
																	detectedEdges.push_back(best.getContour());
																}

								}
}

void FloatingEdges::imgProc(){
								cv::threshold(map, th_all_map, 208, 255, CV_THRESH_BINARY);
								cv::threshold(map, th_wall_map, 204, 255, CV_THRESH_BINARY);
								// remove "micro map holes"
								cv::blur(th_all_map, th_all_map, cv::Size(5,5));
								black = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
}

void FloatingEdges::detectEdges(){
								int lTh = 204;
								cv::Canny(th_all_map, all_edges, lTh, 300, 3);
								cv::Canny(th_wall_map, wall_only_edges, lTh, 300, 3);
								cv::dilate(wall_only_edges, bw_mask, cv::Mat());

								for(int i = 1; i<=1; i++) {
																cv::dilate(bw_mask, bw_mask, cv::Mat());
								}

								cv::bitwise_not(bw_mask, bw_mask);
								cv::bitwise_or(all_edges, black, floating_edges, bw_mask);
}

bool FloatingEdges::findFloatingEdges(const cv::Point& vehicle, const double& yaw, const cv::Rect& roi){
								detectedEdges = contVector();
								hierarchy = std::vector<cv::Vec4i>();

								findContours(floating_edges, detectedEdges, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
								if(detectedEdges.size()==0) {
																return false;
								}else{
																edges = contours();
																int c = 0;
																for(auto current : detectedEdges) {
																								Contour cont(current, map);
																								if(cont.getLength()*mapResolution >= vehicleWidth) {
																																cont.initContour();
																																cont.setScore(vehicle, yaw, roi);
																																edges.push_back(cont);
																								}

																}
																detectedEdges = contVector();
																for(auto current : edges) {
																								detectedEdges.push_back(current.getContour());
																}

								}
								return true;
}

void FloatingEdges::sortByScore() {
								std::list<Contour> sortedContours(edges.begin(), edges.end());
								sortedContours.sort();
								sortedContours.reverse();
								edges = contours(sortedContours.begin(), sortedContours.end());
}

cv::Mat FloatingEdges::drawEdges(){
								cv::Mat out = map.clone();
								cv::cvtColor(out, out, CV_GRAY2RGB);;
								if(detectedEdges.size()==0) {
																return out;
								}else{
																/*
																   for(int idx = 0; idx >=0; idx = hierarchy[idx][0]) {
																        cv::drawContours(out, detectedEdges, idx, cv::Scalar(0, 0, 255), CV_FILLED, 8, hierarchy);
																   }
																 */
																for(int i = 0; i<detectedEdges.size(); i++) {
																								cv::drawContours(out, detectedEdges, i, cv::Scalar(0, 0, 255));
																}
																for(auto current : edges) {
																								//cv::circle(out, current.getCenter(), 3, cv::Scalar(0, 255, 0), -1, 8, 0);
																								std::ostringstream strs;
																								strs << current.getScore();
																								std::string str = strs.str();
																								std::string ostring = str;
																								cv::putText(out, ostring, current.getCentroid(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0) );

																								double length = 25;
																								cv::Point P2;

																								P2.x =  (int)round(current.getCentroid().x + length * cv::cos((-current.getYaw()) * CV_PI / 180.0));
																								P2.y =  (int)round(current.getCentroid().y + length * cv::sin((-current.getYaw()) * CV_PI / 180.0));

																								cv::line(out, current.getCentroid(), P2, cv::Scalar(0, 255, 0));
																}
																return out;
								}

}
