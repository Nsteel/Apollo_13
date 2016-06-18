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

const contours& FloatingEdges::getEdges(const cv::Mat& map, const cv::Point& vehicle, const double& yaw, const cv::Rect& roi){
								if(map.rows>0 && map.cols>0) {
																this->map = map.clone();
																imgProc();
																detectEdges();
																if(findFloatingEdges(vehicle, yaw, roi)) {
																								sortByScore();
																								/*
																								   Contour best = edges[0];
																								   edges = contours();
																								   edges.push_back(best);
																								   detectedEdges = contVector();
																								   detectedEdges.push_back(best.getContour());
																								 */
																								detectedEdges = contVector();
																								for(auto current : edges) {
																									//current.shiftCentroid();
																																detectedEdges.push_back(current.getContour());
																								}
																}

								}
								return edges;
}

void FloatingEdges::imgProc(){
								th_all_map = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
								th_wall_map = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
								all_edges = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
								wall_only_edges = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
								bw_mask = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
								floating_edges = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));
								black = cv::Mat(map.size(), CV_8UC1, cv::Scalar(0));

								cv::threshold(map, th_all_map, 208, 255, CV_THRESH_BINARY);
								cv::threshold(map, th_wall_map, 204, 255, CV_THRESH_BINARY);
								// remove "micro map holes"
								cv::blur(th_all_map, th_all_map, cv::Size(5,5));

}

void FloatingEdges::detectEdges(){
								int lTh = 204;
								// detect obstacles and floating edges
								cv::Canny(th_all_map, all_edges, lTh, 300, 3);
								// detect obstacles only
								cv::Canny(th_wall_map, wall_only_edges, lTh, 300, 3);

								// inflate detected obstacles to ensure clean removal
								cv::dilate(wall_only_edges, bw_mask, cv::Mat());
								for(int i = 1; i<=3; i++) {
																cv::dilate(bw_mask, bw_mask, cv::Mat());
								}
								// create a mask where the obstacles are true and free space is false
								cv::bitwise_not(bw_mask, bw_mask);

								// remove obstacles so only floating edges remain
								cv::bitwise_or(all_edges, black, floating_edges, bw_mask);
}

bool FloatingEdges::findFloatingEdges(const cv::Point& vehicle, const double& yaw, const cv::Rect& roi){
								detectedEdges = contVector();
								hierarchy = std::vector<cv::Vec4i>();

								//inflate edges to merge micro edges and small gaps
								cv::dilate(floating_edges, floating_edges, cv::Mat());

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
																float contrast = 0;
																float invContrast = 0;
																for(int i = 0; i<detectedEdges.size(); i++) {
																								if(detectedEdges.size()!=1) {
																																contrast = ((float)detectedEdges.size()-1-(float)i)/((float)detectedEdges.size()-1);
																																invContrast = 1.0-contrast;
																								}else{
																																contrast = 1;
																																invContrast = 0;
																								}

																								cv::drawContours(out, detectedEdges, i, cv::Scalar(0, ((float)255)*contrast, ((float)255)*invContrast));
																}
																for(auto current : edges) {
																								cv::circle(out, current.getCentroid(), 2, cv::Scalar(255, 0, 0), -1, 8, 0);

																								std::ostringstream strs;
																								strs << current.getScore();
																								std::string str = strs.str();
																								std::string ostring = str;
																								cv::putText(out, ostring, current.getCentroid(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0) );


																								double length = 25;
																								cv::Point P2;

																								P2.x =  (int)round(current.getCentroid().x + length * cv::cos((-current.getYaw()) * CV_PI / 180.0));
																								P2.y =  (int)round(current.getCentroid().y + length * cv::sin((-current.getYaw()) * CV_PI / 180.0));

																								cv::line(out, current.getCentroid(), P2, cv::Scalar(255, 0, 0));
																}
																return out;
								}

}
