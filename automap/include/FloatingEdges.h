/*
 * FloatingEdges.h
 *
 *  Created on: Jun 6, 2016
 *      Author: sebastian
 */

#ifndef FLOATINGEDGES_H_
#define FLOATINGEDGES_H_

#include <vector>
#include <cv.h>
#include <list>
#include "Contour.h"

typedef std::vector<Contour> contours;
typedef std::vector< std::vector<cv::Point> > contVector;

class FloatingEdges {
public:
								FloatingEdges(const double vehicleWidth, const double mapResolution);
								void getEdges(const cv::Mat& map, const cv::Point& vehicle, const double& yaw, const cv::Rect& roi);
								cv::Mat drawEdges();

private:
								double vehicleWidth;
								double mapResolution;
								cv::Mat map, th_all_map, th_wall_map, all_edges, wall_only_edges, bw_mask, black, floating_edges;
								contours edges;
								contVector detectedEdges;
								std::vector<cv::Vec4i>hierarchy;

								void imgProc();
								void detectEdges();
								bool findFloatingEdges(const cv::Point& vehicle, const double& yaw, const cv::Rect& roi);
								void sortByScore();

};

#endif /* FLOATINGEDGES_H_ */
