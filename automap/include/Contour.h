/*
 * FloatingEdge.h
 *
 *  Created on: Jun 6, 2016
 *      Author: sebastian
 */

#ifndef CONTOUR_H_
#define CONTOUR_H_

#include <vector>
#include <cv.h>

typedef std::vector<cv::Point> edge;

class Contour {
public:
								Contour(const edge &vectorOfPoints, const cv::Mat& map);
								Contour(const Contour &other);
								void initContour();
								const edge& getContour();
								const cv::Point& getCentroid() const;
								const double getYaw() const;
								const cv::Point& getMid() const;
								const double getLength() const;

private:
								edge vectorOfPoints;
								cv::Mat map;
								cv::Point centroid;
								cv::Point midEdge;
								double yaw;
								double length;

								void calcCentroid();
								void calcMidEdge();
								void calcLength();
								void calcDirectionToUnknown();
								int makeYaw(const int angle) const;
								int correctYawAngle(const int theta, const int increment) const;
};

#endif /* CONTOUR_H_ */
