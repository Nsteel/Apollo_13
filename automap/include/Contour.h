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
								void setScore(const cv::Point& vehicle, const double& yaw, const cv::Rect& roi);
								const double getScore() const;
								const edge& getContour();
								const cv::Point& getCentroid() const;
								void shiftCentroid();
								const cv::Point getCentroidGlobal(const cv::Rect& roi) const;
								const double getYaw() const;
								const cv::Point& getMid() const;
								const double getLength() const;

								friend bool operator<(const Contour& l, const Contour& r){
									return l.getScore()<r.getScore();
								}

								friend bool operator>(const Contour& l, const Contour& r){
									return l.getScore()>r.getScore();
								}

private:
								edge vectorOfPoints;
								cv::Mat map;
								cv::Point centroid;
								cv::Point midEdge;
								double yaw;
								double length;
								double score;

								void calcCentroid();
								void calcMidEdge();
								void calcLength();
								void calcDirectionToUnknown();
								int makeYaw(const int angle) const;
								double makeYaw(const double angle) const;
								int correctYawAngle(const int theta, const int increment) const;
};

#endif /* CONTOUR_H_ */
