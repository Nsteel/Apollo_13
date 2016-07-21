/*
 * Path.h
 *
 *  Created on: Jul 13, 2016
 *      Author: sebastian
 */

#ifndef PATH_H_
#define PATH_H_

#include <cv.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

class Path {
public:
	Path();
	Path(const Path& other);
	Path(std::vector<cv::Point>& path, double mapResolution);

	const cv::Point& getGridStart() const;
	const cv::Point2f& getWorldStart() const;
	const cv::Point& getGridEnd() const;
	const cv::Point2f& getWorldEnd() const;

	const double getGridLength() const;
	const double getWorldLength() const;

	const std::vector<cv::Point>& getGridPath() const;
	const std::vector<cv::Point2f>& getWorldPath() const;

private:
	cv::Point gridStart;
	cv::Point gridEnd;
	cv::Point2f worldStart;
	cv::Point2f worldEnd;
	double gridLength;
	double worldLength;

	std::vector<cv::Point> gridPath;
	std::vector<cv::Point2f> worldPath;

};

#endif /* PATH_H_ */
