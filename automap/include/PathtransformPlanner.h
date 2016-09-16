/*
 * PathtransformPlanner.h
 *
 *  Created on: Jul 13, 2016
 *      Author: sebastian
 */

#ifndef PATHTRANSFORMPLANNER_H_
#define PATHTRANSFORMPLANNER_H_

#include <cv.h>
#include "Path.h"
#include "PlannerException.h"
#include <automap/ExplorationConfig.h>

namespace PTP {
const static int OCCUPIED_CELL_COLOR = 0;
const static int UNKNOWN_CELL_COLOR = 205;
const static int FREE_CELL_COLOR = 255;
};

class PathtransformPlanner {
public:
								PathtransformPlanner(const nav_msgs::MapMetaData& mapInfo);

								// getters and setters
								void setConfig(automap::ExplorationConfig& config);
								const cv::Rect getRobotFootprint() const;
								void setRobotFootprint(const cv::Rect& robotFootprint);
								const nav_msgs::MapMetaData& getMapInfo() const;
								double getAccuracy() const;
								void setAccuracy(double accuracy);
								double getAlpha() const;
								void setAlpha(double alpha);
								double getMinObstDistance() const;
								void setMinObstDistance(double minObstDistance);
								void setPathTransform(const cv::Mat& pathTransform);
								void setOccupancyGrid(const cv::Mat& occupancyGrid);
								const cv::Mat& getOccupancyGrid() const;
								const cv::Mat& getDistanceTransform() const;
								const cv::Mat& getObstacleTransform() const;
								const cv::Mat& getPathTransform() const;

								// main methods
								void updateTransformMatrices(const cv::Mat& occupancyGrid, const cv::Point& robot);
								Path findPath(const cv::Point& goal) const;
								bool isGoalSafe(const cv::Point& goal) const;


private:
								automap::ExplorationConfig config;
								nav_msgs::MapMetaData mapInfo;
								double accuracy;
								double minObstDistance;
								double alpha;
								cv::Rect robotFootprint;

								bool initialized;

								cv::Mat occupancyGrid;
								cv::Mat distanceTransform;
								cv::Mat obstacleTransform;
								cv::Mat pathTransform;

								cv::Point robot;

								void calcDistanceTransform();
								void calcObstacleTransform();
								void calcPathTransform();

								cv::Point findSteepestDescent(const cv::Point& current) const;

								const float scanWindowFwd(const cv::Mat& transform, const cv::Point& current) const;
								const float scanWindowBwd(const cv::Mat& transform, const cv::Point& current) const;
};

#endif /* PATHTRANSFORMPLANNER_H_ */
