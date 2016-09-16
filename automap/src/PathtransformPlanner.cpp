#include "PathtransformPlanner.h"

PathtransformPlanner::PathtransformPlanner(const nav_msgs::MapMetaData& mapInfo) : mapInfo(mapInfo){

								robotFootprint = cv::Rect();

								cv::Mat occupancyGrid = cv::Mat();
								cv::Mat distanceTransform = cv::Mat();
								cv::Mat obstacleTransform = cv::Mat();
								cv::Mat pathTransform = cv::Mat();

								initialized = false;

}

void PathtransformPlanner::setConfig(automap::ExplorationConfig& config){
								this->config = config;
								this->accuracy = config.planner_downsampling_factor;
								this->minObstDistance = config.planner_min_obstacle_distance / mapInfo.resolution * config.planner_downsampling_factor;
								this->alpha = config.planner_alpha / mapInfo.resolution * config.planner_downsampling_factor;
								this->robotFootprint = cv::Rect(0,0,config.planner_robot_width/mapInfo.resolution, config.planner_robot_length/mapInfo.resolution);
								initialized = true;
}

void PathtransformPlanner::updateTransformMatrices(const cv::Mat& occupancyGrid,
																																																			const cv::Point& robot) {
								std::string msg;

								if (initialized) {
																if (occupancyGrid.cols == 0 || occupancyGrid.rows == 0) {
																								msg = "Occupancy grid ist empty!";
																								throw PlannerException(msg);
																}
																if (robot.x >= occupancyGrid.cols || robot.y >= occupancyGrid.rows
																				|| robot.x < 0 || robot.y < 0) {
																								msg = "Robot not on Map !";
																								throw PlannerException(msg);
																}
																this->occupancyGrid = occupancyGrid.clone();
																this->robot = robot;


																// -> free space arround robot
																robotFootprint.x = robot.x - robotFootprint.width / 2;
																robotFootprint.y = robot.y - robotFootprint.height / 2;
																cv::rectangle(this->occupancyGrid, robotFootprint, cv::Scalar(PTP::FREE_CELL_COLOR),
																														CV_FILLED);

																// downsample occupancy grid by the factor accuracy -> increases performance by a huge magnitude
																cv::resize(this->occupancyGrid, this->occupancyGrid, cv::Size(),
																											accuracy, accuracy, CV_INTER_AREA);
																// AREA interpolation has a small blur effect, threshold sharpens the egdes again
																cv::threshold(this->occupancyGrid, this->occupancyGrid, PTP::UNKNOWN_CELL_COLOR-1, PTP::FREE_CELL_COLOR, CV_THRESH_TOZERO);

																this->robot.x = robot.x * accuracy;
																this->robot.y = robot.y * accuracy;


																// init transform matrices
																distanceTransform = cv::Mat::zeros(this->occupancyGrid.rows,
																																																			this->occupancyGrid.cols, CV_32F)
																																				+ std::numeric_limits<float>::max();
																obstacleTransform = cv::Mat::zeros(this->occupancyGrid.rows,
																																																			this->occupancyGrid.cols, CV_32F)
																																				+ std::numeric_limits<float>::max();
																pathTransform = cv::Mat::zeros(this->occupancyGrid.rows,
																																															this->occupancyGrid.cols, CV_32F)
																																+ std::numeric_limits<float>::max();
																calcDistanceTransform();
																calcObstacleTransform();
																calcPathTransform();

																this->robot = robot;

								} else {
																msg = "PathtransformPlanner has not been initialized yet..";
																throw PlannerException(msg);
								}
}

Path PathtransformPlanner::findPath(const cv::Point& goal) const {
								std::string msg;
								if(!initialized) {
																msg = "PathtransformPlanner has not been initialized yet..";
																throw PlannerException(msg);
								}
								if (goal == robot) {
																msg = "Destination Reached !";
																throw PlannerException(msg);
								}
								if (pathTransform.cols == 0 || pathTransform.rows == 0) {
																msg = "No path transform grid found !";
																throw PlannerException(msg);
								}
								cv::Point goalTransformed = cv::Point(goal.x * accuracy, goal.y * accuracy);

								if (goalTransformed.x >= occupancyGrid.cols || goalTransformed.y >= occupancyGrid.rows
												|| goalTransformed.x < 0 || goalTransformed.y < 0) {
																msg = "Target not on Map !";
																throw PlannerException(msg);
								}
								if (occupancyGrid.at<uchar>(goalTransformed) <= PTP::OCCUPIED_CELL_COLOR) {
																msg = "Target unreachable - Blocked !";
																throw PlannerException(msg);
								}

								std::vector<cv::Point> path;

								path.push_back(goal);

								cv::Point currentTransformed = goalTransformed;
								cv::Point lastTransformed;
								cv::Point current = goal;

								int maxSteps = pathTransform.rows+pathTransform.cols*100;

								while ( 1/accuracy<cv::norm(current-robot)) {
																maxSteps--;
																lastTransformed = currentTransformed;
																currentTransformed = findSteepestDescent(currentTransformed);
																current = cv::Point(currentTransformed.x / accuracy, currentTransformed.y / accuracy);
																path.push_back(current);
																if (lastTransformed == currentTransformed || maxSteps==0) {
																								msg = "Target unreachable !";
																								throw PlannerException(msg);
																}
								}
								if(path[path.size()-1]!=robot) {
																path.push_back(robot);
								}


								std::vector<cv::Point> pathReverse(path.size());
								for (unsigned int i = 0; i < pathReverse.size(); i++) {
																pathReverse[i] = path[path.size() - (i + 1)];
								}

								return Path(pathReverse, mapInfo.resolution);
}

bool PathtransformPlanner::isGoalSafe(const cv::Point& goal) const{
	std::string msg;
	if(!initialized) {
									msg = "PathtransformPlanner has not been initialized yet..";
									throw PlannerException(msg);
	}
	if (goal == robot) {
									msg = "Destination Reached !";
									throw PlannerException(msg);
	}
	if (pathTransform.cols == 0 || pathTransform.rows == 0) {
									msg = "No path transform grid found !";
									throw PlannerException(msg);
	}
	cv::Point goalTransformed = cv::Point(goal.x * accuracy, goal.y * accuracy);

	if (goalTransformed.x >= occupancyGrid.cols || goalTransformed.y >= occupancyGrid.rows
					|| goalTransformed.x < 0 || goalTransformed.y < 0) {
									msg = "Target not on Map !";
									throw PlannerException(msg);
	}
	if (occupancyGrid.at<uchar>(goalTransformed) <= PTP::OCCUPIED_CELL_COLOR) {
									msg = "Target unreachable - Blocked !";
									throw PlannerException(msg);
	}

	float cost = obstacleTransform.at<float>(goalTransformed);
	return cost*1.3>=minObstDistance;

}

cv::Point PathtransformPlanner::findSteepestDescent(
								const cv::Point& current) const {
								int height = distanceTransform.rows;
								int width = distanceTransform.cols;

								std::vector<cv::Point> window;

								//mid right
								if (current.x + 1 < width) {
																window.push_back(cv::Point(current.x + 1, current.y));

								}
								//bottom right
								if (current.x + 1 < width && current.y + 1 < height) {
																window.push_back(cv::Point(current.x + 1, current.y + 1));

								}
								//bottom mid
								if (current.y + 1 < height) {
																window.push_back(cv::Point(current.x, current.y + 1));

								}
								//bottom left
								if (current.y + 1 < height && current.x - 1 >= 0) {
																window.push_back(cv::Point(current.x - 1, current.y + 1));

								}
								//mid left
								if (current.x - 1 >= 0) {
																window.push_back(cv::Point(current.x - 1, current.y));

								}
								//top left
								if (current.y - 1 >= 0 && current.x - 1 >= 0) {
																window.push_back(cv::Point(current.x - 1, current.y - 1));

								}
								//top mid
								if (current.y - 1 >= 0) {
																window.push_back(cv::Point(current.x, current.y - 1));

								}
								//top right
								if (current.y - 1 >= 0 && current.x + 1 < width) {
																window.push_back(cv::Point(current.x + 1, current.y - 1));

								}
								cv::Point minVal = current;

								for (auto c : window) {
																if (pathTransform.at<float>(c) <= pathTransform.at<float>(minVal)) {
																								minVal = c;
																}
								}

								return minVal;

}

void PathtransformPlanner::calcDistanceTransform() {

								distanceTransform.at<float>(robot) = 0;
								int currentVal;
								cv::Point currentPos;

								cv::Mat temp = cv::Mat::zeros(distanceTransform.rows,
																																						distanceTransform.cols,
																																						CV_32F);

								float error = cv::norm(temp, distanceTransform, CV_L2);

								while (error != 0) {
																temp = distanceTransform.clone();
																for (int r = 0; r < occupancyGrid.rows; r++) {

																								for (int c = 0; c < occupancyGrid.cols; c++) {
																																currentPos = cv::Point(c, r);
																																currentVal = occupancyGrid.at<uchar>(currentPos);

																																if (currentVal > PTP::UNKNOWN_CELL_COLOR) {

																																								distanceTransform.at<float>(currentPos) = scanWindowFwd(
																																																distanceTransform, currentPos);
																																}

																								}
																}

																for (int r = occupancyGrid.rows - 1; r >= 0; r--) {

																								for (int c = occupancyGrid.cols - 1; c >= 0; c--) {
																																currentPos = cv::Point(c, r);
																																currentVal = occupancyGrid.at<uchar>(currentPos);
																																if (currentVal > PTP::UNKNOWN_CELL_COLOR) {

																																								distanceTransform.at<float>(currentPos) = scanWindowBwd(
																																																distanceTransform, currentPos);
																																}

																								}
																}

																error = cv::norm(temp, distanceTransform, CV_L2);

								}

}

void PathtransformPlanner::calcObstacleTransform() {

								int currentVal;
								cv::Point currentPos;

								for (int r = 0; r < occupancyGrid.rows; r++) {

																for (int c = 0; c < occupancyGrid.cols; c++) {
																								currentPos = cv::Point(c, r);
																								currentVal = occupancyGrid.at<uchar>(currentPos);

																								if (currentVal <= PTP::UNKNOWN_CELL_COLOR) {
																																obstacleTransform.at<float>(currentPos) = 0;
																								}

																}
								}

								cv::Mat temp = cv::Mat::zeros(obstacleTransform.rows,
																																						obstacleTransform.cols,
																																						CV_32F);

								float error = cv::norm(temp, obstacleTransform, CV_L2);

								while (error != 0) {
																temp = obstacleTransform.clone();
																for (int r = 0; r < occupancyGrid.rows; r++) {

																								for (int c = 0; c < occupancyGrid.cols; c++) {
																																currentPos = cv::Point(c, r);
																																currentVal = occupancyGrid.at<uchar>(currentPos);

																																if (currentVal > PTP::UNKNOWN_CELL_COLOR) {

																																								obstacleTransform.at<float>(currentPos) = scanWindowFwd(
																																																obstacleTransform, currentPos);
																																}

																								}
																}

																for (int r = occupancyGrid.rows - 1; r >= 0; r--) {

																								for (int c = occupancyGrid.cols - 1; c >= 0; c--) {
																																currentPos = cv::Point(c, r);
																																currentVal = occupancyGrid.at<uchar>(currentPos);
																																if (currentVal > PTP::UNKNOWN_CELL_COLOR) {

																																								obstacleTransform.at<float>(currentPos) = scanWindowBwd(
																																																obstacleTransform, currentPos);
																																}

																								}
																}

																error = cv::norm(temp, obstacleTransform, CV_L2);

								}

}

void PathtransformPlanner::calcPathTransform() {

								int currentVal;
								cv::Point currentPos;
								double cost;
								double costFun;

								for (int r = 0; r < occupancyGrid.rows; r++) {

																for (int c = 0; c < occupancyGrid.cols; c++) {
																								currentPos = cv::Point(c, r);
																								currentVal = occupancyGrid.at<uchar>(currentPos);

																								if (currentVal > PTP::UNKNOWN_CELL_COLOR) {

																																cost = obstacleTransform.at<float>(currentPos);

																																if (cost <= minObstDistance) {
																																								costFun = alpha
																																																		* pow(
																																																minObstDistance
																																																- obstacleTransform.at<float>(
																																																								currentPos), 3);
																																} else {
																																								costFun = 0;
																																}

																																pathTransform.at<float>(currentPos) =
																																								distanceTransform.at<float>(currentPos) + costFun;

																								}


																								if (currentPos == robot) {
																																pathTransform.at<float>(currentPos) = 0;
																								}


																}
								}

}

const float PathtransformPlanner::scanWindowFwd(const cv::Mat& transform,
																																																const cv::Point& current) const {
								int width = transform.cols;

								std::vector<float> window;

								//central point
								window.push_back(transform.at<float>(current));
								//mid left
								if (current.x - 1 >= 0) {

																window.push_back(1 + transform.at<float>(current.y, current.x - 1));

								}
								//top left
								if (current.y - 1 >= 0 && current.x - 1 >= 0) {

																window.push_back(
																								1.41 + transform.at<float>(current.y - 1, current.x - 1));

								}
								//top mid
								if (current.y - 1 >= 0) {

																window.push_back(1 + transform.at<float>(current.y - 1, current.x));

								}
								//top right
								if (current.y - 1 >= 0 && current.x + 1 < width) {

																window.push_back(
																								1.41 + transform.at<float>(current.y - 1, current.x + 1));

								}

								float minVal = window[0];
								for (auto current : window) {
																if (current < minVal) {
																								minVal = current;
																}
								}
								return minVal;
}
const float PathtransformPlanner::scanWindowBwd(const cv::Mat& transform,
																																																const cv::Point& current) const {
								int height = transform.rows;
								int width = transform.cols;

								std::vector<float> window;

								//central point
								window.push_back(transform.at<float>(current));
								//mid right
								if (current.x + 1 < width) {
																window.push_back(1 + transform.at<float>(current.y, current.x + 1));

								}
								//bottom right
								if (current.y + 1 < height && current.x + 1 < width) {
																window.push_back(
																								1.41 + transform.at<float>(current.y + 1, current.x + 1));

								}
								//bottom mid
								if (current.y + 1 < height) {
																window.push_back(1 + transform.at<float>(current.y + 1, current.x));

								}
								//bottom left
								if (current.y + 1 < height && current.x - 1 >= 0) {
																window.push_back(
																								1.41 + transform.at<float>(current.y + 1, current.x - 1));

								}
								float minVal = window[0];
								for (auto current : window) {
																if (current < minVal) {
																								minVal = current;
																}
								}
								return minVal;

}

double PathtransformPlanner::getAccuracy() const {
								return accuracy;
}

void PathtransformPlanner::setAccuracy(double accuracy) {
								this->accuracy = accuracy;
}

const cv::Mat& PathtransformPlanner::getDistanceTransform() const {
								return distanceTransform;
}

double PathtransformPlanner::getAlpha() const {
								return alpha;
}

double PathtransformPlanner::getMinObstDistance() const {
								return minObstDistance;
}

const cv::Mat& PathtransformPlanner::getObstacleTransform() const {
								return obstacleTransform;
}

const cv::Mat& PathtransformPlanner::getOccupancyGrid() const {
								return occupancyGrid;
}

const cv::Mat& PathtransformPlanner::getPathTransform() const {
								return pathTransform;
}

const cv::Rect PathtransformPlanner::getRobotFootprint() const {
								return robotFootprint;
}

const nav_msgs::MapMetaData& PathtransformPlanner::getMapInfo() const {
								return mapInfo;
}
