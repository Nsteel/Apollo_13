/*
 * Path.cpp
 *
 *  Created on: Jul 13, 2016
 *      Author: sebastian
 */

#include "Path.h"

Path::Path(){
};

Path::Path(const Path& other) :
								gridStart(other.gridStart), gridEnd(other.gridEnd), worldStart(
																other.worldStart), worldEnd(other.worldEnd), gridLength(
																other.gridLength), worldLength(other.worldLength), gridPath(
																other.gridPath), worldPath(other.worldPath) {

}

Path::Path(std::vector<cv::Point>& path, double mapResolution) :
								gridPath(path) {
								worldPath = std::vector<cv::Point2f>();
								gridLength = 0;

								for (int i = 0; i < path.size() - 1; i++) {
																gridLength += cv::norm(path[i] - path[i + 1]);
																worldPath.push_back(
																								cv::Point2f(path[i].x * mapResolution,
																																				path[i].y * mapResolution));
								}

								gridStart = path[0];
								gridEnd = path[path.size() - 1];

								worldLength = gridLength * mapResolution;
								worldStart = worldPath[0];
								worldEnd = worldPath[worldPath.size() - 1];

}

const cv::Point& Path::getGridStart() const {
								return gridStart;
}
const cv::Point2f& Path::getWorldStart() const {
								return worldStart;
}
const cv::Point& Path::getGridEnd() const {
								return gridEnd;
}
const cv::Point2f& Path::getWorldEnd() const {
								return worldEnd;
}
const double Path::getGridLength() const {
								return gridLength;
}
const double Path::getWorldLength() const {
								return worldLength;
}
const std::vector<cv::Point>& Path::getGridPath() const {
								return gridPath;
}
const std::vector<cv::Point2f>& Path::getWorldPath() const {
								return worldPath;
}
