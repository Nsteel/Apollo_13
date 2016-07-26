#include "Frontier.h"

Frontier::Frontier(){
        centroidGrid = cv::Point();
        centroidWorld = cv::Point2f();
        frontierYaw = 0;
        path = Path();
        score = 0;
        points = frontierPoints();
};

Frontier::Frontier(const Frontier& other) : points(other.points), centroidGrid(other.centroidGrid), centroidWorld(other.centroidWorld), length(other.length),
        path(other.path), score(other.score), frontierYaw(other.frontierYaw){
        //std::cout<<"Copy: "<<centroidGrid<<" score: "<<score<<std::endl;
};

Frontier::Frontier(const frontierPoints& points, const cv::Point& centroidGrid, const cv::Point2f& centroidWorld, const double length,  const double frontierYaw) :
        points(points), centroidGrid(centroidGrid), centroidWorld(centroidWorld), length(length), frontierYaw(frontierYaw){
        path = Path();
        score = 0;
};

const frontierPoints& Frontier::getPoints() const {
        return points;
}

const cv::Point& Frontier::getCentroidGrid() const {
        return centroidGrid;
}
const cv::Point2f& Frontier::getCentroidWorld() const {
        return centroidWorld;
}
void Frontier::setCentroidGrid(const cv::Point& p){
        this->centroidGrid = p;
}
const double Frontier::getFrontierYaw() const {
        return frontierYaw;
}
void Frontier::setFrontierYaw(const double frontierYaw){
        this->frontierYaw = frontierYaw;
}
const double Frontier::getLength() const {
        return length;
}
const Path& Frontier::getPath() const {
        return path;
}
void Frontier::setPath(const Path& path){
        this->path = path;
}
const double Frontier::getScore() const {
        return score;
}
void Frontier::setScore(const double score){
        this->score = score;
}
const frontierPoints& Frontier::getFrontierPoints() const {
        return points;
}
