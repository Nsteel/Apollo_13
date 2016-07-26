#ifndef EXPLORATION_PLANNER_CPP
#define EXPLORATION_PLANNER_CPP

#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include "PathtransformPlanner.h"
#include "FrontierDetector.h"
#include "Frontier.h"
#include "Path.h"
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <list>

class ExplorationPlanner {
public:
        ExplorationPlanner(const PathtransformPlanner * planner, const int windowSize_x=600,
                           const int windowSize_y=600);
        bool findBestPlanSimple(const cv::Mat& occupancyGrid, const cv::Point& robotGridPos, const double robotYaw);
        nav_msgs::Path getBestPlan(std_msgs::Header& h);
        cv::Mat drawFrontiers();
        cv::Mat drawWindow();

private:
        const PathtransformPlanner* planner;
        nav_msgs::MapMetaData mapInfo;
        cv::Rect robotFootprint;
        cv::Rect rollingWindow;
        int occupiedColor;
        int unknownColor;
        int freeColor;

        FrontierDetector localDetector;
        FrontierDetector globalDetector;

        cv::Mat occupancyGrid;
        cv::Mat window;
        cv::Point robotGridPos;
        double robotYaw;
        std::list<Frontier>validFrontiers;
        std::list<Frontier>frontierStack;
        std::list<Path>pathStack;

        bool extractValidFrontiersLocal();
        bool extractValidFrontiersGlobal();
        void scoreValidFrontiersSimple();
        void extractBestFrontier();

        frontierPoints localPtsToGlobal(const frontierPoints& points) const;
        cv::Point2f gridPtToWorld(const cv::Point& point) const;
        double calcFrontierLength(const frontierPoints& points) const;
        double calcScoreSimple(const Frontier& f) const;
        void calcScoreNBV(Frontier& f) const;
        double isInView(const std::vector<cv::Point>& pts, const cv::Point& start, double yaw, double FOV, double maxDistance) const;
        double calcInformationGain(const cv::Point& start, double yaw, double FOV, double angularResolution, double maxDistance) const;
        bool checkProximitryToFrontier(const Frontier& f) const;
        cv::Point calcFrontierCentroid(const frontierPoints& points) const;
        cv::Point shiftCentroid(const cv::Point& point, const double yaw) const;
        double calcFrontierColorGradient(const frontierPoints& points) const;

        int makeYaw(const int angle) const;
        double makeYaw(const double angle) const;
        //int correctYawAngle(const int theta, const int increment) const;
        double correctYawAngle(const double theta, const double increment) const;

};

#endif
