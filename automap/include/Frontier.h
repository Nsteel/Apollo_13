#ifndef FRONTIER_CPP
#define FRONTIER_CPP

#include <vector>
#include <cv.h>
#include "Path.h"

typedef std::vector<cv::Point> frontierPoints;

class Frontier {
public:
        Frontier();
        Frontier(const Frontier &other);
        Frontier(const frontierPoints &points, const cv::Point& centroidGrid, const cv::Point2f& centroidWorld, const double length, const double frontierYaw);

        const frontierPoints& getPoints() const;
        const cv::Point& getCentroidGrid() const;
        const cv::Point2f& getCentroidWorld() const;
        void setCentroidGrid(const cv::Point& p);
        const double getFrontierYaw() const;
        void setFrontierYaw(const double frontierYaw);
        const Path& getPath() const;
        void setPath(const Path& path);
        const double getScore() const;
        void setScore(const double score);
        const frontierPoints& getFrontierPoints() const;
        const double getLength() const;

        friend bool operator<(const Frontier &l, const Frontier &r){
                //std::cout<<l.getScore()<<" < "<<r.getScore()<<std::endl;
                return l.getScore()<r.getScore();
        }

        friend bool operator>(const Frontier &l, const Frontier &r){
                //std::cout<<l.getScore()<<" > "<<r.getScore()<<std::endl;
                return l.getScore()>r.getScore();
        }

private:
        frontierPoints points;
        cv::Point centroidGrid;
        cv::Point2f centroidWorld;
        Path path;
        double length;
        double frontierYaw;
        double score;


};

#endif
