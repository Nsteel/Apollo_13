#include "ExplorationPlanner.h"

ExplorationPlanner::ExplorationPlanner(const PathtransformPlanner * planner) : planner(planner){
        rollingWindow = cv::Rect();
        robotFootprint = cv::Rect();
        mapInfo = planner->getMapInfo();
        unknownColor = PTP::UNKNOWN_CELL_COLOR;
        occupiedColor = PTP::UNKNOWN_CELL_COLOR;
        freeColor = PTP::FREE_CELL_COLOR;

        localDetector = FrontierDetector(freeColor, unknownColor, occupiedColor);
        globalDetector = FrontierDetector(freeColor, unknownColor, occupiedColor);
}

void ExplorationPlanner::setConfig(automap::ExplorationConfig& config){
        this->config = config;
        this->rollingWindow = cv::Rect(0,0,config.exploration_rolling_window_width/mapInfo.resolution, config.exploration_rolling_window_height/mapInfo.resolution);
        this->robotFootprint = planner->getRobotFootprint();
        localDetector.setConfig(config);
        globalDetector.setConfig(config);

}

bool ExplorationPlanner::findBestPlan(const cv::Mat& occupancyGrid, const cv::Point& robotGridPos, const double robotYaw, bool useNBV){

        this->robotYaw = robotYaw/CV_PI*180.0;
        this->occupancyGrid = occupancyGrid;
        this->robotGridPos = robotGridPos;
        validFrontiers = std::list<Frontier>();
        frontierStack = std::list<Frontier>();
        std::cout<<"trying local ..."<<std::endl;
        if(!extractValidFrontiersLocal(useNBV)) {
                std::cout<<"Local didn't work, trying global ..."<<std::endl;
                if(!extractValidFrontiersGlobal(useNBV)) {
                        std::cout<<"Global didn't work, exit !"<<std::endl;
                        return false;
                }
        }

        return true;

}

nav_msgs::Path ExplorationPlanner::getBestPlan(std_msgs::Header& h){
        Frontier best = frontierStack.front();
        std::cout<<"Best frontier score: "<<best.getScore()<<std::endl;
        frontierStack.pop_front();
        double yawGoalRad = best.getFrontierYaw()*CV_PI/180.0;
        double yawRobotRad = robotYaw*CV_PI/180.0;

        nav_msgs::Path path;
        path.header = h;
        std::vector<geometry_msgs::PoseStamped> pVector;

        std::vector<cv::Point> gridPath = best.getPath().getGridPath();
        for(int i = 0; i<gridPath.size(); i++) {
                geometry_msgs::PoseStamped new_goal;

                new_goal.header = h;
                tf::Quaternion goal_quat = tf::createQuaternionFromYaw(0);

                if(i == 0) {
                        goal_quat = tf::createQuaternionFromYaw(yawRobotRad);
                }
                if(i == gridPath.size()-1) {
                        goal_quat = tf::createQuaternionFromYaw(yawGoalRad);
                }
                cv::Point2f map_goal;
                map_goal.x = gridPath[i].x*mapInfo.resolution + mapInfo.origin.position.x;
                map_goal.y = -(gridPath[i].y*mapInfo.resolution + mapInfo.origin.position.y);
                new_goal.pose.position.x = map_goal.x;
                new_goal.pose.position.y = map_goal.y;

                new_goal.pose.orientation.x = goal_quat.x();
                new_goal.pose.orientation.y = goal_quat.y();
                new_goal.pose.orientation.z = goal_quat.z();
                new_goal.pose.orientation.w = goal_quat.w();
                pVector.push_back(new_goal);
        }
        path.poses = pVector;
        return path;
}

bool ExplorationPlanner::extractValidFrontiersLocal(bool useNBV){
        //std::cout<<"local:1 "<<std::endl;
        cv::Rect baseWindow(0, 0, occupancyGrid.rows, occupancyGrid.cols);
        rollingWindow.x = robotGridPos.x-rollingWindow.width/2;
        rollingWindow.y = robotGridPos.y-rollingWindow.height/2;
        rollingWindow = rollingWindow & baseWindow;

        window = occupancyGrid(rollingWindow).clone();
        //std::cout<<"local:2 "<<std::endl;
        //1.) find all frontiers
        validFrontiers = std::list<Frontier>();
        localDetector.processFrontiers(window);
        contours found = localDetector.getFrontiers();
        //std::cout<<"local:3 "<<std::endl;
        if(found.size()==0) {
                std::cout<<"Local-Detected-Frontiers: "<<found.size()<<std::endl;
                return false;
        }

        //2.) generate passable frontier objects only
        std::vector<Frontier>passableFrontiers;

        //std::cout<<robotFootprint<<std::endl;
        double minL = std::sqrt(robotFootprint.height*robotFootprint.height + robotFootprint.width*robotFootprint.width)*mapInfo.resolution*config.frontier_tolerance;
        //std::cout<<"local:4 "<<std::endl;
        for(auto current : found) {
                frontierPoints globalFp = localPtsToGlobal(current);
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(globalFp, center, radius);
                double length = radius*2*mapInfo.resolution;
                //std::cout<<"local:5 "<<std::endl;
                if(length>minL) {
                        cv::Point centroidGlobal = calcFrontierCentroid(globalFp);
                        //std::cout<<"local:6 "<<std::endl;
                        double frontierYaw = calcFrontierColorGradient(globalFp);
                        //std::cout<<"local:7 "<<std::endl;
                        //if(!useNBV) {
                        // this prevents centroids in unknown but also not very smart
                        centroidGlobal = shiftCentroid(centroidGlobal, frontierYaw);
                        //std::cout<<"local:8 "<<std::endl;
                        //}
                        cv::Point2f centroidWorld = gridPtToWorld(centroidGlobal);
                        passableFrontiers.push_back(Frontier(globalFp, centroidGlobal, centroidWorld, length, frontierYaw));
                        //std::cout<<"local:9 "<<std::endl;
                }
        }
        if(passableFrontiers.size()==0) {
                std::cout<<"Local-Passable-Frontiers: "<<passableFrontiers.size()<<std::endl;
                return false;
        }

        //3.) rember reachable frontiers only
        for(auto current : passableFrontiers) {
                if(!checkProximitryToFrontier(current)) {
                  //std::cout<<"local:10 "<<std::endl;
                        try{
                                if(!useNBV) {
                                        Path p = planner->findPath(current.getCentroidGrid());
                                        current.setPath(p);
                                        current.setScore(calcScoreSimple(current));
                                }
                                else{
                                        calcScoreNBV(current);
                                        //std::cout<<"local:11 "<<std::endl;
                                }

                                if(current.getScore()>=0) {
                                  //std::cout<<"local:12 "<<std::endl;
                                        validFrontiers.push_back(current);
                                        //
                                        frontierStack.push_back(current);
                                }else{
                                  std::cout<<"Discarded score: "<<current.getScore()<<" Pos: "<<current.getCentroidGrid()<<std::endl;
                                }

                        }catch(std::exception& e) {
                          //std::cout<<"local:13 "<<std::endl;
                                std::cout<<e.what()<<std::endl;
                        }
                }

        }
        if(validFrontiers.size()==0) {
                std::cout<<"Local-Valid-Frontiers: "<<validFrontiers.size()<<std::endl;
                return false;
        }
        //
        frontierStack.sort();
        frontierStack.reverse();
        //std::cout<<"local:14 "<<std::endl;
        //
        validFrontiers.sort();
        validFrontiers.reverse();
        //std::cout<<"local:15 "<<std::endl;

        return true;

}

bool ExplorationPlanner::extractValidFrontiersGlobal(bool useNBV){
        //1.) find all frontiers
        validFrontiers = std::list<Frontier>();
        globalDetector.processFrontiers(occupancyGrid);
        contours found = globalDetector.getFrontiers();
        if(found.size()==0) {
                std::cout<<"Global-Detected-Frontiers: "<<found.size()<<std::endl;
                return false;
        }

        //2.) generate passable frontier objects only
        std::vector<Frontier>passableFrontiers;
        double minL = std::sqrt(robotFootprint.height*robotFootprint.height + robotFootprint.width*robotFootprint.width)*mapInfo.resolution*config.frontier_tolerance;
        for(auto current : found) {
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(current, center, radius);
                double length = radius*2*mapInfo.resolution*config.frontier_tolerance;
                if(length>minL) {
                        cv::Point centroidGlobal = calcFrontierCentroid(current);
                        double frontierYaw = calcFrontierColorGradient(current);
                        //if(!useNBV) {
                        // this prevents centroids in unknown but also not very smart
                        centroidGlobal = shiftCentroid(centroidGlobal, frontierYaw);
                        //}
                        cv::Point2f centroidWorld = gridPtToWorld(centroidGlobal);
                        passableFrontiers.push_back(Frontier(current, centroidGlobal, centroidWorld, length, frontierYaw));
                }
        }
        if(passableFrontiers.size()==0) {
                std::cout<<"Global-Passable-Frontiers: "<<passableFrontiers.size()<<std::endl;
                return false;
        }

        //3.) rember reachable frontiers only
        for(auto current : passableFrontiers) {
                if(!checkProximitryToFrontier(current)) {
                        try{
                                if(!useNBV) {
                                        Path p = planner->findPath(current.getCentroidGrid());
                                        current.setPath(p);
                                        current.setScore(calcScoreSimple(current));

                                }else{
                                        calcScoreNBV(current);

                                }

                                if(current.getScore()>=0) {
                                        validFrontiers.push_back(current);
                                        //
                                        frontierStack.push_back(current);
                                }else{
                                  std::cout<<"Discarded score: "<<current.getScore()<<" Pos: "<<current.getCentroidGrid()<<std::endl;
                                }


                        }catch(std::exception& e) {
                                std::cout<<e.what()<<std::endl;
                        }
                }

        }
        if(validFrontiers.size()==0) {
                std::cout<<"Global-Valid-Frontiers: "<<validFrontiers.size()<<std::endl;
                return false;
        }
        //
        frontierStack.sort();
        frontierStack.reverse();
        //
        validFrontiers.sort();
        validFrontiers.reverse();

        return true;
}
double ExplorationPlanner::calcScoreSimple(const Frontier& f) const {

        double score = 0;
        double distance = f.getPath().getWorldLength();
        if(distance<=config.planner_robot_length*2){
          return -40;
        }
        double dTh = f.getFrontierYaw()-robotYaw;
        if(dTh<-180.0) {
                dTh += 360.0;
        }else if(dTh>180.0) {
                dTh -= 360.0;
        }
        //double dFun = std::pow(config.exploration_score_simple_dist_base*(1/(1+distance)), config.exploration_score_simple_dist_exp);
        //double yFun = std::pow(config.exploration_score_simple_angle_base*((180.0-std::fabs(dTh))/180.0), config.exploration_score_simple_angle_exp);
        //score =  dFun * yFun;
        score = cv::exp(-config.exploration_score_simple_dist_exp*distance) * cv::exp(config.exploration_score_simple_angle_exp*(180.0-std::fabs(dTh))/180);

        std::cout<<f.getCentroidGrid()<<" // dist: "<<distance<<" // dTh: "<<dTh<<" // score: "<<score<<std::endl;
        return score;


}
void ExplorationPlanner::calcScoreNBV(Frontier& f) const {

        double yaw = f.getFrontierYaw();
        cv::Point centroid = f.getCentroidGrid();

        //check for reachability of the frontier
        try{
                planner->findPath(centroid);

        }catch(std::exception& e) {
                f.setScore(-40);
                return;
        }
        //std::cout<<"NBV:1 "<<std::endl;

        //double bestScore = calcInformationGain(centroid, yaw, 90.0, 0.5, 120.0);
        double bestScore = -10;
        double bestYaw = yaw;
        cv::Point bestCentroid = centroid;

        //fallback strategy if frontier doesn't fit fov:
        double bestFit = -20;
        bool fittingFrontierFound = false;
        double bestFitAngle;
        cv::Point bestFitPoint;

        //Generate and evaluate n random points in close proximitry to the frontier
        cv::RNG rng(cv::getTickCount());
        std::vector<cv::Point> potSensingPositions = getNRandomPoints(centroid, &rng, config.exploration_score_nbv_sensor_min_range,
                                                                      config.exploration_score_nbv_sensor_max_range, config.exploration_score_nbv_sampling_points);
        if(potSensingPositions.size()<config.exploration_score_nbv_sampling_points){
          f.setScore(-90);
          return;
        }
        //std::cout<<"NBV:1.5 "<<std::endl;
        for(auto current : potSensingPositions) {
                int nAngles = config.exploration_score_nbv_angular_scan_range / config.exploration_score_nbv_angular_scan_accuracy;
                //War vorher 90
                double angle = correctYawAngle(yaw, -config.exploration_score_nbv_angular_scan_range / 2.0);
                //double angle = 0;
                for (int a = 0; a < nAngles; a++) {
                        double quality = isInView(f.getPoints(), current, angle, config.exploration_score_nbv_sensor_fov,
                                                  config.exploration_score_nbv_sensor_max_range/mapInfo.resolution);
                                                  //std::cout<<"NBV:loop1 "<<std::endl;
                        if(quality >= 1.0) {
                                double score = calcInformationGain(current, angle, config.exploration_score_nbv_sensor_fov,
                                                                   config.exploration_score_nbv_sensor_angular_accuracy,
                                                                   config.exploration_score_nbv_sensor_max_range/mapInfo.resolution);
                                                                   //std::cout<<"NBV:loop2 "<<std::endl;
                                if (score > bestScore) {
                                        bestScore = score;
                                        bestCentroid = current;
                                        bestYaw = angle;
                                }
                                fittingFrontierFound = true;

                        }else{

                                bool reachable = true;
                                //std::cout<<"NBV:loop1.4 "<<std::endl;

                                try{
                                        planner->findPath(current);

                                }catch(std::exception& e) {
                                        reachable = false;
                                }
                                //std::cout<<"NBV:loop1.5 "<<std::endl;

                                if(quality>bestFit && reachable) {
                                        bestFit = quality;
                                        bestFitAngle = angle;
                                        bestFitPoint = current;
                                }
                                //std::cout<<"NBV:loop1.6 "<<std::endl;
                                //std::cout<<"NBV:loop3 "<<std::endl;
                        }
                        angle = correctYawAngle(angle, config.exploration_score_nbv_angular_scan_accuracy);
                        //std::cout<<"NBV:loop4 "<<std::endl;


                }
        }
        //std::cout<<"NBV:2 "<<std::endl;

        if(!fittingFrontierFound) {
                bestScore = calcInformationGain(bestFitPoint, bestFitAngle, config.exploration_score_nbv_sensor_fov,
                                                config.exploration_score_nbv_sensor_angular_accuracy,
                                                config.exploration_score_nbv_sensor_max_range/mapInfo.resolution);
                bestYaw = bestFitAngle;
                bestCentroid = bestFitPoint;
        }
        //std::cout<<"NBV:3 "<<std::endl;
        /*
        if(bestScore<0) {

          try{
                  f.setPath(planner->findPath(centroid));
                  f.setScore(0);
                  return;

          }catch(std::exception& e) {
                  f.setScore(-50);
                  return;
          }

        }
        */
        //std::cout<<"NBV:4 "<<std::endl;

        f.setScore(bestScore);
        f.setFrontierYaw(bestYaw);
        f.setCentroidGrid(bestCentroid);
        try{
                f.setPath(planner->findPath(bestCentroid));

        }catch(std::exception& e) {
                f.setScore(-60);
                return;
        }
        //std::cout<<"NBV:5 "<<std::endl;

}

std::vector<cv::Point> ExplorationPlanner::getNRandomPoints(const cv::Point& frontierCentroid, cv::RNG* rngSeed, double innerRadius, double outerRadius, int n) const {
        int maxIterations = 1000;
        std::vector<cv::Point> vectorOfPoints;
        int inner = innerRadius/mapInfo.resolution;
        int outer = outerRadius/mapInfo.resolution;
        while(vectorOfPoints.size()<n && maxIterations>0) {
                double r = rngSeed->uniform(inner, outer);
                double phi = CV_PI * rngSeed->uniform(0.0, 2.0);
                int y = r * cv::sin(phi) + frontierCentroid.y;
                int x = r * cv::cos(phi) + frontierCentroid.x;
                cv::Point p(x, y);
                if (p.x < occupancyGrid.cols && p.y < occupancyGrid.rows && p.x >= 0 && p.y >= 0 && occupancyGrid.at<uchar>(p) == PTP::FREE_CELL_COLOR) {
                        try{
                            if(planner->isGoalSafe(p)){
                              vectorOfPoints.push_back(p);
                            }
                        }catch(std::exception& e){
                          std::cout<<e.what()<<std::endl;
                        }

                }
                maxIterations--;
        }
        return vectorOfPoints;

}

double ExplorationPlanner::isInView(const std::vector<cv::Point>& pts, const cv::Point& start, double yaw, double FOV, double maxDistance) const {
        double minAngle = correctYawAngle(yaw, -FOV / 2.0);
        double maxAngle = correctYawAngle(yaw, FOV / 2.0);
        cv::Point P2;
        P2.x = (int) round(
                start.x
                + maxDistance * cv::cos((-minAngle) * CV_PI / 180.0));
        P2.y = (int) round(
                start.y
                + maxDistance * cv::sin((-minAngle) * CV_PI / 180.0));
        cv::Point P3;
        P3.x = (int) round(
                start.x
                + maxDistance * cv::cos((-maxAngle) * CV_PI / 180.0));
        P3.y = (int) round(
                start.y
                + maxDistance * cv::sin((-maxAngle) * CV_PI / 180.0));

        std::vector<cv::Point>viewPoly;
        viewPoly.push_back(start);
        viewPoly.push_back(P2);
        viewPoly.push_back(P3);
        viewPoly.push_back(start);
        int c = 0;
        for(auto current : pts) {
                if(pointPolygonTest(viewPoly, current, false)>=0) {
                        c++;
                }
        }
        return ((double)c)/((double)pts.size());

}

double ExplorationPlanner::calcInformationGain(const cv::Point& start, double yaw, double FOV, double angularResolution, double maxDistance) const {
        cv::Rect baseWindow(0, 0, occupancyGrid.rows, occupancyGrid.cols);
        double viewDistance = config.exploration_score_nbv_sensor_max_range/mapInfo.resolution;
        cv::Rect rw(0, 0, 2.1*viewDistance, 2.1*viewDistance);
        //std::cout<<"NBV:calcInfo0.5 "<<std::endl;
        rw.x = start.x-rw.width/2;
        rw.y = start.y-rw.height/2;
        rw = rw & baseWindow;
        cv::Mat original = occupancyGrid(rw).clone();
        //std::cout<<"NBV:calcInfo1 "<<std::endl;

        int kernelSize = 3;
        for(int b = 0; b<config.exploration_score_nbv_obstacle_inflation; b++) {
                cv::erode(original,original, cv::getStructuringElement( 0,
                                                                        cv::Size( 2*kernelSize + 1, 2*kernelSize + 1 ),
                                                                        cv::Point( kernelSize, kernelSize ) ));
        }
        cv::Mat temp = original.clone();
        //std::cout<<"NBV:calcInfo2 "<<std::endl;

        int nAngles = FOV / angularResolution;
        double angle = correctYawAngle(yaw, -FOV / 2);
        for (int a = 0; a <= nAngles; a++) {
                cv::Point P1(start.x-rw.x, start.y-rw.y);

                cv::Point P2;

                P2.x = (int) round(
                        P1.x
                        + maxDistance * cv::cos((-angle) * CV_PI / 180.0)); // - vor angle entfernt
                P2.y = (int) round(
                        P1.y
                        + maxDistance * cv::sin((-angle) * CV_PI / 180.0)); // - vor angle entfernt


                cv::LineIterator it(original, P1, P2, 8);
                for (int i = 0; i < it.count; i++, ++it) {
                        int current = original.at<uchar>(it.pos());

                        if (current < PTP::UNKNOWN_CELL_COLOR) {
                                break;
                        } else {
                                temp.at<uchar>(it.pos()) = PTP::FREE_CELL_COLOR;

                        }
                }
                angle = correctYawAngle(angle, angularResolution);
        }
        //std::cout<<"NBV:calcInfo3 "<<std::endl;

        try{
                double distance = planner->findPath(start).getWorldLength();
                double dTh = yaw-robotYaw;
                if(dTh<-180.0) {
                        dTh += 360.0;
                }else if(dTh>180.0) {
                        dTh -= 360.0;
                }
                double n = cv::norm(original, temp, CV_L2) + 1;
                //std::cout<<"NBV:calcInfo4 "<<std::endl;

                return n * cv::exp(-config.exploration_score_nbv_alpha*distance) * cv::exp(config.exploration_score_nbv_beta*(180.0-std::fabs(dTh))/180);

        }catch(std::exception& e) {

                return -30;
        }

}
//Prevents endless feedback loops between local planner and ExplorationPlanner
//by ignoring frontiers existing below a certain steering
//precision defined in the local planner.
bool ExplorationPlanner::checkProximitryToFrontier(const Frontier& f) const {
        double d = cv::norm(f.getCentroidGrid() - robotGridPos)*mapInfo.resolution;
        double dTh = f.getFrontierYaw()-robotYaw;
        if(dTh<-180.0) {
                dTh += 360.0;
        }else if(dTh>180.0) {
                dTh -= 360.0;
        }
        dTh = std::fabs(dTh);
        if(d<=config.exploration_goal_accuracy_position && dTh<=config.exploration_goal_accuracy_angle/CV_PI*180.0) {
                return true;
        }
        return false;
}

cv::Mat ExplorationPlanner::drawFrontiers(){
        cv::Mat out = occupancyGrid.clone();
        cv::cvtColor(out, out, CV_GRAY2RGB);
        contours detectedEdges;
        for(auto current : validFrontiers) {
                detectedEdges.push_back(current.getPoints());
        }

        if(detectedEdges.size()==0) {
                return out;
        }else{

                float contrast = 0;
                float invContrast = 0;

                for(int i = 0; i<detectedEdges.size(); i++) {
                        if(detectedEdges.size()!=1) {
                                contrast = ((float)detectedEdges.size()-1-(float)i)/((float)detectedEdges.size()-1);
                                invContrast = 1.0-contrast;
                        }else{
                                contrast = 1;
                                invContrast = 0;
                        }

                        cv::drawContours(out, detectedEdges, i, cv::Scalar(0, ((float)255)*contrast, ((float)255)*invContrast));
                }
                //std::cout<<"Points in des. order:"<<std::endl;
                for(auto current : validFrontiers) {
                        cv::circle(out, current.getCentroidGrid(), 2, cv::Scalar(255, 0, 0), -1, 8, 0);
                        //std::cout<<"P: "<<current.getCentroidGrid()<<" score: "<<current.getScore()<<std::endl;

                        std::ostringstream strs;
                        //strs <<"P: "<<current.getCentroidGrid()
                        strs<<"S: "<<current.getScore();
                        //<<"Y: "<<current.getFrontierYaw()
                        //<<"D: "<<current.getPath().getWorldLength();
                        std::string str = strs.str();
                        std::string ostring = str;
                        cv::putText(out, ostring, current.getCentroidGrid(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0) );


                        double length = 25;
                        cv::Point P2;

                        P2.x =  (int)round(current.getCentroidGrid().x + length * cv::cos((-current.getFrontierYaw()) * CV_PI / 180.0));
                        P2.y =  (int)round(current.getCentroidGrid().y + length * cv::sin((-current.getFrontierYaw()) * CV_PI / 180.0));

                        cv::line(out, current.getCentroidGrid(), P2, cv::Scalar(255, 0, 0));
                }

                cv::circle(out, robotGridPos, 2, cv::Scalar(0, 0, 255), -1, 8, 0);
                //std::ostringstream strs;
                //strs<<"Y: "<<robotYaw;
                //std::string str = strs.str();
                //std::string ostring = str;
                //cv::putText(out, ostring, robotGridPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0) );
                double length = 25;
                cv::Point P2;

                P2.x =  (int)round(robotGridPos.x + length * cv::cos((-robotYaw) * CV_PI / 180.0));
                P2.y =  (int)round(robotGridPos.y + length * cv::sin((-robotYaw) * CV_PI / 180.0));

                cv::line(out, robotGridPos, P2, cv::Scalar(0, 0, 255));


        }

        return out(rollingWindow);

}

cv::Mat ExplorationPlanner::drawWindow(){
        cv::Mat out = window.clone();
        cv::cvtColor(out, out, CV_GRAY2RGB);
        return out;
}

frontierPoints ExplorationPlanner::localPtsToGlobal(const frontierPoints& points) const {
        frontierPoints fp;
        for(auto current : points) {
                cv::Point temp(current.x+rollingWindow.x,current.y+rollingWindow.y);
                fp.push_back(temp);
        }
        return fp;
}

cv::Point2f ExplorationPlanner::gridPtToWorld(const cv::Point& point) const {
        cv::Point2f worldPoint;
        worldPoint.x = point.x*mapInfo.resolution + mapInfo.origin.position.x;
        worldPoint.y = -(point.y*mapInfo.resolution + mapInfo.origin.position.y);
        return worldPoint;
}

cv::Point ExplorationPlanner::calcFrontierCentroid(const frontierPoints& points) const {
        cv::Point result;
        double x_mean = 0;
        double y_mean = 0;
        for(auto current : points) {
                x_mean += (double)current.x;
                y_mean += (double)current.y;
        }
        x_mean /= points.size();
        y_mean /= points.size();
        result.x= (int)x_mean;
        result.y= (int)y_mean;
        return result;
}

cv::Point ExplorationPlanner::shiftCentroid(const cv::Point& point, const double yaw) const {
        double temp_yaw = correctYawAngle(yaw, 180);
        cv::Point end;
        end.x =  (int)round(point.x + 120 * cv::cos((-temp_yaw) * CV_PI / 180.0));
        end.y =  (int)round(point.y + 120 * cv::sin((-temp_yaw) * CV_PI / 180.0));
        cv::LineIterator it(occupancyGrid, point, end, 8);
        for(int i = 0; i<it.count; i++, ++it) {
                if(occupancyGrid.at<uchar>(it.pos()) > PTP::UNKNOWN_CELL_COLOR) {
                        return it.pos();
                }
        }
        return point;
}

double ExplorationPlanner::calcFrontierColorGradient(const frontierPoints& points) const {
        //select region of interest
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(points, center, radius);
        std::vector<cv::Point> circlePoints;
        int max_x = (center.x+radius<occupancyGrid.cols) ? center.x+radius : occupancyGrid.cols-1;
        int min_x = (center.x-radius>=0) ? center.x-radius : 0;
        int max_y = (center.y+radius<occupancyGrid.rows) ? center.y+radius : occupancyGrid.rows-1;
        int min_y = (center.y-radius>=0) ? center.y-radius : 0;

        circlePoints.push_back(cv::Point(min_x, center.y));
        circlePoints.push_back(cv::Point(center.x, max_y));
        circlePoints.push_back(cv::Point(center.x, min_y));
        circlePoints.push_back(cv::Point(max_x, center.y));

        cv::Rect roi = cv::boundingRect(circlePoints);

        //crop region of interest
        cv::Mat contourMap = occupancyGrid(roi).clone();

        //reduce noise
        cv::blur(contourMap, contourMap, cv::Size(3,3));

        //calc gray scale gradient
        cv::Mat grad_x, grad_y, grad;
        int scale = 1;
        int delta = 0;
        int ddepth = CV_64F;

        cv::Sobel( contourMap, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );         //dx
        cv::Sobel( contourMap, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );         //dy

        //find phase angle
        cv::Mat orientation = cv::Mat::zeros(grad_x.rows, grad_y.cols, CV_64F);
        grad_x.convertTo(grad_x,CV_64F);
        grad_y.convertTo(grad_y,CV_64F);
        cv::addWeighted( grad_x, 0.5, grad_y, 0.5, 0, grad );

        phase(grad_x, grad_y, orientation, true);
        orientation.convertTo(orientation,CV_32S);

        //remove useless information
        grad = cv::abs(grad);
        grad.convertTo(grad,CV_8UC1);
        cv::bitwise_not(grad, grad);
        cv::threshold(grad, grad, freeColor-1, freeColor, CV_THRESH_BINARY);
        cv::Mat mask = cv::Mat::zeros(grad.rows, grad.cols, CV_32S);
        cv::bitwise_and(orientation, mask, orientation, grad);

        double yaw = 0;
        double ts = 0;
        double tc = 0;
        // find average over all angles of the gradient
        for(auto current : points) {
                // transform global point to roi point
                cv::Point roiPoint(current.x - roi.x, current.y - roi.y);
                int temp_yaw = orientation.at<int>(roiPoint);
                ts += cv::sin(temp_yaw*CV_PI/180);
                tc += cv::cos(temp_yaw*CV_PI/180);
        }
        // transform angle to yaw angle
        yaw = std::atan2(ts,tc)/CV_PI*180;
        yaw = -correctYawAngle(yaw, 180);

        return yaw;
}

int ExplorationPlanner::makeYaw(const int angle) const {
        int yaw = angle;
        if(yaw<-360) {
                yaw += 360;
        }else if(yaw>180) {
                yaw -= 360;
        }
        return yaw;
}

double ExplorationPlanner::makeYaw(const double angle) const {
        double yaw = angle;
        if(yaw<-360.0) {
                yaw += 360.0;
        }else if(yaw>180.0) {
                yaw -= 360.0;
        }
        return yaw;
}

double ExplorationPlanner::correctYawAngle(const double theta, const double increment) const {
        double yaw = 0;
        double angle = theta + increment;
        if (angle > 180.0) {
                yaw = -180.0 - (180.0 - angle);
        } else if (angle < -180.0) {
                yaw = 180.0 - (-180.0 - angle);
        } else {
                yaw = angle;
        }
        return yaw;
}
