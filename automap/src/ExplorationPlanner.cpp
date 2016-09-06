#include "ExplorationPlanner.h"

ExplorationPlanner::ExplorationPlanner(const PathtransformPlanner * planner, const int windowSize_x,
                                       const int windowSize_y) : planner(planner){
        rollingWindow = cv::Rect(0,0,windowSize_x, windowSize_y);
        robotFootprint = planner->getRobotFootprint();
        mapInfo = planner->getMapInfo();
        unknownColor = PTP::UNKNOWN_CELL_COLOR;
        occupiedColor = PTP::UNKNOWN_CELL_COLOR;
        freeColor = PTP::FREE_CELL_COLOR;

        localDetector = FrontierDetector(freeColor, unknownColor, occupiedColor);
        globalDetector = FrontierDetector(freeColor, unknownColor, occupiedColor);
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

        //scoreValidFrontiersSimple();
        //extractBestFrontier();

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
        cv::Rect baseWindow(0, 0, occupancyGrid.rows, occupancyGrid.cols);
        rollingWindow.x = robotGridPos.x-rollingWindow.width/2;
        rollingWindow.y = robotGridPos.y-rollingWindow.height/2;
        rollingWindow = rollingWindow & baseWindow;

        window = occupancyGrid(rollingWindow).clone();

        //1.) find all frontiers
        validFrontiers = std::list<Frontier>();
        localDetector.processFrontiers(window);
        contours found = localDetector.getFrontiers();
        if(found.size()==0) {
                return false;
        }

        //2.) generate passable frontier objects only
        std::vector<Frontier>passableFrontiers;

        //std::cout<<robotFootprint<<std::endl;
        double minL = std::sqrt(robotFootprint.height*robotFootprint.height + robotFootprint.width*robotFootprint.width)*mapInfo.resolution;
        for(auto current : found) {
                frontierPoints globalFp = localPtsToGlobal(current);
                double length = calcFrontierLength(globalFp);
                double dist = cv::norm(globalFp[0]-globalFp[globalFp.size()-1]);
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(globalFp, center, radius);
                radius = radius*2*mapInfo.resolution;
                //std::cout<<center<<" radius*2: "<<radius<< " length: "<<length<<" gap: "<<dist<<" robotWidth:"<<minL<<std::endl;
                if(radius>minL) { // i think this should only be the distance of the 2 outer points
                        cv::Point centroidGlobal = calcFrontierCentroid(globalFp);
                        double frontierYaw = calcFrontierColorGradient(globalFp); // this is quite accurate but not very smart
                        if(!useNBV){
                          centroidGlobal = shiftCentroid(centroidGlobal, frontierYaw); // this prevents centroids in unknown but also not very smart
                        }
                        cv::Point2f centroidWorld = gridPtToWorld(centroidGlobal);
                        passableFrontiers.push_back(Frontier(globalFp, centroidGlobal, centroidWorld, length, frontierYaw));
                }
        }
        if(passableFrontiers.size()==0) {
                return false;
        }

        //3.) rember reachable frontiers only
        for(auto current : passableFrontiers) {
                if(!checkProximitryToFrontier(current)) {
                        try{
                                if(!useNBV){
                                  Path p = planner->findPath(current.getCentroidGrid());
                                  current.setPath(p);
                                  current.setScore(calcScoreSimple(current));
                                }
                                else{
                                  calcScoreNBV(current);
                                }



                                frontierStack.push_back(current);
                                //
                                validFrontiers.push_back(current);
                        }catch(std::exception& e) {
                                std::cout<<e.what()<<std::endl;
                        }
                }

        }
        if(validFrontiers.size()==0) {
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

bool ExplorationPlanner::extractValidFrontiersGlobal(bool useNBV){
        //1.) find all frontiers
        validFrontiers = std::list<Frontier>();
        globalDetector.processFrontiers(occupancyGrid);
        contours found = globalDetector.getFrontiers();
        if(found.size()==0) {
                return false;
        }

        //2.) generate passable frontier objects only
        std::vector<Frontier>passableFrontiers;
        double minL = std::sqrt(robotFootprint.height*robotFootprint.height + robotFootprint.width*robotFootprint.width)*mapInfo.resolution;
        for(auto current : found) {
                double length = calcFrontierLength(current);
                double dist = cv::norm(current[0]-current[current.size()-1]);
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(current, center, radius);
                radius = radius*2*mapInfo.resolution;
                //std::cout<<center<<" radius*2: "<<radius<< " length: "<<length<<" gap: "<<dist<<" robotWidth:"<<minL<<std::endl;
                if(radius>minL) { // i think this should only be the distance of the 2 outer points
                        cv::Point centroidGlobal = calcFrontierCentroid(current);
                        double frontierYaw = calcFrontierColorGradient(current); // this is quite accurate but not very smart
                        if(!useNBV){
                          centroidGlobal = shiftCentroid(centroidGlobal, frontierYaw); // this prevents centroids in unknown but also not very smart
                        }
                        cv::Point2f centroidWorld = gridPtToWorld(centroidGlobal);
                        passableFrontiers.push_back(Frontier(current, centroidGlobal, centroidWorld, length, frontierYaw));
                }
        }
        if(passableFrontiers.size()==0) {
                return false;
        }

        //3.) rember reachable frontiers only
        for(auto current : passableFrontiers) {
                if(!checkProximitryToFrontier(current)) {
                        try{
                                if(!useNBV){
                                  Path p = planner->findPath(current.getCentroidGrid());
                                  current.setPath(p);
                                  current.setScore(calcScoreSimple(current));

                                }else{
                                  calcScoreNBV(current);
                                }

                                validFrontiers.push_back(current);
                                //
                                frontierStack.push_back(current);

                        }catch(std::exception& e) {
                                std::cout<<e.what()<<std::endl;
                        }
                }

        }
        if(validFrontiers.size()==0) {
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
        double length = f.getLength();
        double dTh = f.getFrontierYaw()-robotYaw;
        if(dTh<-180.0) {
                dTh += 360.0;
        }else if(dTh>180.0) {
                dTh -= 360.0;
        }
        //double dYaw = 180.0-std::fabs(dTh);

        //score = (180-dYaw)*(180-dYaw)* (1/ (1+distance)) *  length;
        //score = (180-dYaw)*(180-dYaw)*  length;
        //score =dYaw +  dYaw*(1/ (1+(distance))) + dYaw*(length * 0.05);
        //std::cout<<f.getCentroidGrid()<<" // dist: "<<distance<<" // dTh: "<<dTh<<" // score: "<<score<<std::endl;
        double dFun = std::pow(2*(1/(1+distance)), 4);
        double yFun = std::pow(2*((180.0-std::fabs(dTh))/180.0), 4);
        score =  6*dFun * yFun;
        //score = dYaw + dYaw*(1/ (1+(distance)));
        //return std::fabs(dTh);
        std::cout<<f.getCentroidGrid()<<" // dist: "<<distance<<" // dTh: "<<dTh<<" // score: "<<score<<std::endl;
        return score;


}
void ExplorationPlanner::calcScoreNBV(Frontier& f) const {

        double yaw = f.getFrontierYaw();
        cv::Point centroid = f.getCentroidGrid();

        //double bestScore = calcInformationGain(centroid, yaw, 90.0, 0.5, 120.0);
        double bestScore = 0;
        double bestYaw = yaw;
        cv::Point bestCentroid = centroid;

        //fallback strategy if frontier doesn't fit fov:
        double bestFit = 0;
        bool fittingFrontierFound = false;
        double bestFitAngle;
        cv::Point bestFitPoint;

        //Generate and evaluate n random points in close proximitry to the frontier
        cv::RNG rng(cv::getTickCount());
        std::vector<cv::Point> potSensingPositions = getNRandomPoints(centroid, &rng, 1, 6, 10);

        for(auto current : potSensingPositions) {
                int nAngles = 180.0 / 5.0;

                double angle = correctYawAngle(yaw, -90.0 / 2.0);
                //double angle = 0;
                for (int a = 0; a < nAngles; a++) {
                        double quality = isInView(f.getPoints(), current, angle, 90.0, 120.0);
                        //std::cout<<"in view.."<<quality<<std::endl;
                        //if(quality >= (1.0-0.3) && quality <= 1.0) {
                        if(quality >= 1.0) {
                                double score = calcInformationGain(current, angle, 90.0, 0.5, 120.0);
                                if (score > bestScore) {
                                        bestScore = score;
                                        bestCentroid = current;
                                        bestYaw = angle;
                                }
                                fittingFrontierFound = true;

                        }else{
                                if(quality>bestFit) {
                                        bestFit = quality;
                                        bestFitAngle = angle;
                                        bestFitPoint = current;
                                }
                        }
                        angle = correctYawAngle(angle, 5.0);


                }
        }

        /*
           for (int i = 0; i < 50; i++) {
                double r = rng.uniform(20.0, 120.0); //60pxls means 2meters, based on the max view distance -> still hard code
                //double phi = correctYawAngle(yaw, rng.uniform(0.0, 90.0)-45.0)/180.0*CV_PI;
                double phi = CV_PI * rng.uniform(0.0, 2.0);
                int y = r * cv::sin(phi) + centroid.y;
                int x = r * cv::cos(phi) + centroid.x;
                //cout<<current.getCentroid()<<" --- "<<x<<" // "<<y<<" ---- " << map.cols<<" // " << map.rows<<endl;


                if (x < occupancyGrid.cols && y < occupancyGrid.rows && x >= 0 && y >= 0) {
                        cv::Point p(x, y);

                        if (occupancyGrid.at<uchar>(p) == PTP::FREE_CELL_COLOR) {
                                //std::cout<<p<<" --- "<<x<<" // "<<y<<" ---- "<<std::endl;
                                //int nAngles = 90.0 / 15.0;
                                int nAngles = 180.0 / 5.0;

                                double angle = correctYawAngle(yaw, -90.0 / 2.0);
                                //double angle = 0;
                                for (int a = 0; a < nAngles; a++) {
                                        double quality = isInView(f.getPoints(), p, angle, 90.0, 120.0);
                                        std::cout<<"in view.."<<quality<<std::endl;
                                        //if(quality >= (1.0-0.3) && quality <= 1.0) {
                                        if(quality >= 1.0) {
                                                double score = calcInformationGain(p, angle, 90.0, 0.5, 120.0);
                                                if (score > bestScore) {
                                                        bestScore = score;
                                                        bestCentroid = p;
                                                        bestYaw = angle;
                                                }
                                                fittingFrontierFound = true;

                                        }else{
                                                if(quality>bestFit) {
                                                        bestFit = quality;
                                                        bestFitAngle = angle;
                                                        bestFitPoint = p;
                                                }
                                        }
                                        angle = correctYawAngle(angle, 5.0);


                                }

                        }
                }

           }
         */
        if(!fittingFrontierFound) {
                bestScore = calcInformationGain(bestFitPoint, bestFitAngle, 90.0, 0.5, 120.0);
                bestYaw = bestFitAngle;
                bestCentroid = bestFitPoint;
        }

        f.setScore(bestScore);
        f.setFrontierYaw(bestYaw);
        f.setCentroidGrid(bestCentroid);
        f.setPath(planner->findPath(bestCentroid));
}

std::vector<cv::Point> ExplorationPlanner::getNRandomPoints(const cv::Point& frontierCentroid, cv::RNG* rngSeed, double innerRadius, double outerRadius, int n) const {
        std::vector<cv::Point> vectorOfPoints;
        int inner = innerRadius/mapInfo.resolution;
        int outer = outerRadius/mapInfo.resolution;
        while(vectorOfPoints.size()<n) {
                double r = rngSeed->uniform(inner, outer);
                double phi = CV_PI * rngSeed->uniform(0.0, 2.0);
                int y = r * cv::sin(phi) + frontierCentroid.y;
                int x = r * cv::cos(phi) + frontierCentroid.x;
                cv::Point p(x, y);
                if (p.x < occupancyGrid.cols && p.y < occupancyGrid.rows && p.x >= 0 && p.y >= 0 && occupancyGrid.at<uchar>(p) == PTP::FREE_CELL_COLOR) {
                        vectorOfPoints.push_back(p);
                }
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
        cv::Rect rw(0, 0, 260, 260);
        rw.x = start.x-rw.width/2;
        rw.y = start.y-rw.height/2;
        rw = rw & baseWindow;
        cv::Mat original = occupancyGrid(rw).clone();
        //cv::Mat temp = occupancyGrid(rw).clone();
        int kernelSize = 3;
        //cv::blur(original,original,cv::Size(kernelSize, kernelSize));
        for(int b = 0; b<2; b++) {
                cv::erode(original,original, cv::getStructuringElement( 0,
                                                                        cv::Size( 2*kernelSize + 1, 2*kernelSize + 1 ),
                                                                        cv::Point( kernelSize, kernelSize ) ));
        }
        //cv::blur(original,original,cv::Size(kernelSize, kernelSize));
        cv::Mat temp = original.clone();

        int nAngles = FOV / angularResolution;
        double angle = correctYawAngle(yaw, -FOV / 2);
        //std::cout<<"start: "<<angle;
        //std::vector<cv::Point>viewPoly;
        //cv::Point movingPxl=start;
        //viewPoly.push_back(start);
        //cv::Point movingPixel(start.x, start.y);
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
                                //movingPixel = cv::Point(it.pos().x+rw.x, it.pos().y+rw.y);
                                //movingPxl = it.pos();

                        }
                }
                //viewPoly.push_back(movingPixel);
                angle = correctYawAngle(angle, angularResolution);
        }
        /*
           viewPoly.push_back(start);
           for(auto blubb : viewPoly){
            std::cout<<blubb<<std::endl;
           }
           //cv::Point s1 = f.getPoints()[0];
           //cv::Point s2 =f.getPoints()[f.getPoints().size()-1];
           bool isInViewPoly = false;
           for(auto pts : f.getPoints()){
           if(pointPolygonTest(viewPoly, pts, false)<0){
            //isInViewPoly = false;
            //break;
           }else{
            isInViewPoly = true;
           }
           }
           //bool isInViewPoly = (pointPolygonTest(viewPoly, s1, false)>=0) && (pointPolygonTest(viewPoly, s2, false)>=0);

           if(!isInViewPoly) {
                std::cout<<" // frontier not in view: "<<isInViewPoly<<" // "<<std::endl;
                return -1;
           }else{
                std::cout<<" // frontier is in view: "<<isInViewPoly<<" // "<<std::endl;
           }
         */

        try{
                double distance = planner->findPath(start).getWorldLength();
                double dTh = yaw-robotYaw;
                if(dTh<-180.0) {
                        dTh += 360.0;
                }else if(dTh>180.0) {
                        dTh -= 360.0;
                }
                double n = cv::norm(original, temp, CV_L2);
                /*
                   std::ostringstream strs;
                   strs<<"P:"<<start<<"-Y:"<<yaw<<"-iterations:"<<nAngles<<"-gain:"<<n<<".png";
                   std::string str = strs.str();
                   std::string imgPath = ros::package::getPath("automap") + "/data/test/" + str;
                   std::vector<int> com_param;
                   com_param.push_back(CV_IMWRITE_PNG_COMPRESSION);
                   com_param.push_back(9);
                   try {
                        cv::imwrite(imgPath, temp, com_param);
                        ROS_INFO("Map written to: %s", imgPath.c_str());

                   } catch (std::runtime_error& ex) {
                        std::cout << "Exception converting img to PNG: " << ex.what() << std::endl;
                   }
                 */
                //return n;
                //return cv::norm(original, temp, CV_L2) * cv::exp(1.0/(1.0+distance) + (180.0-std::fabs(dTh))/180);
                if(distance<=8.0) {
                        return n * cv::exp(-0.1*distance) *  (180.0-std::fabs(dTh))/180;
                }else{
                        return n * cv::exp(-0.1*distance);
                }
        }catch(std::exception& e) {
                //std::cout<<e.what()<<std::endl;
                return -1;
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
        if(d<=1.0 && dTh<=0.5/CV_PI*180.0) {
                return true;
        }
        return false;
}

void ExplorationPlanner::scoreValidFrontiersSimple(){
        for(Frontier current : validFrontiers) {
                double score = 0;
                double distance = current.getPath().getWorldLength();
                double length = current.getLength();
                double dTh = current.getFrontierYaw()-robotYaw;
                if(dTh<-180.0) {
                        dTh += 360.0;
                }else if(dTh>180.0) {
                        dTh -= 360.0;
                }
                double dYaw = 180.0-std::fabs(dTh);

                //score = (180-dYaw)*(180-dYaw)* (1/ (1+distance)) *  length;
                //score = (180-dYaw)*(180-dYaw)*  length;
                //score =dYaw +  dYaw*(1/ (1+(distance))) + dYaw*(length * 0.5);
                //score = dYaw + dYaw*(1/ (1+(distance)));
                score = dYaw;
                //std::cout<<"Frontier: "<<current.getCentroidGrid()<<" // d: "<<distance<<"m // l: "<<length<<"m // dYaw: "<<dYaw<<"deg // score: "<<score<<std::endl;
                current.setScore(score);
                std::cout<<"Frontier: "<<current.getCentroidGrid()<<" // d: "<<distance<<"m // l: "<<length<<"m // dYaw: "<<dYaw<<"deg // score: "<<current.getScore()<<std::endl;
                frontierStack.push_back(current);


        }
        //frontierStack.sort();
        //frontierStack.reverse();
}

void ExplorationPlanner::extractBestFrontier(){
        validFrontiers.sort();
        validFrontiers.reverse();
        Frontier temp = validFrontiers.front();
        std::cout<<"Best from stack: "<<temp.getScore()<<std::endl;
        frontierStack.push_back(temp);
        //validFrontiers.pop_front();
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

double ExplorationPlanner::calcFrontierLength(const frontierPoints& points) const {
        double length = 0;
        for(int i = 0; i+1<points.size(); i++) {
                length += cv::norm(points[i]-points[i+1]);
        }
        return length*mapInfo.resolution/2.0;
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

        //shiftedCentroid.x =  (int)round(point.x + 0.4/0.05 * cv::cos((-temp_yaw) * CV_PI / 180.0));
        //shiftedCentroid.y =  (int)round(point.y + 0.4/0.05 * cv::sin((-temp_yaw) * CV_PI / 180.0));

        //cv::Point shiftedCentroid;
        //return shiftedCentroid;
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
        //since map is in gray scale, this step is not needed
        //cv::cvtColor(contourMap, contourMap, CV_RGB2GRAY);
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

        //draw for testing
        //cv::rectangle(map, roi, cv::Scalar(255, 0, 0), 1, 1);
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
/*
   int ExplorationPlanner::correctYawAngle(const int theta, const int increment) const {
        int yaw = 0;
        int angle = theta + increment;
        if (angle > 180.0) {
                yaw = -180.0 - (180.0 - angle);
        } else if (angle < -180.0) {
                yaw = 180.0 - (-180.0 - angle);
        } else {
                yaw = angle;
        }
        return yaw;
   }
 */
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
