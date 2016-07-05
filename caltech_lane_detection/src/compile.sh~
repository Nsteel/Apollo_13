# This file compiles the lane_detector library and ros node

# Author: Nicolas Acero
# Date: 29/06/2016

path=`rospack find lane_detector`
clean="make clean"
compile="make release"
command="gcc -shared -o libLaneDetector.so CameraInfoOpt.o LaneDetectorOpt.o LaneDetector.o InversePerspectiveMapping.o mcv.o"
copy_library="cp libLaneDetector.so $path/lib/"
copy_headers="cp *.h $path/include/lane_detector/"

$clean
$compile
$command
$copy_library
$copy_headers

#workspace path
cd ~/catkin_ws
catkin_make
