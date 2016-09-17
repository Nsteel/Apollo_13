#include "Dashboard.h"
#include "../../../build/simulation/ui_Dashboard.h"

Dashboard::Dashboard(ros::NodeHandle* nh, QWidget *parent) :
        QMainWindow(parent), ui(new Ui::Dashboard), nh(nh)
{
        ui->setupUi(this);
        ctrl_msg = control_msg();
        commands = nh->advertise<simulation::ctrl_msg>("robot_control", 10);
        automap = nh->advertise<automap_ctrl>("automap/ctrl_msg", 10);
        robotInfo = nh->subscribe<simulation::telemetry_msg>("telemetry", 10, boost::bind(telemetryCallback, _1, ui));

        connect(ui->speedSlider, SIGNAL(valueChanged(int)), this, SLOT(valueChangedSpeed(int)));
        connect(ui->steeringSlider, SIGNAL(valueChanged(int)), this, SLOT(valueChangedSteering(int)));
        connect(ui->maxSpeed, SIGNAL(clicked()), this, SLOT(maxSpeedClicked()));
        connect(ui->minSpeed, SIGNAL(clicked()), this, SLOT(minSpeedClicked()));
        connect(ui->zeroSpeed, SIGNAL(clicked()), this, SLOT(zeroSpeedClicked()));
        connect(ui->maxSteering, SIGNAL(clicked()), this, SLOT(maxSteeringClicked()));
        connect(ui->minSteering, SIGNAL(clicked()), this, SLOT(minSteeringClicked()));
        connect(ui->centerSteering, SIGNAL(clicked()), this, SLOT(centerSteeringClicked()));
        connect(ui->automap_ctrl, SIGNAL(stateChanged(int)), this, SLOT(automapControlClicked()));
        connect(ui->automap_sens, SIGNAL(stateChanged(int)), this, SLOT(automapSensingClicked()));
        connect(ui->automap_nbv, SIGNAL(stateChanged(int)), this, SLOT(automapNBVClicked()));

        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(pollNodeHandle()));
        timer->start(5);

}

Dashboard::~Dashboard()
{
        delete ui;
}

void telemetryCallback(const simulation::telemetry_msg::ConstPtr& tele, Ui::Dashboard* ui){
        ui->speed->display(tele->v_radial);
        ui->steering->display(tele->steering_angle);
        ui->v_x->display(tele->v_linear.x);
        ui->v_y->display(tele->v_linear.y);
        ui->v_angle->display(tele->v_angular.z);
        ui->distance->display(tele->radial_distance);
        ros::spinOnce();
}

void Dashboard::keyPressEvent(QKeyEvent *event){
        ctrl_msg.header.stamp = ros::Time::now();
        int speed = ctrl_msg.speed;
        int steering = ctrl_msg.steering;

        switch ( event->key()) {
        case Qt::Key_W: {
                if(speed<10) {
                        ctrl_msg.speed = speed + 1;
                }
                break;
        }


        case  Qt::Key_S: {
                if(speed>-10) {
                        ctrl_msg.speed = speed - 1;
                }
                break;
        }


        case  Qt::Key_A: {
                if(steering > -50) {
                        ctrl_msg.steering = steering - 2;
                }
                break;
        }


        case Qt::Key_D: {
                if(steering < 50) {
                        ctrl_msg.steering  = steering + 2;
                }
                break;
        }

        }

        ui->speedSlider->setValue(ctrl_msg.speed);
        ui->steeringSlider->setValue(ctrl_msg.steering);

        commands.publish(ctrl_msg);
        ros::spinOnce();
}

void Dashboard::pollNodeHandle(){
        ros::spinOnce();
        timer->start(5);
}

void Dashboard::valueChangedSpeed(int value){
        ctrl_msg.header.stamp = ros::Time::now();
        ctrl_msg.speed=value;
        commands.publish(ctrl_msg);
        ros::spinOnce();
}

void Dashboard::valueChangedSteering(int value){
        ctrl_msg.header.stamp = ros::Time::now();
        ctrl_msg.steering=value;
        commands.publish(ctrl_msg);
        ros::spinOnce();
}

void Dashboard::maxSpeedClicked(){
        ctrl_msg.header.stamp = ros::Time::now();
        ui->speedSlider->setValue(10);
        ctrl_msg.speed=10;
        commands.publish(ctrl_msg);
        ros::spinOnce();
}

void Dashboard::minSpeedClicked(){
        ctrl_msg.header.stamp = ros::Time::now();
        ui->speedSlider->setValue(-10);
        ctrl_msg.speed=-10;
        commands.publish(ctrl_msg);
        ros::spinOnce();
}

void Dashboard::zeroSpeedClicked(){
        ctrl_msg.header.stamp = ros::Time::now();
        ui->speedSlider->setValue(0);
        ctrl_msg.speed=0;
        commands.publish(ctrl_msg);
        ros::spinOnce();
}

void Dashboard::maxSteeringClicked(){
        ctrl_msg.header.stamp = ros::Time::now();
        ui->steeringSlider->setValue(50);
        ctrl_msg.steering=50;
        commands.publish(ctrl_msg);
        ros::spinOnce();
}

void Dashboard::minSteeringClicked(){
        ctrl_msg.header.stamp = ros::Time::now();
        ui->steeringSlider->setValue(-50);
        ctrl_msg.steering=-50;
        commands.publish(ctrl_msg);
        ros::spinOnce();
}

void Dashboard::centerSteeringClicked(){
        ctrl_msg.header.stamp = ros::Time::now();
        ui->steeringSlider->setValue(8);
        ctrl_msg.steering=8;
        commands.publish(ctrl_msg);
        ros::spinOnce();
}

void Dashboard::automapControlClicked(){
        automap_msg.header.stamp = ros::Time::now();
        automap_msg.control_On=ui->automap_ctrl->isChecked();
        automap_msg.detection_On=ui->automap_sens->isChecked();
        automap_msg.NBV_On=ui->automap_nbv->isChecked();
        automap.publish(automap_msg);
        ros::spinOnce();
}

void Dashboard::automapSensingClicked(){
        automap_msg.header.stamp = ros::Time::now();
        automap_msg.control_On=ui->automap_ctrl->isChecked();
        automap_msg.detection_On=ui->automap_sens->isChecked();
        automap_msg.NBV_On=ui->automap_nbv->isChecked();
        automap.publish(automap_msg);
        ros::spinOnce();
}

void Dashboard::automapNBVClicked(){
        automap_msg.header.stamp = ros::Time::now();
        automap_msg.control_On=ui->automap_ctrl->isChecked();
        automap_msg.detection_On=ui->automap_sens->isChecked();
        automap_msg.NBV_On=ui->automap_nbv->isChecked();
        automap.publish(automap_msg);
        ros::spinOnce();
}
