#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <QMainWindow>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <simulation/telemetry_msg.h>
#include <simulation/ctrl_msg.h>
#include <QTimer>
#include <QKeyEvent>
#include <automap/automap_ctrl_msg.h>

typedef geometry_msgs::Twist twist_msg;
typedef simulation::ctrl_msg control_msg;
typedef automap::automap_ctrl_msg automap_ctrl;

namespace Ui {
class Dashboard;
}

//void telemetryCallback(const geometry_msgs::Twist::ConstPtr& tele, Ui::Dashboard* ui);
void telemetryCallback(const simulation::telemetry_msg::ConstPtr& tele, Ui::Dashboard* ui);

class Dashboard : public QMainWindow
{
        Q_OBJECT

public:
        explicit Dashboard(ros::NodeHandle* nh, QWidget *parent = 0);
        ~Dashboard();
        void keyPressEvent(QKeyEvent *event);

private:
        Ui::Dashboard *ui;
        ros::NodeHandle* nh;
        control_msg ctrl_msg;
        automap_ctrl automap_msg;
        ros::Publisher commands;
        ros::Publisher automap;
        ros::Subscriber robotInfo;
        QTimer *timer;

private slots:
        void valueChangedSpeed(int value);
        void valueChangedSteering(int value);
        void maxSpeedClicked();
        void minSpeedClicked();
        void zeroSpeedClicked();
        void maxSteeringClicked();
        void minSteeringClicked();
        void centerSteeringClicked();
        void pollNodeHandle();
        void automapControlClicked();
        void automapSensingClicked();
        void automapNBVClicked();
};

#endif // DASHBOARD_H
