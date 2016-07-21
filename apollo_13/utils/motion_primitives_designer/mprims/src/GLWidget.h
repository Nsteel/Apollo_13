#ifndef GLWidget_h
#define GLWidget_h

#include <list>
#include <vector>
#include <Eigen/Dense>
#include <QtOpenGL>
#include "Pose2.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:

    GLWidget(QWidget* parent = 0);
    GLWidget(QGLContext* context, QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0);
    GLWidget(const QGLFormat& format, QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0);

    QSize sizeHint() const { return QSize(800, 800); }

    const Pose2_disc& disc_min() const { return min_; }
    const Pose2_disc& disc_max() const { return max_; }

    bool discrete_mode() const { return disc_mode_; }
    bool have_selection() const { return false; }

    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

    int start_x() const { return (int)start_.x; }
    int start_y() const { return (int)start_.y; }
    int start_yaw() const { return discretize_angle(start_.yaw, num_angles_); }

    bool goal_selected() const { return selection_.goal_selected; }
    int goal_x() const { return (int)selection_.selected_goal->x; }
    int goal_y() const { return (int)selection_.selected_goal->y; }
    int goal_yaw() const { return discretize_angle(selection_.selected_goal->yaw, num_angles_); }

public slots:

    void toggle_disc_mode();
    void add_discrete_goal();
    void remove_discrete_goal();
    void set_num_angles(int);
    void set_disc_start_angle(int);
    void set_disc_start_x(int);
    void set_disc_start_y(int);
    void set_disc_goal_angle(int);
    void set_disc_goal_x(int);
    void set_disc_goal_y(int);

signals:

    void gui_changed();

private:

    bool disc_mode_;

    bool left_button_down_;
    bool right_button_down_;

    QPointF start_tail_;
    QPointF goal_tail_;

    QPointF start_head_;
    QPointF goal_head_;

    Pose2_disc min_;
    Pose2_disc max_;

    Pose2_cont start_;
    std::list<Pose2_cont> goals_;

    QPointF left_button_down_pos_;
    QPointF right_button_down_pos_;

    struct Selection
    {
        bool start_selected;
        bool goal_selected;
        std::list<Pose2_cont>::iterator selected_goal;
    } selection_;

    int num_angles_;

    void construct();

    bool hits_start(const QPointF& point) const;
    bool hits_goal(std::list<Pose2_cont>::iterator goal_idx, const QPointF& point) const;
    bool hits_arrow(const Pose2_cont& pose, const QPointF& point) const;

    QPointF viewport_to_world(const QPointF& viewport_coord) const;

    void draw_grid();
    void draw_guidelines();
    void draw_selection();
    void draw_arrow(double x, double y, double yaw, double r, double g, double b, double scale = 1.0);
    void draw_arrow_wireframe(double x, double y, double yaw, double r, double g, double b, double scale = 1.0);
    void draw_line(const std::vector<Pose2_cont>& motion);

    double realize_angle(int disc_angle, int num_angles);

    double normalize_angle(double angle) const;
    int discretize_angle(double angle, int num_angles) const;

    bool same_side(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;
    bool point_in_triangle(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) const;

    Pose2_cont discretize(const Pose2_cont& pose);

    void clear_selection();
    void select_at(const QPointF& point);
};

#endif
