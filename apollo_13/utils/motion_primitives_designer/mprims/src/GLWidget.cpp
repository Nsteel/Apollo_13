#include <cmath>
#include <cstdio>
#include <Eigen/Dense>
#include <GL/gl.h>
#include <GL/glu.h>
#include "GLWidget.h"
#include "logging.h"
#include "unicycle_motions.h"

static int discretize(double d, double res)
{
    return (int)(d / res);
}

static double continuize(int i, double res)
{
    return (double)(i * res);
}

GLWidget::GLWidget(QWidget* parent) :
    QGLWidget(parent)
{
    construct();
}

GLWidget::GLWidget(QGLContext* context, QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f) :
    QGLWidget(context, parent, shareWidget, f)
{
    construct();
}

GLWidget::GLWidget(const QGLFormat& format, QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f) :
    QGLWidget(format, parent, shareWidget, f)
{
    construct();
}

void GLWidget::construct()
{
    disc_mode_ = true;
    min_ = Pose2_disc(-15, -15, 0);
    max_ = Pose2_disc(15, 15, 360);

    start_ = Pose2_cont(0.0, 0.0, 0.0);
    goals_.push_back(Pose2_cont(10.0, 0.0, 0.0));

    left_button_down_ = false;
    right_button_down_ = false;
    num_angles_ = 16;
}

void GLWidget::initializeGL()
{
    glClearColor(1.0f, 0.98f, 0.98f, 1.0f);
    glLineWidth(2.0f);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    draw_grid();

    if (disc_mode_)
    {
        draw_guidelines();
    }

    for (const Pose2_cont goal : goals_) {
        std::vector<Pose2_cont> motion = generate_unicycle_motion(start_, goal);
        draw_line(motion);
    }

    // draw the start
    draw_arrow(start_.x, start_.y, start_.yaw, 0.0, 1.0, 0.0);

    // draw the goal
    for (const Pose2_cont goal : goals_) {
        draw_arrow(goal.x, goal.y, goal.yaw, 1.0, 0.0, 0.0);
    }

    // draw the selection
    draw_selection();

    glFlush();
    swapBuffers();
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    QPointF world_point = viewport_to_world(event->posF());

    // select the target pose
    if (event->button() == Qt::LeftButton || event->button() == Qt::RightButton) {
        clear_selection();
        select_at(world_point);
        if (selection_.goal_selected || selection_.start_selected) {
            emit gui_changed();
        }
    }

    if (event->button() == Qt::LeftButton) {
        left_button_down_ = true;
        left_button_down_pos_ = world_point;
        update();
    }
    else if (event->button() == Qt::RightButton) {
        right_button_down_ = true;
        right_button_down_pos_ = world_point;
        update();
    }
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    QPointF world_point = viewport_to_world(event->posF());

    if (left_button_down_) {
        // translate the selected pose
        if (selection_.start_selected) {
            start_.x = world_point.x();
            start_.y = world_point.y();
            DEBUG_PRINT("Moved the start to (%0.3f, %0.3f)", world_point.x(), world_point.y());
        }
        else if (selection_.goal_selected) {
            selection_.selected_goal->x = world_point.x();
            selection_.selected_goal->y = world_point.y();
            DEBUG_PRINT("Moved the selected goal to (%0.3f, %0.3f)", world_point.x(), world_point.y());
        }
    }
    if (right_button_down_) {
        // rotate the selected pose
        double dx = world_point.x() - right_button_down_pos_.x();
        double dy = world_point.y() - right_button_down_pos_.y();
        double angle = atan2(dy, dx);

        if (selection_.start_selected) {
            start_.yaw = angle;
            DEBUG_PRINT("Moved the start yaw to %0.3f", angle);
        }
        else if (selection_.goal_selected) {
            selection_.selected_goal->yaw = angle;
            DEBUG_PRINT("Moved the selected goal yaw to %0.3f", angle);
        }
    }

    update();
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    DEBUG_PRINT("Mouse release event");
    if (disc_mode_) {
        // snap to nearest discrete pose
        DEBUG_PRINT("Snapping to discrete poses");
        start_ = discretize(start_);
        for (Pose2_cont& pose : goals_) {
            pose = discretize(pose);
        }

        emit gui_changed();
    }

    if (event->button() == Qt::LeftButton) {
        left_button_down_ = false;
    }
    else if (event->button() == Qt::RightButton) {
        right_button_down_ = false;
    }

    update();
}

bool GLWidget::hits_start(const QPointF& point) const
{
    return hits_arrow(start_, point);
}

bool GLWidget::hits_goal(std::list<Pose2_cont>::iterator goal_it, const QPointF& point) const
{
    return hits_arrow(*goal_it, point);
}

bool GLWidget::hits_arrow(const Pose2_cont& pose, const QPointF& point) const
{
    bool selected = false;

    Eigen::Vector3d pv(point.x(), point.y(), 0.0);

    Eigen::Affine3d transform = Eigen::Translation3d(pose.x, pose.y, 0.0) * Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ());

    {
        Eigen::Vector3d a(0.166, 0.3, 0.0);
        Eigen::Vector3d b(0.166, -0.3, 0.0);
        Eigen::Vector3d c(0.5, 0.0, 0.0);

        a = transform * a;
        b = transform * b;
        c = transform * c;

        selected |= point_in_triangle(pv, a, b, c);
    }

    {
        Eigen::Vector3d a(-0.5, -0.15, 0.0);
        Eigen::Vector3d b(0.166, -0.15, 0.0);
        Eigen::Vector3d c(-0.15, 0.15, 0.0);

        a = transform * a;
        b = transform * b;
        c = transform * c;

        selected |= point_in_triangle(pv, a, b, c);
    }

    {
        Eigen::Vector3d a(0.166, -0.15, 0.0);
        Eigen::Vector3d b(0.166,  0.15, 0.0);
        Eigen::Vector3d c(-0.5, 0.15, 0.0);

        a = transform * a;
        b = transform * b;
        c = transform * c;

        selected |= point_in_triangle(pv, a, b, c);
    }

    return selected;
}

void GLWidget::toggle_disc_mode()
{
    disc_mode_ = !(disc_mode_);

    DEBUG_PRINT("Toggle Discrete Mode: %s!", (disc_mode_ ? "On" : "Off"));

    // continuous -> discrete mode
    if (disc_mode_) {
        start_ = discretize(start_);
        for (Pose2_cont& pose : goals_) {
            pose = discretize(pose);
        }
    }

    left_button_down_ = false;
    right_button_down_ = false;

    update();
    emit gui_changed();
}

void GLWidget::add_discrete_goal()
{
    goals_.push_back(Pose2_cont(0.0, 0.0, 0.0));
    emit gui_changed();
    update();
}

void GLWidget::remove_discrete_goal()
{
    if (selection_.goal_selected) {
        goals_.erase(selection_.selected_goal);
        selection_.goal_selected = false;
        update();
        emit gui_changed();
    }
}

void GLWidget::set_num_angles(int num_angles)
{
    DEBUG_PRINT("Set Num Angles to %d!", num_angles);
    num_angles_ = num_angles;
    update();
}

void GLWidget::set_disc_start_angle(int angle)
{
    printf("Set Discrete Start Angle to %d!\n", angle);
    start_.yaw = realize_angle(angle, num_angles_);
    update();
}

void GLWidget::set_disc_start_x(int disc_x)
{
    start_.x = (double)disc_x;
    update();
}

void GLWidget::set_disc_start_y(int disc_y)
{
    start_.y = (double)disc_y;
    update();
}

void GLWidget::set_disc_goal_angle(int angle)
{
    printf("Set Discrete Goal Angle to %d!\n", angle);
    assert(selection_.goal_selected);
    selection_.selected_goal->yaw = realize_angle(angle, num_angles_);
    update();
}

void GLWidget::set_disc_goal_x(int disc_x)
{
    assert(selection_.goal_selected);
    selection_.selected_goal->x = (double)disc_x;
    update();
}

void GLWidget::set_disc_goal_y(int disc_y)
{
    assert(selection_.goal_selected);
    selection_.selected_goal->y = (double)disc_y;
    update();
}

void GLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D((GLdouble)min_.x, (GLdouble)max_.x, (GLdouble)min_.y, (GLdouble)max_.y);
    glMatrixMode(GL_MODELVIEW);
}

QPointF GLWidget::viewport_to_world(const QPointF& viewport_coord) const
{
    const double world_width = max_.x - min_.x;
    const double world_height = max_.y - min_.y;
    const double world_x = world_width * viewport_coord.x() / width() + min_.x;
    const double world_y = world_height * (1.0 - viewport_coord.y() / height()) + min_.y;
    return QPointF(world_x, world_y);
}

void GLWidget::draw_grid()
{
    // draw the grid
    glBegin(GL_LINES);
    glColor3f(0.8f, 0.8f, 0.8f);
    for (int x = min_.x; x <= max_.x; ++x) {
        if (x != 0 && (x % 5)) {
            glVertex2d((double)x, min_.y);
            glVertex2d((double)x, max_.y);
        }
    }
    for (int y = min_.y; y <= max_.y; ++y) {
        if (y != 0 && (y % 5)) {
            glVertex2d(min_.x, y);
            glVertex2d(max_.x, y);
        }
    }

    glColor3f(0.5f, 0.5f, 0.5f);
    for (int x = min_.x; x <= max_.x; ++x) {
        if ((x % 5) == 0) {
            glVertex2d((double)x, min_.y);
            glVertex2d((double)x, max_.y);
        }
    }
    for (int y = min_.y; y <= max_.y; ++y) {
        if (y % 5 == 0) {
            glVertex2d(min_.x, y);
            glVertex2d(max_.x, y);
        }
    }

    glColor3f(0.0f, 0.0f, 0.0f);
    glVertex2d(0, min_.y);
    glVertex2d(0, max_.y);
    glVertex2d(min_.x, 0);
    glVertex2d(max_.x, 0);
    glEnd();
}

void GLWidget::draw_selection()
{
    if (selection_.start_selected) {
        draw_arrow_wireframe(start_.x, start_.y, start_.yaw, 0.0, 0.0, 1.0);
    }
    else if (selection_.goal_selected) {
        draw_arrow_wireframe(selection_.selected_goal->x, selection_.selected_goal->y, selection_.selected_goal->yaw, 0.0, 0.0, 1.0);
    }
}

void GLWidget::draw_guidelines()
{
    glColor3f(0.0f, 0.0f, 1.0f);

    if (left_button_down_ || right_button_down_) {
        if (selection_.start_selected) {
            Pose2_cont disc_start = discretize(start_);
            draw_arrow_wireframe(disc_start.x, disc_start.y, disc_start.yaw, 0.5, 0.5, 1.0, 1.5);
        }
        if (selection_.goal_selected) {
            Pose2_cont disc_goal = discretize(*selection_.selected_goal);
            draw_arrow_wireframe(disc_goal.x, disc_goal.y, disc_goal.yaw, 0.5, 0.5, 1.0, 1.5);
        }
    }
}

void GLWidget::draw_arrow(double x, double y, double yaw, double r, double g, double b, double scale)
{
    glPushMatrix();
    glColor3f(r, g, b);
    glLoadIdentity();
    glTranslated(x, y, 0);
    glRotated(yaw * 180.0 / M_PI, 0.0, 0.0, 1.0);
    glScaled(scale, scale, scale);
    glBegin(GL_TRIANGLES);
    glVertex2d(-0.5, 0.15);
    glVertex2d(-0.5, -0.15);
    glVertex2d(0.666 - 0.5, -0.15);

    glVertex2d(0.666 - 0.5, -0.15);
    glVertex2d(0.666 - 0.5, 0.15);
    glVertex2d(-0.5, 0.15);

    glVertex2d(0.666 - 0.5, 0.3);
    glVertex2d(0.666 - 0.5, -0.3);
    glVertex2d(0.5, 0.0);
    glEnd();
    glPopMatrix();
}

void GLWidget::draw_arrow_wireframe(double x, double y, double yaw, double r, double g, double b, double scale)
{
    glPushMatrix();
    glColor3f(r, g, b);
    glLoadIdentity();
    glTranslated(x, y, 0);
    glRotated(yaw * 180.0 / M_PI, 0.0, 0.0, 1.0);
    glScaled(scale, scale, scale);
    glBegin(GL_LINE_LOOP);
    glVertex2d(-0.5, 0.15);
    glVertex2d(-0.5, -0.15);
    glVertex2d(0.666 - 0.5, -0.15);
    glVertex2d(0.666 - 0.5, -0.3);
    glVertex2d(0.5, 0.0);
    glVertex2d(0.666 - 0.5, 0.3);
    glVertex2d(0.666 - 0.5, 0.15);
    glVertex2d(-0.5, 0.15);
    glEnd();
    glPopMatrix();
}

void GLWidget::draw_line(const std::vector<Pose2_cont>& motion)
{
    glColor3f(1.0f, 0.0f, 1.0f);
    glBegin(GL_LINE_STRIP);
    for (const Pose2_cont& pose : motion) {
        glVertex2d(pose.x, pose.y);
    }
    glEnd();
}

double GLWidget::realize_angle(int index, int num_angles)
{
    return index * (2.0 * M_PI) / num_angles;
}

int GLWidget::discretize_angle(double angle, int num_angles) const
{
    double thetaBinSize = 2.0 * M_PI / num_angles;
    return (int)(normalize_angle(angle + thetaBinSize / 2.0) / (2.0 * M_PI) * (num_angles));
}

double GLWidget::normalize_angle(double angle) const
{
    // get to the range from -2PI, 2PI
    if (fabs(angle) > 2 * M_PI) {
        angle = angle - ((int)(angle / (2 * M_PI))) * 2 * M_PI;
    }

    // get to the range 0, 2PI
    if (angle < 0) {
        angle += 2 * M_PI;
    }

    return angle;
}

bool GLWidget::same_side(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
{
    Eigen::Vector3d cp1 = (b - a).cross(p1 - a);
    Eigen::Vector3d cp2 = (b - a).cross(p2 - a);
    return (cp1.dot(cp2) >= 0) ;
}

bool GLWidget::point_in_triangle(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) const
{
    return same_side(p, a, b, c) && same_side(p, b, a, c) && same_side(p, c, a, b);
}

Pose2_cont GLWidget::discretize(const Pose2_cont& pose)
{
    double disc_x = std::round(pose.x);
    double disc_y = std::round(pose.y);
    double disc_angle = realize_angle(discretize_angle(pose.yaw, num_angles_), num_angles_);

    return Pose2_cont(disc_x, disc_y, disc_angle);
}

void GLWidget::clear_selection()
{
    selection_.start_selected = false;
    selection_.goal_selected = false;
    selection_.selected_goal = goals_.end();
}

void GLWidget::select_at(const QPointF& point)
{
    if (hits_start(point)) {
        DEBUG_PRINT("Selected the start");
        selection_.start_selected = true;
    }
    else {
        int idx = 0;
        for (std::list<Pose2_cont>::iterator i = goals_.begin(); i != goals_.end(); ++i) {
            if (hits_goal(i, point)) {
                DEBUG_PRINT("Selected goal %d", idx);
                selection_.goal_selected = true;
                selection_.selected_goal = i;
            }
            ++idx;
        }
    }
}
