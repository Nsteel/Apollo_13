#ifndef MotionPrimitiveDesignerWindow_h
#define MotionPrimitiveDesignerWindow_h

#include <QtGui>

class DiscreteAnglesSpinBox;
class GLWidget;

class MotionPrimitiveDesignerWindow : public QMainWindow
{
    Q_OBJECT

public:

    MotionPrimitiveDesignerWindow(QWidget* parent = 0, Qt::WindowFlags flags = 0);

    // virtual QSize sizeHint() const;

public slots:

    void update_gui();

private slots:

    void update_num_angles(int i);
    void toggle_selection_mode();

private:

    int last_spinbox_value_;

    GLWidget*       render_widget_;
    QDockWidget*    control_panel_dock_widget_;

    QPushButton*    discrete_mode_toggle_button_;
    QPushButton*    add_goal_button_;
    QPushButton*    remove_goal_button_;

    DiscreteAnglesSpinBox*  num_disc_angles_spinbox_;
    QSpinBox*               start_disc_angle_spinbox_;
    QSpinBox*               start_disc_x_spinbox_;
    QSpinBox*               start_disc_y_spinbox_;
    QSpinBox*               goal_disc_angle_spinbox_;
    QSpinBox*               goal_disc_x_spinbox_;
    QSpinBox*               goal_disc_y_spinbox_;

    bool is_pow2(unsigned i);
};

#endif
