#ifndef DiscreteAnglesSpinBox_h
#define DiscreteAnglesSpinBox_h

#include <QtGui>

class DiscreteAnglesSpinBox : public QSpinBox
{
    Q_OBJECT;

public:

    DiscreteAnglesSpinBox(QWidget* parent = 0);

    virtual void stepBy(int steps);

private:

};

#endif
