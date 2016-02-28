#include "DiscreteAnglesSpinBox.h"

DiscreteAnglesSpinBox::DiscreteAnglesSpinBox(QWidget* parent) :
    QSpinBox(parent)
{
}

void DiscreteAnglesSpinBox::stepBy(int steps)
{
    if (steps > 0)
    {
        setValue(value() << 1);
    }
    else
    {
        setValue(value() >> 1);
    }
}
