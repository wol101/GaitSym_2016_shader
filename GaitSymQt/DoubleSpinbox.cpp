#include "doublespinbox.h"

DoubleSpinBox::DoubleSpinBox(QWidget *parent) :
    QDoubleSpinBox(parent)
{
}

// use enter events to grab the keyboard
void DoubleSpinBox::enterEvent ( QEvent * /* event */)
{
    setFocus();
    emit EmitMouseEnter();
}

// use leave events to release the keyboard
void DoubleSpinBox::leaveEvent ( QEvent * /* event */ )
{
    clearFocus();
    emit EmitMouseLeave();
}
