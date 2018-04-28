#include "spinbox.h"

SpinBox::SpinBox(QWidget *parent) :
    QSpinBox(parent)
{
}

// use enter events to grab the keyboard
void SpinBox::enterEvent ( QEvent * /* event */ )
{
    setFocus();
    emit EmitMouseEnter();
}

// use leave events to release the keyboard
void SpinBox::leaveEvent ( QEvent * /* event */ )
{
    clearFocus();
    emit EmitMouseLeave();
}
