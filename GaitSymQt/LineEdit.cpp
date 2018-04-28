#include "lineedit.h"

LineEdit::LineEdit(QWidget *parent) :
    QLineEdit(parent)
{
}

// use enter events to grab the keyboard
void LineEdit::enterEvent ( QEvent * /* event */ )
{
    setFocus();
    emit EmitMouseEnter();
}

// use leave events to release the keyboard
void LineEdit::leaveEvent ( QEvent * /* event */ )
{
    clearFocus();
    emit EmitMouseLeave();
}
