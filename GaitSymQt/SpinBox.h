#ifndef SPINBOX_H
#define SPINBOX_H

#include <QSpinBox>

class SpinBox : public QSpinBox
{
    Q_OBJECT
public:
    explicit SpinBox(QWidget *parent = 0);
    
signals:
    void EmitMouseEnter();
    void EmitMouseLeave();

public slots:

protected:
    void enterEvent(QEvent * event);
    void leaveEvent(QEvent * event);

};

#endif // SPINBOX_H
