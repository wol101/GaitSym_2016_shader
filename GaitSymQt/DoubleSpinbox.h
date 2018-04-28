#ifndef DOUBLESPINBOX_H
#define DOUBLESPINBOX_H

#include <QDoubleSpinBox>

class DoubleSpinBox : public QDoubleSpinBox
{
    Q_OBJECT
public:
    explicit DoubleSpinBox(QWidget *parent = 0);
    
signals:
    void EmitMouseEnter();
    void EmitMouseLeave();
    
public slots:

protected:
    void enterEvent(QEvent * event);
    void leaveEvent(QEvent * event);

};

#endif // DOUBLESPINBOX_H
