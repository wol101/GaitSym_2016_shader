#ifndef LINEEDIT_H
#define LINEEDIT_H

#include <QLineEdit>

class LineEdit : public QLineEdit
{
    Q_OBJECT
public:
    explicit LineEdit(QWidget *parent = 0);
    
signals:
    void EmitMouseEnter();
    void EmitMouseLeave();

public slots:

protected:
    void enterEvent(QEvent * event);
    void leaveEvent(QEvent * event);

};

#endif // LINEEDIT_H
