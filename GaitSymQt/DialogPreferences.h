#ifndef DIALOGPREFERENCES_H
#define DIALOGPREFERENCES_H

#include <QDialog>

class Preferences;

namespace Ui {
    class DialogPreferences;
}

class DialogPreferences : public QDialog {
    Q_OBJECT
public:
    DialogPreferences(QWidget *parent = 0);
    ~DialogPreferences();

    void SetValues(const Preferences &prefs);
    void GetValues(Preferences *prefs);

public slots:

    void graphicsPathBrowse();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::DialogPreferences *m_ui;
};

#endif // DIALOGPREFERENCES_H
