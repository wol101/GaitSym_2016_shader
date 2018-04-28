#ifndef DIALOGINTERFACE_H
#define DIALOGINTERFACE_H

#include <QDialog>

class Preferences;

namespace Ui {
    class DialogInterface;
}

class DialogInterface : public QDialog
{
    Q_OBJECT

public:
    explicit DialogInterface(QWidget *parent = 0);
    ~DialogInterface();

    Ui::DialogInterface *getUI() { return ui; }

    static QColor getIdealTextColour(const QColor& rBackgroundColour);
    static QColor getAlphaColourHint(const QColor& colour);

    void SetValues(const Preferences &prefs);
    void GetValues(Preferences *prefs);

    QColor m_EnvironmentColour;
    QColor m_BodyColour;
    QColor m_JointColour;
    QColor m_GeomColour;
    QColor m_GeomForceColour;
    QColor m_StrapColour;
    QColor m_StrapForceColour;
    QColor m_StrapCylinderColour;
    QColor m_ReporterColour;
    QColor m_DataTargetColour;
    QColor m_BackgroundColour;


public slots:

    void colourEnvironment();
    void colourBody();
    void colourJoint();
    void colourGeom();
    void colourGeomForce();
    void colourStrap();
    void colourStrapForce();
    void colourStrapCylinder();
    void colourReporter();
    void colourDataTarget();
    void backgroundColour();

signals:
    void EmitBackgroundColour(float red, float green, float blue, float alpha);
    void EmitRedrawRequired();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::DialogInterface *ui;
};

#endif // DIALOGINTERFACE_H
