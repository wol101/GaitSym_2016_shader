#include "dialoginterface.h"
#include "ui_dialoginterface.h"
#include "Simulation.h"
#include "Environment.h"
#include "Body.h"
#include "Joint.h"
#include "Geom.h"
#include "Muscle.h"
#include "CylinderWrapStrap.h"
#include "TwoCylinderWrapStrap.h"
#include "Contact.h"
#include "Reporter.h"
#include "DataTarget.h"
#include "preferences.h"

#include <QColorDialog>
#include <QAbstractSpinBox>

// Simulation globals
extern Simulation *gSimulation;

DialogInterface::DialogInterface(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogInterface)
{
    ui->setupUi(this);

    connect(ui->pushButtonBackgroundColor, SIGNAL(clicked()), this, SLOT(backgroundColour()));
    connect(ui->pushButtonEnvironmentColour, SIGNAL(clicked()), this, SLOT(colourEnvironment()));
    connect(ui->pushButtonBodyColour, SIGNAL(clicked()), this, SLOT(colourBody()));
    connect(ui->pushButtonJointColour, SIGNAL(clicked()), this, SLOT(colourJoint()));
    connect(ui->pushButtonGeomColour, SIGNAL(clicked()), this, SLOT(colourGeom()));
    connect(ui->pushButtonGeomForceColour, SIGNAL(clicked()), this, SLOT(colourGeomForce()));
    connect(ui->pushButtonStrapColour, SIGNAL(clicked()), this, SLOT(colourStrap()));
    connect(ui->pushButtonStrapForceColour, SIGNAL(clicked()), this, SLOT(colourStrapForce()));
    connect(ui->pushButtonStrapCylinderColour, SIGNAL(clicked()), this, SLOT(colourStrapCylinder()));
    connect(ui->pushButtonReporterColour, SIGNAL(clicked()), this, SLOT(colourReporter()));
    connect(ui->pushButtonDataTargetColour, SIGNAL(clicked()), this, SLOT(colourDataTarget()));
}

DialogInterface::~DialogInterface()
{
    delete ui;
}

void DialogInterface::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}


void DialogInterface::SetValues(const Preferences &prefs)
{
    ui->lineEditEnvironmentX->setText(QString("%1").arg(prefs.EnvironmentX));
    ui->lineEditEnvironmentY->setText(QString("%1").arg(prefs.EnvironmentY));
    ui->lineEditEnvironmentZ->setText(QString("%1").arg(prefs.EnvironmentZ));
    ui->lineEditBodyX->setText(QString("%1").arg(prefs.BodyX));
    ui->lineEditBodyY->setText(QString("%1").arg(prefs.BodyY));
    ui->lineEditBodyZ->setText(QString("%1").arg(prefs.BodyZ));
    ui->lineEditJointX->setText(QString("%1").arg(prefs.JointX));
    ui->lineEditJointY->setText(QString("%1").arg(prefs.JointY));
    ui->lineEditJointZ->setText(QString("%1").arg(prefs.JointZ));
    ui->lineEditGeomX->setText(QString("%1").arg(prefs.GeomX));
    ui->lineEditGeomY->setText(QString("%1").arg(prefs.GeomY));
    ui->lineEditGeomZ->setText(QString("%1").arg(prefs.GeomZ));
    ui->lineEditGeomForceScale->setText(QString("%1").arg(prefs.GeomForceScale));
    ui->lineEditGeomForceRadius->setText(QString("%1").arg(prefs.GeomForceRadius));
    ui->lineEditStrapRadius->setText(QString("%1").arg(prefs.StrapRadius));
    ui->lineEditStrapForceRadius->setText(QString("%1").arg(prefs.StrapForceRadius));
    ui->lineEditStrapForceScale->setText(QString("%1").arg(prefs.StrapForceScale));
    ui->lineEditStrapCylinderLength->setText(QString("%1").arg(prefs.StrapCylinderLength));
    ui->lineEditReporterX->setText(QString("%1").arg(prefs.ReporterX));
    ui->lineEditReporterY->setText(QString("%1").arg(prefs.ReporterY));
    ui->lineEditReporterZ->setText(QString("%1").arg(prefs.ReporterZ));
    ui->lineEditDataTargetX->setText(QString("%1").arg(prefs.DataTargetX));
    ui->lineEditDataTargetY->setText(QString("%1").arg(prefs.DataTargetY));
    ui->lineEditDataTargetZ->setText(QString("%1").arg(prefs.DataTargetZ));

    const QString COLOUR_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    m_EnvironmentColour = prefs.EnvironmentColour;
    m_BodyColour = prefs.BodyColour;
    m_JointColour = prefs.JointColour;
    m_GeomColour = prefs.GeomColour;
    m_GeomForceColour = prefs.GeomForceColour;
    m_StrapColour = prefs.StrapColour;
    m_StrapForceColour = prefs.StrapForceColour;
    m_StrapCylinderColour = prefs.StrapCylinderColour;
    m_ReporterColour = prefs.ReporterColour;
    m_DataTargetColour = prefs.DataTargetColour;
    m_BackgroundColour = prefs.BackgroundColour;

    ui->pushButtonEnvironmentColour->setStyleSheet(COLOUR_STYLE.arg(m_EnvironmentColour.name()).arg(getIdealTextColour(m_EnvironmentColour).name()).arg(getAlphaColourHint(m_EnvironmentColour).name()));
    ui->pushButtonBodyColour->setStyleSheet(COLOUR_STYLE.arg(m_BodyColour.name()).arg(getIdealTextColour(m_BodyColour).name()).arg(getAlphaColourHint(m_BodyColour).name()));
    ui->pushButtonJointColour->setStyleSheet(COLOUR_STYLE.arg(m_JointColour.name()).arg(getIdealTextColour(m_JointColour).name()).arg(getAlphaColourHint(m_JointColour).name()));
    ui->pushButtonGeomColour->setStyleSheet(COLOUR_STYLE.arg(m_GeomColour.name()).arg(getIdealTextColour(m_GeomColour).name()).arg(getAlphaColourHint(m_GeomColour).name()));
    ui->pushButtonGeomForceColour->setStyleSheet(COLOUR_STYLE.arg(m_GeomForceColour.name()).arg(getIdealTextColour(m_GeomForceColour).name()).arg(getAlphaColourHint(m_GeomForceColour).name()));
    ui->pushButtonStrapColour->setStyleSheet(COLOUR_STYLE.arg(m_StrapColour.name()).arg(getIdealTextColour(m_StrapColour).name()).arg(getAlphaColourHint(m_StrapColour).name()));
    ui->pushButtonStrapForceColour->setStyleSheet(COLOUR_STYLE.arg(m_StrapForceColour.name()).arg(getIdealTextColour(m_StrapForceColour).name()).arg(getAlphaColourHint(m_StrapForceColour).name()));
    ui->pushButtonStrapCylinderColour->setStyleSheet(COLOUR_STYLE.arg(m_StrapCylinderColour.name()).arg(getIdealTextColour(m_StrapCylinderColour).name()).arg(getAlphaColourHint(m_StrapCylinderColour).name()));
    ui->pushButtonReporterColour->setStyleSheet(COLOUR_STYLE.arg(m_ReporterColour.name()).arg(getIdealTextColour(m_ReporterColour).name()).arg(getAlphaColourHint(m_ReporterColour).name()));
    ui->pushButtonDataTargetColour->setStyleSheet(COLOUR_STYLE.arg(m_DataTargetColour.name()).arg(getIdealTextColour(m_DataTargetColour).name()).arg(getAlphaColourHint(m_DataTargetColour).name()));
    ui->pushButtonBackgroundColor->setStyleSheet(COLOUR_STYLE.arg(m_BackgroundColour.name()).arg(getIdealTextColour(m_BackgroundColour).name()).arg(getAlphaColourHint(m_BackgroundColour).name()));
}

void DialogInterface::GetValues(Preferences *prefs)
{
    prefs->EnvironmentX = ui->lineEditEnvironmentX->text().toDouble();
    prefs->EnvironmentY = ui->lineEditEnvironmentY->text().toDouble();
    prefs->EnvironmentZ = ui->lineEditEnvironmentZ->text().toDouble();
    prefs->BodyX = ui->lineEditBodyX->text().toDouble();
    prefs->BodyY = ui->lineEditBodyY->text().toDouble();
    prefs->BodyZ = ui->lineEditBodyZ->text().toDouble();
    prefs->JointX = ui->lineEditJointX->text().toDouble();
    prefs->JointY = ui->lineEditJointY->text().toDouble();
    prefs->JointZ = ui->lineEditJointZ->text().toDouble();
    prefs->GeomX = ui->lineEditGeomX->text().toDouble();
    prefs->GeomY = ui->lineEditGeomY->text().toDouble();
    prefs->GeomZ = ui->lineEditGeomZ->text().toDouble();
    prefs->GeomForceScale = ui->lineEditGeomForceScale->text().toDouble();
    prefs->GeomForceRadius = ui->lineEditGeomForceRadius->text().toDouble();
    prefs->StrapRadius = ui->lineEditStrapRadius->text().toDouble();
    prefs->StrapForceRadius = ui->lineEditStrapForceRadius->text().toDouble();
    prefs->StrapForceScale = ui->lineEditStrapForceScale->text().toDouble();
    prefs->StrapCylinderLength = ui->lineEditStrapCylinderLength->text().toDouble();
    prefs->ReporterX = ui->lineEditReporterX->text().toDouble();
    prefs->ReporterY = ui->lineEditReporterY->text().toDouble();
    prefs->ReporterZ = ui->lineEditReporterZ->text().toDouble();
    prefs->DataTargetX = ui->lineEditDataTargetX->text().toDouble();
    prefs->DataTargetY = ui->lineEditDataTargetY->text().toDouble();
    prefs->DataTargetZ = ui->lineEditDataTargetZ->text().toDouble();

    prefs->BackgroundColour = m_BackgroundColour;
    prefs->EnvironmentColour = m_EnvironmentColour;
    prefs->BodyColour = m_BodyColour;
    prefs->JointColour = m_JointColour;
    prefs->GeomColour = m_GeomColour;
    prefs->GeomForceColour = m_GeomForceColour;
    prefs->StrapColour = m_StrapColour;
    prefs->StrapForceColour = m_StrapForceColour;
    prefs->StrapCylinderColour = m_StrapCylinderColour;
    prefs->ReporterColour = m_ReporterColour;
    prefs->DataTargetColour = m_DataTargetColour;
    prefs->BackgroundColour = m_BackgroundColour;
}


void DialogInterface::backgroundColour()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor Color = QColorDialog::getColor(m_BackgroundColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (Color.isValid())
    {
        ui->pushButtonBackgroundColor->setStyleSheet(Color_STYLE.arg(Color.name()).arg(getIdealTextColour(Color).name()).arg(getAlphaColourHint(Color).name()));
        m_BackgroundColour = Color;
        emit EmitBackgroundColour(m_BackgroundColour.redF(), m_BackgroundColour.greenF(), m_BackgroundColour.blueF(), m_BackgroundColour.alphaF());
        emit EmitRedrawRequired();
    }
}

void DialogInterface::colourEnvironment()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_EnvironmentColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (colour.isValid())
    {
        ui->pushButtonEnvironmentColour->setStyleSheet(Color_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_EnvironmentColour = colour;

        if (gSimulation)
        {
            gSimulation->GetEnvironment()->SetColour(m_EnvironmentColour.redF(), m_EnvironmentColour.greenF(), m_EnvironmentColour.blueF(), m_EnvironmentColour.alphaF());
            gSimulation->GetInterface()->EnvironmentColour.SetColour(m_EnvironmentColour.redF(), m_EnvironmentColour.greenF(), m_EnvironmentColour.blueF(), m_EnvironmentColour.alphaF());
            emit EmitRedrawRequired();
        }
    }
}


void DialogInterface::colourBody()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_BodyColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (colour.isValid())
    {
        ui->pushButtonBodyColour->setStyleSheet(Color_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_BodyColour = colour;

        if (gSimulation)
        {
            std::map<std::string, Body *> *bodyList = gSimulation->GetBodyList();
            std::map<std::string, Body *>::const_iterator bodyIter;
            for (bodyIter = bodyList->begin(); bodyIter != bodyList->end(); bodyIter++) bodyIter->second->SetColour(m_BodyColour.redF(), m_BodyColour.greenF(), m_BodyColour.blueF(), m_BodyColour.alphaF());
            gSimulation->GetInterface()->BodyColour.SetColour(m_BodyColour.redF(), m_BodyColour.greenF(), m_BodyColour.blueF(), m_BodyColour.alphaF());
            emit EmitRedrawRequired();
        }
    }
}


void DialogInterface::colourJoint()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_JointColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (colour.isValid())
    {
        ui->pushButtonJointColour->setStyleSheet(Color_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_JointColour = colour;

        if (gSimulation)
        {
            std::map<std::string, Joint *> *jointList = gSimulation->GetJointList();
            std::map<std::string, Joint *>::const_iterator jointIter;
            for (jointIter = jointList->begin(); jointIter != jointList->end(); jointIter++) jointIter->second->SetColour(m_JointColour.redF(), m_JointColour.greenF(), m_JointColour.blueF(), m_JointColour.alphaF());
            gSimulation->GetInterface()->JointColour.SetColour(m_JointColour.redF(), m_JointColour.greenF(), m_JointColour.blueF(), m_JointColour.alphaF());
            emit EmitRedrawRequired();
        }
    }
}


void DialogInterface::colourGeom()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_GeomColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (colour.isValid())
    {
        ui->pushButtonGeomColour->setStyleSheet(Color_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_GeomColour = colour;

        if (gSimulation)
        {
            std::map<std::string, Geom *> *geomList = gSimulation->GetGeomList();
            std::map<std::string, Geom *>::const_iterator geomIter;
            for (geomIter = geomList->begin(); geomIter != geomList->end(); geomIter++) geomIter->second->SetColour(m_GeomColour.redF(), m_GeomColour.greenF(), m_GeomColour.blueF(), m_GeomColour.alphaF());
            gSimulation->GetInterface()->GeomColour.SetColour(m_GeomColour.redF(), m_GeomColour.greenF(), m_GeomColour.blueF(), m_GeomColour.alphaF());
            emit EmitRedrawRequired();
        }
    }
}


void DialogInterface::colourGeomForce()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_GeomForceColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (colour.isValid())
    {
        ui->pushButtonGeomForceColour->setStyleSheet(Color_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_GeomForceColour = colour;

        if (gSimulation)
        {
            std::vector<Contact *> *contactList = gSimulation->GetContactList();
            for (unsigned int c = 0; c < contactList->size(); c++) (*contactList)[c]->SetColour(m_GeomForceColour.redF(), m_GeomForceColour.greenF(), m_GeomForceColour.blueF(), m_GeomForceColour.alphaF());
            gSimulation->GetInterface()->GeomForceColour.SetColour(m_GeomColour.redF(), m_GeomColour.greenF(), m_GeomColour.blueF(), m_GeomColour.alphaF());
            emit EmitRedrawRequired();
        }
    }
}


void DialogInterface::colourStrap()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_StrapColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (colour.isValid())
    {
        ui->pushButtonStrapColour->setStyleSheet(Color_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_StrapColour = colour;

        if (gSimulation)
        {
            std::map<std::string, Muscle *> *muscleList = gSimulation->GetMuscleList();
            std::map<std::string, Muscle *>::const_iterator muscleIter;
            for (muscleIter = muscleList->begin(); muscleIter != muscleList->end(); muscleIter++)
            {
                muscleIter->second->GetStrap()->SetColour(m_StrapColour.redF(), m_StrapColour.greenF(), m_StrapColour.blueF(), m_StrapColour.alphaF());
                muscleIter->second->GetStrap()->SetLastDrawTime(-1);
            }
            gSimulation->GetInterface()->StrapColour.SetColour(m_StrapColour.redF(), m_StrapColour.greenF(), m_StrapColour.blueF(), m_StrapColour.alphaF());
            emit EmitRedrawRequired();
        }
    }
}


void DialogInterface::colourStrapForce()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_StrapForceColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (colour.isValid())
    {
        ui->pushButtonStrapForceColour->setStyleSheet(Color_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_StrapForceColour = colour;

        if (gSimulation)
        {
            gSimulation->GetInterface()->StrapForceColour.SetColour(m_StrapForceColour.redF(), m_StrapForceColour.greenF(), m_StrapForceColour.blueF(), m_StrapForceColour.alphaF());
            std::map<std::string, Muscle *> *muscleList = gSimulation->GetMuscleList();
            std::map<std::string, Muscle *>::const_iterator muscleIter;
            for (muscleIter = muscleList->begin(); muscleIter != muscleList->end(); muscleIter++)
            {
                muscleIter->second->GetStrap()->SetForceColour(gSimulation->GetInterface()->StrapForceColour);
                muscleIter->second->GetStrap()->SetLastDrawTime(-1);
            }
            emit EmitRedrawRequired();
        }
    }
}


void DialogInterface::colourStrapCylinder()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_StrapCylinderColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (colour.isValid())
    {
        ui->pushButtonStrapCylinderColour->setStyleSheet(Color_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_StrapCylinderColour = colour;

        if (gSimulation)
        {
            gSimulation->GetInterface()->StrapCylinderColour.SetColour(m_StrapCylinderColour.redF(), m_StrapCylinderColour.greenF(), m_StrapCylinderColour.blueF(), m_StrapCylinderColour.alphaF());
            std::map<std::string, Muscle *> *muscleList = gSimulation->GetMuscleList();
            std::map<std::string, Muscle *>::const_iterator muscleIter;
            for (muscleIter = muscleList->begin(); muscleIter != muscleList->end(); muscleIter++)
            {
                CylinderWrapStrap *cylinderWrapStrap = dynamic_cast<CylinderWrapStrap *>(muscleIter->second->GetStrap());
                if (cylinderWrapStrap)
                {
                    cylinderWrapStrap->SetCylinderColour(gSimulation->GetInterface()->StrapCylinderColour);
                    cylinderWrapStrap->SetLastDrawTime(-1);
                }
                else
                {
                    TwoCylinderWrapStrap *twoCylinderWrapStrap = dynamic_cast<TwoCylinderWrapStrap *>(muscleIter->second->GetStrap());
                    if (twoCylinderWrapStrap)
                    {
                        twoCylinderWrapStrap->SetCylinderColour(gSimulation->GetInterface()->StrapCylinderColour);
                        twoCylinderWrapStrap->SetLastDrawTime(-1);
                    }
                }
            }
            emit EmitRedrawRequired();
        }
    }
}

void DialogInterface::colourReporter()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_ReporterColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (colour.isValid())
    {
        ui->pushButtonReporterColour->setStyleSheet(Color_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_ReporterColour = colour;

        if (gSimulation)
        {
            std::map<std::string, Reporter *> *reporterList = gSimulation->GetReporterList();
            std::map<std::string, Reporter *>::const_iterator reporterIter;
            for (reporterIter = reporterList->begin(); reporterIter != reporterList->end(); reporterIter++) reporterIter->second->SetColour(m_ReporterColour.redF(), m_ReporterColour.greenF(), m_ReporterColour.blueF(), m_ReporterColour.alphaF());
            gSimulation->GetInterface()->ReporterColour.SetColour(m_ReporterColour.redF(), m_ReporterColour.greenF(), m_ReporterColour.blueF(), m_ReporterColour.alphaF());
            emit EmitRedrawRequired();
        }
    }
}

void DialogInterface::colourDataTarget()
{
    const QString Color_STYLE("QPushButton { background-color : %1; color : %2; border: 4px solid %3; }");

    QColor colour;
    colour = QColorDialog::getColor(m_DataTargetColour, this, "Select Color", QColorDialog::ShowAlphaChannel /* | QColorDialog::DontUseNativeDialog */);
    if (colour.isValid())
    {
        ui->pushButtonDataTargetColour->setStyleSheet(Color_STYLE.arg(colour.name()).arg(getIdealTextColour(colour).name()).arg(getAlphaColourHint(colour).name()));
        m_DataTargetColour = colour;

        if (gSimulation)
        {
            std::map<std::string, DataTarget *> *reporterList = gSimulation->GetDataTargetList();
            std::map<std::string, DataTarget *>::const_iterator reporterIter;
            for (reporterIter = reporterList->begin(); reporterIter != reporterList->end(); reporterIter++) reporterIter->second->SetColour(m_DataTargetColour.redF(), m_DataTargetColour.greenF(), m_DataTargetColour.blueF(), m_DataTargetColour.alphaF());
            gSimulation->GetInterface()->DataTargetColour.SetColour(m_DataTargetColour.redF(), m_DataTargetColour.greenF(), m_DataTargetColour.blueF(), m_DataTargetColour.alphaF());
            emit EmitRedrawRequired();
        }
    }
}

// return an ideal label Color, based on the given background Color.
// Based on http://www.codeproject.com/cs/media/IdealTextColor.asp
QColor DialogInterface::getIdealTextColour(const QColor& rBackgroundColor)
{
    const int THRESHOLD = 105;
    int BackgroundDelta = (rBackgroundColor.red() * 0.299) + (rBackgroundColor.green() * 0.587) + (rBackgroundColor.blue() * 0.114);
    return QColor((255- BackgroundDelta < THRESHOLD) ? Qt::black : Qt::white);
}

QColor DialogInterface::getAlphaColourHint(const QColor& Color)
{
    // (source * Blend.SourceAlpha) + (background * Blend.InvSourceAlpha)
    QColor background;
    background.setRgbF(1.0, 1.0, 1.0);
    QColor hint;
    hint.setRedF((Color.redF() * Color.alphaF()) + (background.redF() * (1 - Color.alphaF())));
    hint.setGreenF((Color.greenF() * Color.alphaF()) + (background.greenF() * (1 - Color.alphaF())));
    hint.setBlueF((Color.blueF() * Color.alphaF()) + (background.blueF() * (1 - Color.alphaF())));
    return hint;
}



