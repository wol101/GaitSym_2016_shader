#include "dialogpreferences.h"
#include "ui_dialogpreferences.h"

#include <QString>
#include <QFileDialog>

#include "customfiledialogs.h"
#include "preferences.h"
#include "glwidget.h"

DialogPreferences::DialogPreferences(QWidget *parent) :
    QDialog(parent),
    m_ui(new Ui::DialogPreferences)
{
    m_ui->setupUi(this);

#ifndef __APPLE__
    // quicktime only available for Mac version
    m_ui->radioButtonQuicktime->setVisible(false);
#endif

}

DialogPreferences::~DialogPreferences()
{
    delete m_ui;
}

void DialogPreferences::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void DialogPreferences::graphicsPathBrowse()
{
    QString folder = CustomFileDialogs::getExistingDirectory(this, tr("Select Folder"), "", QFileDialog::DontUseNativeDialog);
    if (folder.isNull() == false)
    {
        m_ui->lineEditGraphicPath->setText(folder);
    }
}

void DialogPreferences::SetValues(const Preferences &prefs)
{
    m_ui->lineEditGraphicPath->setText(prefs.GraphicsPath);
    m_ui->lineEditMovieFolder->setText(prefs.MoviePath);

    m_ui->doubleSpinBoxFrontClip->setValue(prefs.CameraFrontClip);
    m_ui->doubleSpinBoxBackClip->setValue(prefs.CameraBackClip);
    m_ui->doubleSpinBoxCursorRadius->setValue(prefs.CursorRadius);
    m_ui->doubleSpinBoxNudge->setValue(prefs.Nudge);

    m_ui->spinBoxSkip->setValue(prefs.MovieSkip);
    m_ui->doubleSpinBoxFramerate->setValue(prefs.QuicktimeFramerate);

    m_ui->checkBoxYUp->setChecked(prefs.YUp);
    m_ui->checkBoxTracking->setChecked(prefs.DisplayTracking);
    m_ui->checkBoxOverlay->setChecked(prefs.DisplayOverlay);
    m_ui->checkBoxContactForce->setChecked(prefs.DisplayContactForces);
    m_ui->checkBoxMuscleForce->setChecked(prefs.DisplayMuscleForces);
    m_ui->checkBoxFramerate->setChecked(prefs.DisplayFramerate);
    m_ui->checkBoxActivationColours->setChecked(prefs.DisplayActivation);
    m_ui->checkBoxOpenCL->setChecked(prefs.OpenCLUseOpenCL);
    m_ui->checkBoxOpenCLLog->setChecked(prefs.OpenCLUseOpenCLLog);

    m_ui->radioButtonPPM->setChecked(static_cast<GLWidget::MovieFormat>(prefs.MovieFormat == GLWidget::PPM));
    m_ui->radioButtonTIFF->setChecked(static_cast<GLWidget::MovieFormat>(prefs.MovieFormat == GLWidget::TIFF));
    m_ui->radioButtonPOVRay->setChecked(static_cast<GLWidget::MovieFormat>(prefs.MovieFormat == GLWidget::POVRay));
    m_ui->radioButtonOBJ->setChecked(static_cast<GLWidget::MovieFormat>(prefs.MovieFormat == GLWidget::OBJ));

    m_ui->spinBoxOpenCLDeviceNumber->setValue(prefs.OpenCLDeviceDeviceNumber);
    m_ui->spinBoxOpenCLTargetPlatform->setValue(prefs.OpenCLTargetPlatform);
    m_ui->comboBoxOpenCLDevice->setCurrentIndex(m_ui->comboBoxOpenCLDevice->findText(prefs.OpenCLDeviceType));
}

void DialogPreferences::GetValues(Preferences *prefs)
{
    prefs->GraphicsPath = m_ui->lineEditGraphicPath->text();
    prefs->MoviePath = m_ui->lineEditMovieFolder->text();

    prefs->CameraFrontClip = m_ui->doubleSpinBoxFrontClip->value();
    prefs->CameraBackClip = m_ui->doubleSpinBoxBackClip->value();
    prefs->CursorRadius = m_ui->doubleSpinBoxCursorRadius->value();
    prefs->Nudge = m_ui->doubleSpinBoxNudge->value();

    prefs->MovieSkip = m_ui->spinBoxSkip->value();
    prefs->QuicktimeFramerate = m_ui->doubleSpinBoxFramerate->value();

    prefs->YUp = m_ui->checkBoxYUp->isChecked();
    prefs->DisplayTracking = m_ui->checkBoxTracking->isChecked();
    prefs->DisplayOverlay = m_ui->checkBoxOverlay->isChecked();
    prefs->DisplayContactForces = m_ui->checkBoxContactForce->isChecked();
    prefs->DisplayMuscleForces = m_ui->checkBoxMuscleForce->isChecked();
    prefs->DisplayFramerate = m_ui->checkBoxFramerate->isChecked();
    prefs->DisplayActivation = m_ui->checkBoxActivationColours->isChecked();
    prefs->OpenCLUseOpenCL = m_ui->checkBoxOpenCL->isChecked();
    prefs->OpenCLUseOpenCLLog = m_ui->checkBoxOpenCLLog->isChecked();

    if (m_ui->radioButtonPPM->isChecked()) prefs->MovieFormat = static_cast<int>(GLWidget::PPM);
    if (m_ui->radioButtonTIFF->isChecked()) prefs->MovieFormat = static_cast<int>(GLWidget::TIFF);
    if (m_ui->radioButtonPOVRay->isChecked()) prefs->MovieFormat = static_cast<int>(GLWidget::POVRay);
    if (m_ui->radioButtonOBJ->isChecked()) prefs->MovieFormat = static_cast<int>(GLWidget::OBJ);

    prefs->OpenCLDeviceDeviceNumber = m_ui->spinBoxOpenCLDeviceNumber->value();
    prefs->OpenCLTargetPlatform = m_ui->spinBoxOpenCLTargetPlatform->value();
    prefs->OpenCLDeviceType = m_ui->comboBoxOpenCLDevice->currentText();
}
