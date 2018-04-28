#ifndef CUSTOMFILEDIALOGS_H
#define CUSTOMFILEDIALOGS_H

#include <QString>
#include <QFileDialog>
#include <QStringList>
#include <QWidget>

class CustomFileDialogs
{
public:
    static QString getOpenFileName(QWidget * parent = 0, const QString & caption = QString(), const QString & dir = QString(),
                                   const QString & filter = QString(), QString * selectedFilter = 0, QFileDialog::Options options = 0);
    static QStringList getOpenFileNames(QWidget * parent = 0, const QString & caption = QString(), const QString & dir = QString(),
                                       const QString & filter = QString(), QString * selectedFilter = 0, QFileDialog::Options options = 0);
    static QString getExistingDirectory(QWidget * parent = 0, const QString & caption = QString(), const QString & dir = QString(),
                                        QFileDialog::Options options = QFileDialog::ShowDirsOnly);
    static QString getSaveFileName(QWidget * parent = 0, const QString & caption = QString(), const QString & dir = QString(),
                                   const QString & filter = QString(), QString * selectedFilter = 0, QFileDialog::Options options = 0);

private:
    static bool m_forceNativeDialogs;
};

#endif // CUSTOMFILEDIALOGS_H
