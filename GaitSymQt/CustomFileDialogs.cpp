#include <QList>
#include <QUrl>
#include <QStandardPaths>
#include <QFileDialog>

#include "customfiledialogs.h"

bool CustomFileDialogs::m_forceNativeDialogs = true;

// this provides some hooks so I have a little more control over dialogs
// in particular I want to have specific things in the side bars
// and I want a global overide on native dialog use

QString CustomFileDialogs::getOpenFileName(QWidget *parent,
                                           const QString &caption,
                                           const QString &dir,
                                           const QString &filter,
                                           QString *selectedFilter,
                                           QFileDialog::Options options)
{
#ifndef WIN32
    if (m_forceNativeDialogs || !(options & QFileDialog::DontUseNativeDialog))
    {
        return QFileDialog::getOpenFileName(parent, caption, dir, filter, selectedFilter, options & (!QFileDialog::DontUseNativeDialog));
    }
#endif

    QFileDialog dialog(parent);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setFileMode(QFileDialog::ExistingFile);

#if defined(Q_OS_MAC)
    QList<QUrl> urls;
    urls << QUrl::fromLocalFile(QStandardPaths::writableLocation(QStandardPaths::HomeLocation))
         << QUrl::fromLocalFile(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation))
         << QUrl::fromLocalFile("/") << QUrl::fromLocalFile("/Volumes");
    dialog.setSidebarUrls(urls);
#endif

    dialog.setViewMode(QFileDialog::Detail);
    dialog.setWindowTitle(caption);
    dialog.setDirectory(dir);
    dialog.setNameFilter(filter);
    if (selectedFilter && !selectedFilter->isEmpty())
        dialog.selectNameFilter(*selectedFilter);
    if (dialog.exec() == QDialog::Accepted)
    {
        if (selectedFilter) *selectedFilter = dialog.selectedNameFilter();
        return dialog.selectedFiles().value(0);
    }
    return QString();
}

QStringList CustomFileDialogs::getOpenFileNames(QWidget *parent,
                                                const QString &caption,
                                                const QString &dir,
                                                const QString &filter,
                                                QString *selectedFilter,
                                                QFileDialog::Options options)
{
#ifndef WIN32
    if (m_forceNativeDialogs || !(options & QFileDialog::DontUseNativeDialog))
    {
        return QFileDialog::getOpenFileNames(parent, caption, dir, filter, selectedFilter, options & (!QFileDialog::DontUseNativeDialog));
    }
#endif

    QFileDialog dialog(parent);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setFileMode(QFileDialog::ExistingFiles);

#if defined(Q_OS_MAC)
    QList<QUrl> urls;
    urls << QUrl::fromLocalFile(QStandardPaths::writableLocation(QStandardPaths::HomeLocation))
         << QUrl::fromLocalFile(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation))
         << QUrl::fromLocalFile("/") << QUrl::fromLocalFile("/Volumes");
    dialog.setSidebarUrls(urls);
#endif

    dialog.setViewMode(QFileDialog::Detail);
    dialog.setWindowTitle(caption);
    dialog.setDirectory(dir);
    dialog.setNameFilter(filter);
    if (selectedFilter && !selectedFilter->isEmpty())
        dialog.selectNameFilter(*selectedFilter);
    if (dialog.exec() == QDialog::Accepted)
    {
        if (selectedFilter)
            *selectedFilter = dialog.selectedNameFilter();
        return dialog.selectedFiles();
    }
    return QStringList();
}

QString CustomFileDialogs::getExistingDirectory(QWidget *parent,
                                                const QString &caption,
                                                const QString &dir,
                                                QFileDialog::Options options)
{
#ifndef WIN32
    if (m_forceNativeDialogs || !(options & QFileDialog::DontUseNativeDialog))
    {
        return QFileDialog::getExistingDirectory(parent, caption, dir, options & (!QFileDialog::DontUseNativeDialog));
    }
#endif

    QFileDialog dialog(parent);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setOption(QFileDialog::ShowDirsOnly, true);
    dialog.setFileMode(QFileDialog::DirectoryOnly);

#if defined(Q_OS_MAC)
    QList<QUrl> urls;
    urls << QUrl::fromLocalFile(QStandardPaths::writableLocation(QStandardPaths::HomeLocation))
         << QUrl::fromLocalFile(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation))
         << QUrl::fromLocalFile("/") << QUrl::fromLocalFile("/Volumes");
    dialog.setSidebarUrls(urls);
#endif

    dialog.setViewMode(QFileDialog::Detail);
    dialog.setWindowTitle(caption);
    dialog.setDirectory(dir);
    if (dialog.exec() == QDialog::Accepted)
    {
        return dialog.selectedFiles().value(0);
    }
    return QString();
}

QString CustomFileDialogs::getSaveFileName(QWidget *parent,
                                           const QString &caption,
                                           const QString &dir,
                                           const QString &filter,
                                           QString *selectedFilter,
                                           QFileDialog::Options options)
{
#ifndef WIN32
    if (m_forceNativeDialogs || !(options & QFileDialog::DontUseNativeDialog))
    {
        return QFileDialog::getSaveFileName(parent, caption, dir, filter, selectedFilter, options & (!QFileDialog::DontUseNativeDialog));
    }
#endif

    QFileDialog dialog(parent);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setFileMode(QFileDialog::AnyFile);

#if defined(Q_OS_MAC)
    QList<QUrl> urls;
    urls << QUrl::fromLocalFile(QStandardPaths::writableLocation(QStandardPaths::HomeLocation))
         << QUrl::fromLocalFile(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation))
         << QUrl::fromLocalFile("/") << QUrl::fromLocalFile("/Volumes");
    dialog.setSidebarUrls(urls);
#endif

    dialog.setViewMode(QFileDialog::Detail);
    dialog.setWindowTitle(caption);
    dialog.setDirectory(dir);
    dialog.setNameFilter(filter);

    dialog.setAcceptMode(QFileDialog::AcceptSave);
    if (selectedFilter && !selectedFilter->isEmpty())
        dialog.selectNameFilter(*selectedFilter);
    if (dialog.exec() == QDialog::Accepted)
    {
        if (selectedFilter)
            *selectedFilter = dialog.selectedNameFilter();
        return dialog.selectedFiles().value(0);
    }
    return QString();
}

