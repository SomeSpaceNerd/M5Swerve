#ifndef SETTINGSMANAGER_H
#define SETTINGSMANAGER_H

#include <QHash>
#include <QSettings>
#include <QString>

class SettingsManager
{
public:
    SettingsManager();

    bool persistInterfaceName(QString interface);
    bool persistRobotId(int id);
    bool persistControlIndex(QString name, int index);
    bool removeControlIndex(QString name);

    QString savedInterface();
    int savedRobotId();
    int savedControlIndex(QString name);
    QHash<QString, int> savedControls();

    void sync(); // Forces any pending writes out to disk right now

private:
    QHash<QString, int> readControls();
    bool writeControls(const QHash<QString, int> &controls);

    QSettings m_settings;
};

#endif // SETTINGSMANAGER_H

