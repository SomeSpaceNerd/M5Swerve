#include "settingsmanager.h"

namespace
{
constexpr char kInterfaceKey[]  = "network/interface";
constexpr char kRobotIdKey[]    = "network/robotId";
constexpr char kControlsGroup[] = "controls";
}

SettingsManager::SettingsManager() : m_settings("SomeSpaceNerd", "bellman Driver Station")
{
}

bool SettingsManager::persistInterfaceName(QString interface)
{
    m_settings.setValue(kInterfaceKey, interface);
    return m_settings.status() == QSettings::NoError;
}

bool SettingsManager::persistRobotId(int id)
{
    m_settings.setValue(kRobotIdKey, id);
    return m_settings.status() == QSettings::NoError;
}

bool SettingsManager::persistControlIndex(QString name, int index)
{
    if (name.isEmpty() || index < 0)
        return false;

    QHash<QString, int> controls = readControls();
    controls.insert(name, index);
    return writeControls(controls);
}

bool SettingsManager::removeControlIndex(QString name)
{
    QHash<QString, int> controls = readControls();
    if (controls.remove(name) == 0) {return true;}

    return writeControls(controls);
}

QString SettingsManager::savedInterface()
{
    return m_settings.value(kInterfaceKey, "").toString();
}

int SettingsManager::savedRobotId()
{
    return m_settings.value(kRobotIdKey, -1).toInt();
}

int SettingsManager::savedControlIndex(QString name)
{
    return readControls().value(name, -1);
}

QHash<QString, int> SettingsManager::savedControls()
{
    return readControls();
}

void SettingsManager::sync()
{
    m_settings.sync();
}

QHash<QString, int> SettingsManager::readControls()
{
    QHash<QString, int> result;

    int size = m_settings.beginReadArray(kControlsGroup);
    for (int i = 0; i < size; ++i)
    {
        m_settings.setArrayIndex(i);
        QString name = m_settings.value("name").toString();
        int index = m_settings.value("index", -1).toInt();
        if (!name.isEmpty() && index >= 0)
            result.insert(name, index);
    }
    m_settings.endArray();

    return result;
}

bool SettingsManager::writeControls(const QHash<QString, int> &controls)
{
    m_settings.remove(kControlsGroup); // clear the old array out before rewriting it

    m_settings.beginWriteArray(kControlsGroup);
    int i = 0;
    for (QHash<QString, int>::const_iterator it = controls.constBegin(); it != controls.constEnd(); ++it, ++i)
    {
        m_settings.setArrayIndex(i);
        m_settings.setValue("name", it.key());
        m_settings.setValue("index", it.value());
    }
    m_settings.endArray();

    return m_settings.status() == QSettings::NoError;
}

