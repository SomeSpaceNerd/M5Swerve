#ifndef LOGGER_H
#define LOGGER_H

#include <QTextBrowser>
#include <QScrollBar>
#include <QDebug>

void logMessage(QTextBrowser *browser, QString level, QString caller, QString message);

#endif // LOGGER_H

