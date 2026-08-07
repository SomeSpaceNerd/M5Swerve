#include "logger.h"

void logMessage(QTextBrowser *browser, QString level, QString caller, QString message)
{
    if (!browser)
        return;

    QColor color;
    if (level == "DEBUG") {color = QColor("magenta");}
    else if (level == "INFO") {color = QColor("white");}
    else if (level == "WARNING") {color = QColor("yellow");}
    else if (level == "ERROR") {color = QColor("red");}
    else if (level == "FATAL") {color = QColor("darkRed");}
    else {color = QColor("lightBlue");}

    QString formatted = QString("[%1] %2").arg(caller.toHtmlEscaped(), message.toHtmlEscaped());
    QString html = QString("<span style='color:%1;'>%2</span>").arg(color.name(), formatted);

    browser->append(html);
    qDebug() << formatted;

    // Force scroll to bottom
    QScrollBar *sb = browser->verticalScrollBar();
    sb->setValue(sb->maximum());
}

