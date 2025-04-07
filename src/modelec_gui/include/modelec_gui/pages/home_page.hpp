#pragma once

#include <QWidget>
#include <QLabel>

namespace ModelecGUI {
class HomePage : public QWidget
{
    Q_OBJECT
public:
    HomePage(QWidget *parent = nullptr);

protected:
    QLayout* m_layout;
};
}