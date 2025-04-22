#include <modelec_gui/pages/home_page.hpp>

#include <QVBoxLayout>

namespace ModelecGUI
{

    HomePage::HomePage(QWidget* parent) : QWidget(parent)
    {
        // label with default text
        m_layout = new QVBoxLayout(this);
        m_layout->addWidget(new QLabel("Welcome to Modelec GUI!"));

        this->setLayout(m_layout);
    }

}