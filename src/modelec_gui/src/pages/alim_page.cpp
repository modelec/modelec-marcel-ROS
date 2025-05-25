#include <modelec_gui/pages/alim_page.hpp>

namespace ModelecGUI
{
    AlimPage::AlimPage(rclcpp::Node::SharedPtr node, QWidget* parent) :
        QWidget(parent),
        node_(node)
    {
    }

    AlimPage::~AlimPage()
    = default;
}
