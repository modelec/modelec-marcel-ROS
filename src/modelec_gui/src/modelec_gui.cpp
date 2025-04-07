#include "modelec_gui/modelec_gui.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QMenuBar>
#include <utility>
#include <modelec_gui/pages/home_page.hpp>
#include <modelec_gui/pages/test_page.hpp>

namespace ModelecGUI {

    ROS2QtGUI::ROS2QtGUI(rclcpp::Node::SharedPtr node, QWidget *parent)
        : QMainWindow(parent), node_(std::move(node)), stackedWidget(new QStackedWidget(this)) {

        // Add pages to stack
        stackedWidget->addWidget(new HomePage(this));
        stackedWidget->addWidget(new TestPage(get_node(), this));
        setCentralWidget(stackedWidget);

        setupMenu();

        resize(800, 600);
    }

    void ROS2QtGUI::setupMenu() {
        QMenuBar *menuBar = this->menuBar();

        QMenu *optionsMenu = menuBar->addMenu("Options");

        home_action_ = new QAction("Home", this);
        test_action_ = new QAction("Test", this);
        exit_action_ = new QAction("Exit", this);

        optionsMenu->addAction(home_action_);
        optionsMenu->addAction(test_action_);
        optionsMenu->addSeparator();
        optionsMenu->addAction(exit_action_);

        connect(home_action_, &QAction::triggered, this, [this]() {
            stackedWidget->setCurrentIndex(0);
        });

        connect(test_action_, &QAction::triggered, this, [this]() {
            stackedWidget->setCurrentIndex(1);
        });

        connect(exit_action_, &QAction::triggered, this, [this]() {
            close();
        });
    }

    ROS2QtGUI::~ROS2QtGUI() = default;

}
