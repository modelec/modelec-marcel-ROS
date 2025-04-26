#include "modelec_gui/modelec_gui.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QMenuBar>
#include <utility>
#include <modelec_gui/pages/home_page.hpp>
#include <modelec_gui/pages/map_page.hpp>
#include <modelec_gui/pages/test_page.hpp>

namespace ModelecGUI {

    ROS2QtGUI::ROS2QtGUI(rclcpp::Node::SharedPtr node, QWidget *parent)
        : QMainWindow(parent), node_(std::move(node)), stackedWidget(new QStackedWidget(this)) {

        // Add pages to stack
        stackedWidget->addWidget(new HomePage(this));
        stackedWidget->addWidget(new TestPage(get_node(), this));
        stackedWidget->addWidget(new MapPage(get_node(), this));
        setCentralWidget(stackedWidget);

        setupMenu();

        resize(1000, 700);
    }

    void ROS2QtGUI::setupMenu() {
        QMenuBar *menuBar = this->menuBar();

        QMenu *optionsMenu = menuBar->addMenu("View");

        home_action_ = new QAction("Home", this);
        test_action_ = new QAction("Test", this);
        map_action_ = new QAction("Map", this);

        playmat_map_menu_ = new QMenu("playmat");
        playmat_map_ = new QAction("Map", this);
        playmat_grid_ = new QAction("Grid", this);

        toggle_show_obstacle_action_ = new QAction("Toggle Show Obstacle", this);

        exit_action_ = new QAction("Exit", this);

        optionsMenu->addAction(home_action_);
        optionsMenu->addAction(test_action_);
        optionsMenu->addSeparator();
        optionsMenu->addAction(map_action_);
        optionsMenu->addMenu(playmat_map_menu_);
        playmat_map_menu_->addAction(playmat_map_);
        playmat_map_menu_->addAction(playmat_grid_);
        optionsMenu->addAction(toggle_show_obstacle_action_);
        optionsMenu->addSeparator();
        optionsMenu->addAction(exit_action_);

        connect(home_action_, &QAction::triggered, this, [this]() {
            stackedWidget->setCurrentIndex(0);
        });

        connect(test_action_, &QAction::triggered, this, [this]() {
            stackedWidget->setCurrentIndex(1);
        });

        connect(map_action_, &QAction::triggered, this, [this]() {
            stackedWidget->setCurrentIndex(2);
        });

        connect(playmat_map_, &QAction::triggered, this, [this]() {
            auto map_page = dynamic_cast<MapPage *>(stackedWidget->currentWidget());
            if (map_page) {
                map_page->setPlaymatMap();
            }
        });

        connect(playmat_grid_, &QAction::triggered, this, [this]() {
            auto map_page = dynamic_cast<MapPage *>(stackedWidget->currentWidget());
            if (map_page) {
                map_page->setPlaymatGrid();
            }
        });

        connect(toggle_show_obstacle_action_, &QAction::triggered, this, [this]() {
            auto map_page = dynamic_cast<MapPage *>(stackedWidget->currentWidget());
            if (map_page)
            {
                map_page->toggleShowObstacle();
            }
        });

        connect(exit_action_, &QAction::triggered, this, [this]() {
            close();
        });
    }

    ROS2QtGUI::~ROS2QtGUI() = default;

}
