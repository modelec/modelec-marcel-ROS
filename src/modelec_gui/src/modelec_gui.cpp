#include "modelec_gui/modelec_gui.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QMenuBar>
#include <utility>


namespace ModelecGUI {

    ROS2QtGUI::ROS2QtGUI(rclcpp::Node::SharedPtr node, QWidget *parent)
        : QMainWindow(parent), node_(std::move(node)), stackedWidget(new QStackedWidget(this)) {

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto client = node_->create_client<std_srvs::srv::Empty>("enemy_manager/ping");
        int timeout = 3;
        while (!client->wait_for_service(std::chrono::seconds(1)) && timeout-- > 0)
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
        }

        if (timeout <= 0)
        {
            // Quit app
            this->close();
            return;
        }

        // Add pages to stack
        resize(1200, 800);

        home_page_ = new HomePage(get_node(), this);
        odo_page_ = new OdoPage(get_node(), this);
        test_map_page_ = new TestMapPage(get_node(), this);
        map_page_ = new MapPage(get_node(), this);
        action_page_ = new ActionPage(get_node(), this);
        alim_page_ = new AlimPage(get_node(), this);

        stackedWidget->addWidget(home_page_);
        stackedWidget->addWidget(odo_page_);
        stackedWidget->addWidget(test_map_page_);
        stackedWidget->addWidget(map_page_);
        stackedWidget->addWidget(action_page_);
        stackedWidget->addWidget(alim_page_);
        setCentralWidget(stackedWidget);

        setupMenu();

        connect(home_page_, &HomePage::TeamChoose, this, [this]()
        {
            stackedWidget->setCurrentIndex(3);
        });
    }

    void ROS2QtGUI::setupMenu() {
        QMenuBar *menuBar = this->menuBar();

        QMenu *optionsMenu = menuBar->addMenu("View");

        home_action_ = new QAction("Home", this);
        odo_action_ = new QAction("Odometrie", this);
        action_action_ = new QAction("Action", this);
        alim_action_ = new QAction("Alim", this);
        map_action_ = new QAction("Map", this);

        playmat_map_menu_ = new QMenu("playmat");
        playmat_map_ = new QAction("Map", this);
        playmat_grid_ = new QAction("Grid", this);

        toggle_show_obstacle_action_ = new QAction("Toggle Show Obstacle", this);

        exit_action_ = new QAction("Exit", this);

        optionsMenu->addAction(home_action_);
        optionsMenu->addAction(odo_action_);
        optionsMenu->addAction(action_action_);
        optionsMenu->addAction(alim_action_);
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
            home_page_->Init();
            map_page_->Reset();
        });

        connect(odo_action_, &QAction::triggered, this, [this]() {
            stackedWidget->setCurrentIndex(1);
        });

        connect(action_action_, &QAction::triggered, this, [this]() {
            stackedWidget->setCurrentIndex(4);
        });

        connect(alim_action_, &QAction::triggered, this, [this]() {
            stackedWidget->setCurrentIndex(5);
        });

        connect(map_action_, &QAction::triggered, this, [this]() {
            stackedWidget->setCurrentIndex(2);
        });

        connect(playmat_map_, &QAction::triggered, this, [this]() {
            auto map_page = dynamic_cast<TestMapPage *>(stackedWidget->currentWidget());
            if (map_page) {
                map_page->setPlaymatMap();
            }
        });

        connect(playmat_grid_, &QAction::triggered, this, [this]() {
            auto map_page = dynamic_cast<TestMapPage *>(stackedWidget->currentWidget());
            if (map_page) {
                map_page->setPlaymatGrid();
            }
        });

        connect(toggle_show_obstacle_action_, &QAction::triggered, this, [this]() {
            auto map_page = dynamic_cast<TestMapPage *>(stackedWidget->currentWidget());
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
