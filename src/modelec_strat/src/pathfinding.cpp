#include <modelec_strat/pathfinding.hpp>

namespace Modelec {

    struct AStarNode
    {
        int x, y;
        double g = std::numeric_limits<double>::infinity();  // Cost from start
        double f = std::numeric_limits<double>::infinity();  // g + heuristic
        int parent_x = -1;
        int parent_y = -1;

        bool operator>(const AStarNode& other) const
        {
            return f > other.f;
        }
    };

    double heuristic(int x1, int y1, int x2, int y2)
    {
        return std::hypot(x1 - x2, y1 - y2);  // Euclidean distance
    }

    Pathfinding::Pathfinding()
    {
    }

    Pathfinding::Pathfinding(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        robot_length_mm_ = 300.0f;
        robot_width_mm_ = 300.0f;

        grid_width_ = 300;
        grid_height_ = 200;

        if (!LoadObstaclesFromXML("/home/acki/ros2_ws/src/modelec_strat/map/obstacles.xml"))
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to load obstacles from XML");
        }

        map_pub_ = node_->create_publisher<modelec_interfaces::msg::Obstacle>(
            "nav/obstacle", 40);

        ask_obstacle_srv_ = node_->create_service<std_srvs::srv::Empty>(
            "nav/ask_map_obstacle",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                const std::shared_ptr<std_srvs::srv::Empty::Response> response) {
                RCLCPP_INFO(node_->get_logger(), "Ask obstacle request received");
                HandleAskObstacleRequest(request, response);
            });

        map_srv_ = node_->create_service<modelec_interfaces::srv::Map>(
            "nav/map",
            [this](const std::shared_ptr<modelec_interfaces::srv::Map::Request> request,
                const std::shared_ptr<modelec_interfaces::srv::Map::Response> response) {
                RCLCPP_INFO(node_->get_logger(), "Map request received");
                HandleMapRequest(request, response);
            });

        map_size_srv_ = node_->create_service<modelec_interfaces::srv::MapSize>(
            "nav/map_size",
            [this](const std::shared_ptr<modelec_interfaces::srv::MapSize::Request> request,
                const std::shared_ptr<modelec_interfaces::srv::MapSize::Response> response) {
                RCLCPP_INFO(node_->get_logger(), "Map size request received");
                HandleMapSizeRequest(request, response);
            });

    }

    rclcpp::Node::SharedPtr Pathfinding::getNode() const
    {
        return node_;
    }

    WaypointListMsg Pathfinding::FindPath(const PosMsg::SharedPtr& start, const PosMsg::SharedPtr& goal)
    {
        WaypointListMsg waypoints;

        // Constantes de ta map
        constexpr float map_width_mm = 3000.0f;  // mm
        constexpr float map_height_mm = 2000.0f; // mm
        const float cell_size_mm_x = map_width_mm / grid_width_;
        const float cell_size_mm_y = map_height_mm / grid_height_;

        // Robot dimensions
        const int inflate_x = std::ceil((robot_width_mm_ / 2.0f) / cell_size_mm_x);
        const int inflate_y = std::ceil((robot_length_mm_ / 2.0f) / cell_size_mm_y);

        // 1. Build fresh empty grid
        grid_.clear();
        grid_.resize(grid_height_);
        for (auto& row : grid_)
        {
            row.assign(grid_width_, 0); // 0 = free
        }

        // 2. Fill obstacles with inflation
        for (const auto& [id, obstacle] : obstacle_map_)
        {
            int x_start = std::max(0, (int)(obstacle.x / cell_size_mm_x) - inflate_x);
            int y_start = std::max(0, (int)(obstacle.y / cell_size_mm_y) - inflate_y);
            int x_end = std::min(grid_width_ - 1, (int)((obstacle.x + obstacle.width) / cell_size_mm_x) + inflate_x);
            int y_end = std::min(grid_height_ - 1, (int)((obstacle.y + obstacle.height) / cell_size_mm_y) + inflate_y);

            // Inverser l'axe Y
            y_start = (grid_height_ - 1) - y_start;
            y_end = (grid_height_ - 1) - y_end;
            if (y_start > y_end) std::swap(y_start, y_end);

            for (int y = y_start; y <= y_end; ++y)
            {
                for (int x = x_start; x <= x_end; ++x)
                {
                    grid_[y][x] = 1; // mark as obstacle
                }
            }
        }

        // 3. Convert start and goal (avec inversion Y)
        const int start_x = start->x / cell_size_mm_x;
        const int start_y = (grid_height_ - 1) - (start->y / cell_size_mm_y);
        const int goal_x = goal->x / cell_size_mm_x;
        const int goal_y = (grid_height_ - 1) - (goal->y / cell_size_mm_y);

        if (start_x < 0 || start_y < 0 || goal_x < 0 || goal_y < 0 ||
            start_x >= grid_width_ || start_y >= grid_height_ ||
            goal_x >= grid_width_ || goal_y >= grid_height_)
        {
            RCLCPP_WARN(node_->get_logger(), "Start or Goal out of bounds");
            return waypoints;
        }

        if (grid_[start_y][start_x] == 1 || grid_[goal_y][goal_x] == 1)
        {
            RCLCPP_WARN(node_->get_logger(), "Start or Goal inside an obstacle");
            return waypoints;
        }

        // 4. A* algorithm
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
        std::unordered_map<int64_t, AStarNode> all_nodes;

        auto hash = [](int x, int y) {
            return (int64_t)x << 32 | (uint32_t)y;
        };

        auto neighbors = [](int x, int y) {
            return std::vector<std::pair<int, int>>{
                {x+1, y}, {x-1, y}, {x, y+1}, {x, y-1},
                {x+1, y+1}, {x-1, y+1}, {x+1, y-1}, {x-1, y-1} // diagonals
            };
        };

        AStarNode start_node{start_x, start_y};
        start_node.g = 0;
        start_node.f = heuristic(start_x, start_y, goal_x, goal_y);

        open_list.push(start_node);
        all_nodes[hash(start_x, start_y)] = start_node;

        bool path_found = false;

        while (!open_list.empty())
        {
            AStarNode current = open_list.top();
            open_list.pop();

            if (current.x == goal_x && current.y == goal_y)
            {
                path_found = true;
                break;
            }

            for (const auto& [nx, ny] : neighbors(current.x, current.y))
            {
                if (nx < 0 || ny < 0 || ny >= (int)grid_.size() || nx >= (int)grid_[0].size())
                    continue;

                if (grid_[ny][nx] == 1)
                    continue;

                double tentative_g = current.g + heuristic(current.x, current.y, nx, ny);
                int64_t neighbor_hash = hash(nx, ny);

                if (all_nodes.find(neighbor_hash) == all_nodes.end() || tentative_g < all_nodes[neighbor_hash].g)
                {
                    AStarNode neighbor;
                    neighbor.x = nx;
                    neighbor.y = ny;
                    neighbor.g = tentative_g;
                    neighbor.f = tentative_g + heuristic(nx, ny, goal_x, goal_y);
                    neighbor.parent_x = current.x;
                    neighbor.parent_y = current.y;
                    all_nodes[neighbor_hash] = neighbor;
                    open_list.push(neighbor);
                }
            }
        }

        if (!path_found)
        {
            RCLCPP_WARN(node_->get_logger(), "No path found");
            return waypoints;
        }

        // 5. Reconstruct path
        std::vector<std::pair<int, int>> path;
        int cx = goal_x;
        int cy = goal_y;

        while (!(cx == start_x && cy == start_y))
        {
            path.emplace_back(cx, cy);
            auto it = all_nodes.find(hash(cx, cy));
            if (it == all_nodes.end())
                break;
            cx = it->second.parent_x;
            cy = it->second.parent_y;
        }
        path.emplace_back(start_x, start_y);
        std::reverse(path.begin(), path.end());

        // 6. Path smoothing
        std::vector<std::pair<int, int>> smooth_path;
        size_t current = 0;
        while (current < path.size())
        {
            size_t next = current + 1;
            while (next < path.size())
            {
                bool clear = true;
                int x0 = path[current].first;
                int y0 = path[current].second;
                int x1 = path[next].first;
                int y1 = path[next].second;

                int dx = std::abs(x1 - x0);
                int dy = -std::abs(y1 - y0);
                int sx = x0 < x1 ? 1 : -1;
                int sy = y0 < y1 ? 1 : -1;
                int err = dx + dy;
                int x = x0;
                int y = y0;

                while (true)
                {
                    if (grid_[y][x] == 1)
                    {
                        clear = false;
                        break;
                    }
                    if (x == x1 && y == y1)
                        break;
                    int e2 = 2 * err;
                    if (e2 >= dy)
                    {
                        err += dy;
                        x += sx;
                    }
                    if (e2 <= dx)
                    {
                        err += dx;
                        y += sy;
                    }
                }

                if (!clear)
                    break;
                next++;
            }

            smooth_path.push_back(path[current]);
            if (next == path.size())
            {
                smooth_path.push_back(path.back());
                break;
            }
            current = next - 1;
        }

        // 7. Fill Waypoints (reconvertir grille -> millimètres)
        int id = 0;
        for (size_t i = 0; i < smooth_path.size(); ++i)
        {
            const auto& [x, y] = smooth_path[i];

            // Skip first point if it's too close to robot position
            if (i == 0)
            {
                float world_x = x * cell_size_mm_x;
                float world_y = (grid_height_ - 1 - y) * cell_size_mm_y;

                float dx = std::abs(world_x - start->x);
                float dy = std::abs(world_y - start->y);

                if (dx < 50 && dy < 50) // <== seuil de 50mm
                    continue;
            }

            WaypointMsg wp;
            wp.x = x * cell_size_mm_x;
            wp.y = (grid_height_ - 1 - y) * cell_size_mm_y;
            wp.theta = 0;
            wp.id = id++;
            wp.is_end = false;
            waypoints.push_back(wp);
        }
        if (!waypoints.empty())
        {
            waypoints.back().is_end = true;
        }

        return waypoints;
    }


    bool Pathfinding::LoadObstaclesFromXML(const std::string& filename)
    {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load obstacle XML file: %s", filename.c_str());
            return false;
        }

        tinyxml2::XMLElement* root = doc.FirstChildElement("Obstacles");
        if (!root)
        {
            RCLCPP_ERROR(node_->get_logger(), "No <Obstacles> root element in file");
            return false;
        }

        for (tinyxml2::XMLElement* obstacleElem = root->FirstChildElement("Obstacle");
             obstacleElem != nullptr;
             obstacleElem = obstacleElem->NextSiblingElement("Obstacle"))
        {
            modelec_interfaces::msg::Obstacle obs;

            if (obstacleElem->QueryIntAttribute("id", &obs.id) != tinyxml2::XML_SUCCESS ||
                obstacleElem->QueryIntAttribute("x", &obs.x) != tinyxml2::XML_SUCCESS ||
                obstacleElem->QueryIntAttribute("y", &obs.y) != tinyxml2::XML_SUCCESS ||
                obstacleElem->QueryIntAttribute("width", &obs.width) != tinyxml2::XML_SUCCESS ||
                obstacleElem->QueryIntAttribute("height", &obs.height) != tinyxml2::XML_SUCCESS)
            {
                RCLCPP_WARN(node_->get_logger(), "Incomplete obstacle definition in XML, skipping one obstacle");
                continue;
            }

            obstacle_map_[obs.id] = obs;
        }

        RCLCPP_INFO(node_->get_logger(), "Loaded %zu obstacles from XML", obstacle_map_.size());
        return true;
    }

    void Pathfinding::HandleMapRequest(const std::shared_ptr<modelec_interfaces::srv::Map::Request>,
                                       const std::shared_ptr<modelec_interfaces::srv::Map::Response> response)
    {
        response->width = grid_[0].size();
        response->height = grid_.size();
        response->map = std::vector<int>(grid_.size() * grid_[0].size());
        for (size_t i = 0; i < grid_.size(); i++)
        {
            for (size_t j = 0; j < grid_[i].size(); j++)
            {
                response->map[i * grid_[i].size() + j] = grid_[i][j];
            }
        }
    }

    void Pathfinding::HandleMapSizeRequest(const std::shared_ptr<modelec_interfaces::srv::MapSize::Request>,
        const std::shared_ptr<modelec_interfaces::srv::MapSize::Response> response)
    {
        response->width = grid_width_;
        response->height = grid_height_;
    }

    void Pathfinding::HandleAskObstacleRequest(const std::shared_ptr<std_srvs::srv::Empty::Request>,
        const std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        for (auto & [index, obs] : obstacle_map_)
        {
            map_pub_->publish(obs);
        }
    }

    Waypoint::Waypoint(const modelec_interfaces::msg::OdometryPos& pos, int index, bool isLast)
    {
        id = index;
        x = pos.x;
        y = pos.y;
        theta = pos.theta;
        is_end = isLast;
        reached = false;
    }

    Waypoint::Waypoint(const WaypointMsg& waypoint)
    {
        id = waypoint.id;
        x = waypoint.x;
        y = waypoint.y;
        theta = waypoint.theta;
        is_end = waypoint.is_end;
        reached = false;
    }

    WaypointMsg Waypoint::toMsg() const
    {
        return static_cast<OdometryAddWaypoint_<std::allocator<void>>>(*this);
    }
}
