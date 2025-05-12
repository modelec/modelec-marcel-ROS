#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_strat/pathfinding.hpp>
#include <modelec_strat/obstacle/column.hpp>
#include <modelec_utils/config.hpp>

namespace Modelec
{
    struct AStarNode
    {
        int x, y;
        double g = std::numeric_limits<double>::infinity(); // Cost from start
        double f = std::numeric_limits<double>::infinity(); // g + heuristic
        int parent_x = -1;
        int parent_y = -1;

        bool operator>(const AStarNode& other) const
        {
            return f > other.f;
        }
    };

    double heuristic(int x1, int y1, int x2, int y2)
    {
        return std::hypot(x1 - x2, y1 - y2); // Euclidean distance
    }

    Pathfinding::Pathfinding()
    {
    }

    Pathfinding::Pathfinding(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
        map_width_mm_ = Config::get<int>("config.map.size.map_width_mm", 3000);
        map_height_mm_ = Config::get<int>("config.map.size.map_height_mm", 2000);

        robot_length_mm_ = Config::get<int>("config.robot.size.length_mm", 300);
        robot_width_mm_ = Config::get<int>("config.robot.size.width_mm", 300);
        margin_mm_ = Config::get<int>("config.robot.size.margin_mm", 100);

        enemy_length_mm_ = Config::get<int>("config.enemy.size.length_mm", 300);
        enemy_width_mm_ = Config::get<int>("config.enemy.size.width_mm", 300);

        enemy_margin_mm_ = Config::get<int>("config.enemy.size.margin_mm", 50);

        factor_close_enemy_ = Config::get<float>("config.enemy.factor_close_enemy", -0.5f);

        grid_width_ = Config::get<int>("config.map.size.grid_width", 300);
        grid_height_ = Config::get<int>("config.map.size.grid_height", 200);

        std::string obstacles_path = ament_index_cpp::get_package_share_directory("modelec_strat") +
            "/data/obstacles.xml";
        if (!LoadObstaclesFromXML(obstacles_path))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load obstacles from XML");
        }

        obstacle_add_sub_ = node_->create_subscription<modelec_interfaces::msg::Obstacle>(
            "obstacle/add", 10,
            [this](const modelec_interfaces::msg::Obstacle::SharedPtr msg)
            {
                RCLCPP_INFO(node_->get_logger(), "Obstacle add request received");
                AddObstacle(std::make_shared<Obstacle>(*msg));
            });

        obstacle_add_pub_ = node_->create_publisher<modelec_interfaces::msg::Obstacle>(
            "nav/obstacle/added", 40);

        obstacle_remove_sub_ = node_->create_subscription<modelec_interfaces::msg::Obstacle>(
            "obstacle/remove", 10,
            [this](const modelec_interfaces::msg::Obstacle::SharedPtr msg)
            {
                RCLCPP_INFO(node_->get_logger(), "Obstacle remove request received");
                RemoveObstacle(msg->id);
            });

        obstacle_remove_pub_ = node_->create_publisher<modelec_interfaces::msg::Obstacle>(
            "nav/obstacle/removed", 40);

        ask_obstacle_srv_ = node_->create_service<std_srvs::srv::Empty>(
            "nav/ask_map_obstacle",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   const std::shared_ptr<std_srvs::srv::Empty::Response> response)
            {
                RCLCPP_INFO(node_->get_logger(), "Ask obstacle request received");
                HandleAskObstacleRequest(request, response);
            });

        map_srv_ = node_->create_service<modelec_interfaces::srv::Map>(
            "nav/map",
            [this](const std::shared_ptr<modelec_interfaces::srv::Map::Request> request,
                   const std::shared_ptr<modelec_interfaces::srv::Map::Response> response)
            {
                RCLCPP_INFO(node_->get_logger(), "Map request received");
                HandleMapRequest(request, response);
            });

        map_size_srv_ = node_->create_service<modelec_interfaces::srv::MapSize>(
            "nav/map_size",
            [this](const std::shared_ptr<modelec_interfaces::srv::MapSize::Request> request,
                   const std::shared_ptr<modelec_interfaces::srv::MapSize::Response> response)
            {
                RCLCPP_INFO(node_->get_logger(), "Map size request received");
                HandleMapSizeRequest(request, response);
            });

        waypoint_pub_ = node_->create_publisher<WaypointMsg>(
            "odometry/add_waypoint", 100);
    }

    rclcpp::Node::SharedPtr Pathfinding::getNode() const
    {
        return node_;
    }

    /*bool Pathfinding::EnemyOnPath(const modelec_interfaces::msg::OdometryPos& enemy_pos)
    {
        int ex = enemy_pos.x;
        int ey = enemy_pos.y;

        for (const auto& wp : current_waypoints_)
        {
            int wx = wp.x;
            int wy = wp.y;

            if (std::abs(wx - ex) + std::abs(wy - ey) <= 300)
                return true;
        }
        return false;
    }*/

    /*void Pathfinding::Replan()
    {
        if (!current_start_ || !current_goal_)
            return;

        current_waypoints_ = FindPath(current_start_, current_goal_);

        for (const auto& wp : current_waypoints_)
        {
            waypoint_pub_->publish(wp);
        }
    }*/

    std::pair<int, WaypointListMsg> Pathfinding::FindPath(const PosMsg::SharedPtr& start, const PosMsg::SharedPtr& goal,
                                                          bool isClose, int collisionMask)
    {
        if (!start || !goal)
        {
            RCLCPP_WARN(node_->get_logger(), "Start or Goal position is null");
            return {-3, WaypointListMsg()};
        }

        WaypointListMsg waypoints;

        const float cell_size_mm_x = map_width_mm_ / grid_width_;
        const float cell_size_mm_y = map_height_mm_ / grid_height_;

        // Robot dimensions
        int inflate_x;
        int inflate_y;

        if (isClose)
        {
            inflate_x = std::ceil(((robot_width_mm_) / 2.0f) / cell_size_mm_x);
            inflate_y = std::ceil(((robot_length_mm_) / 2.0f) / cell_size_mm_y);
        }
        else
        {
            inflate_x = std::ceil(((robot_width_mm_ + margin_mm_) / 2.0f) / cell_size_mm_x);
            inflate_y = std::ceil(((robot_length_mm_ + margin_mm_) / 2.0f) / cell_size_mm_y);
        }

        // 1. Build fresh empty grid
        grid_.clear();
        grid_.resize(grid_height_);
        for (auto& row : grid_)
        {
            row.assign(grid_width_, FREE);
        }

        if (has_enemy_pos_)
        {
            int ex = (last_enemy_pos_.x / cell_size_mm_x);
            int ey = ((map_height_mm_ - last_enemy_pos_.y) / cell_size_mm_y);

            const int inflate_enemy_x = std::ceil((enemy_margin_mm_ + (enemy_width_mm_ / 2)) / cell_size_mm_x) +
                inflate_x;
            const int inflate_enemy_y = std::ceil((enemy_margin_mm_ + (enemy_length_mm_ / 2)) / cell_size_mm_y) +
                inflate_y;

            for (int y = ey - inflate_enemy_y; y <= ey + inflate_enemy_y; ++y)
            {
                for (int x = ex - inflate_enemy_x; x <= ex + inflate_enemy_x; ++x)
                {
                    if (x >= 0 && y >= 0 && x < grid_width_ && y < grid_height_)
                    {
                        grid_[y][x] |= ENEMY;
                    }
                }
            }
        }

        // Bord gauche et droit
        for (int y = 0; y < grid_height_; ++y)
        {
            for (int x = 0; x < inflate_x; ++x)
            {
                grid_[y][x] |= WALL;
                grid_[y][grid_width_ - 1 - x] |= WALL;
            }
        }

        // Bord haut et bas
        for (int x = 0; x < grid_width_; ++x)
        {
            for (int y = 0; y < inflate_y; ++y)
            {
                grid_[y][x] |= WALL;
                grid_[grid_height_ - 1 - y][x] |= WALL;
            }
        }

        // 2. Fill obstacles with inflation
        // TODO some bug exist with the inflate
        for (const auto& [id, obstacle] : obstacle_map_)
        {
            float cx = obstacle->x();
            float cy = obstacle->y();
            float width = obstacle->width() + inflate_x * 2 * cell_size_mm_x;
            float height = obstacle->height() + inflate_y * 2 * cell_size_mm_y;
            float theta = M_PI_2 - obstacle->theta();

            float dx = width / 2.0f;
            float dy = height / 2.0f;

            // Compute corners in local space and rotate+translate to world
            std::vector<std::pair<float, float>> corners = {
                {-dx, -dy}, {dx, -dy}, {dx, dy}, {-dx, dy}
            };

            for (auto& [x, y] : corners)
            {
                float rx = x * std::cos(theta) - y * std::sin(theta);
                float ry = x * std::sin(theta) + y * std::cos(theta);
                x = rx + cx;
                y = ry + cy;
            }

            // Compute bounding box in grid space
            float min_x = corners[0].first;
            float max_x = corners[0].first;
            float min_y = corners[0].second;
            float max_y = corners[0].second;

            for (const auto& [x, y] : corners)
            {
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
            }

            int x_start = std::max(0, (int)(min_x / cell_size_mm_x));
            int x_end = std::min(grid_width_ - 1, (int)(max_x / cell_size_mm_x));
            int y_start = std::max(0, (int)(min_y / cell_size_mm_y));
            int y_end = std::min(grid_height_ - 1, (int)(max_y / cell_size_mm_y));

            // Mark cells that fall inside rotated rectangle
            for (int y = y_start; y <= y_end; ++y)
            {
                for (int x = x_start; x <= x_end; ++x)
                {
                    // Convert cell center to world space
                    float wx = (x + 0.5f) * cell_size_mm_x;
                    float wy = (y + 0.5f) * cell_size_mm_y;

                    // Inverse transform: world -> obstacle local space
                    float dx_ = wx - cx;
                    float dy_ = wy - cy;

                    float lx = dx_ * std::cos(-theta) - dy_ * std::sin(-theta);
                    float ly = dx_ * std::sin(-theta) + dy_ * std::cos(-theta);

                    if (std::abs(lx) <= dx + 1 && std::abs(ly) <= dy + 1)
                    {
                        int gy = (grid_height_ - 1) - y;
                        if (x >= 0 && gy >= 0 && x < grid_width_ && gy < grid_height_)
                        {
                            grid_[gy][x] |= OBSTACLE;
                        }
                    }
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
            return {-2, waypoints};
        }

        if (!TestCollision(start_x, start_y, collisionMask) || !TestCollision(goal_x, goal_y, collisionMask))
        {
            if (!TestCollision(start_x, start_y, collisionMask))
            {
                RCLCPP_WARN(node_->get_logger(), "Start inside an obstacle");
                return {grid_[start_y][start_x], waypoints};
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Goal inside an obstacle");
                return {grid_[goal_y][goal_x], waypoints};
            }
        }

        // 4. A* algorithm
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
        std::unordered_map<int64_t, AStarNode> all_nodes;

        auto hash = [](int x, int y)
        {
            return (int64_t)x << 32 | (uint32_t)y;
        };

        auto neighbors = [](int x, int y)
        {
            return std::vector<std::pair<int, int>>{
                {x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1},
                {x + 1, y + 1}, {x - 1, y + 1}, {x + 1, y - 1}, {x - 1, y - 1} // diagonals
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

                if (!TestCollision(nx, ny, collisionMask))
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
            return {-1, waypoints};
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
                    if (!TestCollision(x, y, collisionMask))
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
            wp.theta = i == smooth_path.size() - 1 ? goal->theta : 0.0f; // last waypoint takes goal theta
            wp.id = id++;
            wp.is_end = false;
            waypoints.push_back(wp);
        }
        if (!waypoints.empty())
        {
            waypoints.back().is_end = true;
        }

        return {FREE, waypoints};
    }

    void Pathfinding::SetCurrentPos(const PosMsg::SharedPtr& pos)
    {
        current_start_ = pos;
    }

    std::shared_ptr<Obstacle> Pathfinding::GetObstacle(int id) const
    {
        return obstacle_map_.at(id);
    }

    void Pathfinding::RemoveObstacle(int id)
    {
        obstacle_map_.erase(id);

        modelec_interfaces::msg::Obstacle msg;
        msg.id = id;
        obstacle_remove_pub_->publish(msg);
    }

    void Pathfinding::AddObstacle(const std::shared_ptr<Obstacle>& obstacle)
    {
        obstacle_map_[obstacle->id()] = obstacle;
        modelec_interfaces::msg::Obstacle msg = obstacle->toMsg();
        obstacle_add_pub_->publish(msg);
    }

    std::shared_ptr<ColumnObstacle> Pathfinding::GetClosestColumn(const PosMsg::SharedPtr& pos,
                                                                  const std::vector<int>& blacklistedId)
    {
        // TODO score (count dist and dist with enemy)
        std::shared_ptr<ColumnObstacle> closest_obstacle = nullptr;
        auto robotPos = Point(pos->x, pos->y, pos->theta);
        auto enemyPos = Point(last_enemy_pos_.x, last_enemy_pos_.y, last_enemy_pos_.theta);
        float score = std::numeric_limits<float>::max();

        for (const auto& [id, obstacle] : obstacle_map_)
        {
            if (auto obs = std::dynamic_pointer_cast<ColumnObstacle>(obstacle))
            {
                if (!obs->IsAtObjective() && std::find(blacklistedId.begin(), blacklistedId.end(), obs->id()) ==
                    blacklistedId.end())
                {
                    for (auto possiblePos : obs->GetAllPossiblePositions())
                    {
                        auto dist = Point::distance(robotPos, possiblePos);
                        auto distEnemy = Point::distance(enemyPos, possiblePos);

                        auto s = dist + distEnemy * factor_close_enemy_;

                        if (s < score)
                        {
                            score = s;
                            closest_obstacle = obs;
                        }
                    }
                }
            }
        }

        return closest_obstacle;
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
        for (auto& [index, obs] : obstacle_map_)
        {
            obstacle_add_pub_->publish(obs->toMsg());
        }
    }

    void Pathfinding::OnEnemyPosition(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        last_enemy_pos_ = *msg;
        has_enemy_pos_ = true;
    }

    void Pathfinding::OnEnemyPositionLongTime(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        Point enemyPos(msg->x, msg->y, msg->theta);
        for (auto& [index, obs] : obstacle_map_)
        {
            if (auto column = std::dynamic_pointer_cast<ColumnObstacle>(obs))
            {
                if (Point::distance(enemyPos, column->GetPosition()) < enemy_width_mm_ + (column->width() / 2) +
                    enemy_margin_mm_)
                {
                    RemoveObstacle(column->id());
                }
            }
        }
    }

    bool Pathfinding::TestCollision(int x, int y, int collisionMask)
    {
        if (y < 0 || y >= static_cast<int>(grid_.size()) ||
            x < 0 || x >= static_cast<int>(grid_[y].size()))
        {
            RCLCPP_WARN(node_->get_logger(), "TestCollision: access out of bounds x=%d y=%d", x, y);
            return false; // ou true, selon ce que tu veux (false = pas de collision)
        }

        return (grid_[y][x] & collisionMask) && !(grid_[y][x] & ~collisionMask);
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
            std::shared_ptr<Obstacle> obs = std::make_shared<Obstacle>(obstacleElem);
            obstacle_map_[obs->id()] = obs;
        }

        for (tinyxml2::XMLElement* obstacleElem = root->FirstChildElement("Gradin");
             obstacleElem != nullptr;
             obstacleElem = obstacleElem->NextSiblingElement("Gradin"))
        {
            std::shared_ptr<ColumnObstacle> obs = std::make_shared<ColumnObstacle>(obstacleElem);
            obstacle_map_[obs->id()] = obs;
        }

        RCLCPP_INFO(node_->get_logger(), "Loaded %zu obstacles from XML", obstacle_map_.size());
        return true;
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
