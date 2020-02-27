#include <iostream>
#include <fstream>
#include <sstream>
#include <istream>

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rione_msgs/msg/command.hpp>
#include <rione_msgs/msg/location.hpp>
#include <rione_msgs/srv/request_location.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "picojson/picojson.h"

using namespace std;

class LocationRegister :
    public rclcpp::Node 
{
        private:
            rclcpp::Service<rione_msgs::srv::RequestLocation>::SharedPtr _request_service;

            void handleService_(
                const shared_ptr<rmw_request_id_t> request_header,
                const shared_ptr<rione_msgs::srv::RequestLocation::Request> request,
                const shared_ptr<rione_msgs::srv::RequestLocation::Response> response
            );

            short check_command(string command);

            vector<rione_msgs::msg::Location> load_location(string file_name);

            bool save_data(string data, string file);

            bool regist_location(vector<rione_msgs::msg::Location> locations, string file);

            string location2json(rione_msgs::msg::Location location, string file);

            vector<rione_msgs::msg::Location> get_location(vector<rione_msgs::msg::Location> locations, string file);

            vector<rione_msgs::msg::Location> get_all_location(string file);

            rione_msgs::msg::Location analyze_content(string content);

            geometry_msgs::msg::Point json2coordinate(picojson::value value);

            vector<rione_msgs::msg::Location> search_position(vector<rione_msgs::msg::Location> locations, vector<rione_msgs::msg::Location> search);

            bool send_goal_location(rione_msgs::msg::Location location);

            string log;
            geometry_msgs::msg::PoseStamped goal_position = geometry_msgs::msg::PoseStamped();

            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_location_publisher;

        public:
            LocationRegister();
};
