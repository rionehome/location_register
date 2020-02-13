#include <iostream>
#include <fstream>
#include <sstream>
#include <istream>

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rione_msgs/msg/command.hpp>
#include <rione_msgs/msg/location.hpp>
#include <rione_msgs/msg/request.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std;

class LocationRegister :
    public rclcpp::Node {
        private:

            rclcpp::Subscription<rione_msgs::msg::Request>::SharedPtr _register_current_position;
            rclcpp::Subscription<rione_msgs::msg::Request>::SharedPtr _request_location;
            rclcpp::Subscription<rione_msgs::msg::Request>::SharedPtr _request_current_location;
            rclcpp::Subscription<rione_msgs::msg::Request>::SharedPtr _request_location_list;

            rclcpp::Subscription<rione_msgs::msg::Command>::SharedPtr _save_location;
            rclcpp::Subscription<rione_msgs::msg::Command>::SharedPtr _load_location;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _localization;

            rclcpp::Publisher<rione_msgs::msg::Location>::SharedPtr location_publisher;
            rclcpp::Publisher<rione_msgs::msg::Request>::SharedPtr request_publisher;

            void regist_current_position(rione_msgs::msg::Request::SharedPtr msg);
            void request_location(rione_msgs::msg::Request::SharedPtr msg);
            void request_current_location(rione_msgs::msg::Request::SharedPtr msg);
            void request_location_list(rione_msgs::msg::Request::SharedPtr msg);

            void get_localization(nav_msgs::msg::Odometry::SharedPtr msg);
            bool is_register(string command);
            bool is_request(string command);
            vector<string> analize_content(string content, char separator);
            void save_location(rione_msgs::msg::Request::SharedPtr msg);
            vector<string> load_location(string file_name);

            geometry_msgs::msg::Point position = geometry_msgs::msg::Point();

            string position_name;
            string file_name;

        public:
            LocationRegister();

};
