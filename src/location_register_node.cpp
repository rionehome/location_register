#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "location_register/location_register.hpp"

using namespace std;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = make_shared<LocationRegister>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
