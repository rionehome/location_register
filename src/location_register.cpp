#include "location_register.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std;

LocationRegister::LocationRegister() : Node("location_register") {
    _register_current_position = this->create_subscription<rione_msgs::msg::Request>(
        "/location/register_current_location",
        10,
        [this](rione_msgs::msg::Request::SharedPtr msg){
            regist_current_position(msg);
        }
    );

    _request_location = this->create_subscription<rione_msgs::msg::Request>(
        "/location/request_location",
        10,
        [this](rione_msgs::msg::Request::SharedPtr msg){
            request_location(msg);
        }
    );

    _request_current_location = this->create_subscription<rione_msgs::msg::Request>(
        "/location/request_current_location",
        10,
        [this](rione_msgs::msg::Request::SharedPtr msg){
            request_current_location(msg);
        }
    );

    _request_location_list = this->create_subscription<rione_msgs::msg::Request>(
        "/location/request_location_list",
        10,
        [this](rione_msgs::msg::Request::SharedPtr msg){
            request_location_list(msg);
        }
    );

    _localization = this->create_subscription<nav_msgs::msg::Odometry>(
        "/localization",
        10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg){
            get_localization(msg);
        }
    );

    location_publisher = this->create_publisher<rione_msgs::msg::Location>(
        "/location/answer",
        10
    );

    request_publisher = this->create_publisher<rione_msgs::msg::Request>(
        "/location/answer",
        10
    );

    RCLCPP_INFO(this->get_logger(), "START LOCATION REGISTER");
}

/*
 * regist_current_location
 * :param msg:
 *
 */
void LocationRegister::regist_current_position(rione_msgs::msg::Request::SharedPtr msg){

    msg->locations[0].position = position;
    
    save_location(msg);
}

void LocationRegister::request_location(rione_msgs::msg::Request::SharedPtr msg){
    string name = msg->locations[0].name;
    string file_name = msg->file;

    vector<string> positions = load_location(file_name);

    int pos;
    for(pos=0; pos < (int)positions.size(); pos++) {
        vector<string> contents = analize_content(positions[pos], ':');

        if (contents[0] == name) {
            vector<string> pose = analize_content(contents[1], ',');
            rione_msgs::msg::Location position = rione_msgs::msg::Location();

            position.name = name;
            position.position.x = stod(pose[0]);
            position.position.y = stod(pose[1]);
            position.position.z = stod(pose[2]);

            location_publisher->publish(position);

            string log = "LOAD LOCATION : " + contents[0] + " "+ contents[1];
            RCLCPP_INFO(this->get_logger(), log);
        }
    }
}

void LocationRegister::request_current_location(rione_msgs::msg::Request::SharedPtr msg){
    rione_msgs::msg::Location location = rione_msgs::msg::Location();

    location.position = position;

    location_publisher->publish(location);
}

void LocationRegister::request_location_list(rione_msgs::msg::Request::SharedPtr msg){
    string name = msg->locations[0].name;
    string file_name = msg->file;

    vector<string> positions = load_location(file_name);

    rione_msgs::msg::Request answer = rione_msgs::msg::Request();
    answer.file = file_name;

    int pos;
    for(pos=0; pos < (int)positions.size(); pos++) {
        vector<string> contents = analize_content(positions[pos], ':');

        vector<string> pose = analize_content(contents[1], ',');

        rione_msgs::msg::Location position = rione_msgs::msg::Location();

        position.name = name;
        position.position.x = stod(pose[0]);
        position.position.y = stod(pose[1]);
        position.position.z = stod(pose[2]);

        answer.locations.push_back(position);
    }

    request_publisher->publish(answer);
}

vector<string> LocationRegister::load_location(string file_name){
    ifstream reading_file;
    reading_file.open(file_name, ios::in);

    vector<string> locations;

    string buffer;

    RCLCPP_INFO(this->get_logger(), "LOADING LOCATION FILE");

    if (reading_file.fail()) {
        string log = "NO SUCH FILE : " + file_name;
        RCLCPP_INFO(this->get_logger(), log);
        return vector<string>();
    }

    while (getline(reading_file, buffer)) {
        locations.push_back(buffer);
    }

    reading_file.close();

    return locations;
}

void LocationRegister::save_location(rione_msgs::msg::Request::SharedPtr msg) {

    ofstream writing_file;
    writing_file.open(msg->file, ios::app);

    int pos;
    for (pos=0; pos<(int)msg->locations.size(); pos++) {
        string name = msg->locations[pos].name;
        geometry_msgs::msg::Point pose = msg->locations[pos].position;

        string log = name + ":" + to_string(pose.x) + "," + to_string(pose.y) + "," + to_string(pose.z);

        RCLCPP_INFO(this->get_logger(), "SAVE DATA");
        RCLCPP_INFO(this->get_logger(), log);

        writing_file << log << endl;

    }

    writing_file.close();
}

vector<string> LocationRegister::analize_content(string content, char separator){
    vector<string> words;

    stringstream ss{content};
    string buffer;

    while (getline(ss, buffer, separator)){
        words.push_back(buffer);
    }

    return words;
}

void LocationRegister::get_localization(nav_msgs::msg::Odometry::SharedPtr msg){
    position = msg->pose.pose.position;
}

bool LocationRegister::is_register(string command){
    if ( command=="REGISTER" ) {
        return true;
    }

    return false;
}

bool LocationRegister::is_request(string command){
    if ( command=="REQUEST" ) {
        return true;
    }

    return false;
}
