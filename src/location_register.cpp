#include "location_register.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace std::placeholders;

LocationRegister::LocationRegister() : Node("location_register")
{
    _request_service = this->create_service<rione_msgs::srv::RequestData>(
        "/location_register",
        [this](
            const shared_ptr<rmw_request_id_t> request_header,
            const shared_ptr<rione_msgs::srv::RequestData::Request> request,
            const shared_ptr<rione_msgs::srv::RequestData::Response> response)
        {
            (void)request_header;
            handleService_(request_header, request, response);
        }
    );

    goal_location_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal",
        10
    );

    RCLCPP_INFO(this->get_logger(), "START LOCATION REGISTER");
}

// 
//
//
void LocationRegister::handleService_(
                const shared_ptr<rmw_request_id_t> request_header,
                const shared_ptr<rione_msgs::srv::RequestData::Request> request,
                const shared_ptr<rione_msgs::srv::RequestData::Response> response)
{
    short command_number = check_command(request->command);

    RCLCPP_DEBUG(this->get_logger(), "COMMAND NUMBER IS %d", command_number);

    switch(command_number) {
        case 0:
            regist_location(request->locations, request->file);
            break;
        case 1:
            response->locations = get_location(request->locations, request->file);
            break;
        case 2:
            if(1==(int)request->locations.size())
            {
                vector<rione_msgs::msg::Location> locations;
                locations = get_location(request->locations, request->file);
                response->flag = send_goal_location(locations[0]);
            }
            else if((int)request->locations.size()<=0)
            {
                RCLCPP_INFO(this->get_logger(), "THERE IS NO GOAL POSITION");
            }
            else if(1<(int)request->locations.size())
            {
                RCLCPP_INFO(this->get_logger(), "TOO MANY GOAL POSITIONS");
            }
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "NOT FOUND COMMAND");
    }
}

//
//
//
short LocationRegister::check_command(string command)
{
    if (command == "REGIST")
    {
        return 0;
    } 
    else if (command == "GET")
    {
        return 1;
    }
    else if (command == "SEND_GOAL")
    {
        log = "RECIEVE " + command + " REQUEST";
        RCLCPP_INFO(this->get_logger(), log);
        return 2;
    }
    else
    {
        return -1;
    }
}

//
//
//
vector<rione_msgs::msg::Location> LocationRegister::load_location(string file)
{
    ifstream reading_file;
    reading_file.open(file, ios::in);

    vector<rione_msgs::msg::Location> locations;

    string buffer;

    RCLCPP_INFO(this->get_logger(), "LOADING LOCATION FILE");

    if (reading_file.fail())
    {
        log = "NO SUCH FILE : " + file;
        RCLCPP_INFO(this->get_logger(), log);
        return vector<rione_msgs::msg::Location>();
    }

    while (getline(reading_file, buffer))
    {
        locations.push_back(analize_content(buffer));
    }

    reading_file.close();

    return locations;
}

//
//
//
bool LocationRegister::save_location(rione_msgs::msg::Location location, string file)
{

    ofstream writing_file;
    writing_file.open(file, ios::app);

    if (writing_file.fail())
    {
        log = "NO SUCH FILE : " + file;
        RCLCPP_INFO(this->get_logger(), log);
        return false;
    }

    string name = location.name;
    geometry_msgs::msg::Point pose = location.position;

    string log = "SAVE DATA TO " + file;
    RCLCPP_INFO(this->get_logger(), log);

    log = name + ":" + to_string(pose.x) + "," + to_string(pose.y) + "," + to_string(pose.z);

    RCLCPP_INFO(this->get_logger(), log);

    writing_file << log << endl;


    writing_file.close();

    return true;
}

//
//
//
bool LocationRegister::regist_location(vector<rione_msgs::msg::Location> locations, string file)
{
    log = "RECIEVE REGIST REQUEST";
    RCLCPP_INFO(this->get_logger(), log);

    int count;    

    for(count=0; count<(int)locations.size(); count++)
    {
        save_location(locations[count], file);
    }

    return true;
}

//
//
//
vector<rione_msgs::msg::Location> LocationRegister::get_location(vector<rione_msgs::msg::Location> locations, string file)
{
    log = "RECIEVE GET REQUEST";
    RCLCPP_INFO(this->get_logger(), log);

    if(0 < locations.size())
    {
        vector<rione_msgs::msg::Location> result = get_all_location(file);
        return search_position(result, locations);
    }
    else 
    {
        return get_all_location(file);
    }
}

//
//
//
vector<rione_msgs::msg::Location> LocationRegister::get_all_location(string file)
{
    RCLCPP_DEBUG(this->get_logger(), "GET ALL LOCATION");
    vector<rione_msgs::msg::Location> locations = load_location(file);
    int pos;
    for(pos=0; pos<(int)locations.size(); pos++)
    {
        log = locations[pos].name + ":" +
                to_string(locations[pos].position.x) + "," +
                to_string(locations[pos].position.y) + "," +
                to_string(locations[pos].position.z);
        RCLCPP_DEBUG(this->get_logger(), log);
    }

    return locations;
}

//
//
//
rione_msgs::msg::Location LocationRegister::analize_content(string content)
{
    RCLCPP_DEBUG(this->get_logger(), "NOW SEPARATE CONTENT");
    rione_msgs::msg::Location location = rione_msgs::msg::Location();
    string buffer;

    vector<string> separated_data;
    stringstream ss{content};

    while (getline(ss, buffer, ':'))
    {
        separated_data.push_back(buffer);
    }

    location.name = separated_data[0];
    vector<float> position_data;
    ss = stringstream(separated_data[1]);

    while (getline(ss, buffer, ','))
    {
        position_data.push_back(stof(buffer));
    }

    location.position.x = position_data[0];
    location.position.y = position_data[1];
    location.position.z = position_data[2];

    return location;
}

//
//
//
vector<rione_msgs::msg::Location> LocationRegister::search_position(
    vector<rione_msgs::msg::Location> locations, vector<rione_msgs::msg::Location> search)
{
    vector<rione_msgs::msg::Location> result;

    int pos;
    int count;
    for(pos=0; pos<(int)locations.size(); pos++)
    {
        for(count=0; count<(int)search.size(); count++)
        {
            if(locations[pos].name == search[count].name)
            {
                RCLCPP_DEBUG(this->get_logger(), "GET SPECIFICAL POSITION");
                log = locations[pos].name + ":" +
                    to_string(locations[pos].position.x) + "," +
                    to_string(locations[pos].position.y) + "," +
                    to_string(locations[pos].position.z);
                RCLCPP_DEBUG(this->get_logger(), log);
                result.push_back(locations[pos]);
            }
        }
    }
    
    return result;
}

//
//
//
bool LocationRegister::send_goal_location(rione_msgs::msg::Location location){
    string name = location.name;

    
    goal_position.pose.position = location.position;
    goal_location_publisher->publish(goal_position);

    string log = "SENT GOAL POSITION : " + name + " : " +
        to_string(location.position.x) + "," +
        to_string(location.position.y) + "," +
        to_string(location.position.z);
    RCLCPP_INFO(this->get_logger(), log);

    return true;
}
