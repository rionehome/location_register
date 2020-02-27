#include "location_register/location_register.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std;

LocationRegister::LocationRegister() : Node("location_register")
{
    _request_service = this->create_service<rione_msgs::srv::RequestLocation>(
        "/location_register",
        [this](
            const shared_ptr<rmw_request_id_t> request_header,
            const shared_ptr<rione_msgs::srv::RequestLocation::Request> request,
            const shared_ptr<rione_msgs::srv::RequestLocation::Response> response)
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
                const shared_ptr<rione_msgs::srv::RequestLocation::Request> request,
                const shared_ptr<rione_msgs::srv::RequestLocation::Response> response)
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
    else if (command == "REMOVE")
    {
        RCLCPP_INFO(this->get_logger(), "RECIEVE REMOVE REUEST");
        return 3;
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
        RCLCPP_ERROR(this->get_logger(), log);
        return vector<rione_msgs::msg::Location>();
    }

    while (getline(reading_file, buffer))
    {
        locations.push_back(analyze_content(buffer));
    }

    reading_file.close();

    return locations;
}

//
//
//
bool LocationRegister::save_data(string data, string file)
{
    ofstream writing_file;
    writing_file.open(file, ios::app);

    if (writing_file.fail())
    {
        log = "NO SUCH FILE : " + file;
        RCLCPP_ERROR(this->get_logger(), log);
        return false;
    }

    string log = "SAVE DATA TO " + file;
    RCLCPP_INFO(this->get_logger(), log);

    RCLCPP_DEBUG(this->get_logger(), data);

    writing_file << data << endl;


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
        string data = location2json(locations[count], file);

        save_data(data, file);
    }

    return true;
}

//
//
//
string LocationRegister::location2json(rione_msgs::msg::Location location, string file)
{
    picojson::object root;
    picojson::object place;
    picojson::object coordinate;
    picojson::object data;
    picojson::array datalist;
    picojson::array contents;
 
    coordinate.insert(make_pair("x", picojson::value(location.position.x)));
    coordinate.insert(make_pair("y", picojson::value(location.position.y)));
    coordinate.insert(make_pair("z", picojson::value(location.position.z)));
    place.insert(make_pair("coordinate", picojson::value(coordinate)));

    datalist.push_back(picojson::value(place));

    int count;
    for(count=0; count<location.contents.size(); count++)
    {
        contents.push_back(picojson::value(location.contents[count]));
    }

    if(0<count)
    {
        data.insert( make_pair( "contents", picojson::value( contents ) ) );
        datalist.push_back(picojson::value(data));
    }
 
    root.insert(make_pair(location.name, picojson::value(datalist)));
 
    return picojson::value(root).serialize();
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
rione_msgs::msg::Location LocationRegister::analyze_content(string content)
{
    RCLCPP_DEBUG(this->get_logger(), "NOW SEPARATE CONTENT");
    rione_msgs::msg::Location location = rione_msgs::msg::Location();

    
    picojson::value v;
    std::string err = picojson::parse(v, content);
    if (! err.empty()) {
	    RCLCPP_ERROR(this->get_logger(), "FAILED PARSE");
        return location;
    }

    map<string, picojson::value> location_object = v.get<picojson::object>();
    location.name = location_object.begin()->first;

    picojson::array datas = location_object.begin()->second.get<picojson::array>();
    int count;
    for(count=0; count<datas.size(); count++)
    {
        map<string, picojson::value> state = datas[count].get<picojson::object>();
        if("coordinate"==state.begin()->first)
        {
            location.position = json2coordinate(state.begin()->second);
        }
        else if("contents"==state.begin()->first)
        {
            vector<picojson::value> contents = state.begin()->second.get<picojson::array>();

            int i;
            for(i=0; i<contents.size(); i++)
            {
                location.contents.push_back(contents[i].serialize());
            }
        }
    }

    return location;
}

//
//
//
geometry_msgs::msg::Point LocationRegister::json2coordinate(picojson::value value)
{
    geometry_msgs::msg::Point point = geometry_msgs::msg::Point();

    map<string, picojson::value> coordinate = value.get<picojson::object>();
    point.x = coordinate["x"].get<double>();
    point.y = coordinate["y"].get<double>();
    point.z = coordinate["z"].get<double>();

    return point;
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
