#include "ros/ros.h"
#include "ros/package.h"
#include <cstdint>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <set>
#include <thread>
#include "point.hpp"
#include "common.hpp"
#include "visualization_msgs/Marker.h"
#include "nord_messages/Vector2.h"
#include "nord_messages/PlanSrv.h"
#include "nord_messages/PromptEvidenceReportingSrv.h"
#include "nord_messages/EvidenceSrv.h"
#include "nord_messages/NextNode.h"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/ObjectArray.h"
#include "nord_messages/MotorTwist.h"
#include "nord_messages/NextNode.h"
#include "std_msgs/Int64.h"

std::vector<std::pair<std::string, point<2>>> load_objects(std::string filename)
{
    std::ifstream file(filename);
    std::string l;
    std::vector<std::pair<std::string, point<2>>> objects;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        float x, y;
        iss >> x >> y;
        std::string name;
        std::getline(iss, name);
        name.erase(name.begin());
        objects.emplace_back(name, point<2>(x, y));
        std::cout << x << " " << y << " " << name << std::endl;
    }
    return objects;
}

int main(int argc, char** argv)
{
    using namespace std::literals;
    ros::init(argc, argv, "nord_stupid_two");
    ros::NodeHandle n;

    point<2> position;
    point<2> stddev(10, 10);
    assert(argc >= 3);
    point<2> exit_position = point<2>(std::stod(argv[1]), std::stod(argv[2]));
    std::vector<point<2>> path;
    std::vector<point<2>> dijkstra_path;
    float time_left = 180;
    auto objects = load_objects(ros::package::getPath("nord_vision")+"/data/objects.txt");
    bool planned_escape = false;
    bool position_known = false;

    ros::Publisher point_pub(n.advertise<nord_messages::NextNode>("/nord/control/point", 10));
    ros::Publisher espeak_pub(n.advertise<std_msgs::String>("/espeak/string", 10));
    ros::Publisher map_pub(n.advertise<visualization_msgs::Marker>("/nord/map", 10));
    ros::Publisher motor_pub(n.advertise<nord_messages::MotorTwist>("/nord/motor_controller/twist", 10));
    ros::Publisher pf_pub(n.advertise<std_msgs::Int64>("/nord/estimation/set_num_particles", 10));
    
    ros::Subscriber pose_sub(n.subscribe<nord_messages::PoseEstimate>("/nord/estimation/pose_estimation", 10,
        [&](const nord_messages::PoseEstimate::ConstPtr& msg) {
            position = point<2>(msg->x.mean, msg->y.mean);
            stddev = point<2>(msg->x.stddev, msg->y.stddev);
            if (position_known)
            {
                if (dijkstra_path.size() > 0 && (position - dijkstra_path.front()).length() < 0.02)
                {
                    dijkstra_path.erase(dijkstra_path.begin());
                    if (dijkstra_path.size() > 0)
                    {
                        go_to(dijkstra_path.front(), point_pub);
                    }
                    else
                    {
                        path.erase(path.begin());
                    }
                }
                if (path.size() > 0 && dijkstra_path.size() == 0)
                {
                    if ((path.front() - position).length() < 0.1)
                        path.erase(path.begin());
                    if (path.size() > 0)
                    {
                        std::cout << "planning dijkstra from " << position << " to " << path.front() << std::endl;
                        dijkstra_path = plan_path(position, path.front(), true, n);
                        if (dijkstra_path.size() > 0)
                        {
                            if (dijkstra_path.size() > 1)
                            {
                                dijkstra_path.erase(dijkstra_path.begin());
                            }
                            go_to(dijkstra_path.front(), point_pub);
                        }
                        else
                        {
                            time_left = 0;
                        }
                    }
                }

                for (size_t i = 0; i < objects.size(); i++)
                {
                    if ((objects[i].second - position).length() < 0.5)
                    {
                        std_msgs::String msg;
                        msg.data = objects[i].first;
                        espeak_pub.publish(msg);
                        objects.erase(objects.begin() + i);
                        i--;
                    }
                }
            }
    }));

    ros::Timer timer(n.createTimer(ros::Duration(1),
        [&](const ros::TimerEvent& e) {
            time_left -= 1;
            create_path_message(path);
            if (time_left < exit_heuristic() && !planned_escape)
            {
                planned_escape = true;
                dijkstra_path = plan_path(position, exit_position, true, n);
            }
    }));

    std::this_thread::sleep_for(2s);

    ros::Rate r(1);
    while (ros::ok())
    {
        if (stddev.x() < 0.015 && stddev.y() < 0.015)
        {
            std::cout << "found position!" << std::endl;
            position_known = true;
            break;
        }

        nord_messages::MotorTwist msg;
        msg.angular_vel = M_PI / 2;
        motor_pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }

    std_msgs::Int64 msg;
    msg.data = 10000;
    pf_pub.publish(msg);

    nord_messages::MotorTwist msg2;
    msg2.angular_vel = 0;
    motor_pub.publish(msg2);

    path = plan_path(position, exit_position, false, n);

    ros::spin();

    return 0;
}
