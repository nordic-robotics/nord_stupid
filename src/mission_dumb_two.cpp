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
#include "point.hpp"
#include "common.hpp"
#include "visualization_msgs/Marker.h"
#include "nord_messages/Vector2.h"
#include "nord_messages/PlanSrv.h"
#include "nord_messages/PromptEvidenceReportingSrv.h"
#include "nord_messages/EvidenceSrv.h"
#include "nord_messages/NextNode.h"
#include "nord_messages/MotorTwist.h"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/ObjectArray.h"
#include "std_msgs/Empty.h"

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

std::string classify(size_t id, std::vector<std::pair<size_t, point<2>>>& unknown, ros::NodeHandle& n)
{
    std::cout << "classify" << std::endl;
    ros::ServiceClient client = n.serviceClient<nord_messages::PromptEvidenceReportingSrv>(
        "/nord/vision/prompt_evidence_reporting_service", false);
    nord_messages::PromptEvidenceReportingSrv srv;
    srv.request.id = id;
    if (!client.call(srv))
    {
        std::cout << "call failed!" << std::endl;
        return "";
    }
    std::string new_class = srv.response.name;

    if (new_class == "")
    {
        std::cout << "classification failed!" << std::endl;
        return "";
    }

    point<2> obj_location(-1, -1);
    for (size_t i = 0; i < unknown.size(); i++)
    {
        if (unknown[i].first == id)
        {
            obj_location = unknown[i].second;
        }
    }

    std::cout << "snapping evidence" << std::endl;
    ros::ServiceClient client2 = n.serviceClient<nord_messages::EvidenceSrv>(
        "/nord/evidence_service", false);
    nord_messages::EvidenceSrv srv2;
    // fill in
    srv2.request.data.id = id;
    srv2.request.data.x = obj_location.x();
    srv2.request.data.y = obj_location.y();
    srv2.request.data.objectId.data = new_class;
    if (!client2.call(srv2))
    {
        std::cout << "evidence call failed!" << std::endl;
    }

    return new_class;
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
    bool seen_past_objects = true;
    std::set<size_t> seen_ids;
    std::vector<std::pair<size_t, point<2>>> unknown;

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
                    if ((path.front() - position).length() < 0.2)
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
                

                for (int i = 0; i < objects.size(); i++)
                {
                    if ((objects[i].second - position).length() < 0.5)
                    {
                        seen_past_objects = true;
                        std_msgs::String msg;
                        msg.data = objects[i].first;
                        espeak_pub.publish(msg);
                        objects.erase(objects.begin() + i);
                        i--;
                    }
                }
                if (!seen_past_objects)
                {
                    for (int i = 0; i < unknown.size(); i++)
                    {
                        if ((position - unknown[i].second).length() < 1.0)
                        {
                            auto name = classify(unknown[i].first, unknown, n);

                            if (name != "")
                            {
                                std_msgs::String msg;
                                msg.data = name;
                                espeak_pub.publish(msg);
                                unknown.erase(unknown.begin() + i);
                                i--;
                            }
                        }
                    }
                }
                
            }
        }
    }));

    ros::Subscriber obj_sub(n.subscribe<nord_messages::ObjectArray>("/nord/vision/igo", 9,
        [&](const nord_messages::ObjectArray::ConstPtr& msg) {
            std::cout << "got obj" << std::endl;
            unknown.clear();
            unknown.reserve(msg->data.size());
            for (auto& d : msg->data)
            {
                if (seen_ids.find(d.id) == seen_ids.end())
                {
                    unknown.push_back(std::make_pair(d.id, point<2>(d.x, d.y)));
                    seen_ids.insert(d.id);
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
