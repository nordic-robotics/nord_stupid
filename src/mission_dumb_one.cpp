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
#include "nord_messages/MotorTwist.h"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/ObjectArray.h"
#include "std_msgs/Empty.h"

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
    ros::init(argc, argv, "nord_stupid_one");
    ros::NodeHandle n;

    point<2> position;
    assert(argc >= 3);
    point<2> exit_position = point<2>(std::stod(argv[1]), std::stod(argv[2]));
    std::vector<std::pair<size_t, point<2>>> unknown;
    auto path = read_path(ros::package::getPath("nord_planning")+"/data/plan.txt");
    std::vector<point<2>> dijkstra_path;
    float time_left = 300;
    bool planned_escape = false;
    std::set<size_t> seen_ids;

    ros::Publisher point_pub(n.advertise<nord_messages::NextNode>("/nord/control/point", 10));
    ros::Publisher espeak_pub(n.advertise<std_msgs::String>("/espeak/string", 10));
    ros::Publisher map_pub(n.advertise<visualization_msgs::Marker>("/nord/map", 10));
    ros::Publisher motor_pub(n.advertise<nord_messages::MotorTwist>("/nord/motor_controller/twist", 10));
    
    ros::Subscriber replan_sub(n.subscribe<std_msgs::Empty>("/nord/houston/mission_abort", 10,
        [&](const std_msgs::Empty::ConstPtr& msg) {
            dijkstra_path.clear();
    }));
    ros::Subscriber pose_sub(n.subscribe<nord_messages::PoseEstimate>("/nord/estimation/pose_estimation", 10,
        [&](const nord_messages::PoseEstimate::ConstPtr& msg) {
            position = point<2>(msg->x.mean, msg->y.mean);
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
                if (path.size() > 0)
                {
                    if ((path.front() - position).length() < 0.1)
                    {
                        path.erase(path.begin());
                    }
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
            }

            for (size_t i = 0; i < unknown.size(); i++)
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
    ros::spin();

    return 0;
}
