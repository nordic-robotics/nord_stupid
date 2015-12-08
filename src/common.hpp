#pragma once

#include "visualization_msgs/Marker.h"
#include "point.hpp"
#include "nord_messages/PlanSrv.h"
#include "nord_messages/NextNode.h"

inline visualization_msgs::Marker create_path_message(const std::vector<point<2>>& path)
{
    visualization_msgs::Marker line_list;
    line_list.id = 204;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.color.a = 0.7;
    line_list.color.r = 1.0;
    line_list.color.g = 0.8;
    line_list.color.b = 0.1;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "houston_path";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.lifetime = ros::Duration();
    line_list.scale.x = 0.01;

    if (path.size() < 2)
        return line_list;

    for (size_t i = 0; i < path.size() - 1; i++)
    {
        geometry_msgs::Point p0, p1;
        p0.x = path[i].x();
        p0.y = path[i].y();
        p1.x = path[i + 1].x();
        p1.y = path[i + 1].y();
        p0.z = p1.z = 0;
        line_list.points.push_back(p0);
        line_list.points.push_back(p1);
    }

    return line_list;
}

inline visualization_msgs::Marker create_objects_message(const std::vector<std::pair<std::string, point<2>>> objects)
{
    visualization_msgs::Marker line_list;
    line_list.id = 206;
    line_list.type = visualization_msgs::Marker::CUBE_LIST;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "houston_objects";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.lifetime = ros::Duration();
    line_list.scale.x = 0.05;
    line_list.scale.y = 0.05;
    line_list.scale.z = 0.05;
    line_list.color.r = line_list.color.g = line_list.color.b = line_list.color.a = 1.0;

    for (auto& o : objects)
    {
        geometry_msgs::Point p;
        p.x = o.second.x();
        p.y = o.second.y();
        p.z = 0;

        std_msgs::ColorRGBA c;
        c.a = 1.0;

        if (o.first == "green cube")
        {
            c.r = 0.0;
            c.g = 1.0;
            c.b = 0.0;
        }
        else if (o.first == "red cube")
        {
            c.r = 1.0;
            c.g = 0.0;
            c.b = 0.0;
        }
        else if (o.first == "patric")
        {
            c.r = 1.0;
            c.g = 0.5;
            c.b = 0.0;
        }
        std::cout << p.x << " " << p.y << " " << c.g << std::endl;
        line_list.points.push_back(p);
        line_list.colors.push_back(c);
    }

    return line_list;
}

std::vector<point<2>> read_path(std::string filename)
{
    std::ifstream file(filename);
    std::string l;
    std::vector<point<2>> path;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        float x, y;
        iss >> x >> y;
        path.emplace_back(x, y);
    }
    return path;
}

void go_to(const point<2>& p, ros::Publisher& pub, int move = 1)
{
    std::cout << "go_to (" << p.x() << ", " << p.y() << ")" << std::endl;
    nord_messages::NextNode msg;
    msg.x = p.x();
    msg.y = p.y();
    msg.move = move;

    pub.publish(msg);
}

std::vector<point<2>> plan_path(const point<2>& current, const point<2>& target, bool direct, ros::NodeHandle& n)
{
    ros::ServiceClient client = n.serviceClient<nord_messages::PlanSrv>(
        "/nord_planning/plan_service", false);
    nord_messages::PlanSrv srv;
    srv.request.start.x = current.x();
    srv.request.start.y = current.y();
    srv.request.end.x = target.x();
    srv.request.end.y = target.y();
    srv.request.direct = direct;
    if (!client.call(srv))
    {
        std::cout << "plan call failed!" << std::endl;
        return std::vector<point<2>>();
    }
    std::cout << "new plan:" << std::endl;
    std::vector<point<2>> result;
    std::transform(srv.response.path.begin(), srv.response.path.end(),
                   std::back_inserter(result),
        [](const nord_messages::Vector2& p) {
            std::cout << "\t" << p.x << " " << p.y << std::endl;
            return point<2>(p.x, p.y);
    });
    return result;
}

float exit_heuristic()
{
    return 120;
}