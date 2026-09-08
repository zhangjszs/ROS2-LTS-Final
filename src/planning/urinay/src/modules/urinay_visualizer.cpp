/**
 * @file Visualization.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief UrinayVisualizer 类成员函数的实现
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "modules/urinay_visualizer.hpp"

/* ----------------------------- 私有方法 ---------------------------- */

void UrinayVisualizer::setTimestamp(const rclcpp::Time& stamp) {
    this->stamp_ = stamp;
}

/* ------------------------------ 公有方法 ---------------------------- */

UrinayVisualizer& UrinayVisualizer::getInstance() {
    static UrinayVisualizer vis;
    return vis;
}

void UrinayVisualizer::init(rclcpp::Node::SharedPtr node, const UrinayParams::Visualization& params) {
    params_ = params;
    if (params.publish_markers) {
        trianglesPub = node->create_publisher<visualization_msgs::msg::MarkerArray>(params_.triangulation_topic, 1);
        midpointsPub = node->create_publisher<visualization_msgs::msg::MarkerArray>(params_.midpoints_topic, 1);
        wayPub = node->create_publisher<visualization_msgs::msg::MarkerArray>(params_.way_topic, 1);
    }
}

void UrinayVisualizer::visualize(const TriangleSet& triSet) const {
    if (not this->params_.publish_markers)
        return;
    if (!trianglesPub || trianglesPub->get_subscription_count() == 0)
        return;

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.reserve(1 + 5 * triSet.size());
    visualization_msgs::msg::Marker mTriangulation, mCircumCenter, mMidpoint;
    size_t id = 0;
    mTriangulation.header.stamp = this->stamp_;
    mTriangulation.header.frame_id = "map";
    mTriangulation.color.a = 1.0;
    mTriangulation.color.r = 1.0;
    mTriangulation.pose.orientation.w = 1.0;
    mTriangulation.scale.x = 0.1;
    mTriangulation.scale.y = 0.1;
    mTriangulation.scale.z = 0.01;
    mTriangulation.id = id++;
    mTriangulation.action = visualization_msgs::msg::Marker::DELETEALL;
    mTriangulation.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ma.markers.push_back(mTriangulation);
    mTriangulation.action = visualization_msgs::msg::Marker::ADD;

    mCircumCenter = mTriangulation;
    mCircumCenter.type = visualization_msgs::msg::Marker::CYLINDER;
    mCircumCenter.scale.x = 0.1;
    mCircumCenter.scale.y = 0.1;
    mCircumCenter.scale.z = 0.05;
    mCircumCenter.color.r = 0.0;
    mCircumCenter.color.g = 0.0;
    mCircumCenter.color.b = 1.0;

    mMidpoint = mCircumCenter;
    mMidpoint.type = visualization_msgs::msg::Marker::CUBE;
    mMidpoint.color.r = 0.0;
    mMidpoint.color.g = 1.0;
    mMidpoint.color.b = 0.0;
    for (const Triangle& t : triSet) {
        // 三角形本身
        mTriangulation.points.clear();
        mTriangulation.points.reserve(4);
        mTriangulation.id = id++;
        mTriangulation.points.push_back(t.nodes[0].pointGlobal().gmPoint());
        mTriangulation.points.push_back(t.nodes[1].pointGlobal().gmPoint());
        mTriangulation.points.push_back(t.nodes[2].pointGlobal().gmPoint());
        mTriangulation.points.push_back(t.nodes[0].pointGlobal().gmPoint());
        ma.markers.push_back(mTriangulation);

        // 外接圆圆心
        mCircumCenter.pose.position = t.circumCenterGlobal().gmPoint();
        mCircumCenter.id = id++;
        ma.markers.push_back(mCircumCenter);

        // 边的中点
        for (const Edge& e : t.edges) {
            mMidpoint.pose.position = e.midPointGlobal().gmPoint();
            mMidpoint.id = id++;
            ma.markers.push_back(mMidpoint);
        }
    }
    trianglesPub->publish(ma);
}

void UrinayVisualizer::visualize(const EdgeSet& edgeSet) const {
    if (not this->params_.publish_markers)
        return;
    if (!midpointsPub || midpointsPub->get_subscription_count() == 0)
        return;

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.reserve(edgeSet.size() + 1);
    visualization_msgs::msg::Marker mMidpoint;
    size_t id = 0;
    mMidpoint.header.stamp = this->stamp_;
    mMidpoint.header.frame_id = "map";
    mMidpoint.color.a = 1.0;
    mMidpoint.color.r = 1.0;
    mMidpoint.pose.orientation.w = 1.0;
    mMidpoint.scale.x = 0.04;
    mMidpoint.scale.y = 0.04;
    mMidpoint.scale.z = 0.1;
    mMidpoint.type = visualization_msgs::msg::Marker::CYLINDER;
    mMidpoint.id = id++;
    mMidpoint.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(mMidpoint);
    mMidpoint.action = visualization_msgs::msg::Marker::ADD;

    for (const Edge& e : edgeSet) {
        mMidpoint.pose.position = e.midPointGlobal().gmPoint();
        mMidpoint.id = id++;
        ma.markers.push_back(mMidpoint);
    }
    midpointsPub->publish(ma);
}

void UrinayVisualizer::visualize(const Way& way) const {
    if (not this->params_.publish_markers)
        return;
    if (!wayPub || wayPub->get_subscription_count() == 0)
        return;

    visualization_msgs::msg::MarkerArray ma;
    ma.markers.reserve(3 * way.size() + 1);
    visualization_msgs::msg::Marker mMidpoints, mLeft, mRight;
    size_t id = 0;
    mMidpoints.header.stamp = this->stamp_;
    mMidpoints.header.frame_id = "map";
    mMidpoints.color.a = 1.0;
    mMidpoints.color.g = 1.0;
    mMidpoints.pose.orientation.w = 1.0;
    mMidpoints.scale.x = 0.15;
    mMidpoints.scale.y = 0.15;
    mMidpoints.scale.z = 0.15;
    mMidpoints.type = visualization_msgs::msg::Marker::LINE_STRIP;
    mMidpoints.id = id++;
    mMidpoints.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(mMidpoints);
    mMidpoints.action = visualization_msgs::msg::Marker::ADD;
    mLeft = mMidpoints;
    mLeft.color.g = 0.0;
    mLeft.color.b = 0.7;
    mLeft.id = id++;
    mRight = mMidpoints;
    mRight.color.r = 0.7;
    mRight.color.g = 0.7;

    mMidpoints.color.a = 0.5;
    mMidpoints.id = id++;
    for (const Point& p : way.getPath()) {
        mMidpoints.points.push_back(p.gmPoint());
    }
    ma.markers.push_back(mMidpoints);

    mRight.id = id++;
    const Tracklimits tracklimits = way.getTracklimits();

    for (const Node& n : tracklimits.left) {
        mLeft.points.push_back(n.pointGlobal().gmPoint());
    }
    ma.markers.push_back(mLeft);

    for (const Node& n : tracklimits.right) {
        mRight.points.push_back(n.pointGlobal().gmPoint());
    }
    ma.markers.push_back(mRight);

    wayPub->publish(ma);
}
