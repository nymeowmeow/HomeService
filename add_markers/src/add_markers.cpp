#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <math.h>

#define THRESHOLD 0.25f

class Marker
{
public:
    Marker(ros::Publisher& marker_pub, const uint32_t& shape, int x, int y);
   ~Marker() {}

    void setPosition(double x, double y);
    bool publish(uint action);
private:
    //disable copy/operator=
    Marker(const Marker&);
    void operator=(const Marker&);

    visualization_msgs::Marker marker_;
    ros::Publisher& marker_pub_;
    const uint32_t& shape_;
};

Marker::Marker(ros::Publisher& marker_pub, const uint32_t& shape, int x, int y) 
      : marker_pub_(marker_pub), shape_(shape)
{
    // Set the fame ID and timestamp.
    marker_.header.frame_id = "map";
    marker_.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker. This serves to create
    // a unique ID. Any marker sent with the same namespace and id will
    // overwrite the old one
    marker_.ns = "marker_shapes";
    marker_.id = 0;

    // set the marker type.
    marker_.type = shape;

    //set the pose of the marker. This is a full 6DOF pose relative to the
    //frame/time specified in the header
    marker_.pose.position.x = x;
    marker_.pose.position.y = y;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    // set the scale of the marker in m on a side
    marker_.scale.x = 0.25;
    marker_.scale.y = 0.25;
    marker_.scale.z = 0.25;

    // set the color. to be sure to set alpha to something non zero
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;

    marker_.lifetime = ros::Duration();
}

void
Marker::setPosition(double x, double y) 
{
    marker_.pose.position.x = x;
    marker_.pose.position.y = y;
}

bool
Marker::publish(uint action)
{
    //set the marker action. Options are ADD, DELETE
    marker_.action = action;
    //publish the marker
    while (marker_pub_.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
           return false;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub_.publish(marker_);

    return true; 
}

class Controller
{
public:
    enum STATE { STARTING = 0, PICKUP, DROPOFF };
    Controller(ros::Publisher& marker_pub, uint32_t shape, 
               int pickupx, int pickupy, 
               int dropoffx, int dropoffy);
   ~Controller() {}

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    STATE getCurrentState() const { return state_; }
    int getPickUpX() const { return pickupx_; }
    int getPickUpY() const { return pickupy_; }
    int getDropOffX() const { return dropoffx_; }
    int getDropOffY() const { return dropoffy_; }
private:
    Controller(const Controller&);
    void operator=(const Controller&);

    int pickupx_;
    int pickupy_;
    int dropoffx_;
    int dropoffy_;

    STATE state_;
    Marker marker_;
};

Controller::Controller(ros::Publisher& marker_pub, uint32_t shape, 
                       int pickupx, int pickupy, 
                       int dropoffx, int dropoffy)
    : marker_(marker_pub, shape, pickupx, pickupy),
      pickupx_(pickupx), pickupy_(pickupy), 
      dropoffx_(dropoffx),
      dropoffy_(dropoffy), state_(STARTING)
{
}

void
Controller::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    float posx = msg->pose.pose.position.x;
    float posy = msg->pose.pose.position.y;

    STATE newstate = state_;
    if (state_ == STARTING)
    {
        float frompickup = pow(pow(pickupx_ - posx, 2) + pow(pickupy_ - posy, 2), 0.5);
        if (fabs(frompickup) < THRESHOLD)
            newstate = PICKUP; 
    } else if (state_ == PICKUP) {
        float fromdropoff = pow(pow(dropoffx_ - posx, 2) + pow(dropoffy_ - posy, 2), 0.5);
        if (fabs(fromdropoff) < THRESHOLD*3)
            newstate = DROPOFF;
    }
    if (state_ != newstate)
    {
        if (newstate == PICKUP)
        {
            ROS_INFO("publish marker to pickup location");
            marker_.publish(visualization_msgs::Marker::ADD);
        } else if (newstate == DROPOFF) {
            ROS_INFO("publish marker to dropoff location");
            //remove marker at pickup location
            marker_.publish((uint)visualization_msgs::Marker::DELETE);
            //publish marker to dropoff location
            marker_.setPosition(dropoffx_, dropoffy_);
            marker_.publish((uint)visualization_msgs::Marker::ADD);
        }
        printf("state change from %d to %d\n",state_, newstate);
        state_ = newstate;
    } 
}
 
int main( int argc, char** argv )
{
   ros::init(argc, argv, "add_markers");
   ros::NodeHandle n;
   ros::Rate r(1);

   // Set our initial shape type to be a cube
   uint32_t shape = visualization_msgs::Marker::CUBE;

   ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   Controller controller(marker_pub, shape, 2, 5, -5, 5.0);
   ros::Subscriber sub = n.subscribe("/odom", 1000, &Controller::odom_callback, &controller);

   ros::spin();

   return 0; 
}

