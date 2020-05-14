#include <ros/ros.h>
#include <fake_ar_publisher/ARMarker.h>
#include <titan_ur5e/LocalizePart.h>
#include <tf/transform_listener.h>

class Localizer
{
public:
    Localizer(ros::NodeHandle &nh)
    {
        _ar_sub = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 10, &Localizer::visionCallback, this);
        _server = nh.advertiseService("localize_part", &Localizer::localizePart, this);
    }

    void visionCallback(const fake_ar_publisher::ARMarkerConstPtr &msg)
    {
        _last_msg = msg;
        // ROS_INFO_STREAM(_last_msg->pose.pose);
    }

    bool localizePart(titan_ur5e::LocalizePart::Request &req, titan_ur5e::LocalizePart::Response &res)
    {
        tf::Transform cam_to_target;
        tf::poseMsgToTF(_last_msg->pose.pose, cam_to_target);

        tf::StampedTransform req_to_cam;
        _listener.lookupTransform(req.base_frame, _last_msg->header.frame_id, ros::Time(0), req_to_cam);

        tf::Transform req_to_target;
        req_to_target = req_to_cam * cam_to_target;

        tf::poseTFToMsg(req_to_target, res.pose);
        return true;
    }

private:
    ros::Subscriber _ar_sub;
    ros::ServiceServer _server;
    fake_ar_publisher::ARMarkerConstPtr _last_msg;
    tf::TransformListener _listener;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    Localizer localizer(nh);
    ROS_INFO_STREAM("Vision node starting");

    ros::spin();
}
