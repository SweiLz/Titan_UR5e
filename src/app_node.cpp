#include <ros/ros.h>
#include <titan_ur5e/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>

class ScanNPlan
{
public:
    ScanNPlan(ros::NodeHandle &nh) //: _ac("scaled_pos_traj_controller/follow_joint_trajectory", true)
    {
        _vision_client = nh.serviceClient<titan_ur5e::LocalizePart>("localize_part");
        // _cartesian_client = nh.serviceClient<app_core::PlanCartesianPath>("plan_path");
    }

    void start(const std::string &base_frame)
    {
        ROS_INFO_STREAM("Attempting to localize part");
        titan_ur5e::LocalizePart srv;
        srv.request.base_frame = base_frame;
        ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);
        if (!_vision_client.call(srv))
        {
            ROS_ERROR_STREAM("Could not localize part");
            return;
        }
        // ROS_INFO_STREAM("part localized: " << srv.response);
        geometry_msgs::Pose move_target = srv.response.pose;
        geometry_msgs::Pose move_target2 = move_target;
        move_target2.position.x += 0.2;

        geometry_msgs::Pose pre_move_target = move_target;
        pre_move_target.position.z += 0.1;

        geometry_msgs::Pose pre_move_target2 = move_target2;
        pre_move_target2.position.z += 0.1;

        // std::vector<geometry_msgs::Pose> target_list;
        // target_list.push_back(pre_move_target);
        // target_list.push_back(move_target);
        // target_list.push_back(pre_move_target);

        moveit::planning_interface::MoveGroupInterface::Options opt("manipulator", "ur1/robot_description");
        moveit::planning_interface::MoveGroupInterface move_group("manipulator");
        move_group.setPoseReferenceFrame(base_frame);

        geometry_msgs::Pose move_to = move_target;
        move_to.position.z += 0.05;
        for (int i = 0; i < 3; i++)
        {
            move_to.position.y = move_target.position.y - (0.08 * i);
            for (int j = 0; j < 3; j++)
            {
                move_to.position.x = move_target.position.x + (0.08 * j);
                move_group.setPoseTarget(move_to);
                move_group.move();

                move_to.position.z -= 0.05;
                move_group.setPoseTarget(move_to);
                move_group.move();

                move_to.position.z += 0.05;
                move_group.setPoseTarget(move_to);
                move_group.move();
            }
        }

        // move_group.setPoseTarget(move_target);
        // move_group.move();

        // move_group.setPoseTarget(pre_move_target);
        // move_group.move();

        // move_group.setPoseTarget(pre_move_target2);
        // move_group.move();

        // move_group.setPoseTarget(move_target2);
        // move_group.move();

        // move_group.setPoseTarget(pre_move_target2);
        // move_group.move();

        // ros::Duration(3.0).sleep();
        move_group.setNamedTarget("Home");
        move_group.move();

        // app_core::PlanCartesianPath cartesian_srv;
        // cartesian_srv.request.pose = move_target;
        // if (!_cartesian_client.call(cartesian_srv))
        // {
        //     ROS_ERROR("Could not plan for path");
        //     return;
        // }
        // ROS_INFO_STREAM("Got cart path, executing");
        // control_msgs::FollowJointTrajectoryGoal goal;
        // goal.trajectory = cartesian_srv.response.trajectory;
        // _ac.sendGoal(goal);
        // _ac.waitForResult();
        ROS_INFO_STREAM("Done");
    }

private:
    ros::ServiceClient _vision_client;
    // ros::ServiceClient _cartesian_client;
    // actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> _ac;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "app_node");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~");
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();
    std::string base_frame;
    _nh.param<std::string>("base_frame", base_frame, "world");

    ROS_INFO_STREAM("ScanNPlan node has been initialized");
    ScanNPlan app(nh);
    ros::Duration(0.5).sleep();
    app.start(base_frame);
    ros::waitForShutdown();
    // ros::spin();
}