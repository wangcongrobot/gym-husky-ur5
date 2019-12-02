
#include "motion_planner/motion_planner.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_planner_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    JRCMotionPlanner motion_planner(nh);

    ros::waitForShutdown();

}