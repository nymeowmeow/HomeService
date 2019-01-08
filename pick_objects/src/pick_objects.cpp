#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal 
createGoal(double x, double y, double z, double w)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = z;
    goal.target_pose.pose.orientation.w = w;

    return goal;
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //move_base_msgs::MoveBaseGoal pickUpGoal = createGoal(2.0, 4.0, -0.000246, 1);
  move_base_msgs::MoveBaseGoal pickUpGoal = createGoal(2.0, 5.0, -0.000246, 1);
  move_base_msgs::MoveBaseGoal dropOffGoal = createGoal(-5.0, 5.0, -0.000247, 1);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(pickUpGoal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("pickup reached the goal!");
    //wait for 5 seconds
    ros::Duration(5.0).sleep();

    ROS_INFO("Sending Drop Off Goal");
    ac.sendGoal(dropOffGoal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Robot reached the drop off location!");
    } else {
       ROS_INFO("Robot fail to reach the drop off location!");
    }
  } else {
    ROS_INFO("pickup unable to reach the goal!");
  }

  return 0;
}
