#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <gate_detect/GateCVAction.h>
#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_gate_detect");

  // create the action client
  actionlib::SimpleActionClient<gate_detect::GateCVAction> ac("gate_detect");
  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  gate_detect::GateCVGoal goal;
  goal.samples = 1;
  ac.sendGoal(goal);


  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  ac.cancelAllGoals();

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;
}
