#ifndef SARA_SHIELD_PANEL2_H
#define SARA_SHIELD_PANEL2_H

#include <ros/ros.h>

#include <rviz/panel.h>
#include "std_msgs/Bool.h"

class QLineEdit;
class QLabel;
class QTimer;

namespace rviz_plugin_sara_shield
{

class DriveWidget;

class SaraShieldPanel: public rviz::Panel
{

Q_OBJECT
public:
  SaraShieldPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

protected Q_SLOTS:

  void publishGoalPose();

  void resetSafeLabel();

  void sendNoHumanInScene(bool on);

  void forceSave(bool on);

  void forceUnsave(bool on);

  void sendDummyHuman(bool on);

  void safeFlagCallback(const std_msgs::Bool & msg);


  //protected member variables
protected:

  std::vector<QLineEdit*> goal_line_edits_;
  QLabel* safe_status_label_;

  QTimer* safe_flag_timer_;

  // The ROS publishers and subsribers
  ros::Publisher goal_pub_;
  ros::Publisher no_human_in_scene_pub_;
  ros::Publisher force_safe_pub_;
  ros::Publisher force_unsafe_pub_;
  ros::Publisher send_dummy_human_pub_;
  ros::Subscriber safe_flag_sub_;

  // The ROS node handle.
  ros::NodeHandle nh_;

};

} // end namespace rviz_plugin_sara_shield

#endif // SARA_SHIELD_PANEL_H
