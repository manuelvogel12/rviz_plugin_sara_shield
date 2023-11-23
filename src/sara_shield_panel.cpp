#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QRadioButton>
#include <QButtonGroup>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

#include "sara_shield_panel.h"

namespace rviz_plugin_sara_shield
{

SaraShieldPanel::SaraShieldPanel( QWidget* parent )
  : rviz::Panel( parent )
{

  // Box #1: status of sara shield
  QHBoxLayout* status_layout = new QHBoxLayout;
  status_layout->addWidget( new QLabel( "Sara-shield status:" ));
  safe_status_label_ = new QLabel("-");
  safe_status_label_->setStyleSheet("border: 1px solid black;");
  status_layout->addWidget(safe_status_label_);

  // Box #2: Label for Goal Joint Pos
  QHBoxLayout* goal_text_layout = new QHBoxLayout;
  goal_text_layout->addWidget( new QLabel( "Goal Joint Pos:" ));

  // Box #3: Input Boxes for Goal Joint Pos
  QHBoxLayout* goal_input_layout = new QHBoxLayout;
  int number_robot_joints = 6;
  for(int i=0; i<number_robot_joints; i++){
    QLineEdit* line_edit = new QLineEdit("0.0");
    goal_line_edits_.push_back(line_edit);
    goal_input_layout->addWidget( line_edit);
  }

  // Box #4: Button for Goal Joint Pos
  QHBoxLayout* goal_send_layout = new QHBoxLayout;
  QPushButton* goal_push_button = new QPushButton("Send Goal Pos");
  goal_send_layout->addWidget( goal_push_button);

  // Box #5: Various other buttons
  QHBoxLayout* addional_button_layout = new QHBoxLayout;
  QButtonGroup* non_exclusive_button_group = new QButtonGroup;
  non_exclusive_button_group->setExclusive(false);
  QRadioButton* no_human_button = new QRadioButton("No Human in Scene");
  QRadioButton* force_safe_button = new QRadioButton("Force safe");
  QRadioButton* force_unsafe_button = new QRadioButton("Force unsafe");
  force_unsafe_button->setEnabled(false); // not inmplemented yet
  QRadioButton* send_dummy_human_button = new QRadioButton("Send Dummy Human");
  non_exclusive_button_group->addButton(no_human_button);
  non_exclusive_button_group->addButton(force_safe_button);
  non_exclusive_button_group->addButton(force_unsafe_button);
  non_exclusive_button_group->addButton(send_dummy_human_button);
  addional_button_layout->addWidget( no_human_button);
  addional_button_layout->addWidget( force_safe_button);
  addional_button_layout->addWidget( force_unsafe_button);
  addional_button_layout->addWidget( send_dummy_human_button);


  // Add all boxes to the layout
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( status_layout );
  layout->addLayout( goal_text_layout );
  layout->addLayout( goal_input_layout );
  layout->addLayout( goal_send_layout );
  layout->addLayout( addional_button_layout );
  setLayout( layout );

  // Have a timer to see if sara shield is running
  safe_flag_timer_ = new QTimer( this );
  safe_flag_timer_->setSingleShot(true);

  // Signal/slot connections.
  connect( goal_push_button, SIGNAL(clicked()), this, SLOT(publishGoalPose()));
  connect(safe_flag_timer_, SIGNAL(timeout()), this, SLOT(resetSafeLabel()));
  connect( no_human_button, SIGNAL(clicked(bool)), this, SLOT(sendNoHumanInScene(bool)));
  connect( force_safe_button, SIGNAL(clicked(bool)), this, SLOT(forceSave(bool)));
  connect( force_unsafe_button, SIGNAL(clicked(bool)), this, SLOT(forceUnsave(bool)));
  connect( send_dummy_human_button, SIGNAL(clicked(bool)), this, SLOT(sendDummyHuman(bool)));

  //ROS pubs and subs
  goal_pub_ = nh_.advertise<std_msgs::Float32MultiArray>( "sara_shield/goal_joint_pos", 1 );
  no_human_in_scene_pub_ = nh_.advertise<std_msgs::Bool>( "sara_shield/humans_in_scene", 1 );
  force_safe_pub_ = nh_.advertise<std_msgs::Bool>( "sara_shield/safe_flag", 1 );
  send_dummy_human_pub_ = nh_.advertise<std_msgs::Bool>( "sara_shield/send_dummy_meas", 1 );

  safe_flag_sub_ = nh_.subscribe("/sara_shield/is_safe", 100, & SaraShieldPanel::safeFlagCallback, this);
}


void SaraShieldPanel::safeFlagCallback(const std_msgs::Bool & msg){
    if(msg.data){
        safe_status_label_->setText("SAFE");
        safe_status_label_->setStyleSheet("border: 2px solid green;");
    }
    else{
        safe_status_label_->setText("NOT SAFE");
        safe_status_label_->setStyleSheet("border: 2px solid red;");
    }
    safe_flag_timer_->start(200);
}


void SaraShieldPanel::resetSafeLabel(){
  safe_status_label_->setText("-");
  safe_status_label_->setStyleSheet("border: 2px solid black;");
}


void SaraShieldPanel::publishGoalPose(){
    std_msgs::Float32MultiArray goal_pos_msg;
    for(QLineEdit* le: goal_line_edits_){
        double a = le->text().toDouble();
        goal_pos_msg.data.push_back(a);
    }
    goal_pub_.publish(goal_pos_msg);
    std::cout<<"goal pose message send"<<std::endl;
}


void SaraShieldPanel::sendNoHumanInScene(bool on){
  std_msgs::Bool msg;
  msg.data=!on; // button: on when no human in scene, msg: on, when human in scene
  no_human_in_scene_pub_.publish(msg);
}

void SaraShieldPanel::forceSave(bool on){
  std_msgs::Bool msg;
  msg.data=on;
  force_safe_pub_.publish(msg);
}

void SaraShieldPanel::forceUnsave(bool on){
  std::cout<<"NOT IMPLEMENTED YET"<<std::endl;
}

void SaraShieldPanel::sendDummyHuman(bool on){
  std_msgs::Bool msg;
  msg.data=on;
  send_dummy_human_pub_.publish(msg);
}



// Save all configuration data from this panel to the given
// Config object.
void SaraShieldPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  //config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void SaraShieldPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  //if( config.mapGetString( "Topic", &topic ))
  //{
   // output_topic_editor_->setText( topic );
    //updateTopic();
  //}
}

} // end namespace rviz_plugin_sara_shield

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_sara_shield::SaraShieldPanel, rviz::Panel )
