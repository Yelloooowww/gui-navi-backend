#include "sub_artimap.h"
#include "mainwindow.h"

sub_artimap::sub_artimap(QObject *parent, NodeHandle *nh, MainWindow *gui, string topic_name, bool use_service) : QThread(parent)
{
  this->nh = nh;
  this->gui = gui;
  this->topic_name = topic_name;
  this->use_service = use_service;
  b_count = 0;
  g_count = 0;
  v_count = 0;
  s_count = 0;
  p_count = 0;

  // stringstream ss;
  // ss << "subscriber to " << topic_name;
  // ROS_INFO(ss.str().c_str());
}

void sub_artimap::run()
{
  if (this->use_service)
  {
    timer = nh->createTimer(ros::Duration(2), &sub_artimap::timer_callback, this);
  }
  else
  {
    sub = nh->subscribe<subt_msgs::ArtifactPoseArray>(topic_name, 1, &sub_artimap::cb_arti, this);
  }
  ros::AsyncSpinner spinner(1);
  ros::Rate r(10);

  spinner.start();
  ros::waitForShutdown();
}

void sub_artimap::timer_callback(const ros::TimerEvent &event)
{
  subt_msgs::artifact arti;
  if (ros::service::call("artifact_srv", arti))
  {
    // ROS_INFO("receive artifacts: %d",arti.response.artifacts.count);
    for (int i = 0; i < arti.response.artifacts.count; i++)
    {
      stringstream ss, sst;
      sst << arti.response.artifacts.pose_array[i].Class << ", ";
      sst << arti.response.artifacts.pose_array[i].pose.position.x << ", ";
      sst << arti.response.artifacts.pose_array[i].pose.position.y << ", ";
      sst << arti.response.artifacts.pose_array[i].pose.position.z << ", ";
      //ROS_INFO(sst.str().c_str());

      if (arti.response.artifacts.pose_array[i].Class == "backpack")
      {
        ss << b_count++ << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.x << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.y << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.z << ",";
        ss << "Backpack"
           << ",";
        ss << arti.response.artifacts.pose_array[i].appear_count << ",";
        emit gui->add_to_backpack(QString::fromStdString(ss.str()));
      }
      else if (arti.response.artifacts.pose_array[i].Class == "survivor")
      {
        ss << s_count++ << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.x << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.y << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.z << ",";
        ss << "Survivor"
           << ",";
        ss << arti.response.artifacts.pose_array[i].appear_count << ",";
        emit gui->add_to_survivor(QString::fromStdString(ss.str()));
      }
      else if (arti.response.artifacts.pose_array[i].Class == "vent")
      {
        ss << v_count++ << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.x << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.y << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.z << ",";
        ss << "Vent"
           << ",";
        ss << arti.response.artifacts.pose_array[i].appear_count << ",";
        emit gui->add_to_vent(QString::fromStdString(ss.str()));
      }
      else if (arti.response.artifacts.pose_array[i].Class == "phone")
      {
        ss << p_count++ << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.x << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.y << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.z << ",";
        ss << "Cell Phone"
           << ",";
        ss << arti.response.artifacts.pose_array[i].appear_count << ",";
        emit gui->add_to_phone(QString::fromStdString(ss.str()));
      }
      else if (arti.response.artifacts.pose_array[i].Class == "CO2")
      {
        ss << g_count++ << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.x << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.y << ",";
        ss << arti.response.artifacts.pose_array[i].pose.position.z << ",";
        ss << "Gas"
           << ",";
        ss << arti.response.artifacts.pose_array[i].appear_count << ",";
        emit gui->add_to_co2(QString::fromStdString(ss.str()));
      }
    }
  }
  else
  {
    ROS_INFO("fail to call artifact service");
  }
}

void sub_artimap::cb_arti(const subt_msgs::ArtifactPoseArray::ConstPtr &msg)
{
  for (int i = 0; i < msg->count; i++)
  {
    stringstream ss, sst;
    sst << msg->pose_array[i].Class << ", ";
    sst << msg->pose_array[i].pose.position.x << ", ";
    sst << msg->pose_array[i].pose.position.y << ", ";
    sst << msg->pose_array[i].pose.position.z << ", ";
    // ROS_INFO(sst.str().c_str());

    if (msg->pose_array[i].Class == "backpack")
    {
      ss << b_count++ << ",";
      ss << msg->pose_array[i].pose.position.x << ",";
      ss << msg->pose_array[i].pose.position.y << ",";
      ss << msg->pose_array[i].pose.position.z << ",";
      ss << "Backpack"
         << ",";
      ss << msg->pose_array[i].appear_count << ",";
      emit gui->add_to_backpack(QString::fromStdString(ss.str()));
    }
    else if (msg->pose_array[i].Class == "survivor")
    {
      ss << s_count++ << ",";
      ss << msg->pose_array[i].pose.position.x << ",";
      ss << msg->pose_array[i].pose.position.y << ",";
      ss << msg->pose_array[i].pose.position.z << ",";
      ss << "Survivor"
         << ",";
      ss << msg->pose_array[i].appear_count << ",";
      emit gui->add_to_survivor(QString::fromStdString(ss.str()));
    }
    else if (msg->pose_array[i].Class == "vent")
    {
      ss << v_count++ << ",";
      ss << msg->pose_array[i].pose.position.x << ",";
      ss << msg->pose_array[i].pose.position.y << ",";
      ss << msg->pose_array[i].pose.position.z << ",";
      ss << "Vent"
         << ",";
      ss << msg->pose_array[i].appear_count << ",";
      emit gui->add_to_vent(QString::fromStdString(ss.str()));
    }
    else if (msg->pose_array[i].Class == "phone")
    {
      ss << p_count++ << ",";
      ss << msg->pose_array[i].pose.position.x << ",";
      ss << msg->pose_array[i].pose.position.y << ",";
      ss << msg->pose_array[i].pose.position.z << ",";
      ss << "Cell Phone"
         << ",";
      ss << msg->pose_array[i].appear_count << ",";
      emit gui->add_to_phone(QString::fromStdString(ss.str()));
    }
    else if (msg->pose_array[i].Class == "CO2")
    {
      ss << g_count++ << ",";
      ss << msg->pose_array[i].pose.position.x << ",";
      ss << msg->pose_array[i].pose.position.y << ",";
      ss << msg->pose_array[i].pose.position.z << ",";
      ss << "Gas"
         << ",";
      ss << msg->pose_array[i].appear_count << ",";
      emit gui->add_to_co2(QString::fromStdString(ss.str()));
    }
  }
}
