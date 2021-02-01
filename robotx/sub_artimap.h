#ifndef SUB_ARTIMAP_H
#define SUB_ARTIMAP_H

#include <QObject>
#include <QThread>
#include <QImage>
#include <QCheckBox>

#include <sstream>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "subt_msgs/artifact.h"
#include "subt_msgs/ArtifactPose.h"
#include "subt_msgs/ArtifactPoseArray.h"

using namespace std;
using namespace ros;

class MainWindow;

class sub_artimap : public QThread
{

  Q_OBJECT

public:
  sub_artimap(QObject *parent, NodeHandle *nh, MainWindow *gui, string topic_name, bool use_service);
  void run();
  void cb_arti(const subt_msgs::ArtifactPoseArray::ConstPtr &msg);
  void timer_callback(const ros::TimerEvent &event);

private:
  ros::Timer timer;
  NodeHandle *nh;
  MainWindow *gui;
  Subscriber sub;
  string topic_name;
  QImage *img;
  bool use_service;
  int my_id;
  int b_count;
  int v_count;
  int g_count;
  int s_count;
  int p_count;
};

#endif // SUB_ARTIMAP_H
