#include <ros/ros.h>
#include <stdio.h>
#include <conductor/Song.h>
#include <conductor/Note.h>

using namespace std;
using namespace ros;
using namespace conductor;

int main(int argc, char** argv){
  ros::init(argc, argv, "send_song");

  Publisher piano = ros::NodeHandle().advertise<conductor::Song>("song_piano", 1);
  Publisher drum = ros::NodeHandle().advertise<conductor::Song>("song_drum", 1);
  Duration(5.0).sleep();

  Song p;
  Note n;
  n.time = Duration(1);
  n.key = "60";
  p.song.push_back(n);
  printf("%f\n", n.time.toSec());

  Song d;
  n.time = Duration(1);
  n.key = "hi-hat";
  d.song.push_back(n);

/*
  n.time = Duration(15);
  n.key = "snare";
  s.song.push_back(n);
*/

  piano.publish(p);
  drum.publish(d);
  ROS_INFO("sent song");
}
