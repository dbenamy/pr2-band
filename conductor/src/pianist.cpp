#include <ros/ros.h>
#include <stdio.h>
#include <conductor/Song.h>
#include <conductor/arm.h>
#include <conductor/head.h>
#include <std_msgs/String.h>

using namespace std;
using namespace ros;

double NATURAL_X = 0.43;
double PREP_HEIGHT = 0.925;
double FLAT_X = 0.47;
double FLAT_PREP_HEIGHT = 0.935;
double MID_KBD = -0.4;
double MOVE_SPEED = 0.8 / 4;
double move_time = 0.5;
double hit_time = 0.2;
double HIT_HEIGHT = 0.015;
double prep_pose[6] = {0, 0, 0, -0.1, 0, 0};
double hit_pose[6]  = {0, 0, 0, -0.1, 0, 0};

double DRUMS_START_MID[7] = {1.5837059819274157, 0.88477830074949138, 0.60882772339705271, -1.6672974898423472, 0.20512577032668747, -0.10723804504697487, 1.5487932443509185};
double PIANO_START_MID[7] = {-0.74999703778556692, 0.50765413354983246, 0.18909789261950177, -1.9310697259831349, 66.378375334029158, -1.1785883062193605, -34.819901405579095};
//double PIANO_START_MID[7] = {-0.71335231291377921, 0.73377636300803983, 0.088715771311952393, -2.1254967913712122, 72.851059434188215, -1.4433400850253904, -34.761294890204944};
double PIANO_START_DOWN[7] = {-1.4861248932982436, 0.96971160354636399, -0.5903869183645154, 0.0027774366780290516, 75.083552060871199, -0.026385084141899995, -36.187038692948555};
//double DRUMS_START_MID[7] = {0.64636376608933488, 1.0382334912609721, -0.27536878920649999, -2.3207924810275915, 3.1899776098886585, -1.356944757067625, -1.6763924758717161};
double DRUMS_START_DOWN[7] = {1.583540168692702, 0.88503208551880463, 0.60946914270253216, -0.08394021944070422, 0.20524146450058869, -0.026746542862725597, 1.5488802621911169};
double DRUMS_START_UP[7] = {0.54911430392959049, 1.0293510243350044, 0.03187105811820206, -2.0916204449074671, 3.0962653290287503, -1.0480749332806045, 4.6146057902487385};
double PIANO_START_UP[7] = {-0.37260611557656775, 0.78258763363931017, 3.9552329405652945e-05, -1.9304906431376005, 66.378317486942223, -1.178501288379163, -34.819901405579103};

double key_to_y[128][3];
Arm* arm;
Arm* left_arm;
Head* head;


void nap(double t) {
  ros::Duration(t).sleep();
}


void move_arm(double pose[6], double move_time)
{
//  printf("%f, %f, %f, %f, %f, %f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
  arm->sendArmToPose(pose, move_time);
  nap(move_time);
}


void prep_key(int key)
{
  prep_pose[0] = key_to_y[key][0];
  double old_y = prep_pose[1];
  double new_y = key_to_y[key][1];
//  printf("prep %d (y=%f)\n", key, new_y);
  double dy = abs(old_y - new_y);
  double move_time = dy / MOVE_SPEED + 0.2; // 0.2 - 4.2
  if (dy > 5.0 / 42.0) {
    double mid_y = dy / 2 + min(old_y, new_y);
//    printf("from %.3f to %.3f mid %.3f\n", old_y, new_y, mid_y);
    prep_pose[1] = mid_y;
    double mid_z = key_to_y[key][2] + 0.1 * dy / 0.8;
    prep_pose[2] = mid_z;
    move_arm(prep_pose, move_time / 2);
    prep_pose[1] = key_to_y[key][1];
    prep_pose[2] = key_to_y[key][2];
    move_arm(prep_pose, move_time / 2);
  }
  else {
    prep_pose[1] = key_to_y[key][1];
    prep_pose[2] = key_to_y[key][2];
    move_arm(prep_pose, move_time);
  }
}

void hit_key(int key)
{
//  printf("hit\n");
  for (int i = 0; i < 6; i++) {
    hit_pose[i] = prep_pose[i];
  }
  hit_pose[2] = prep_pose[2] - HIT_HEIGHT;
  move_arm(hit_pose, hit_time);
  move_arm(prep_pose, hit_time);
}


void song_callback(const conductor::SongConstPtr& s)
{
  ROS_INFO("Received song with %d notes.", int(s->song.size()));

  double start_time = s->start_sec;

  for(unsigned int i = 0; i < s->song.size() && ros::ok(); i++){
    int key = atoi(s->song[i].key.c_str());
    if(key_to_y[key][1] == 0) {
      printf("WARNING: Skipping midi key %d\n", key);
      continue;
    }
    prep_key(key);

    double wait_time = s->song[i].time.toSec() - (ros::Time::now().toSec() - start_time);
//    printf("next note in %f secs\n", wait_time);
    if (wait_time < 0) {
      printf("WARNING: Missed note's start time (%f).\n", wait_time);
    } else {
      nap(wait_time);
    }

    hit_key(key);
//    ros::Duration(2.0).sleep();
  }
  printf("Done playing song.\n");
}

double key_i_to_y(int i) {
  return -(double(i) / 43 + .05);
}

void play_mary()
{
  prep_key(64);
  hit_key(64);
  prep_key(62);
  hit_key(62);
  prep_key(60);
  hit_key(60);
  prep_key(62);
  hit_key(62);
  prep_key(64);
  hit_key(64);
  prep_key(64);
  hit_key(64);
  prep_key(64);
  hit_key(64);
  prep_key(62);
  hit_key(62);
  prep_key(62);
  hit_key(62);
  prep_key(62);
  hit_key(62);
  prep_key(64);
  hit_key(64);
  prep_key(67);
  hit_key(67);
  prep_key(67);
  hit_key(67);
  prep_key(64);
  hit_key(64);
  prep_key(62);
  hit_key(62);
  prep_key(60);
  hit_key(60);
  prep_key(62);
  hit_key(62);
  prep_key(64);
  hit_key(64);
  prep_key(64);
  hit_key(64);
  prep_key(64);
  hit_key(64);
  prep_key(64);
  hit_key(64);
  prep_key(62);
  hit_key(62);
  prep_key(62);
  hit_key(62);
  prep_key(64);
  hit_key(64);
  prep_key(62);
  hit_key(62);
  prep_key(60);
  hit_key(60);
}


void go_to_mid_c()
{
  prep_key(60); nap(2); hit_key(60); // middle c
}

void go_to_low()
{
  prep_key(36); nap(2); hit_key(36);
}

void go_to_high()
{
  prep_key(95); nap(2); hit_key(95);
}


void test_range()
{
  prep_key(36); nap(2); hit_key(0); // low key
  prep_key(60); nap(2); hit_key(0); // middle c
  prep_key(73); nap(2); hit_key(0); // sharp
  prep_key(95); nap(2); hit_key(0); // high key
  prep_key(66); nap(2); hit_key(0); // sharp

  prep_key(39); nap(2); hit_key(0); // d sharp
  prep_key(56); nap(2); hit_key(0); // g sharp
  prep_key(70); nap(2); hit_key(0); // a sharp
}

void arms_down()
{ 
  head->lookAt("base_link", 1.0, 0.0, 1.2);
  nap(2.0);
  arm->sendArmToConfiguration(PIANO_START_MID,2);
  left_arm->sendArmToConfiguration(DRUMS_START_MID,2);
  nap(2.0);
  arm->sendArmToConfiguration(PIANO_START_DOWN,2);
  left_arm->sendArmToConfiguration(DRUMS_START_DOWN,2);
  nap(2.0);  
  head->lookAt("base_link", 1.0, 0.0, 0.55);
  nap(1.0);
  head->lookAt("base_link", 1.0, 0.0, 1.2);
}

void arms_up()
{ 
  head->lookAt("base_link", 1.0, 0.0, 0.55);
  nap(5.0);
  head->lookAt("base_link", 1.0, 0.0, 1.2);
  nap(1);
  arm->sendArmToConfiguration(PIANO_START_MID,2);
  left_arm->sendArmToConfiguration(DRUMS_START_MID,2);
  nap(2.0);
  left_arm->sendArmToConfiguration(DRUMS_START_UP,2);
  arm->sendArmToConfiguration(PIANO_START_UP,2);
  nap(2.0);
  head->lookAt("base_link", 1.0, 0.0, 0.55);
}

void util_callback(const std_msgs::StringConstPtr& s)
{
  if (s->data == "mid_c") go_to_mid_c();
  if (s->data == "low") go_to_low();
  if (s->data == "high") go_to_high();
  if (s->data == "test_range") test_range();
  if (s->data == "play_mary") play_mary();
  
  if (s->data == "arms_down") arms_down();
  if (s->data == "arms_up") arms_up();
//  if (s->data == "look_down") look_down();
//  if (s->data == "look_up") look_up();
//  if (s->data == "bow") bow();
}


int main(int argc, char** argv){
  int midi_key_template[7] = {0, 2, 4, 5, 7, 9, 11};
  int key_i = 0;

  //Naturals
  for (int i = 2; i <= 6; i++) {
    for (int j = 0; j < 7; j++) {
      int midi_key = (i + 1) * 12 + midi_key_template[j];
      key_to_y[midi_key][0] = NATURAL_X;
      key_to_y[midi_key][1] = key_i_to_y(key_i++);
      key_to_y[midi_key][2] = PREP_HEIGHT;
    }
  }

  //C#
  for(int i=1; i<122; i+=12){
    key_to_y[i][0] = FLAT_X;
    key_to_y[i][1] = key_to_y[i-1][1]*.75 + key_to_y[i+1][1]*.25;
    key_to_y[i][2] = FLAT_PREP_HEIGHT;
  }

  //D#
  for(int i=3; i<124; i+=12){
    key_to_y[i][0] = FLAT_X;
    key_to_y[i][1] = key_to_y[i-1][1]*.25 + key_to_y[i+1][1]*.75;
    key_to_y[i][2] = FLAT_PREP_HEIGHT;
  }

  //F#
  for(int i=6; i<127; i+=12){
    key_to_y[i][0] = FLAT_X;
    key_to_y[i][1] = key_to_y[i-1][1]*.75 + key_to_y[i+1][1]*.25;
    key_to_y[i][2] = FLAT_PREP_HEIGHT;
  }
  
  //G#
  for(int i=8; i<117; i+=12){
    key_to_y[i][0] = FLAT_X;
    key_to_y[i][1] = key_to_y[i-1][1]*.5 + key_to_y[i+1][1]*.5;
    key_to_y[i][2] = FLAT_PREP_HEIGHT;
  }

  //A#
  for(int i=10; i<119; i+=12){
    key_to_y[i][0] = FLAT_X;
    key_to_y[i][1] = key_to_y[i-1][1]*.25 + key_to_y[i+1][1]*.75;
    key_to_y[i][2] = FLAT_PREP_HEIGHT;
  }
  
ros::init(argc, argv, "pianist");
  arm = new Arm("right");
  left_arm = new Arm("left");
  head = new Head();

  ros::Subscriber song_sub = ros::NodeHandle().subscribe("song_piano", 1, &song_callback);
  ros::Subscriber song_sub2 = ros::NodeHandle().subscribe("piano_util", 1, &util_callback);

//  prep_key(66); return 0;
  ros::spin();

  delete arm;
  delete left_arm;
  delete head;
  return 0;
}

