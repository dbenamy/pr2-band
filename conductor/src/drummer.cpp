#include <ros/ros.h>
#include <stdio.h>
#include <conductor/Song.h>
#include <conductor/arm.h>
#include <conductor/head.h>

using namespace std;
using namespace ros;

double snare_prep[7]  = {0.3074, 1.2040, 0.0275, -2.0793, 3.1659, -0.8600, -1.6435};
double snare_hit[7]   = {0.3074, 1.2040, 0.0275, -2.0793, 3.1659, -1.0600, -1.6435};
double hi_hat_prep[7] = {0.5500, 1.2040, 0.0275, -2.0793, 3.1659, -0.7800, -1.6435};
double hi_hat_hit[7]  = {0.5500, 1.2040, 0.0275, -2.0793, 3.1659, -0.9800, -1.6435};
double move_time = 0.5;
double hit_time = 0.1;

double c4[7] = {-1.2078073788308425, -0.0064292141559385571, -1.5863507449478789, -1.3163732854486758, -1.6127767841816274, -0.022295245652539302, -0.054968978400179863};
double d4[7] = {-1.2143570016020442, -0.0068521887714608322, -1.58474719668418, -1.2809044611597116, -1.5883074664015417, -0.019728219366662608, -0.094257533250113656};
double e4[7] = {-1.2170929199748248, -0.0060908344635207589, -1.5855489708160295, -1.2368941648991192, -1.5884231605754422, -0.023165424054531786, -0.093082792407427201};
double f4[7] = {-1.2146886280714724, -0.0068521887714608322, -1.5844264870314404, -1.1914361615246916, -1.5884231605754422, -0.023165424054531786, -0.093082792407427201};
double g4[7] = {-1.2170929199748248, -0.0062600243097296008, -1.5786537132821241, -1.1488735723779342, -1.5871505246625297, -0.023165424054531786, -0.093082792407427201};
double a5[7] = {-1.2186681457046076, -0.0056678598479984257, -1.5690324236999311, -1.1055871296742594, -1.5871505246625297, -0.023165424054531786, -0.093082792407427201};
double b5[7] = {-1.2216527839294591, -0.0054986700017895275, -1.5677495850889718, -1.0647617890641046, -1.5871505246625297, -0.023165424054531786, -0.093082792407427201};
double c5[7] = {-1.2192484920261064, -0.0043143410783271783, -1.5677495850889718, -1.0169874543075406, -1.5871505246625297, -0.023165424054531786, -0.093082792407427201};
double d5[7] = {-1.2223160368683148, -0.010320580618743459, -1.5693531333526709, -0.9797813814819738, -1.6279327209626751, -0.027342280384091172, -0.070197100435040766};
double e5[7] = {-1.2228134765724568, -0.011504909542205809, -1.5640614240824646, -0.93359952455062856, -1.6279327209626751, -0.027342280384091172, -0.070197100435040766};

double c4_down[7] = {-1.2078073788308425, -0.0064292141559385571, -1.5863507449478789, -1.3163732854486758, -1.6127767841816274, -0.37436942709845472, -0.054968978400179863};
double d4_down[7] = {-1.2143570016020442, -0.0068521887714608322, -1.5847471966841800, -1.2809044611597116, -1.5883074664015417, -0.37436942709845472, -0.094257533250113656};
double e4_down[7] = {-1.2170929199748248, -0.0060908344635207589, -1.5855489708160295, -1.2368941648991192, -1.5884231605754422, -0.37436942709845472, -0.093082792407427201};
double f4_down[7] = {-1.2146886280714724, -0.0068521887714608322, -1.5844264870314404, -1.1914361615246916, -1.5884231605754422, -0.37436942709845472, -0.093082792407427201};
double g4_down[7] = {-1.2170929199748248, -0.0062600243097296008, -1.5786537132821241, -1.1488735723779342, -1.5871505246625297, -0.37436942709845472, -0.093082792407427201};
double a5_down[7] = {-1.2186681457046076, -0.0056678598479984257, -1.5690324236999311, -1.1055871296742594, -1.5871505246625297, -0.37436942709845472, -0.093082792407427201};
double b5_down[7] = {-1.2216527839294591, -0.0054986700017895275, -1.5677495850889718, -1.0647617890641046, -1.5871505246625297, -0.37436942709845472, -0.093082792407427201};
double c5_down[7] = {-1.2192484920261064, -0.0043143410783271783, -1.5677495850889718, -1.0169874543075406, -1.5871505246625297, -0.37436942709845472, -0.093082792407427201};
double d5_down[7] = {-1.2223160368683148, -0.0103205806187434590, -1.5693531333526709, -0.9797813814819738, -1.6279327209626751, -0.37436942709845472, -0.070197100435040766};
double e5_down[7] = {-1.2228134765724568, -0.011504909542205809, -1.5640614240824646, -0.93359952455062856, -1.6279327209626751, -0.37436942709845472, -0.070197100435040766};




Arm* left_arm;
Arm* right_arm;
Head* head;


void nap(double t) {
  ros::Duration(t).sleep();
}


void prep_hi_hat()
{
  left_arm->sendArmToConfiguration(hi_hat_prep, move_time);
  nap(move_time);
}

void hit_hi_hat()
{
  nap(hit_time);
  left_arm->sendArmToConfiguration(hi_hat_hit, hit_time);
  nap(hit_time);
}

void prep_snare()
{
  left_arm->sendArmToConfiguration(snare_prep, move_time);
  nap(move_time);
}

void hit_snare()
{
  nap(hit_time);
  left_arm->sendArmToConfiguration(snare_hit, hit_time);
  nap(hit_time);
}


void prep_c4(){ right_arm->sendArmToConfiguration(c4, move_time); nap(move_time); }
void prep_d4(){ right_arm->sendArmToConfiguration(d4, move_time); nap(move_time); }
void prep_e4(){ right_arm->sendArmToConfiguration(e4, move_time); nap(move_time); }
void prep_f4(){ right_arm->sendArmToConfiguration(f4, move_time); nap(move_time); }
void prep_g4(){ right_arm->sendArmToConfiguration(g4, move_time); nap(move_time); }
void prep_a5(){ right_arm->sendArmToConfiguration(a5, move_time); nap(move_time); }
void prep_b5(){ right_arm->sendArmToConfiguration(b5, move_time); nap(move_time); }
void prep_c5(){ right_arm->sendArmToConfiguration(c5, move_time); nap(move_time); }
void prep_d5(){ right_arm->sendArmToConfiguration(d5, move_time); nap(move_time); }
void prep_e5(){ right_arm->sendArmToConfiguration(e5, move_time); nap(move_time); }

void hit_c4(){ right_arm->sendArmToConfiguration(c4_down, hit_time); nap(hit_time); }
void hit_d4(){ right_arm->sendArmToConfiguration(d4_down, hit_time); nap(hit_time); }
void hit_e4(){ right_arm->sendArmToConfiguration(e4_down, hit_time); nap(hit_time); }
void hit_f4(){ right_arm->sendArmToConfiguration(f4_down, hit_time); nap(hit_time); }
void hit_g4(){ right_arm->sendArmToConfiguration(g4_down, hit_time); nap(hit_time); }
void hit_a5(){ right_arm->sendArmToConfiguration(a5_down, hit_time); nap(hit_time); }
void hit_b5(){ right_arm->sendArmToConfiguration(b5_down, hit_time); nap(hit_time); }
void hit_c5(){ right_arm->sendArmToConfiguration(c5_down, hit_time); nap(hit_time); }
void hit_d5(){ right_arm->sendArmToConfiguration(d5_down, hit_time); nap(hit_time); }
void hit_e5(){ right_arm->sendArmToConfiguration(e5_down, hit_time); nap(hit_time); }

void songCallback(const conductor::SongConstPtr& s){

  ROS_INFO("Received song with %d notes.", s->song.size());

  double start_time = s->start_sec;

  for(unsigned int i = 0; i < s->song.size() && ros::ok(); i++){
    //printf("playing %s\n", s->song[i].key.c_str());
    if(s->song[i].key == "snare"){
      printf("prep snare\n");
      prep_snare();
    }
    else if(s->song[i].key == "hi-hat")
    {
      printf("prep hi-hat\n");
      prep_hi_hat();
      //head->tilt(1);
    }
    else if(s->song[i].key == "c4")
    {
      printf("prep c4\n");
      prep_c4();
    }
    else if(s->song[i].key == "d4")
    {
      printf("prep d4\n");
      prep_d4();
    }
    else if(s->song[i].key == "e4")
    {
      printf("prep e4\n");
      prep_f4();
    }
    else if(s->song[i].key == "g4")
    {
      printf("prep g4\n");
      prep_g4();
    }
    else if(s->song[i].key == "a5")
    {
      printf("prep a5\n");
      prep_a5();
    }
    else if(s->song[i].key == "b5")
    {
      printf("prep b5\n");
      prep_b5();
    }
    else if(s->song[i].key == "c5")
    {
      printf("prep c5\n");
      prep_c5();
    }
    else if(s->song[i].key == "d5")
    {
      printf("prep d5\n");
      prep_d5();
    }
    else if(s->song[i].key == "e5")
    {
      printf("prep e5\n");
      prep_e5();
    }

    double wait_time = s->song[i].time.toSec() - (ros::Time::now().toSec() - start_time);
    printf("next note in %f secs\n", wait_time);
    if(wait_time > 0) {
      nap(wait_time);
    }

    if(s->song[i].key == "snare"){
      printf("hit snare\n");
      hit_snare();
    }
    else if(s->song[i].key == "hi-hat")
    {
      printf("hit hi-hat\n");
      hit_hi_hat();
      //head->tilt(8);
    }
    else if(s->song[i].key == "c4")
    {
      printf("hit c4\n");
      hit_c4();
    }
    else if(s->song[i].key == "d4")
    {
      printf("hit d4\n");
      hit_d4();
    }
    else if(s->song[i].key == "e4")
    {
      printf("hit e4\n");
      hit_f4();
    }
    else if(s->song[i].key == "g4")
    {
      printf("hit g4\n");
      hit_g4();
    }
    else if(s->song[i].key == "a5")
    {
      printf("hit a5\n");
      hit_a5();
    }
    else if(s->song[i].key == "b5")
    {
      printf("hit b5\n");
      hit_b5();
    }
    else if(s->song[i].key == "c5")
    {
      printf("hit c5\n");
      hit_c5();
    }
    else if(s->song[i].key == "d5")
    {
      printf("hit d5\n");
      hit_d5();
    }
    else if(s->song[i].key == "e5")
    {
      printf("hit e5\n");
      hit_e5();
    }
//    ros::Duration(2.0).sleep();
  }
  printf("done playing song\n");

}

int main(int argc, char** argv){

  ros::init(argc,argv,"drummer");
  left_arm = new Arm("left");
  right_arm = new Arm("right");
  head = new Head();
  ros::Subscriber song_sub = ros::NodeHandle().subscribe("song_drum", 1, &songCallback);
  
//  double angle = atof(argv[1]);
//  hi_hat_hit[5] = angle;
/*
  for (int i = 0; i < 20; i++) {
    prep_hi_hat();
    hit_hi_hat();
    prep_hi_hat();
    hit_hi_hat();
    prep_hi_hat();
    hit_hi_hat();
    prep_snare();
    hit_snare();
  }
*/
/*
  prep_c4();
  hit_c4();
  prep_d4();
  hit_d4();
  prep_e4();
  hit_e4();
  prep_f4();
  hit_f4();
  prep_g4();
  hit_g4();
  prep_a5();
  hit_a5();
  prep_b5();
  hit_b5();
  prep_c5();
  hit_c5();
  prep_d5();
  hit_d5();
  prep_e5();
  hit_e5();

  return 0;
*/
  ros::spin();

  delete left_arm;
  delete head;
  return 0;
}

