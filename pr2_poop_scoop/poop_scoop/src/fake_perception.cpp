#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>

#define NUM_POOPS 30

bool parsePoopFile(FILE* file, std::vector<std::vector<float> > &poops)
{
  char sTemp[128];
  std::vector<float> poo(2,0);

  if(file == NULL)
  {
    ROS_ERROR("Failed to open the poops file. Exiting.\n");
    return false;
  }

  if(fscanf(file,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1.\n");
  while(!feof(file) && strlen(sTemp) != 0)
  {
    poo[0] = atof(sTemp);

    if(fscanf(file,"%s", sTemp) < 1)
      ROS_WARN("Parsed string has length < 1.\n");
    
    poo[1] = atof(sTemp);
    
    poops.push_back(poo);
    ROS_INFO("%0.3f %0.3f", poo[0], poo[1]);

    if(fscanf(file,"%s",sTemp) < 1)
      ROS_WARN("Parsed string has length < 1.\n");
  }

  ROS_INFO("Parsed %d poops.", int(poops.size()));
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_perception");
  ros::NodeHandle nh;
  tf::TransformListener tf;

  ros::Publisher poop_pub = nh.advertise<sensor_msgs::PointCloud>("poop_perception", 3);
  geometry_msgs::Point32 poop;
  sensor_msgs::PointCloud cloud;
sensor_msgs::ChannelFloat32 channel;
  poop.z = 0;
  FILE* pfile;
  std::vector<std::vector<float> > poops;

  if(argc > 1)
    pfile = fopen(argv[1], "r");
  else
  {
    ROS_INFO("usage: $prog <poop_filename>");
    pfile = fopen("/home/poopscoop/ros/pr2_poop_soop/poop_scoop/poops.txt", "w");
    if(pfile == NULL)
    {
      ROS_ERROR("No filename given & can't open default file. Exiting");
      return 0;
    }
  }

  if(!parsePoopFile(pfile, poops) && poops.size() > 0)
  {
    ROS_ERROR("Failed to parse poop file or there are no poops in the file.");
    if(pfile != NULL)
      fclose(pfile);
    return 0;
  }

  sleep(1.0);

  ROS_INFO("Publishing %d poops", int(poops.size()));


  channel.name="intensities";
  channel.values.resize(NUM_POOPS, 2000);
  cloud.channels.push_back(channel);
  channel.name="index";
  channel.values.clear();
  channel.values.resize(NUM_POOPS, 0);
  cloud.channels.push_back(channel);
  channel.name="distances";
  channel.values.clear();
  channel.values.resize(NUM_POOPS, 2);
  cloud.channels.push_back(channel);
  channel.name="stamps";
  channel.values.clear();
  channel.values.resize(NUM_POOPS, 0.002);
  cloud.channels.push_back(channel);


  // publish poops
  ros::Rate r(1.0);
  while(true)
  {
	ROS_INFO("Publishing %d poops", int(poops.size()));  
	cloud.header.stamp = ros::Time::now();
	cloud.header.frame_id = "odom_combined";
	cloud.points.clear();
	  for(size_t i = 0; i < NUM_POOPS; i++)
	  {
		if(i < poops.size())
		{
			/*
			geometry_msgs::PointStamped pin, pout;
			pin.point.x = poops[i][0];
			pin.point.y = poops[i][1];
			pin.point.z = 0;
			pin.header.frame_id = "map";
			pin.header.stamp = ros::Time::now();

			tf.transformPoint("odom_combined", pin, pout);
			cloud.points.push_back(pout.point);
			*/ 

			poop.x = poops[i][0];
			poop.y = poops[i][1];
			poop.z = 0;
			cloud.points.push_back(poop);
		}
		else
		{
			poop.x = 25.0;
			poop.y = 25.0;
			poop.z = 25.0;
			cloud.points.push_back(poop);
		}
	  }
	  poop_pub.publish(cloud);

	  r.sleep();
  }

  fclose(pfile);
  return 0;
}
