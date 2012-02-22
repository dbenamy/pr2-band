#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include "poo_blobs.cpp"

#define NUM_POOPS 30

using namespace cv;
using namespace std;

void normalize(Point3d& v)
{
  double len = norm(v);
  v.x /= len;
  v.y /= len;
  v.z /= len;
}

class PooSeer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  tf::TransformListener tf_listener_;
  sensor_msgs::CvBridge bridge_;
  image_geometry::PinholeCameraModel cam_model_;
  ros::Publisher pooPub_;
  ros::Publisher pooPub2D_;

  // These should be uint8_t, but there is no overload for getParam
  // at that type.
  int grassHue, grassBrightness, grassThreshold, pooThreshold, minPooSize;
  double robotHeight;
  bool maskGrass;

  public:
  PooSeer()
    : it_(nh_)
  {
    string image_topic = nh_.resolveName("/prosilica/image_rect_color");
    sub_ = it_.subscribeCamera(image_topic, 10, &PooSeer::imageCb, this);
    pooPub_ = nh_.advertise<sensor_msgs::PointCloud>("poo_view", 3);
    pooPub2D_ = nh_.advertise<sensor_msgs::PointCloud>("poo_2d", 3);
    ros::NodeHandle pnh_("~");

    if(!pnh_.getParam("grassHue", grassHue)) grassHue = 38;
    if(!pnh_.getParam("grassBrightness", grassBrightness)) grassBrightness = 120;
    if(!pnh_.getParam("grassThreshold", grassThreshold)) grassThreshold = 10;
    if(!pnh_.getParam("pooThreshold", pooThreshold)) pooThreshold = 10;
    if(!pnh_.getParam("robotHeight", robotHeight)) robotHeight = 1.5;
    pnh_.param("maskGrass", maskGrass, true);
    pnh_.param("minPooSize", minPooSize, 15*12);
  }

  void imageCb(sensor_msgs::ImageConstPtr const& image_msg,
      sensor_msgs::CameraInfoConstPtr const& info_msg)
  {
    IplImage* image = NULL;
    try {
      image = bridge_.imgMsgToCv(image_msg, "bgr8");
    } catch (sensor_msgs::CvBridgeException& ex) {
      ROS_ERROR("perceive_poo failed to convert image");
      return;
    }
    cam_model_.fromCameraInfo(info_msg);
    tf::StampedTransform tCamToBase, tBaseToCam;
    string base_frame = "/map";
    //string base_frame = "/odom_combined";
    //string base_frame = "/base_footprint";

    // The high_def_optical_frame attached to the prosilica is
    // giving me some trouble. As an alternative, we can compute
    // camera-relative rays and use the wide_stereo_optical_frame
    // to transform into another frame.

    //ROS_WARN("Using the cam_model_.tfFrame() camera_frame transform not a hard-coded one.");
    string camera_frame = cam_model_.tfFrame();
    //string camera_frame = "wide_stereo_optical_frame";

    //printf("Camera frame = %s\n", camera_frame.c_str());

    try {
      tf_listener_.waitForTransform(base_frame, camera_frame,
          image_msg->header.stamp, 
          ros::Duration(1));
      tf_listener_.lookupTransform(camera_frame, base_frame, 
          image_msg->header.stamp, tCamToBase);
      tf_listener_.lookupTransform(base_frame, camera_frame, 
          image_msg->header.stamp, tBaseToCam);
    } catch(tf::TransformException& ex) {
      ROS_WARN("perceive_poo TF exception:\n%s", ex.what());
      return;
    }

    // 2D image blob AABBs
    vector<CvRect> boxes; 
    Mat imgSmall;
    Mat imageMat = image;
    resize(imageMat, imgSmall, Size(640, 480));
    float scaleX = (float)image_msg->width / 640.0f;
    float scaleY = (float)image_msg->height / 480.0f;

    IplImage imageIpl = imgSmall;
    find_poo(&imageIpl, grassHue, grassBrightness, grassThreshold, 
             pooThreshold, maskGrass, minPooSize, boxes);

    // Compute camera height above ground.
    tf::Vector3 cameraOrigin = tBaseToCam.getOrigin();
    cameraOrigin.setY(cameraOrigin.getY()); // Was adding + 0.14 to it because it was using the wide_stereo

    float cameraHeight = cameraOrigin.getZ();
    /*
    printf("Camera origin = %.2f %.2f %.2f\n", 
        cameraOrigin.getX(), cameraOrigin.getY(), cameraOrigin.getZ());
    printf("Camera height = %.2fm\n", cameraHeight);

    printf("Base origin = %.2f %.2f %.2f\n",
        tCamToBase.getOrigin().getX(),
        tCamToBase.getOrigin().getY(),
        tCamToBase.getOrigin().getZ());
    */
    vector<geometry_msgs::Point32> pooSightings;
    vector<geometry_msgs::Point32> poo2DSightings;
    for(uint32_t i = 0; i < boxes.size(); i++)
    {
      CvRect r = boxes[i];
      int cx = r.x + r.width / 2;
      int cy = r.y + r.height / 2;

      // projectPixelTo3dRay doesn't know about any resizing
      // we've done, so we must rescale the coordinates of the
      // objects we've found.
      cx = (int)((float)cx * scaleX + 0.5*scaleX);
      cy = (int)((float)cy * scaleY + 0.5*scaleY);
      Point3d ray = cam_model_.projectPixelTo3dRay(Point(cx,cy));
      normalize(ray);

      // Transform the poo-ray into the base coordinate frame
      //geometry_msgs::PointStamped cameraRay, baseRay;
      tf::Vector3 cameraRay, baseRay;
      //cameraRay.header.frame_id = camera_frame;
      //cameraRay.header.stamp = image_msg->header.stamp;
      cameraRay.setX(ray.x);
      cameraRay.setY(ray.y);
      cameraRay.setZ(ray.z);
      //tf_listener_.transformPoint(base_frame, cameraRay, baseRay);
      baseRay = tBaseToCam(cameraRay);

      // We want the ray relative to the camera origin in the
      // base coordinate frame


      baseRay -= cameraOrigin;
      // baseRay.point.x -= cameraOrigin.getX();
      // baseRay.point.y -= cameraOrigin.getY();
      // baseRay.point.z -= cameraOrigin.getZ();

      //	    printf("Ray to poo: %.3f %.3f %.3f\n", 
      //   baseRay.point.x, baseRay.point.y, baseRay.point.z);

      // We are interested in the point where the ray hits the ground
      if(baseRay.getZ() < 0)
      {
        float s = cameraHeight / -baseRay.getZ();
        geometry_msgs::Point32 poo;
        poo.x = cameraOrigin.getX() + baseRay.getX() * s;
        poo.y = cameraOrigin.getY() + baseRay.getY() * s;
        float dx = poo.x - cameraOrigin.getX();
        float dy = poo.y - cameraOrigin.getY();
        double dist = sqrt(dx*dx + dy*dy);
        //printf("dist = %f\n", dist);
        //if(dist > 1.1 && dist < 9)
        if(dist < 6)
        {
          //ROS_INFO("I see poo in the image at (%d,%d), ", cx, cy);
          //ROS_INFO("in the world at (%.3f,%.3f)\n",
          //    poo.x, poo.y);
          pooSightings.push_back(poo);
          geometry_msgs::Point32 poo2D;
          poo2D.x = cx;
          poo2D.y = cy;
          poo2DSightings.push_back(poo2D);
        }
      }
    }
    ROS_INFO("I see %d poops.", (int)(pooSightings.size()));
    if(pooSightings.size() > 0)
    {
      geometry_msgs::Point32 fakePoo;
      fakePoo.x = 25; fakePoo.y = 25; fakePoo.z = 25;
      pooSightings.resize(NUM_POOPS, fakePoo);
      poo2DSightings.resize(NUM_POOPS, fakePoo);

      sensor_msgs::PointCloud pc;
      pc.header.stamp = ros::Time();
      pc.header.frame_id = base_frame;
      pc.points = pooSightings;

      sensor_msgs::PointCloud pc2D;
      pc2D.header.stamp = ros::Time();
      pc2D.header.frame_id = base_frame;
      pc2D.points = poo2DSightings;

      sensor_msgs::ChannelFloat32 channel;
      channel.name="intensities";
      channel.values.resize(NUM_POOPS, 2000);
      pc.channels.push_back(channel);
      pc2D.channels.push_back(channel);

      channel.name="index";
      channel.values.clear();
      channel.values.resize(NUM_POOPS, 0);
      pc.channels.push_back(channel);
      pc2D.channels.push_back(channel);

      channel.name="distances";
      channel.values.clear();
      channel.values.resize(NUM_POOPS, 2);
      pc.channels.push_back(channel);
      pc2D.channels.push_back(channel);

      channel.name="stamps";
      channel.values.clear();
      channel.values.resize(NUM_POOPS, 0.002);
      pc.channels.push_back(channel);
      pc2D.channels.push_back(channel);

      /*
      geometry_msgs::TwistConstPtr base_msg = ros::topic::waitForMessage<geometry_msgs::Twist>("base_controller/command");
      if(base_msg
      */
      pooPub_.publish(pc);
      pooPub2D_.publish(pc2D);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "perceive_poo");
  PooSeer poo;
  ros::spin();

  // Test with a saved image
  //Mat img = imread("/media/sf_VBoxShared/poo_raw2.png");
  // Mat img = imread("/u/poopscoop/ros/pr2_poop_scoop/perceive_poo/poo_raw.png");
  // IplImage imgIpl = img;
  // vector<CvRect> boxes;
  // find_poo(&imgIpl, 71, 190, 10, 10, boxes);
  // return 0;
}
