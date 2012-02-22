#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include "IteratedMean.cpp"
#include "BlobResult.h"

using namespace cv;
using namespace std;

// Extract blob bounding boxes
void poo_blobs(Mat& img, vector<CvRect>& boxes)
{
    CBlobResult blobs;
    IplImage old;
    Mat colMat, imgMorph;
    IplImage color;
    
    // erode(img,imgMorph,Mat(),
    // 	 old = imgMorph;
    cvtColor(img, colMat, CV_GRAY2BGR);
    color = colMat;
    blobs = CBlobResult(&old, NULL, 0);
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, 12*12);
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetLength(), B_LESS, 3);
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetLength(), B_GREATER, 12);
    boxes.resize(blobs.GetNumBlobs());
    for(int i = 0; i < blobs.GetNumBlobs(); i++)
    {
        CBlob* currentBlob = blobs.GetBlob(i);
        boxes[i] = currentBlob->GetBoundingBox();
        currentBlob->FillBlob(&color, CV_RGB(255,0,0));
    }
#ifdef DEBUG_IMAGES
    //imwrite("/tmp/poo_laser_blobs.png", img);
    imwrite("/tmp/poo_laser_blobs.png", colMat);
#endif
}

void normalize(Vec3f& v)
{
    double len = norm(v);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
}

class NotGroundPred {
    float minH,maxH;
public:
    NotGroundPred(float minH, float maxH) : minH(minH), maxH(maxH) {}
    bool operator ()(float h) {return (h < minH || h > maxH); }
};

class PooLaser
{
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    ros::Publisher markerPub_;   // Publish visualization markers
    ros::Subscriber sub_;        // Subscription to /tilt_scan
    Mat grid;                    // Heightmap quantized at 4cm^2 grid
    const float cellSize; // Heightmap cell side length in meters
    const float cellsPerMeter;

    // Poo height bounds in meters.
    double minPooHeight, maxPooHeight;

public:
    PooLaser() : cellSize(0.02), cellsPerMeter(1 / cellSize)
    {
        sub_ = nh_.subscribe("/tilt_scan", 1, &PooLaser::laserCb, this);
        markerPub_ = nh_.advertise<visualization_msgs::Marker>("poo_laser", 8);
	ros::NodeHandle pnh_("~");
        if(!pnh_.getParam("minPooHeight", minPooHeight)) minPooHeight = 0.02;
        if(!pnh_.getParam("maxPooHeight", maxPooHeight)) maxPooHeight = 0.05;
        grid = Mat::zeros(100,100,CV_32F);
        printf("Using poo height bound [%f,%f]\n", minPooHeight, maxPooHeight);
    }

    // bool notGround(float h) { return (h < minPooHeight || h > maxPooHeight); }

    // Track the tilting scanner's tilt angle and save an image of the
    // grid whenever the tilt direction of change flips.
    void monitorTilt(Vec3f const& laserOrigin, 
                     geometry_msgs::PointStamped const& laserTest)
    {
        static float prevTiltAngle = 0.0f;
        static float tiltDirection = 0.0f;

        Vec3f probe(laserTest.point.x, laserTest.point.y, laserTest.point.z);
        probe -= laserOrigin;
        normalize(probe);
        float newTiltAngle = atan2(probe[2],probe[0]);
	/*
        printf("Tilt angle = %.2f\n", newTiltAngle * 180 / 3.14159);
        printf("Tilt increment = %.4f\n", 
               (newTiltAngle - prevTiltAngle)*180/3.14159);
	*/

        // Save a grid image when the tilt direction flips
        uint8_t saveGridImage = 0;
        if((newTiltAngle - prevTiltAngle) > 0) {
            if(tiltDirection <= 0) {
                tiltDirection = 1;
                saveGridImage = 1;
            }
        }
        else if(tiltDirection >= 0) {
            tiltDirection = -1;
            saveGridImage = 1;
        }
        prevTiltAngle = newTiltAngle;
        if(saveGridImage) {
            printf("Saving grid image\n");
            Mat gray(100,100,CV_8U);
            Mat gridT(100,100,CV_32F);
            threshold(grid,gridT,minPooHeight,0,THRESH_TOZERO);
            threshold(gridT,gridT,maxPooHeight,0,THRESH_TOZERO_INV);
            convertScaleAbs(gridT,gray,3000,0);
            //imwrite("/media/sf_VBoxShared/poop/pooGridGray.png", gray);
	    imwrite("/tmp/pooGridGray.png", gray);
            //grid.setTo(0.0f);

            vector<CvRect> boxes;
            poo_blobs(gray, boxes);
            ROS_INFO("Found %d boxes", boxes.size());
        }
    }

    void laserCb(sensor_msgs::LaserScan const& msg)
    {
        tf::StampedTransform tLaserToBase;
        tf::StampedTransform tBaseToLaser;
	string base_frame = "/base_footprint";
        //string base_frame = "/odom_combined";

        try {
          tf_listener_.waitForTransform(base_frame, msg.header.frame_id,
                                        msg.header.stamp, ros::Duration(1));
          tf_listener_.lookupTransform(msg.header.frame_id, base_frame, 
                                       msg.header.stamp, tLaserToBase);
          tf_listener_.lookupTransform(base_frame, msg.header.frame_id,
                                       msg.header.stamp, tBaseToLaser);
        } catch(tf::TransformException& ex) {
            ROS_WARN("poo_laser TF exception:\n%s", ex.what());
            return;
        }

        // Render laser rays into the heightmap. Each grid cell
        // contains a float representing the cell's metric height.
        tf::Vector3 laserOriginTF = tBaseToLaser.getOrigin();
        Vec3f laserOrigin(laserOriginTF.getX(), 
                          laserOriginTF.getY(), 
                          laserOriginTF.getZ());
        geometry_msgs::PointStamped laserPoint, gridPoint;
        laserPoint.header.stamp = msg.header.stamp;//ros::Time(0);
        laserPoint.header.frame_id = msg.header.frame_id;
        laserPoint.point.z = 0;
        uint32_t half = msg.ranges.size() / 2;
        float angle = msg.angle_min;
        
        // test point to check angles;
        laserPoint.point.x = 1;
        laserPoint.point.y = 0;
        tf_listener_.transformPoint(base_frame, laserPoint, gridPoint);
        monitorTilt(laserOrigin, gridPoint);

	/*
        printf("MinAngle = %.4f, MaxAngle = %.4f, AngleStep = %.4f, NumRanges = %d\n", 
               msg.angle_min, msg.angle_max, msg.angle_increment, msg.ranges.size());
	*/

        vector<Vec3f> pts;
        vector<float> heights;
        vector<float> intense;
        pts.reserve(msg.ranges.size());
        heights.reserve(msg.ranges.size());
        intense.reserve(msg.intensities.size());
        uint32_t i;

        tf::Vector3 laserPointV, gridPointV;
        laserPointV.setZ(0);

        for(i = 0; i < msg.ranges.size(); i++, angle += msg.angle_increment)
        {
            // Compute the point in the laser coordinate frame given
            // the range.
            float r = msg.ranges[i];
            if(r < msg.range_min || r > msg.range_max) continue;
            if(r < 1 || r > 5) continue;
            // if(msg.intensities[i] < 2000 || msg.intensities[i] > 2700) continue;

            laserPoint.point.x = r*cos(angle);
            laserPoint.point.y = r*sin(angle);
            //tf_listener_.transformPoint(base_frame, laserPoint, gridPoint);
            laserPointV.setX(r*cos(angle));
            laserPointV.setY(r*sin(angle));
            gridPointV = tBaseToLaser(laserPointV);
            Vec3f v(gridPointV.getX(), gridPointV.getY(), gridPointV.getZ());
            pts.push_back(v);
            heights.push_back(v[2]);
            intense.push_back(msg.intensities[i]);
        }
	/*
        printf("min intensity = %f, max intensity = %f\n",
               *min_element(intense.begin(), intense.end()), 
               *max_element(intense.begin(), intense.end()));

        vector<float> i2 = intense;
        float meanIntensity = iteratedMean(i2.begin(),
                                           i2.end(),
                                           10, 0.0f);
        printf("mean intensity = %f\n", meanIntensity);
	*/

        NotGroundPred gPred(-0.03, maxPooHeight);
        vector<float>::iterator gend = remove_if(heights.begin(), 
                                                 heights.end(), 
                                                 gPred);
        if(gend - heights.begin() < 100) {
            printf("Scan missed the ground?!\n");
            return;
        }
        float groundHeight = iteratedMean(heights.begin(), gend, 2.0, 0.0f);
        // printf("Estimated ground height = %f\n", groundHeight);
        
        i = 0;
        for(vector<Vec3f>::const_iterator it = pts.begin();
            it < pts.end();
            ++it, ++i)
        {
            int gx = (int)((*it)[0] * cellsPerMeter);
            int gy = (int)((*it)[1] * cellsPerMeter) + 50;
            float z = (*it)[2] - groundHeight;
            if(gx >= 0 && gx < 100 && gy >= 0 && gy < 100 && 
               z > -0.1 && z < 0.2) {
                grid.at<float>(gx,gy) = max(z, grid.at<float>(gx,gy));
                // grid.at<float>(gx, gy) = v[2];
            }
	    /*
            if(i == half) {
                printf("Point on grid (%d,%d)\n", gx, gy);
                printf("Point at (%f,%f,%f)\n", 
                       (*it)[0], (*it)[1], z);
            }
	    */
        }
        // Find blobs in the heightmap

        // 2D image blob AABBs
        vector<CvRect> boxes; 
        // IplImage imageIpl = grid;
        // threshold

        int pooDetections = 0;
        for(uint32_t i = 0; i < boxes.size(); i++)
        {
            //CvRect r = boxes[i];
            //int cx = r.x + r.width / 2;
            //int cy = r.y + r.height / 2;
        }
        ROS_INFO("I shot %d poops with lasers\n", pooDetections);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "poo_laser");
    PooLaser poo;
    ros::spin();
    return 0;
}
