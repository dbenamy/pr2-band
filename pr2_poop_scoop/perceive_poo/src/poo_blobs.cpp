////////////////////////////////////////////////////////////////////////////////
// Image filtering and blob extraction
////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>
#include "BlobResult.h"

using namespace cv;
using namespace std;

// Extract blob bounding boxes
void poo_blobs(IplImage* orig, Mat& img, int minPooSize, vector<CvRect>& boxes)
{
    CBlobResult blobs;
    IplImage old = img;
    Mat foo = orig;
    blobs = CBlobResult(&old, NULL, 0);
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, 50*50);
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, minPooSize); 
    boxes.resize(blobs.GetNumBlobs());
    for(int i = 0; i < blobs.GetNumBlobs(); i++)
    {
        CBlob* currentBlob = blobs.GetBlob(i);
        boxes[i] = currentBlob->GetBoundingBox();
        currentBlob->FillBlob(orig, CV_RGB(255,0,0));
	float cx = boxes[i].x + boxes[i].width / 2;
	float cy = boxes[i].y + boxes[i].height / 2;
	rectangle(foo, Point(cx-2,cy-2), Point(cx+2,cy+2),
		  Scalar(0,255,0),CV_FILLED,1,0);
    }
#ifdef DEBUG_IMAGES
    Mat origMat = orig;
    imwrite("/tmp/poo_blobs.png", origMat);
#endif
}

// Fill a mask such that the interior of large blobs with aspect
// ratios less than 4 is white, and all other pixels are black.
void mask_big_blobs(Mat& img, Mat& mask)
{
    CBlobResult blobs;
    mask = Mat::zeros(img.rows, img.cols, CV_8U);
    IplImage orig = img;
    IplImage maskImg = mask;
    blobs = CBlobResult(&orig, NULL, 0);
    blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 800);
    for(int i = 0; i < blobs.GetNumBlobs(); i++)
    {
        CBlob *blob = blobs.GetBlob(i);
        CvBox2D ell = blob->GetEllipse();
        
        float aspect;
        if(ell.size.width > ell.size.height) 
            aspect = ell.size.width / ell.size.height;
        else
            aspect = ell.size.height / ell.size.width;
        if(aspect < 4)
            blob->FillBlob(&maskImg, CV_RGB(255,255,255));
    }
}

// Finds poo-colored regions in OpenCV images
void find_poo(IplImage* imgIpl, int grassHue, int grassBrightness, 
              int grassThreshold, int pooThreshold, bool maskGrass,
              int minPooSize,
              vector<CvRect>& boxes)
{
    Mat img = cvarrToMat(imgIpl);
#ifdef DEBUG_IMAGES
    imwrite("/tmp/poo_raw.png", img);
#endif
    
    Mat hsv(img.rows, img.cols, CV_8UC3);
    cvtColor(img, hsv, CV_BGR2HSV);
    int mux[2] = {0,0};
    Mat hue(img.rows, img.cols, CV_8U);
    mixChannels(&hsv, 1, &hue, 1, (const int*)mux, 1);

    int muxV[2] = {2,0};
    Mat val(img.rows, img.cols, CV_8U);
    mixChannels(&hsv, 1, &val, 1, (const int*)muxV, 1);
    Mat valMask;
    threshold(val, valMask, grassBrightness, 255, THRESH_BINARY_INV);
#ifdef DEBUG_IMAGES
    imwrite("/tmp/poo_ValMask.png", valMask);
#endif
    
    // Green hue is 120degrees. We're fitting into uint8_t, so
    // cvtColor divides hue in half, putting pure green at 60.
    Mat diffGreen;
    //absdiff(hue, 60, diffGreen);
    absdiff(hue, grassHue, diffGreen); // What grass looks like in the planlab
#ifdef DEBUG_IMAGES
    imwrite("/tmp/poo_grassHue.png", diffGreen);
#endif

    // Find grass: We first eliminate pixels with a high value (bright
    // pixels), then de-noise via erosion, then close holes. Finally,
    // we keep only large connected components.
    Mat grassRaw;
    threshold(diffGreen, grassRaw, grassThreshold, 255, THRESH_BINARY_INV);
    //threshold(diffGreen, grassRaw, grassThreshold, 255, THRESH_BINARY_INV | THRESH_OTSU);

    grassRaw &= valMask;
    erode(grassRaw, grassRaw, Mat(), Point(-1,-1), 3);
    dilate(grassRaw, grassRaw, Mat(), Point(-1,-1), 16);
    erode(grassRaw, grassRaw, Mat(), Point(-1,-1), 13);
    
    // Look for a big connected component of grass
    Mat grass = grassRaw;
    //mask_big_blobs(grassRaw, grass);
    

#ifdef DEBUG_IMAGES
    imwrite("/tmp/poo_grassMask.png", grass);
#endif

    // Grass RGB = <82, 94, 51>
    // M = 94 (G)
    // m = 51 (B)
    // C = 43
    // H' = (51-82)/43 + 2 = 1.28
    // H = 60 * H' = 76.7 degrees
    // In general: H = 60 * ((B-R)/(G - min(B,R))+2)
    // 51,66,25
    // H = 60*(((25-51) / (66-25))+2) = 82

    // Threshold and find connected components.
    Mat pooThresh;
    threshold(diffGreen, pooThresh, pooThreshold, 255, THRESH_BINARY);
    if(maskGrass)
        pooThresh &= grass;

    erode(pooThresh, pooThresh, Mat(), Point(-1,-1), 3);  //added this in to try to get rid of false poops with 1-pixel connections
    dilate(pooThresh, pooThresh, Mat(), Point(-1,-1), 6);
    erode(pooThresh, pooThresh, Mat(), Point(-1,-1), 2);

#ifdef DEBUG_IMAGES
    imwrite("/tmp/poo_pooThresh.png", pooThresh);
#endif

    poo_blobs(imgIpl, pooThresh, minPooSize, boxes);
}
