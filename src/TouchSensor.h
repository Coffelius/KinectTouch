#ifndef INCLUDED_TOUCHSENSOR_H
#define INCLUDED_TOUCHSENSOR_H
#include <iostream>
#include <vector>
#include <map>
using namespace std;

// openCV
//#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/blobtrack.hpp>

using namespace cv;

// openNI
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
using namespace xn;

#include <ofxCvKalman.h>


class TouchSensor  {
public:
    unsigned short touchDepthMin;
    unsigned short touchDepthMax;
    unsigned int touchMinArea;

    CvSeq *imageKeypoints, *imageDescriptors;
    CvPoint src_corners[4];
    int calibrating;
    CvPoint dst_corners[4];
    double hMat[9], hIMat[9];

        int xMin;
    int xMax;
    int yMin;
    int yMax;

    Mat /*1s*/ background;//(480, 640)
    Mat /*1s*/ foreground;//(480, 640);
    Mat /*1s*/ wforeground; //(480, 853);
    Mat /*1b*/ arm; //(480, 853);

    xn::DepthGenerator dGen;
    xn::ImageGenerator iGen;

    vector<Point3f> touchPoints;
    vector<double> areas;
    vector<vector<Point2i> > lastBlobs;
    vector<int>lastLabels;
    vector<bool>newTouch;


    int highLabel;
    TouchSensor(xn::ImageGenerator &image, xn::DepthGenerator &_depth);
    void calibrate(void);
    void initMarker(void);
    void update(void);
    void blobStuff(Mat &arm);
    void startCalibration(void);


    ofxCvKalman *tuioPointSmoothed[32*2];

    void initKalman(void);
    bool updateKalman(int id, Point3f &p);
    void clearKalman(int id);


};

#endif
