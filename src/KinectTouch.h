#ifndef INCLUDED_KINECTTOUCH_H
#define INCLUDED_KINECTTOUCH_H
#include <iostream>
#include <vector>
#include <map>
using namespace std;

// openCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

// openNI
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
using namespace xn;
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

// TUIO
#include "TuioServer.h"
#include "TouchSensor.h"
using namespace TUIO;

class KinectTouch {
protected:
    int initOpenNI(const XnChar* fname);


    xn::Context xnContext;
    xn::DepthGenerator xnDepthGenerator;
    xn::ImageGenerator xnImgeGenertor;
    TouchSensor *touchSensor;
    TuioServer* tuio;

public:
    KinectTouch(void);
    ~KinectTouch(void);
    void update(void);
    void calibrate(void);
};
#endif
