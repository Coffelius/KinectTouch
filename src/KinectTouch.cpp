//============================================================================
// Name        : KinectTouch.cpp
// Author      : github.com/robbeofficial
// Version     : 0.something
// Description : recognizes touch points on arbitrary surfaces using kinect
// 				 and maps them to TUIO cursors
// 				 (turns any surface into a touchpad)
//============================================================================

/*
 * 1. point your kinect from a higher place down to your table
 * 2. start the program (keep your hands off the table for the beginning)
 * 3. use your table as a giant touchpad
 */

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

// TODO smoothing using kalman filter

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
//#define DEBUG
// OpenNI
xn::Context xnContext;
xn::DepthGenerator xnDepthGenerator;
xn::ImageGenerator xnImgeGenertor;

bool mousePressed = false;

//---------------------------------------------------------------------------
// Functions
//---------------------------------------------------------------------------

int initOpenNI(const XnChar* fname)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // initialize context
    nRetVal = xnContext.InitFromXmlFile(fname);
    CHECK_RC(nRetVal, "InitFromXmlFile");

    // initialize depth generator
    nRetVal = xnContext.FindExistingNode(XN_NODE_TYPE_DEPTH, xnDepthGenerator);
    CHECK_RC(nRetVal, "FindExistingNode(XN_NODE_TYPE_DEPTH)");

    // initialize image generator
    nRetVal = xnContext.FindExistingNode(XN_NODE_TYPE_IMAGE, xnImgeGenertor);
    CHECK_RC(nRetVal, "FindExistingNode(XN_NODE_TYPE_IMAGE)");

    return 0;
}

int main()
{
    const bool localClientMode = true; 					// connect to a local client
    initOpenNI("niConfig.xml");


    // TUIO server object
    TuioServer* tuio;
    if (localClientMode)
    {
        tuio = new TuioServer();
    }
    else
    {
        tuio = new TuioServer("192.168.0.2",3333,false);
    }
    TuioTime time;

    TouchSensor touchSensor(xnImgeGenertor, xnDepthGenerator);
    touchSensor.initMarker();
    touchSensor.startCalibration();

    char key=0;

    while ( key != 27 )
    {
        key=waitKey(1);
        if(key=='c') touchSensor.startCalibration();
        //else if(key=='b') background=depth.clone();

        xnContext.WaitAndUpdateAll();
        touchSensor.update();

        vector<Point3f> touchPoints=touchSensor.touchPoints;

        // send TUIO cursors
        time = TuioTime::getSessionTime();
        tuio->initFrame(time);

        for (unsigned int i=0; i<touchPoints.size(); i++)   // touch points
        {
            if(touchPoints[i].z<touchSensor.touchZ) continue;
            CvPoint2D32f c=cvPoint2D32f(touchPoints[i].x ,
                                        touchPoints[i].y);


            float cursorX=1-c.x/ 853.0f;
            float cursorY=c.y/ 480.0f;

            if(cursorX<0 || cursorY< 0 || cursorX>1 || cursorY>1) continue;

            if(touchSensor.newTouch[i])
                tuio->addTuioCursor(cursorX,cursorY);
            else {
                TuioCursor* cursor = tuio->getClosestTuioCursor(cursorX,cursorY);
                if(cursor) {
                    tuio->updateTuioCursor(cursor, cursorX, cursorY);
                }
            }

        }

        tuio->stopUntouchedMovingCursors();
        tuio->removeUntouchedStoppedCursors();
        tuio->commitFrame();

    }

    return 0;
}
