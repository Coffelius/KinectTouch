


#include "KinectTouch.h"

KinectTouch::~KinectTouch(void) {
    delete touchSensor;
}


KinectTouch::KinectTouch(void)
{
    initOpenNI("niConfig.xml");
    const bool localClientMode = true; 					// connect to a local client

    touchSensor=new TouchSensor(xnImgeGenertor, xnDepthGenerator);
    touchSensor->initMarker();
    touchSensor->startCalibration();

    // TUIO server object
    if (localClientMode)
    {
        tuio = new TuioServer();
    }
    else
    {
        tuio = new TuioServer("192.168.0.2",3333,false);
    }
}

int KinectTouch::initOpenNI(const XnChar* fname)
{
    XnStatus nRetVal = XN_STATUS_OK;

    // initialize context
    nRetVal = xnContext.InitFromXmlFile(fname);
    CHECK_RC(nRetVal, "InitFromXmlFile");

    // initialize depth genera tor
    nRetVal = xnContext.FindExistingNode(XN_NODE_TYPE_DEPTH, xnDepthGenerator);
    CHECK_RC(nRetVal, "FindExistingNode(XN_NODE_TYPE_DEPTH)");

    // initialize image generator
    nRetVal = xnContext.FindExistingNode(XN_NODE_TYPE_IMAGE, xnImgeGenertor);
    CHECK_RC(nRetVal, "FindExistingNode(XN_NODE_TYPE_IMAGE)");
    return nRetVal;
}


void KinectTouch::update(void)
{
    xnContext.WaitAndUpdateAll();
    touchSensor->update();

    vector<Point3f> touchPoints=touchSensor->touchPoints;

    TuioTime time;

    // send TUIO cursors
    time = TuioTime::getSessionTime();
    tuio->initFrame(time);

    for (unsigned int i=0; i<touchPoints.size(); i++)   // touch points
    {
        if(touchPoints[i].z<touchSensor->touchZ) continue;
        CvPoint2D32f c=cvPoint2D32f(touchPoints[i].x ,
                                    touchPoints[i].y);


        float cursorX=1-c.x/ 853.0f;
        float cursorY=c.y/ 480.0f;

        if(cursorX<0 || cursorY< 0 || cursorX>1 || cursorY>1) continue;

        if(touchSensor->newTouch[i])
            tuio->addTuioCursor(cursorX,cursorY);
        else
        {
            TuioCursor* cursor = tuio->getClosestTuioCursor(cursorX,cursorY);
            if(cursor)
            {
                tuio->updateTuioCursor(cursor, cursorX, cursorY);
            }
        }

    }

    tuio->stopUntouchedMovingCursors();
    tuio->removeUntouchedStoppedCursors();
    tuio->commitFrame();
}

void KinectTouch::calibrate(void) {
    touchSensor->startCalibration();
}
