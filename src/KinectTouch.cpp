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
#include <highgui.h>
#include <cv.h>
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
using namespace TUIO;

// TODO smoothing using kalman filter

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
#define DEBUG
// OpenNI
xn::Context xnContext;
xn::DepthGenerator xnDepthGenerator;
xn::ImageGenerator xnImgeGenertor;

bool mousePressed = false;

//---------------------------------------------------------------------------
// Functions
//---------------------------------------------------------------------------

int initOpenNI(const XnChar* fname) {
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

void average(vector<Mat1s>& frames, Mat1s& mean) {
	Mat1d acc(mean.size());
	Mat1d frame(mean.size());

	for (unsigned int i=0; i<frames.size(); i++) {
		frames[i].convertTo(frame, CV_64FC1);
		acc = acc + frame;
	}

	acc = acc / frames.size();

	acc.convertTo(mean, CV_16SC1);
}


CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
CvPoint src_corners[4];
void initMarker(void) {
    //Mat1b image=imread("marker.png", 0);
    IplImage* image = cvLoadImage( "marker640.png", CV_LOAD_IMAGE_GRAYSCALE );
    src_corners = {{0,0}, {image->width,0}, {image->width, image->height}, {0, image->height}};

    CvSURFParams params = cvSURFParams(500, 1);

    CvMemStorage *storage=cvCreateMemStorage(0);
    cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );

//    imshow("Ajuste", image);
    cvReleaseImage(&image);

}

int calibrating=5;
CvPoint dst_corners[4];


double hMat[8], hIMat[8];

inline CvPoint2D32f trans(CvPoint2D32f *src, double *h) {


        double x = src->x, y = src->y;
        double Z = 1./(h[6]*x + h[7]*y + h[8]);
        double X = (h[0]*x + h[1]*y + h[2])*Z;
        double Y = (h[3]*x + h[4]*y + h[5])*Z;
    //return cvPoint2D32f(cvRound(X), cvRound(Y));
    return cvPoint2D32f(X, Y);

}
int
locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                    const CvSeq* imageKeypoints, const CvSeq* imageDescriptors,
                    const CvPoint src_corners[4], CvPoint dst_corners[4], double *h );
void calibrate(void)
{
    if(!calibrating) return;
    calibrating--;
    Mat3b rgb(480, 640);
    rgb.data = (uchar *)xnImgeGenertor.GetImageMap();//.GetRGB24ImageMap();//xnImgeGenertor.GetImageMap();
    if(!rgb.data) return;
#ifdef DEBUG

    Mat3b out(480,640);
    cvtColor(rgb, out, CV_BGR2RGB);
    IplImage iOut=out;
#endif
    //cvCopy(&((IplImage)rgb), &((IplImage)out), NULL);

    Mat1b gray(480,640);
    cvtColor(rgb, gray, CV_RGB2GRAY);
   // imshow("gray", gray);
    CvSeq *keypoints = 0, *descriptors = 0;

    CvSURFParams params = cvSURFParams(500, 1);

    CvMemStorage *storage=cvCreateMemStorage(0);
    cvExtractSURF( &(IplImage)gray, 0, &keypoints, &descriptors, storage, params );

    if( locatePlanarObject(  imageKeypoints,
        imageDescriptors,keypoints, descriptors, src_corners, dst_corners, hMat ))
    {

        CvMat _hMat = cvMat(3, 3, CV_64F, hMat);
        CvMat _hIMat = cvMat(3,3, CV_64F, hIMat);

        cvInvert(&_hMat, &_hIMat);

#ifdef DEBUG

        for( int i = 0; i < 4; i++ )
        {
            CvPoint r1 = dst_corners[i%4];
            CvPoint r2 = dst_corners[(i+1)%4];
            cvLine(&(IplImage)out, cvPoint(r1.x, r1.y ),
                cvPoint(r2.x, r2.y ), cvScalar(0,0,255,0) );
        }
#endif
    }
#ifdef DEBUG
    imshow("out", out);
#endif
}
int main() {

	const unsigned int nBackgroundTrain = 30;
	const unsigned short touchDepthMin = 20;
	const unsigned short touchDepthMax = 35;

	//const unsigned short touchDepthMin = 30;
	//const unsigned short touchDepthMax = 50;
	const unsigned int touchMinArea = 30;

	const bool localClientMode = true; 					// connect to a local client

	const double debugFrameMaxDepth = 4000; // maximal distance (in millimeters) for 8 bit debug depth frame quantization
	const char* windowName = "Debug";
	const Scalar debugColor0(0,0,128);
	const Scalar debugColor1(255,0,0);
	const Scalar debugColor2(255,255,255);

	int xMin = 0;
	int xMax = 640;
	int yMin = 0;
	int yMax = 480;

	Mat1s depth(480, 640); // 16 bit depth (in millimeters)
	Mat1b depth8(480, 640); // 8 bit depth
	//Mat3b rgb(480, 640); // 8 bit depth

	Mat3b debug(480, 640); // debug visualization

	Mat1s foreground(640, 480);
	Mat1b foreground8(640, 480);

	Mat1b touch(640, 480); // touch mask

	Mat1s background(480, 640);

    Mat1s arm(640, 480);

	vector<Mat1s> buffer(nBackgroundTrain);

	initOpenNI("niConfig.xml");

	// TUIO server object
	TuioServer* tuio;
	if (localClientMode) {
		tuio = new TuioServer();
	} else {
		tuio = new TuioServer("192.168.0.2",3333,false);
	}
	TuioTime time;

    initMarker();
	// create some sliders
	#ifdef DEBUG
	namedWindow(windowName);
	#endif
	/*
	createTrackbar("xMin", windowName, &xMin, 640);
	createTrackbar("xMax", windowName, &xMax, 640);
	createTrackbar("yMin", windowName, &yMin, 480);
	createTrackbar("yMax", windowName, &yMax, 480);
*/
    xnImgeGenertor.SetPixelFormat( XN_PIXEL_FORMAT_RGB24);
    xnDepthGenerator.GetAlternativeViewPointCap().SetViewPoint(xnImgeGenertor);
	// create background model (average depth)
/*	for (unsigned int i=0; i<nBackgroundTrain; i++) {
		xnContext.WaitAndUpdateAll();
		depth.data = (uchar*) xnDepthGenerator.GetDepthMap();
		buffer[i] = depth;
	}
	average(buffer, background);
*/

    char key=0;
	while ( key != 27 ) {

		// read available data
		xnContext.WaitAndUpdateAll();

		// update 16 bit depth matrix
		depth.data = (uchar*) xnDepthGenerator.GetDepthMap();
		//xnImgeGenertor.GetGrayscale8ImageMap()

        key=waitKey(1);
	    if(key=='c') calibrating=5;
	    else if(key=='b') background=depth.clone();

		// update rgb image
		//rgb.data = (uchar*) xnImgeGenertor.GetRGB24ImageMap(); // segmentation fault here
		//cvtColor(rgb, rgb, CV_RGB2BGR);
		calibrate();

		// extract foreground by simple subtraction of very basic background model
		foreground = background - depth;

		// find touch mask by thresholding (points that are close to background = touch points)
		arm =  (foreground>touchDepthMin);
		touch =(foreground>touchDepthMin) & (foreground < touchDepthMax);

		// extract ROI
		//Rect roi(xMin, yMin, xMax - xMin, yMax - yMin);
		Mat touchRoi = touch; //touch(roi);

		// find touch points
		vector< vector<Point2i> > contours;
		vector<Point2f> touchPoints;
		findContours(touchRoi, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point2i(xMin, yMin));
		for (unsigned int i=0; i<contours.size(); i++) {
			Mat contourMat(contours[i]);
			// find touch points by area thresholding
			if ( contourArea(contourMat) > touchMinArea ) {
				Scalar center = mean(contourMat);
				Point2i touchPoint(center[0], center[1]);
				touchPoints.push_back(touchPoint);
			}
		}

		// send TUIO cursors
		time = TuioTime::getSessionTime();
		tuio->initFrame(time);

		for (unsigned int i=0; i<touchPoints.size(); i++) { // touch points
			//float cursorX = (touchPoints[i].x - xMin) / (xMax - xMin);
			//float cursorY = 1 - (touchPoints[i].y - yMin)/(yMax - yMin);

            CvPoint2D32f c=cvPoint2D32f(touchPoints[i].x - xMin,
                                            touchPoints[i].y - yMin);

            c=trans(&c, hIMat);

            float cursorX=1-c.x/ 853.0f;
            float cursorY=c.y/ 480.0f;

            cursorY-=0.5;
            cursorY*=1.0;
            cursorY+=0.5;

			if(cursorX<0 || cursorY< 0 || cursorX>1 || cursorY>1) continue;
			TuioCursor* cursor = tuio->getClosestTuioCursor(cursorX,cursorY);


			// TODO improve tracking (don't move cursors away, that might be closer to another touch point)
			if (cursor == NULL || cursor->getTuioTime() == time) {
				tuio->addTuioCursor(cursorX,cursorY);
			} else {
				tuio->updateTuioCursor(cursor, cursorX, cursorY);
			}
		}

		tuio->stopUntouchedMovingCursors();
		tuio->removeUntouchedStoppedCursors();
		tuio->commitFrame();


#ifdef DEBUG
		// draw debug frame
		arm.convertTo(depth8, CV_8U, 255 / debugFrameMaxDepth); // render depth to debug frame
		cvtColor(depth8, debug, CV_GRAY2BGR);
		debug.setTo(debugColor0, touch);  // touch mask
		        for( int i = 0; i < 4; i++ )
        {
            CvPoint r1 = dst_corners[i%4];
            CvPoint r2 = dst_corners[(i+1)%4];
            cvLine(&(IplImage)debug, cvPoint(r1.x, r1.y ),
                cvPoint(r2.x, r2.y ), cvScalar(255,0,0,0) );
        }
		//rectangle(debug, roi, debugColor1, 2); // surface boundaries
		for (unsigned int i=0; i<touchPoints.size(); i++) { // touch points
			circle(debug, touchPoints[i], 5, debugColor2, CV_FILLED);
		}

		// render debug frame (with sliders)
		imshow(windowName, debug);
		//imshow("image", rgb);

#endif

	}

	return 0;
}
