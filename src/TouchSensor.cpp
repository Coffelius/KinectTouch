#include "TouchSensor.h"
//#include <Blob.h>
//#include <BlobResult.h>


    const Scalar debugColor0(0,0,128);
    const Scalar debugColor1(255,0,0);
    const Scalar debugColor2(255,255,255);

int
locatePlanarObject( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
                    const CvSeq* imageKeypoints, const CvSeq* imageDescriptors,
                    const CvPoint src_corners[4], CvPoint dst_corners[4], double *h );


inline float distanceFromBorder(Point2i &p)
{
    float d0,d1,d2,d3;
    d0=p.x;
    d1=p.y;
    d2=853-p.x;
    d3=480-p.y;
    return MIN(MIN(d0, d1), MIN(d2,d3));
}
inline CvPoint2D32f trans(CvPoint2D32f *src, double *h)
{
    double x = src->x, y = src->y;
    double Z = 1./(h[6]*x + h[7]*y + h[8]);
    double X = (h[0]*x + h[1]*y + h[2])*Z;
    double Y = (h[3]*x + h[4]*y + h[5])*Z;
    return cvPoint2D32f(X, Y);
}


void TouchSensor::calibrate(void)
{
    if(!calibrating) return;
    calibrating--;
    Mat3b rgb(480, 640);
    rgb.data = (uchar *)iGen.GetImageMap();//.GetRGB24ImageMap();//xnImgeGenertor.GetImageMap();
    if(!rgb.data) return;
    imshow("RGB", rgb);

    Mat1b gray(640,480);
    cvtColor(rgb, gray, CV_RGB2GRAY);
    // imshow("gray", gray);
    CvSeq *keypoints = 0, *descriptors = 0;

    CvSURFParams params = cvSURFParams(500, 1);

    CvMemStorage *storage=cvCreateMemStorage(0);

    IplImage igray=gray;
    cvExtractSURF( &igray, 0, &keypoints, &descriptors, storage, params );

    if( locatePlanarObject(  imageKeypoints,
                             imageDescriptors,keypoints, descriptors, src_corners, dst_corners, hMat ))
    {

        CvMat _hMat = cvMat(3, 3, CV_64F, hMat);
        CvMat _hIMat = cvMat(3,3, CV_64F, hIMat);

        cvInvert(&_hMat, &_hIMat);
    }
    //if(!calibrating) iGen.StopGenerating();
}



/*
void testCloud(Mat1s &depth) {
    int i,j;
    Mat_<Point3f> src(480*640,1);
    float x, y, z;
    const float scaleFactor=0.0021;
    const float minDistance=-10;
    float *psrc=(float*)src.data;
    short *pdepth=(short*)depth.data;
    for(j=0;j<480;j++) {
        for(i=0;i<640;i++) {
            z=*pdepth++;
            x=i * (z+minDistance) * scaleFactor;
            y=j * (z+minDistance) * scaleFactor;
            *psrc++=x;
            *psrc++=y;
            *psrc++=z;
        }
    }
    Mat _hIMat(3, 3, CV_64F, hIMat, 0);

    Mat_<Point3f> dst;
    perspectiveTransform(src, dst, _hIMat);

    Mat1b points(480,853);
    points=0;
    float *pdst=(float*)dst.data;
    uchar *pp=points.data;
    for(i=0;i<480*640;i++) {
        x=*pdst++;
        y=*pdst++;
        z=*pdst++;

        if(x>0 && y>0 && x<853 && y<480) {
            pp[(int)(x+y*853)]=z;
        }
    }

    Mat3b rgb(480,853);
    cvtColor(points, rgb, CV_GRAY2RGB);
    imshow("cloud", rgb);

}*/
//CvScalar colors[8]={CV_RGB(255,0,0), CV_RGB(0,255,0), CV_RGB(0,0,255), CV_RGB(255,255,0), CV_RGB(255,0,255), CV_RGB(0,255,255), CV_RGB(255,255,255), CV_RGB(128,128,128)};

//CBlobResult lastBlobs;
void TouchSensor::blobStuff(Mat &arm) {
    /*
    IplImage iarm=(IplImage)arm;
    if(tracker==NULL) {
        tracker=cvCreateBlobTrackerCC();
    }
    tracker->Process(&iarm,&iarm);

    int i;
    printf("%d blobs\n", tracker->GetBlobNum());
    for(i=0;i<tracker->GetBlobNum();i++)    {
        CvBlob *blob=tracker->GetBlob(i);


    }
    */

    /*
    int i;
    CBlob *currentBlob;
    CBlobResult blobs;

    Mat3b rgb(480, 853);
    memset(rgb.data, 0, 480*853*3);
    IplImage irgb=rgb;
    // find non-white blobs in thresholded image
    blobs = CBlobResult( &(IplImage)arm, NULL, 0 );
    // exclude the ones smaller than param2 value
    blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 1000 );

    if(blobs.GetNumBlobs()>lastBlobs.GetNumBlobs()) {
        // new blob

    } else if(blobs.GetNumBlobs()<lastBlobs.GetNumBlobs()) {
        // removed blob

    } else {
        // same blobs
    }






    for (i = 0; i < blobs.GetNumBlobs(); i++ )
    {
        currentBlob = blobs.GetBlob(i);
        int t=currentBlob->GetID();

        currentBlob->FillBlob( &irgb, colors[t%8]);
    }

    imshow("Blobs", rgb);
    */
}

bool overlapped(vector<Point2i>&c1, vector<Point2i>&c2) {
    //Mat curve;
    //approxPolyDP(c1, )

    for(int i=0;i<c1.size();i++) {
        if(pointPolygonTest((Mat)c2, c1[i],0)>=0)  {
            return true;
        }
    }
    return false;
}
void TouchSensor::update(void)
{


    vector<vector<Point2i> > contours;
    vector<vector<Point2i> > blobs;
    vector<Point3f> touchPoints;
    vector<Point2f> oppositeTouchPoints;
    vector<int>labels;

    Mat depth(480, 640, CV_16UC1, (uchar*)dGen.GetDepthMap(), 0);
    //depth.data = (uchar*)dGen.GetDepthMap();



    if(calibrating) {
        calibrate();
        background=depth.clone();
    }

    // extract foreground by simple subtraction of very basic background model
    foreground = background - depth;
    Mat _hIMat(3, 3, CV_64F, hIMat, 0);
    warpPerspective(foreground, wforeground, _hIMat, Size(853, 480), INTER_LINEAR);


    arm =  (wforeground>15);

    /*Mat1b lala(480,853);
    lala=wforeground;
    threshold(lala, lala, 15, 255, THRESH_BINARY_INV);

    //lala=wforeground<=15;
    blobStuff(lala);
    */
    //touch =(wforeground>10) & (wforeground < 20);

    Mat1b touchRoi=arm.clone();
    //warpPerspective(, Mat& dst, const Mat& M, Size dsize, int
    // find touch points

    //	erode(touchRoi, touchRoi,  getStructuringElement(MORPH_RECT, Size(1,1)));
    findContours(touchRoi, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point2i(xMin, yMin));


    //blur(wforeground, wforeground, Size(853,480));

    vector<double> areas;
    for (unsigned int i=0; i<contours.size(); i++)
    {
        Mat contourMat(contours[i]);
        double area=contourArea(contourMat);


        // find touch points by area thresholding
        if ( area > 2000/*touchMinArea*/ )
        {

            float mind=0;
            unsigned int minj=-1, mink=-1;
            Size size=contourMat.size();

            vector<Point2i> curve=contours[i];

            for(unsigned int j=0; j<curve.size(); j++)
            {
                for(unsigned int k=0; k<curve.size(); k++)
                {
                    if(j==k) continue;
                    int x=curve[j].x-curve[k].x;
                    int y=curve[j].y-curve[k].y;
                    //x/=2;y/=2;
                    //x-=853/2;
                    //y-=240;
                    float d=sqrt( x*x+y*y);
                    if(d>mind)
                    {
                        minj=j;
                        mink=k;
                        mind=d;
                    }
                }
            }

            if(minj!=-1)
            {
                int x=curve[minj].x;
                int y=curve[minj].y;


                float dk, dj;
                dj=distanceFromBorder(curve[minj]);
                dk=distanceFromBorder(curve[mink]);
                //medianBlur(wforeground, wforeground, 3);

                int p, op;

                if(dj>dk)
                {
                    p=minj;
                    op=mink;
                }
                else
                {
                    p=mink;
                    op=minj;
                }


                Point2f p2=Point2f(
                               curve[op].x - curve[p].x,
                               curve[op].y - curve[p].y);
                float dist=sqrt(p2.x*p2.x+p2.y*p2.y);
                p2.x/=dist;
                p2.y/=dist;

                Point2i p3=Point2i(curve[p].x + p2.x*10, curve[p].y + p2.y*10);

                short d=((short*)wforeground.data)[p3.x + p3.y*853];
                //if(d<40)
                //{
                    touchPoints.push_back(Point3f(curve[p].x, curve[p].y, d<40 ? 1:0));
                    oppositeTouchPoints.push_back(curve[op]);
                    areas.push_back(area);

                //}
                blobs.push_back(contours[i]);
                labels.push_back(0);
            }
        }
    }
//#define DEBUG
#ifdef DEBUG
        if(!calibrating)
        {
            Mat3b rgb(480, 640);
            Mat3b bgr(480, 640);
            //Mat1b touch(480, 853); // touch mask

            bgr.data = (uchar *)iGen.GetImageMap();//.GetRGB24ImageMap();//xnImgeGenertor.GetImageMap();
            if(bgr.data)
            {
                cvtColor(bgr, rgb, CV_BGR2RGB);
                warpPerspective(rgb, debug, _hIMat, Size(853, 480), INTER_LINEAR);



                debug.setTo(debugColor0, wforeground>25);
                debug.setTo(debugColor1, touch);
                //Mat1f distanceMap(480, 853);
                //distanceTransform( arm, distanceMap, CV_DIST_L1, 3);
                wforeground.convertTo(depth8, CV_8U, 255 / 200);
                cvtColor(depth8, rgb, CV_GRAY2RGB);
                rgb.copyTo(debug, arm);
                //debug=rgb;
                for (unsigned int i=0; i<touchPoints.size(); i++)   // touch points
                {
                    circle(debug, touchPoints[i], 5, debugColor0, CV_FILLED);
                    line(debug, touchPoints[i], oppositeTouchPoints[i], Scalar(255,0,0,0));


                    Point2f p2=Point2f(
                                       oppositeTouchPoints[i].x - touchPoints[i].x,
                                       oppositeTouchPoints[i].y - touchPoints[i].y);
                    float dist=sqrt(p2.x*p2.x+p2.y*p2.y);
                    p2.x/=dist;
                    p2.y/=dist;

                    circle(debug, touchPoints[i]+p2*20, 5, debugColor0, CV_FILLED);
                }
                flip(debug,debug,1);
                imshow("image", debug);
            }
        }
#endif



    int i,j;
    for(i=0;i<labels.size();i++) {
        for(j=0;j<lastLabels.size();j++) {
            if(overlapped(blobs[i], lastBlobs[j])) {
                labels[i]=lastLabels[j];
                break;
            }
        }
    }

    int maxi=0;
    for(i=0;i<labels.size();i++) {
        if(labels[i]>maxi) maxi=labels[i];
    }

    highLabel=maxi;

    vector <bool>newTouch;
    for(i=0;i<labels.size();i++) {
        if(labels[i]==0) {
            labels[i]=++maxi;
        }
            //newTouch.push_back(true);

    }

    bool b;
    for(i=0;i<labels.size();i++) {
        b=false;
        for(j=0;j<lastLabels.size();j++) {
            if(labels[i]==lastLabels[j]) {
                if(touchPoints[i].z && !this->touchPoints[j].z) {
                    b=true;
                }
                break;
            }
        }
        newTouch.push_back(b);
    }
    //printf("Blobs %d\n", blobs.size());

    this->newTouch=newTouch;
    lastBlobs=blobs;
    this->touchPoints=touchPoints;
    //this->lastOTP=oppositeTouchPoints;
    lastLabels=labels;
    //this->areas=areas;
}
void TouchSensor::startCalibration(void) {
    //iGen.StartGenerating();

    calibrating=5;
}

TouchSensor::TouchSensor(xn::ImageGenerator &image, xn::DepthGenerator &_depth)
{
    iGen=image;
    dGen=_depth;
//    tracker=NULL;
    iGen.SetPixelFormat( XN_PIXEL_FORMAT_RGB24);
    dGen.GetAlternativeViewPointCap().SetViewPoint(iGen);


    touchDepthMin =10;
    touchDepthMax =30;
    touchMinArea = 40;

    imageKeypoints = 0;
    imageDescriptors = 0;

    calibrating=5;

    xMin = 0;
    xMax = 853;
    yMin = 0;
    yMax = 480;

    background.create(480,640, CV_16UC1); //=Mat1s(480,640);
    foreground.create(480,640, CV_16UC1); //=Mat1s(480, 640);
    wforeground.create(480,640, CV_16UC1); //=Mat1s(480, 853);
    arm.create(480,853, CV_8UC1); //=Mat1b(480, 853);

}


void TouchSensor::initMarker(void)
{
    //Mat1b image=imread("marker.png", 0);
    IplImage* image = cvLoadImage( "marker640.png", CV_LOAD_IMAGE_GRAYSCALE );
    src_corners = {{0,0}, {image->width,0}, {image->width, image->height}, {0, image->height}};

    CvSURFParams params = cvSURFParams(500, 1);

    CvMemStorage *storage=cvCreateMemStorage(0);
    cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );

//    imshow("Ajuste", image);
    cvReleaseImage(&image);

}
