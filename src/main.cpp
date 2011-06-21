#include "KinectTouch.h"

int main()
{

    KinectTouch kinectTouch;
    char key=0;

    while ( key != 27 )
    {
        key=waitKey(1);
        if(key=='c') kinectTouch.calibrate();
        kinectTouch.update();

    }

    return 0;
}
