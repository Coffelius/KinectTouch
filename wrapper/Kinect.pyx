cdef extern from "KinectTouch.h":
    ctypedef struct c_KinectTouch "KinectTouch":
        void update()
        void calibrate()
    c_KinectTouch *new_KinectTouch "new KinectTouch" ()
    void del_KinectTouch "delete" (c_KinectTouch *kinect)

cdef class KinectTouch:
    cdef c_KinectTouch *thisptr

    def __cinit__(self):
        self.thisptr=new_KinectTouch()

    def __dealloc__(self):
        del_KinectTouch(self.thisptr)

    def update(self):
        self.thisptr.update()

    def calibrate(self):
        self.thisptr.calibrate()
