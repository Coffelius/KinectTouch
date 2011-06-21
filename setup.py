from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

setup(
  name = 'Demos',
  ext_modules=[
    Extension("KinectTouch",
                sources=["wrapper/Kinect.pyx",
                "libs/ofxCvKalman.cpp",
                "src/KinectTouch.cpp",
                "src/surfcalibration.cpp",
                "src/TUIO/TuioClient.cpp",
                "src/TUIO/TuioServer.cpp",
                "src/TUIO/TuioTime.cpp",
                "src/TouchSensor.cpp",
                "src/oscpack/ip/IpEndpointName.cpp",
                "src/oscpack/ip/posix/NetworkingUtils.cpp",
                "src/oscpack/ip/posix/UdpSocket.cpp",
                "src/oscpack/osc/OscOutboundPacketStream.cpp",
                "src/oscpack/osc/OscPrintReceivedElements.cpp",
                "src/oscpack/osc/OscReceivedElements.cpp",
                "src/oscpack/osc/OscTypes.cpp"

              ],

              # Note, you can link against a c++ library instead of including the source
              libraries=["glut", "OpenNI", "XnVNite_1_3_1", "opencv_core", "opencv_highgui", "opencv_legacy"],
              include_dirs=["src", "/usr/include/nite", "/usr/include/ni", "src/TUIO", "src/oscpack", "libs"],
              language="c++"),
    ],
  cmdclass = {'build_ext': build_ext},

)
