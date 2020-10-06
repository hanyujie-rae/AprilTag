#ifndef _APRILTAGLOCATOR
#define _APRILTAGLOCATOR

#include <vector>
#include <opencv2/opencv.hpp>
#include"apriltag.h"



extern "C" {
#include "apriltag_pose.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h" 
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "common/homography.h"
}

using namespace cv;
using namespace std;

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

enum tagFamily{kTAG16H5, kTAG25H9, kTAG36H11, kTAGCircle21H7, kTAGCircle49H12, kTAGCustom48H12,kTAGStandard41H12, kTAGStandard52H13};

struct AprilTagInfo
{
    Vec6d pose;
    String id;
};


class AprilTagLocator
{
    public:
        /*************************************************
        Function: AprilTagLocator
        Description: 构造函数，生成识别某种AprilTag的定位器
        Input: tagFamily --代表AprilTag的种类
        Output: None
        Others: tagFamily种类可见 https://github.com/AprilRobotics/apriltag-imgs
        *************************************************/
        AprilTagLocator(tagFamily tag=kTAG36H11);
        
        /*************************************************
        Function:getAllPos
        Description:获取图像信息
        Input:Mat &frame--图像的地址
        Output: None
        *************************************************/
        vector<AprilTagInfo> getAllPos(const Mat &frame);  

        /*************************************************
        Function: setCameraParameters
        Description:设置标定的相机参数
        Input: tagsize--代表标识的实际尺寸  fx，fy，cx，cy--代表相机的四元参数
        Output: None
        Others:关于相机的四元参数可见 https://docs.opencv.org/4.4.0/d4/d94/tutorial_camera_calibration.html
        *************************************************/

        void setCameraParameters(double tagsize, double fx, double fy, double cx, double cy);
        
        /*************************************************
        Function:setExtraParameters
        Description:设置其他参数
        Input:decimate--Decimate input image by this factor; 
        blur--Apply low-pass blur to input; threads--Use this many CPU threads;debug--Enable debugging output (slow);refine_edges--Spend more time trying to align edges of tags
        Output: None
        *************************************************/
        void setExtraParameters(float decimate,float blur,int threads,int debug,int refine_edges);

    private:
        Mat _gray;
        void initFamily(tagFamily tag);
        void initExtraParameters();
        void initAllPose();
        cv::Vec6d poseToVec(apriltag_pose_t pose);


        double standardRad(double t);
       
        
        void outputTagNumber();
        void drawOutlines();
        
        // detector of apriltag
        apriltag_family_t *tf;
        apriltag_detector_t *td;
        // parameters of the camera
        apriltag_detection_info_t info;

        // extra function
        bool debug=true;                // Enable debugging output (slow)
        bool quiet=false;               // Reduce output
        tagFamily family=kTAG36H11;     // Tag family to use
        int threads = 1;                // Use this many CPU threads
        double decimate = 2.0;          // Decimate input image by this factor
        double blur = 0.0;              // Apply low-pass blur to input
        bool refine_edges = true;       // Spend more time trying to align edges of tags

        double x=0;
        double y=0;
        double z=0;
        double yaw=0;
        double pitch=0;
        double roll=0;

        int detectedNum = 0;
};


#endif