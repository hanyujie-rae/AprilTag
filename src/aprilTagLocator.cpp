

#include"aprilTagLocator.hpp"
using namespace std;


AprilTagLocator::AprilTagLocator(tagFamily tag){
    initFamily(tag);
    initExtraParameters();
    setCameraParameters(106, 5531,5531,320,240);
}

vector<AprilTagInfo> AprilTagLocator::getAllPos(const Mat &frame){
    cvtColor(frame, _gray, COLOR_BGR2GRAY);
    image_u8_t im = { 
        .width = _gray.cols,        
        .height = _gray.rows,
        .stride = _gray.cols,
        .buf = _gray.data
    };  

    zarray_t *detections = apriltag_detector_detect(td, &im);
    detectedNum = zarray_size(detections);

    vector<AprilTagInfo>  allVecPose;
    apriltag_pose_t _pose;
    for (size_t i = 0; i < detectedNum; i++)
    {
        AprilTagInfo _tag;
        apriltag_detection_t *det; 
        zarray_get(detections, i, &det);
        info.det = det;
        estimate_pose_for_tag_homography(&info, &_pose);
        _tag.pose = poseToVec(_pose);
        _tag.id = det->id;
        allVecPose.push_back(_tag);
    }
    return allVecPose;
}  





cv::Vec6d AprilTagLocator::poseToVec(apriltag_pose_t pose){
    estimate_pose_for_tag_homography(&info, &pose);
    double yaw = 180*standardRad(atan2(pose.R->data[3], pose.R->data[0]));
    double pitch=180*standardRad(sin(pose.R->data[6]));
    double roll=180*standardRad(atan2(pose.R->data[7],pose.R->data[8]));

    yaw=yaw/PI;
    pitch=pitch/PI;
    roll=roll/PI; 

    double x_relevant = pose.t->data[0];
    double y_relevant = pose.t->data[1];
    double z_relevant = pose.t->data[2];

    Vec6d pose_vec = {x_relevant,y_relevant,z_relevant,yaw,pitch,roll};
    return pose_vec;
}

double AprilTagLocator::standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

 void AprilTagLocator::outputTagNumber()
 {   
     
    //  while (true) 
    //     {
            
    //     VideoCapture cap(0);
    //     cap >> frame;
    //     cvtColor(frame, gray, COLOR_BGR2GRAY);

    //     // Make an image_u8_t header for the Mat data , and turn the gray image into the candidate of the tags waiting to be recongnized(&im)
    //     image_u8_t im = { .width = gray.cols,        
    //         .height = gray.rows,
    //         .stride = gray.cols,
    //         .buf = gray.data
        // }                  

        // zarray_t *detections = apriltag_detector_detect(td, &im);    //parttern recongnize to start
        // cout << zarray_size(detections) << " tags detected" << endl;
        // // }
 }



// void AprilTagLocator::drawOutlines()
// {       
    
//     for (int i = 0; i < zarray_size(detections); i++) 
//         {
//             apriltag_detection_t *det;                     
//             zarray_get(detections, i, &det);
//             line(frame, Point(det->p[0][0], det->p[0][1]),
//                      Point(det->p[1][0], det->p[1][1]),
//                      Scalar(0, 0xff, 0), 2);
//             line(frame, Point(det->p[0][0], det->p[0][1]),
//                      Point(det->p[3][0], det->p[3][1]),
//                      Scalar(0, 0, 0xff), 2);
//             line(frame, Point(det->p[1][0], det->p[1][1]),
//                      Point(det->p[2][0], det->p[2][1]),
//                      Scalar(0xff, 0, 0), 2);
//             line(frame, Point(det->p[2][0], det->p[2][1]),
//                      Point(det->p[3][0], det->p[3][1]),
//                      Scalar(0xff, 0, 0), 2);
            
//             stringstream ss;
//             ss << det->id;
//             String text = ss.str();
            
//         }
            
// }
/*****************************************/

void AprilTagLocator::initFamily(tagFamily tag)

{

    tf = NULL;
    switch (tag)
    {
    case kTAG16H5:
        tf = tag16h5_create();
        break;
    case kTAG25H9:
        tf = tag25h9_create();
        break;
    case kTAG36H11:
        tf = tag36h11_create();
        break;
    case kTAGCircle21H7:
        tf = tagCircle21h7_create();
        break;
    case  kTAGCircle49H12:
        tf = tagCircle49h12_create();
        break;
    case kTAGCustom48H12:
        tf = tagCustom48h12_create();
        break;
    case kTAGStandard41H12:
        tf = tagStandard41h12_create();
        break;
    case kTAGStandard52H13:
        tf = tagStandard52h13_create();
        break;
    
    default:
        break;
    }
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);  
}



void AprilTagLocator::setExtraParameters(float decimate,float blur,int threads,int debug,int refine_edges){
    td->quad_decimate = decimate;       
    td->quad_sigma = blur;
    td->nthreads = threads;
    td->debug = debug;
    td->refine_edges = refine_edges;
}

void AprilTagLocator::initExtraParameters(){
    td->quad_decimate = decimate;       
    td->quad_sigma = blur;
    td->nthreads = threads;
    td->debug = debug;
    td->refine_edges = refine_edges;
}

void AprilTagLocator::setCameraParameters(double tagsize, double fx, double fy, double cx, double cy){
    info.tagsize = tagsize;
    info.fx = fx;
    info.fy = fy;
    info.cx = cx;
    info.cy = cy;
}