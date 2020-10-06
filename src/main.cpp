#include<vector>
#include<aprilTagLocator.hpp>
#include<opencv2/opencv.hpp>


int main()
{
    
    AprilTagLocator locator = AprilTagLocator();
    VideoCapture cap(0);
    VideoWriter writer = VideoWriter();
    Mat src;

    cap>>src;
    writer.open("./test.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),20,src.size(),true);
    

    while (waitKey(20)!='q')
    {
        cap>>src;
        
        vector<AprilTagInfo> allPose = locator.getAllPos(src);
        
        char text[128];
        if (allPose.size()>0)
        { 
            AprilTagInfo target = allPose.front();
            sprintf(text,"x:%.2f\n y:%.2f\n z:%.2f\n yaw:%.2f \npitch:%.2f \nroll:%.2f",target.pose[0],target.pose[1],target.pose[2],target.pose[3],target.pose[4],target.pose[5]);
        }else{
            strcpy(text,"no targer detected");
        }

        putText(src, text, Point(0,30),
                    FONT_HERSHEY_PLAIN, 1, Scalar(0xff, 0x99, 0), 1);

        if (!src.empty())
        {
            imshow("test",src);
        }else
        {
            cout<<"camera wrong";
        }

       writer.write(src);
    }

    writer.release();
        
      
    return 0;
}