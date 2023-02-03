#include "RuneDetector.h"
#include "Fitting.h"
#include "iostream"

using namespace cv;

#define ACQ_FRAME_WIDTH   1200
#define ACQ_FRAME_HEIGHT  800 

#define RATIO 2

#define ISVIDEO 30 //30为正常转速，0为静止


int main()
{
    
    VideoCapture cap("../Algorithm/buff.mp4");
    RuneDetector runevison;
    FitTool fittool;
    uint32_t timestamps = 1e7;
    //VideoWriter writer("../3-28.avi", VideoWriter::fourcc('M', 'J', 'P', 'G') ,60, srcimg.size());
    int width = cap.get(CAP_PROP_FRAME_WIDTH);//640
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);//640
    cout<<"width:"<<width*RATIO<<"\theight:"<<height*RATIO<<endl;
    while(1)
    {
        //图片准备
        Mat_time image;
        cap >> image;
        if(image.empty())
        {
            cout<<"No video"<<endl;
            break;
        }
        resize(image,image, Size(width*RATIO, height*RATIO));
        int delay = ISVIDEO;
        bool result = runevison.run(image);
        runevison.target.gyro_pose.timestamp = timestamps;
        vector<cv::Point2f> nextPosition;
        fittool.runNormalRune(runevison.target, nextPosition, runevison.state);
        //cout<<fittool.fitting_data.size()<<endl;
        circle(image, runevison.target.circle_center, 8, Scalar(0, 255, 0), -1);
        circle(image, runevison.target.armor_center, 8, Scalar(255, 0, 255), -1);
        for (int i = 0; i < 4; i++)
        {
            {
                line(image, runevison.target.points[i], runevison.target.points[(i + 1) % 4], Scalar(0, 0, 255));
                circle(image, runevison.target.points[i], 4 * (i + 1), Scalar(0, 255, 255), -1);
            }
            circle(image, runevison.target.circle_center, calDistance(runevison.target.circle_center, runevison.target.armor_center), Scalar(255, 0, 0));    // 大符轨迹
        }
        
        if(!nextPosition.empty())
        {
            Point2f center = (nextPosition[0] + nextPosition[1] + nextPosition[2] + nextPosition[3]) / 4;
            circle(image, center, 8, Scalar(255, 255, 255), -1);
        }
        timestamps += 300;
        imshow("image",image);
        waitKey(ISVIDEO) & 0xFF;
    }
}