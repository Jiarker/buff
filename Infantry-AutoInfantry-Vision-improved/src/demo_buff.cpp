#include "RuneDetector.h"
#include "Fitting.h"
#include "iostream"

using namespace cv;

#define ACQ_FRAME_WIDTH   1200
#define ACQ_FRAME_HEIGHT  800 

#define RATIO 2

#define ISVIDEO 100 //30为正常转速，0为静止


int main()
{
    
    VideoCapture cap("../Algorithm/buff.mp4");
    RuneDetector runevison;
    //VideoWriter writer("../3-28.avi", VideoWriter::fourcc('M', 'J', 'P', 'G') ,60, srcimg.size());
    int width = cap.get(CAP_PROP_FRAME_WIDTH);//640
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);//640
    cout<<"width:"<<width<<"\theight:"<<height<<endl;
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
        runevison.target.points; 
        circle(image, runevison.target.circle_center, 5, Scalar(0, 255, 0), -1);
        circle(image, runevison.target.armor_center, 5, Scalar(255, 0, 255), -1);
        for (int i = 0; i < 4; i++)
        {
            {
                line(image, runevison.target.points[i], runevison.target.points[(i + 1) % 4], Scalar(0, 0, 255));
                circle(image, runevison.target.points[i], 2 * (i + 1), Scalar(0, 0, 255), -1);
            }
            circle(image, runevison.target.circle_center, calDistance(runevison.target.circle_center, runevison.target.armor_center), Scalar(255, 0, 0));    // 大符轨迹
        }
        imshow("image",image);
        waitKey(ISVIDEO) & 0xFF;
    }
}