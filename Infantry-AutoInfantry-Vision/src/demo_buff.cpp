#include "RuneDetector.h"
#include "Fitting.h"
#include "iostream"

using namespace cv;

#define ACQ_FRAME_WIDTH   1200
#define ACQ_FRAME_HEIGHT  800 

#define RATIO 2

#define ISVIDEO 0 //30为正常转速，0为静止


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
        runevison.run(image);
        //imshow("buff",image);
        waitKey(ISVIDEO) & 0xFF;
    }
}