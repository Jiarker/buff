#include "ArmorDetector.h"
#include <thread>
#include <mutex>
#include "mercure_driver.h"

using namespace std;
using namespace cv;

#define VIDEO_IN
//#define CAMERA_IN

typedef struct data
{
    Mat frame[5];
}DATA;

void imageProduce(DATA &);
void imageConsume(DATA&);

VideoWriter writer;
#ifdef CAMERA_IN
camera::MercureDriver cap;
#endif
#ifdef VIDEO_IN
VideoCapture cap("/home/shanzoom/RM_DATA/RM-video/2021-西北站-步兵实录.mp4");
#endif
ArmorParam armor_param;
ArmorDetector detector = ArmorDetector();
ArmorBox armor;
int ProIdx = 0;
int ConIdx = 0;
std::mutex _mutex;
int main()
{
    DATA data_1;
    

    armor_param.loadParam();
    string path("/home/shanzoom/Infantry_Project/Configure/NUM_Classifier.onnx");
    detector = ArmorDetector(COLOR::BLUE,path,armor_param);
    Mat frame; frame.create(900,1600,CV_8UC3);

    cap >> frame;
    writer.open("/home/shanzoom/01.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),25.0,frame.size());

    thread thread_1(imageProduce,ref(data_1));
    thread thread_2(imageConsume,ref(data_1));

    thread_1.join();
    thread_2.join();    
}

void imageProduce(DATA &data)
{
    
    while (1)
    {
    #ifdef CAMERA_IN
        Mat_time src(ACQ_FRAME_HEIGHT,ACQ_FRAME_WIDTH,CV_8UC3);
    #endif
    #ifdef VIDEO_IN
        Mat src;
    #endif
    while (ProIdx - ConIdx >= 5)
    {
        this_thread::sleep_for(chrono::milliseconds(5));
    }
    
    _mutex.lock();
        cap >> src;
        data.frame[ProIdx%5] = src;
        ProIdx++;
    _mutex.unlock();

    }
    
    
}
void imageConsume(DATA &data)
{
    
    while (1)
    {
        while (ConIdx >= ProIdx);
        _mutex.lock();
        if (ConIdx > 5 && ProIdx > 5 && ProIdx-ConIdx==1 && ProIdx%5 != 0)
        {
            ProIdx = ProIdx%5;  ConIdx = ConIdx%5;
            std::cout << ProIdx << " " << ConIdx << endl;
            std::cout << "Error" << endl;
        }
        

        double t = getTickCount();
        int ImageIdx = ProIdx - 1;
    
        detector.setRoi(data.frame[ImageIdx%5],armor.box);
        if (detector.findArmorBox(armor))
        {
            Point2f pts[4];
            for(int j = 0; j < 1; j++)
            {
                detector._armor_boxes[j].rect.points(pts);
                for(int i = 0; i<4 ; i++)
                {
                    line(data.frame[ImageIdx%5],pts[i%4],pts[(i+1)%4], Scalar(0,0,255),2);
                }
                string text;
                
                    text = "ID: " + to_string(armor.id) + " Angle: " + to_string(armor.rect.angle);
                
                putText(data.frame[ImageIdx%5], text, armor.box.tl(), FONT_HERSHEY_PLAIN,2.0, Scalar(0,0,255));
                std::cout << "armor width: " << armor.box.width << endl;
                std::cout << "armor height: "<< armor.box.height << endl;
                std::cout << "armor angle: " << armor.rect.angle << endl;
                std::cout << "armor type: " << armor.type << endl;
                std::cout << "armor ID: " << armor.id << endl;
                
            }
            std::cout << "armor_num: " << detector._armor_boxes.size() << endl;
            std::cout << endl;
        }
    imshow("frame", data.frame[ImageIdx%5]);
    writer << data.frame[ImageIdx%5];
    std::cout << "Tick: " << (double)(getTickCount()-t)/getTickFrequency() << endl;
    std::cout << endl;
    if (waitKey(1) == 'q') exit(0);
        ConIdx = ImageIdx + 1;
    _mutex.unlock();
    }
    
    
}