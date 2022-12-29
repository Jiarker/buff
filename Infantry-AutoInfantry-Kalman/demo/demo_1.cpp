#include"mercure_driver.h"
#include"ArmorDetector.h"

using namespace std;
using namespace cv;

#define DEBUG_SHOW
//#define DEBUG_SHOW_ARMORS

int main()
{
    VideoWriter writer;

    camera::MercureDriver cap;
    ArmorParam armor_param;
    armor_param.loadParam();
    string path("/home/shanzoom/Infantry_Project/Configure/NUM_Classifier.onnx");
    ArmorDetector detector = ArmorDetector(COLOR::BLUE,path,armor_param);
    Mat frame;
    ArmorBox armor;
    VideoCapture video("/home/shanzoom/RM_DATA/RM-video/2021-西北站-步兵实录.mp4");
	video >> frame;
    writer.open("/home/shanzoom/01.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),25.0,frame.size());
    while (true)
    {
        double t = getTickCount();
        //frame.create(ACQ_FRAME_HEIGHT,ACQ_FRAME_WIDTH,CV_8UC3);
        video >> frame;
        
        Point2f pts[4];
        detector.setRoi(frame, armor.box);
if (detector.findArmorBox(armor))
{        
	#ifndef DEBUG_SHOW_ARMORS
        armor.rect.points(pts);
        for(int i = 0; i<4 ; i++)
        {
            line(frame,pts[i%4],pts[(i+1)%4], Scalar(0,0,255),2);
        }
        string text;

            text = "ID: " + to_string(armor.id) + " Angle: " + to_string(armor.rect.angle);
        
        putText(frame, text, armor.box.tl(), FONT_HERSHEY_PLAIN,2.0, Scalar(0,0,255));
        cout << "armor width: " << armor.box.width << endl;
        cout << "armor height: "<< armor.box.height << endl;
        cout << "armor angle: " << armor.rect.angle << endl;
        cout << "armor type: " << armor.type << endl;
        cout << "armor ID: " << armor.id << endl;

        cout << endl;
	#else
		for (int i = 0; i < detector._armor_boxes.size(); i++)
		{
			detector._armor_boxes[i].rect.points(pts);
			for(int j = 0; j<4 ; j++)
			{
				line(frame,pts[j%4],pts[(j+1)%4], Scalar(0,0,255),1);
			}
			string text;

            	text = "ID: " + to_string(detector._armor_boxes[i].id) + " Angle: " + to_string(detector._armor_boxes[i].rect.angle);
        
			putText(frame, text, detector._armor_boxes[i].box.tl(), FONT_HERSHEY_PLAIN,1.0, Scalar(0,0,255));

		}
	#endif
}
        cout << "Speed: " << (double)(getTickCount() -t) / getTickFrequency() << endl;
        if(frame.empty())   cout << "Frame Empty!" << endl;
#ifdef DEBUG_SHOW
        imshow("frame", frame);
#endif
        writer << frame;
#ifdef DEBUG_SHOW
        if(waitKey(1) == 'q')   break;
#endif
    }
    destroyAllWindows();
}
