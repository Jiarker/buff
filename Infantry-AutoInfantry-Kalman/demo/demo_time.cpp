#include <mercure_driver.h>

int main()
{
    camera::MercureDriver cap;

    while (true)
    {
        Mat_time image(ACQ_FRAME_HEIGHT, ACQ_FRAME_WIDTH, CV_8UC3);
        cap >> image;
        cout << image.produced_time.time_since_epoch().count() << endl;
        cv::imshow("frame", image);
        if (cv::waitKey(1) == 'q')
            break;
    }
    
}