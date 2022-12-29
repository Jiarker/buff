#include "ImageProgress.h"
#include <thread>

int main()
{    
    ImageProgress main_progress;

    thread produce(&ImageProgress::ImageProducer,&main_progress);
    thread consume(&ImageProgress::ImageConsumer2, &main_progress);
    thread readport(&ImageProgress::ReadIMU2, &main_progress);
    thread sendport(&ImageProgress::SendData2, &main_progress);

    produce.join();
    consume.join();
    readport.join();
    sendport.join();
}