#include "Serial.h"
#include <thread>
#include <mutex>
Serial serial_port;
float commands[6];
float data[2] = {-54.5, 170};
void SendThread(int);
void ReceiveThread(int);

int main()
{    
    serial_port.openPort();
    
    thread st(SendThread,5);
    thread rt(ReceiveThread,5);

    st.join();
    rt.join();
}

void SendThread(int)
{
    mutex _mutex;
    while (true)
    {
        _mutex.lock();
        serial_port.send(data, 0x0F);
        // serial_port.debugsend();
        this_thread::sleep_for(chrono::milliseconds(100));
        
        _mutex.unlock();
    }
    
    
}
void ReceiveThread(int)
{
    uint8_t CMD[12];
    int a, b;
    mutex _mutex;
    int speed, color, mode;
    while (true)
    {
        _mutex.lock();
        if(serial_port.receive(commands, speed, color, mode) != Serial::OK)
        { 
                this_thread::sleep_for(chrono::milliseconds(100));
        }
        // serial_port.receive(commands, speed, color, mode);
        // else
        //     serial_port.getCMD(CMD);
        _mutex.unlock();
    }
}