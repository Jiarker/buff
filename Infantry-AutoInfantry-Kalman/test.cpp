#include <iostream>
#include <chrono>
#include <memory.h>

using namespace std;

int main() {
    u_char buf[3] = {0x01, 0x02, 0x00};
    uint16_t t = 0;
    memcpy(&t, &buf[1], 2);

    cout << t << endl;
}
