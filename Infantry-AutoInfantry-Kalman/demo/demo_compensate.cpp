#include "Predictor.h"

int main()
{
    CoordPredictor predictor;
    predictor.setBulletSpeed(15);

    cv::Point3f _coord(400, 300, -300);
    

    for (int i = 0; i < 6; i++)
    {
        cv::Point3f coord = _coord;
        coord.x += i * 50;
        // coord.y += i * 50;
        predictor.compensate(coord, 0.003);
        cout << coord << endl;
    }
}