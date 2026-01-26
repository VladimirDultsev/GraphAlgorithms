#ifndef DISTANCE_H
#define DISTANCE_H

#define PI 3.14159265358979323846
#define RADIO_TERRESTRE 6372797.56085
#define GRADOS_RADIANES PI / 180
#define RADIANES_GRADOS 180 / PI

namespace Distance {
    /// Функция подсчёта расстояния по гаверсинусной формуле
    double calcGPSDistance(double longitude_new, double latitude_new, 
                          double longitude_old, double latitude_old);
}

#endif