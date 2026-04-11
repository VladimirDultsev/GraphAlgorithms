#include "distance.h"
#include <cmath>

namespace Distance {
    /// Функция подсчёта расстояния по гаверсинусной формуле
    double calcGPSDistance(double longitude_new, double latitude_new, double longitude_old, double latitude_old)
    {
        double lat_new = latitude_old * GRADOS_RADIANES;
        double lat_old = latitude_new * GRADOS_RADIANES;
        double lat_diff = (latitude_new - latitude_old) * GRADOS_RADIANES;
        double lng_diff = (longitude_new - longitude_old) * GRADOS_RADIANES;

        double a = sin(lat_diff / 2) * sin(lat_diff / 2) + cos(lat_new) * cos(lat_old) * sin(lng_diff / 2) * sin(lng_diff / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return abs(RADIO_TERRESTRE * c);
    }
}