#ifndef MYFALCON_H
#define MYFALCON_H

#include <falcon/core/FalconDevice.h>

bool initialise(libnifalcon::FalconDevice* falcon);

class myFalcon {
   private:
    double max_x, max_y, max_z;
    double min_x, min_y, min_z;
    double x, y, z;
    bool button1Down, button2Down, button3Down, button4Down;
    int button1, button2, button3, button4;

   public:
    myFalcon(libnifalcon::FalconDevice* falcon);
    void update(libnifalcon::FalconDevice* falcon);
    void get(double* x, double* y, double* z, int* button1, int* button2, int* button3, int* button4);
};

#endif