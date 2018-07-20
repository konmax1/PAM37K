#ifndef GILBERT_H
#define GILBERT_H

//#include "arm_math.h" 

#define GILBERT_POR     32                      // порядок фильтра
#define GILBERT_POR2    GILBERT_POR/2           // половина от порядка фильтра

float GilbertTransform(float &sample);  // преобразование Гильберта

#endif
