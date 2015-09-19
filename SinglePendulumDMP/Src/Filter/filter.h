#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f4xx.h"
#include "stdlib.h"




void kalman(int n, int m, int len, double f[], double d[], double u[], double b[], double q[], double h[], double r, double z[], double x[], double p[], double g[]);






#endif
