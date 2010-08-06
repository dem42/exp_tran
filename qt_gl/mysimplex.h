#ifndef MY_SIMPLEX_H
#define MY_SIMPLEX_H

#include <vector>
#include <iostream>
#include <fstream>
#include "errorfunction.h"

/*
 * nelder mead downhill simplex (numerical optimization that doesnt need gradients)
 * @param func     function to optimize
 * @param start    initial vector
 * @param n        size of initial vector (dimension of the problem )
 * @param scale    used for calculating the vertices of the simplex
 * @return min     function minimum
 */
double mysimplex(ErrorFunction &func, std::vector<double>& start, int n, double scale);

#endif /*MY_SIMPLEX_H*/
