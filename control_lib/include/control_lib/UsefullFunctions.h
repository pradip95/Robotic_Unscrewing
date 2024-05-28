#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <thread>
#include <utility>
#include <vector>


void printArray(double* theArray, int numberOfRows, int numberOfColumns);

template <typename containerType>
void printContainer(containerType& container);

void RadiantoDegree(std::vector<double> radVector);

void write_csv(std::string filename,
               std::vector<std::pair<std::string, std::vector<double> > > dataset);

void wait(unsigned int milliseconds);
