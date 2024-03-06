#if !defined(HYPERCUVISFUNCTIONS_H)
#define HYPERCUVISFUNCTIONS_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include "hyperfunctions.h"
#include "cuvis.hpp"
#include <cassert>
#include <cmath>
#if use_cuda
#include "../src/hypergpufunctions.cu"
#endif

using namespace std;
using namespace cv;

#if use_cuda
class HyperFunctionsCuvis : public HyperFunctionsGPU 
#else
class HyperFunctionsCuvis : public HyperFunctions 
#endif
{



public:

    

    cuvis::ProcessingArgs procArgs;   

    string cubert_settings="../settings/ultris20";  //camera settings file 
    string factory_dir="../settings/ultris20"; // requires init.daq file
    string output_dir="../../HyperImages/export/";
    string cubert_img;
    string dark_img;
    string white_img;
    string dist_img;

    float dist_val_mm = 1000; // distance to target in mm

    void LoadImageHyper(string file_name, bool isImage1 );
    void TakeImageHyper1(string file_name, const int exposure_ms, const int num_images);
    void ExportTiff();
    void ReprocessImage(string file_name, bool isImage1 ); 


};


#endif
