#include <iostream>
#include "opencv2/opencv.hpp"
#include <cmath>
#include "../src/hypercuvisfunctions.cpp"
#include "cuvis.hpp"
#include <cassert>

using namespace cv;
using namespace std;


int main (int argc, char *argv[])
{

    HyperFunctionsCuvis HyperFunctions1;

    // below is for ultris 5 example
    // HyperFunctions1.cubert_img = "../../HyperImages/export/Test_001.cu3s";
    // HyperFunctions1.dark_img = "../../HyperImages/Calib100/dark.cu3s";
    // HyperFunctions1.white_img = "../../HyperImages/Calib100/white.cu3s";
    // HyperFunctions1.dist_img = "../../HyperImages/Calib100/distance.cu3s";
    
    // // below are needed if the ultris5 is used instead of the ultris 20
    // HyperFunctions1.cubert_settings="../settings/ultris5";  //camera settings file 
    // HyperFunctions1.factor_dir="../settings/ultris5"; // requires init.daq file


    // ultris 20 example
    HyperFunctions1.cubert_img = "../../HyperImages/cornfields/session_002/session_002_490.cu3";
    HyperFunctions1.dark_img = "../../HyperImages/cornfields/Calibration/dark__session_002_003_snapshot16423119279414228.cu3";
    HyperFunctions1.white_img = "../../HyperImages/cornfields/Calibration/white__session_002_752_snapshot16423136896447489.cu3";
    HyperFunctions1.dist_img = "../../HyperImages/cornfields/Calibration/distanceCalib__session_000_790_snapshot16423004058237746.cu3";



    // below is for taking an image
    // string file_name_base, const int exposure_ms, const int num_image
    // for some reason cannot take image and reprocess image in the same run
    // HyperFunctions1.TakeImageHyper1("test",100, 1);



    // below is for reprocessing the image in the case when raw mode is required
    HyperFunctions1.ReprocessImage( HyperFunctions1.cubert_img);  

    // below loads the hyperspectral image
    // this is used when the image has already been processed
    // HyperFunctions1.cubert_img = "../../HyperImages/vegetation_000_000_snapshot.cu3";
    // HyperFunctions1.LoadImageHyper(HyperFunctions1.cubert_img);

    
   
  
    // below generates false rgb of image
    HyperFunctions1.false_img_b=2;
    HyperFunctions1.false_img_g=13;
    HyperFunctions1.false_img_r=31;
    HyperFunctions1.GenerateFalseImg();
    imshow("test",  HyperFunctions1.false_img);
    cv::waitKey();
    // cv::imwrite(HyperFunctions1.output_dir+"test_img.png", HyperFunctions1.false_img);


    // below is to export tiff image
    // HyperFunctions1.ExportTiff();


    // below only works for ultris20 images due to different number of layers in default spectral database
    // below loads spectral database and performs classification
    // string spec_database= "../json/spectral_database_U5.json";
    // string spec_database= "../json/spectral_database_U20.json";
    // HyperFunctions1.read_ref_spec_json(spec_database);

    // generate spectral similarity image
   	// HyperFunctions1.SpecSimilParent();
    // HyperFunctions1.DispSpecSim();

    // generate classified image
    // HyperFunctions1.SemanticSegmenter();
    // HyperFunctions1.DispClassifiedImage();
    cv::waitKey();

  
cout<<"finished"<<endl;
  return 0;
}
