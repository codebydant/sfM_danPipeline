//***********************************************
//HEADERS
//***********************************************

#include "Segmentation.h"

class BundleAdjustment {

  private:

  public:
    BundleAdjustment(){

    }

    ~BundleAdjustment(){

    }

    static void adjustBundle(std::vector<Point3D>& pointCloud, std::vector<cv::Matx34f>& cameraPoses,
                      CameraData& intrinsics,const std::vector<Feature>& image2dFeature);

};







