//***********************************************
//HEADERS
//***********************************************

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "Visualizer.h"

class BundleAdjustment : public Utilities {

  private:

  public:
    BundleAdjustment(){

    }

    ~BundleAdjustment(){

    }

    void adjustBundle(std::vector<Point3D>& pointCloud, std::vector<cv::Matx34f>& cameraPoses,CameraData& intrinsics,const std::vector<Feature>& image2dFeature);

    // Templated pinhole camera model for used with Ceres.  The camera is
    // parameterized using 7 parameters: 3 for rotation, 3 for translation, 1 for
    // focal length. The principal point is not modeled (assumed be located at the
    // image center, and already subtracted from 'observed'), and focal_x = focal_y.

};







