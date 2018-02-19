#include <opencv2/opencv.hpp>
#include <opencv_lib.hpp>
#include <opencv2/viz.hpp>
#include <iostream>

//widget_accessor‚ðŽg‚í‚È‚¢‚Æ‚«VTK‚Í•s—v
//#include <opencv2/viz/widget_accessor.hpp>
//#include <opencv_vtk_lib.hpp>
//#include <vtkPoints.h>
//#include <vtkTriangle.h>
//#include <vtkCellArray.h>
//#include <vtkPolyData.h>
//#include <vtkPolyDataMapper.h>
//#include <vtkIdList.h>
//#include <vtkActor.h>
//#include <vtkProp.h>

using namespace cv;
using namespace std;

int main()
{

	viz::Viz3d myWindow("Point Cloud");

	int height = 480, width = 640;
	Mat pCloud(height, width, CV_32FC3);

	float fx = 525.0f, // default
		  fy = 525.0f,
		  cx = 319.5f,
		  cy = 239.5f;

	Mat colorImage;
	Mat depth, depth_flt;

	colorImage = imread("rgb.png");
	depth = imread("depth.png", -1);
	imshow("rgb", colorImage);
	imshow("depth", depth);

	depth.convertTo(depth_flt, CV_32FC1, 1.f / 5000.f);
	depth_flt.setTo(std::numeric_limits<float>::quiet_NaN(), depth == 0);
	depth = depth_flt;

	for (int y = 0; y < 480; y++){
		for (int x = 0; x < 640; x++){
			if (depth.at<float>(y, x) < 8.0 && depth.at<float>(y, x) > 0.4){
				//RGB-D Dataset
				float Z = depth.at<float>(y, x);
				float X = (x - cx) * Z / fx;
				float Y = (y - cy) * Z / fy;
				pCloud.at<Vec3f>(y, x) = Vec3f(X, Y, Z);
			}
			else{
				//RGB-D Dataset
				pCloud.at<Vec3f>(y, x) = Vec3f(0.f, 0.f, 0.f);
			}
		}
	}

	viz::WCloud wcloud(pCloud, colorImage);
	myWindow.showWidget("CLOUD", wcloud);
	myWindow.spin();

	return 0;
}
