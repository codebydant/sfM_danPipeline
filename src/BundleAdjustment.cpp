#include "../include/BundleAdjustment.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <mutex>
#include <glog/logging.h>

struct SimpleReprojectionError {

  SimpleReprojectionError(double observed_x, double observed_y) :
            observed_x(observed_x), observed_y(observed_y) {}
    template<typename T>
  bool operator()(const T* const intrinsics,
                      const T* const extrinsics,
                      const T* const point,
                      T* residules) const
      {
          const T& focal_length       = intrinsics[OFFSET_FOCAL_LENGTH];
          const T& principal_point_x  = intrinsics[OFFSET_PRINCIPAL_POINT_X];
          const T& principal_point_y  = intrinsics[OFFSET_PRINCIPAL_POINT_Y];
          const T& k1                 = intrinsics[OFFSET_K1];
          const T& k2                 = intrinsics[OFFSET_K2];
          const T& k3                 = intrinsics[OFFSET_K3];
          const T& p1                 = intrinsics[OFFSET_P1];
          const T& p2                 = intrinsics[OFFSET_P2];

          // compute projective coordinates: x = RX + t.
          // extrinsics[0, 1, 2]: axis-angle
          // extrinsics[3, 4, 5]: translation
          T x[3];
          ceres::AngleAxisRotatePoint(extrinsics, point, x);
          x[0] += extrinsics[3];
          x[1] += extrinsics[4];
          x[2] += extrinsics[5];

          // compute normalized coordinates
          T xn = x[0] / x[2];
          T yn = x[1] / x[2];

          T predicted_x, predicted_y;

          // apply distortion to the normalized points to get (xd, yd)
          // do something for zero distortion
          apply_radio_distortion_camera_intrinsics(focal_length,
                                                   focal_length,
                                                   principal_point_x,
                                                   principal_point_y,
                                                   k1, k2, k3,
                                                   p1, p2,
                                                   xn, yn,
                                                   &predicted_x,
                                                   &predicted_y);

          residules[0] = predicted_x - T(observed_x_);
          residules[1] = predicted_y - T(observed_y_);
          return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x, const double observed_y) {
        return (new ceres::AutoDiffCostFunction<SimpleReprojectionError, 2, 8, 6, 3>(
                new SimpleReprojectionError(observed_x, observed_y)));
    }
    double observed_x;
    double observed_y;
};

void adjustBundle(std::vector<Point3D>& pointCloud,std::vector<cv::Matx34f>& cameraPoses,CameraData& intrinsics,const std::vector<Features>& image2dFeatures) {

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;

    //Convert camera pose parameters from [R|t] (3x4) to [Angle-Axis (3), Translation (3), focal (1)] (1x7)
    typedef cv::Matx<double, 1, 6> CameraVector;
    std::vector<CameraVector> cameraPoses6d;
    cameraPoses6d.reserve(cameraPoses.size());
    for (size_t i = 0; i < cameraPoses.size(); i++) {
        const cv::Matx34f& pose = cameraPoses[i];

        if (pose(0, 0) == 0 and pose(1, 1) == 0 and pose(2, 2) == 0) {
            //This camera pose is empty, it should not be used in the optimization
            cameraPoses6d.push_back(CameraVector());
            continue;
        }
        cv::Vec3f t(pose(0, 3), pose(1, 3), pose(2, 3));
        cv::Matx33f R = pose.get_minor<3, 3>(0, 0);
        float angleAxis[3];
        ceres::RotationMatrixToAngleAxis<float>(R.t().val, angleAxis); //Ceres assumes col-major...

        cameraPoses6d.push_back(CameraVector(angleAxis[0],angleAxis[1],angleAxis[2],t(0),t(1),t(2)));
    }

    //focal-length factor for optimization
    double focal = intrinsics.K.at<float>(0, 0);

    std::vector<cv::Vec3d> points3d(pointCloud.size());

    for (int i = 0; i < pointCloud.size(); i++) {
        const Point3D& p = pointCloud[i];
        points3d[i] = cv::Vec3d(p.pt.x, p.pt.y, p.pt.z);

        for (const auto& kv : p.idxImage) {
            //kv.first  = camera index
            //kv.second = 2d feature index
            cv::Point2f p2d = image2dFeatures[kv.first].pt2D[kv.second];

            //subtract center of projection, since the optimizer doesn't know what it is
            p2d.x -= intrinsics.K.at<float>(0, 2);
            p2d.y -= intrinsics.K.at<float>(1, 2);

            // Each Residual block takes a point and a camera as input and outputs a 2
            // dimensional residual. Internally, the cost function stores the observed
            // image location and compares the reprojection against the observation.
            ceres::CostFunction* cost_function = SimpleReprojectionError::Create(p2d.x, p2d.y);

            problem.AddResidualBlock(cost_function,NULL,cameraPoses6d[kv.first].val,points3d[i].val,&focal);
        }
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 500;
    options.eta = 1e-2;
    options.max_solver_time_in_seconds = 10;
    options.logging_type = ceres::LoggingType::SILENT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    if (not (summary.termination_type == ceres::CONVERGENCE)) {
        std::cerr << "Bundle adjustment failed." << std::endl;
        return;
    }

    //update optimized focal
    intrinsics.K.at<float>(0, 0) = focal;
    intrinsics.K.at<float>(1, 1) = focal;

    //Implement the optimized camera poses and 3D points back into the reconstruction
    for (size_t i = 0; i < cameraPoses.size(); i++) {
        cv::Matx34f& pose = cameraPoses[i];
        cv::Matx34f poseBefore = pose;

        if (pose(0, 0) == 0 and pose(1, 1) == 0 and pose(2, 2) == 0) {
            //This camera pose is empty, it was not used in the optimization
            continue;
        }

        //Convert optimized Angle-Axis back to rotation matrix
        double rotationMat[9] = { 0 };
        ceres::AngleAxisToRotationMatrix(cameraPoses6d[i].val, rotationMat);

        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                pose(c, r) = rotationMat[r * 3 + c]; //`rotationMat` is col-major...
            }
        }

        //Translation
        pose(0, 3) = cameraPoses6d[i](3);
        pose(1, 3) = cameraPoses6d[i](4);
        pose(2, 3) = cameraPoses6d[i](5);
    }

    for (int i = 0; i < pointCloud.size(); i++) {
        pointCloud[i].pt.x = points3d[i](0);
        pointCloud[i].pt.y = points3d[i](1);
        pointCloud[i].pt.z = points3d[i](2);
    }
}


