#include "include/BundleAdjustment.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct SimpleReprojectionError {
    SimpleReprojectionError(double observed_x, double observed_y) :
            observed_x(observed_x), observed_y(observed_y) {
    }
    template<typename T>
    bool operator()(const T* const camera,
                                const T* const point,
                                        const T* const focal,
                                                  T* residuals) const {
        T p[3];
        // Rotate: camera[0,1,2] are the angle-axis rotation.
        ceres::AngleAxisRotatePoint(camera, point, p);

        // Translate: camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Perspective divide
        const T xp = p[0] / p[2];
        const T yp = p[1] / p[2];

        // Compute final projected point position.
        const T predicted_x = *focal * xp;
        const T predicted_y = *focal * yp;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x, const double observed_y) {
        return (new ceres::AutoDiffCostFunction<SimpleReprojectionError, 2, 6, 3, 1>(
                new SimpleReprojectionError(observed_x, observed_y)));
    }
    double observed_x;
    double observed_y;
};

void BundleAdjustment::adjustBundle(std::vector<Point3D>& pointCloud,std::vector<cv::Matx34d>&           cameraPoses, Intrinsics& intrinsics, const std::vector<std::vector<cv::Point2d>>& image2dFeatures) {

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::Problem problem;

    //Convert camera pose parameters from [R|t] (3x4) to [Angle-Axis (3), Translation (3), focal (1)] (1x7)
    using CameraVector = cv::Matx<double,1,6>;
    std::vector<CameraVector> cameraPoses6d;
    cameraPoses6d.reserve(cameraPoses.size());
    for(size_t i = 0; i < cameraPoses.size(); i++){
        const cv::Matx34d& pose = cameraPoses[i];

        if(pose(0, 0) == 0 and pose(1, 1) == 0 and pose(2, 2) == 0) {
            //This camera pose is empty, it should not be used in the optimization
            cameraPoses6d.push_back(CameraVector());
            continue;
        }
        cv::Vec3d t(pose(0, 3), pose(1, 3), pose(2, 3));
        cv::Matx33d R = pose.get_minor<3, 3>(0, 0);
        double angleAxis[3];
        ceres::RotationMatrixToAngleAxis<double>(R.t().val, angleAxis); //Ceres assumes col-major...

        cameraPoses6d.push_back(CameraVector(
                angleAxis[0],
                angleAxis[1],
                angleAxis[2],
                t(0),
                t(1),
                t(2)));
    }

    //focal-length factor for optimization
    double focal = (double)intrinsics.K.at<double>(0, 0);

    std::vector<cv::Vec3d> points3d(pointCloud.size());

    for(int i = 0; i < pointCloud.size(); i++) {
        const Point3D& p = pointCloud[i];
        points3d[i] = cv::Vec3d(p.pt.x, p.pt.y, p.pt.z);

        for(const std::pair<const int,int>& kv : p.idxImage) {
            //kv.first  = camera index
            //kv.second = 2d feature index

            cv::Point2d p2d = image2dFeatures[kv.first][kv.second];

            //subtract center of projection, since the optimizer doesn't know what it is
            double cx = (double)intrinsics.K.at<double>(0, 2);
            double cy = (double)intrinsics.K.at<double>(1, 2);
            p2d.x -= cx;
            p2d.y -= cy;

            // Each Residual block takes a point and a camera as input and outputs a 2
            // dimensional residual. Internally, the cost function stores the observed
            // image location and compares the reprojection against the observation.
            ceres::CostFunction* cost_function = SimpleReprojectionError::Create(p2d.x, p2d.y);

            problem.AddResidualBlock(cost_function,
                    NULL ,
                    cameraPoses6d[kv.first].val,
                    points3d[i].val,
                                        &focal);
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

    if(not (summary.termination_type == ceres::CONVERGENCE)) {
        std::cerr << "Bundle adjustment failed." << std::endl;
        return;
    }

    std::cout << "Current K=\n" << intrinsics.K << std::endl;
    //update optimized focal
    intrinsics.K.at<double>(0, 0) = (double)focal;
    intrinsics.K.at<double>(1, 1) = (double)focal;
    std::cout << "New Optimized K=\n" << intrinsics.K << std::endl;

    //Implement the optimized camera poses and 3D points back into the reconstruction
    for(size_t i = 0; i < cameraPoses.size(); i++) {
        cv::Matx34d& pose = cameraPoses[i];
        cv::Matx34d poseBefore = pose;

        if(pose(0, 0) == 0 and pose(1, 1) == 0 and pose(2, 2) == 0) {
            //This camera pose is empty, it was not used in the optimization
            continue;
        }

        //Convert optimized Angle-Axis back to rotation matrix
        double rotationMat[9] = { 0 };
        Eigen::Matrix3f R;
        ceres::AngleAxisToRotationMatrix(cameraPoses6d[i].val,rotationMat);

        for(int r = 0; r < 3; r++){
            for(int c = 0; c < 3; c++){
                pose(c, r) = rotationMat[r * 3 + c]; //`rotationMat` is col-major...
            }
        }

/*
        //Rotation
        pose(0,0)=rotationMat[0]; pose(0,1)=rotationMat[3]; pose(0,2)=rotationMat[6];
        pose(1,0)=rotationMat[1]; pose(1,1)=rotationMat[4]; pose(1,2)=rotationMat[7];
        pose(2,0)=rotationMat[2]; pose(2,1)=rotationMat[5]; pose(2,2)=rotationMat[8];
*/
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





