#include "../include/Sfm.h"

/********************************************
 PIPELINE
********************************************/

std::vector<cv::Point3d> StructFromMotion::recon(cv::Mat& img1, cv::Mat& img2){

  // **(1) FEATURE EXTRACTION
  std::vector<cv::KeyPoint> keypoints1 = StructFromMotion::obtenerKeypoints(img1);
  std::vector<cv::KeyPoint> keypoints2 = StructFromMotion::obtenerKeypoints(img2);

  // **(2) MATCHES
  std::vector<cv::DMatch> good_matches = StructFromMotion::obtenerMatches(img1,img2,keypoints1,keypoints2);
  cv::Mat matchImage = StructFromMotion::imageMatching(img1,keypoints1,img2, keypoints2,good_matches);
  StructFromMotion::matchingImShow(matchImage);

  // **(3) KEYPOINTS --> only pt.x and pt.y [x,y] without angle, point size, etc...
  std::vector<cv::Point2f> points1 = StructFromMotion::keypoints2F(keypoints1,good_matches);
  std::vector<cv::Point2f> points2 = StructFromMotion::keypoints2F(keypoints2,good_matches);

  // **(4) CAMERA MATRIX
  cv::Mat_<double> matrixK = StructFromMotion::getCameraMatrix();
  double f = matrixK.at<double>(0,0);
  double cx= matrixK.at<double>(0,2);
  double cy = matrixK.at<double>(1,2);

  // **(5) ESSENTIAL MATRIX
  cv::Mat_<double> matrixE = StructFromMotion::findEssentialMatrix(points1,points2,matrixK);

  // **(6) CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
  cv::Mat relativeRotationCam,relativeTranslaCam,inliers;
  StructFromMotion::cameraPose(points1,points2,f,cx,cy,relativeRotationCam,relativeTranslaCam,inliers,matrixE );

  // **(7) PROJECTION MATRIX
  cv::Mat_<double> projection1,projection2;
  StructFromMotion::projection(relativeRotationCam,relativeTranslaCam, projection1,projection2);

  // **(8) TRIANGULATION
  std::vector<cv::Point3d> cloud = StructFromMotion::triangulation(points1,points2,projection1,projection2,inliers);

  // **(9) POINTCLOUD VISUALIZER
  cv::Matx33d matrixCam = (cv::Matx33d)matrixK;
  StructFromMotion::visualizerPointCloud(matrixCam,img1,img2,relativeRotationCam,relativeTranslaCam,cloud);


}


/********************************************
 FUNCIONES
********************************************/

void StructFromMotion::setConstructor(cv::Mat& img1,cv::Mat& img2){

  GaussianBlur(img1,img1, cv::Size(7,7),1.5,1.5);
  GaussianBlur(img2,img2, cv::Size(7,7),1.5,1.5);
  /*
  this->image1 = img1;
  this->image2 = img2;
  cv::Mat matrixE23 = findEssentialMatrix();

  cv::Mat matrixTotal;
  matrixTotal =  this->matrixE12*this->matrixE23;
 // std::cout << "matrixE12 anterior: " << matrixE12 << std::endl;
  this->matrixE12 = matrixTotal;
  this->matrixETotal = matrixTotal;
 // std::cout << "matrixE12 actual: " << matrixE12 << std::endl;
 // std::cout << "matrixE23 " << matrixE23 << std::endl;
 */
}

std::vector<cv::Point2f> StructFromMotion::keypoints2F(std::vector<cv::KeyPoint>& keypoints,std::vector<cv::DMatch>& matches){

  std::vector<cv::Point2f> points2F;

for (std::vector<cv::DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it){
//for(int n=0;n<keypoints.size();n++){
      // Get the position of left keypoints
                   float x= keypoints[it->queryIdx].pt.x;
                   float y= keypoints[it->queryIdx].pt.y;
                   points2F.push_back(cv::Point2f(x,y));

            }
return points2F;
}

std::vector<cv::KeyPoint> StructFromMotion::obtenerKeypoints (cv::Mat& image){

  std::vector<cv::KeyPoint> keypoints;
  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);
  ptrFeature2D->detect(image,keypoints);

  return keypoints;
}

std::vector<cv::DMatch> StructFromMotion::obtenerMatches(cv::Mat& img1,cv::Mat& img2,std::vector<cv::KeyPoint>& keypoints1,std::vector<cv::KeyPoint>& keypoints2){

  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);

  keypoints1=obtenerKeypoints(img1);
  keypoints2=obtenerKeypoints(img2);

  cv::Mat descriptors1,descriptors2;
  ptrFeature2D->compute(img1,keypoints1,descriptors1);
  ptrFeature2D->compute(img2,keypoints2,descriptors2);

  cv::Ptr<cv::DescriptorMatcher> matcherFlan = cv::DescriptorMatcher::create("FlannBased");
  std::vector<cv::DMatch> matches12,matches21,buenosMatches;
  matcherFlan ->match(descriptors1,descriptors2,matches12);
  matcherFlan ->match(descriptors2,descriptors1,matches21);

  for (size_t i=0; i < matches12.size(); i++){
      cv::DMatch forward = matches12[i];
      cv::DMatch backward = matches21[forward.trainIdx];
      if(backward.trainIdx==forward.queryIdx){
          buenosMatches.push_back(forward);
      }
    } 

return buenosMatches;

}

cv::Mat StructFromMotion::imageMatching(cv::Mat& img1,std::vector<cv::KeyPoint>& keypoints1,cv::Mat& img2, std::vector<cv::KeyPoint>& keypoints2,std::vector<cv::DMatch>& matches){

  cv::Mat matchImage; 
  cv::drawMatches(img1,keypoints1,img2,keypoints2,matches,matchImage,
                  cv::Scalar::all(-1),cv::Scalar::all(-1),std::vector<char>(),2);

  return matchImage;
}

void StructFromMotion::visualizerPointCloud(cv::Matx33d& cameraMatrix,cv::Mat& img1,cv::Mat& img2,cv::Mat& cameraR,cv::Mat& cameraT,std::vector<cv::Point3d>& pointcloud){

  // Create a viz window
  cv::viz::Viz3d visualizer("Viz window");
  cv::viz::WCoordinateSystem ucs(100);

  //Create a virtual camera

  cv::viz::WCameraPosition cam1(cameraMatrix, img1, 150,cv::viz::Color::white());
  cam1.setRenderingProperty(cv::viz::LINE_WIDTH,2);
  cv::viz::WCameraPosition cam2(cameraMatrix, img2, 150,cv::viz::Color::white());
  cam2.setRenderingProperty(cv::viz::LINE_WIDTH,2);
  cv::viz::WCloud point3d(pointcloud, cv::viz::Color::green());
  point3d.setRenderingProperty(cv::viz::POINT_SIZE, 1.0);

  visualizer.setBackgroundColor(cv::viz::Color::black());
  visualizer.showWidget("Coordinate Widget", ucs);
  visualizer.showWidget("Point3D", point3d);
  visualizer.showWidget("Camera1", cam1);
  visualizer.showWidget("Camera2", cam2);

  cv::Affine3d pose(cameraR,cameraT);

  visualizer.setWidgetPose("Camera2", pose);
  visualizer.setWidgetPose("Point3D", pose);

  // visualization loop
  while(cv::waitKey(0) && !visualizer.wasStopped()){

    visualizer.spin();
  }
}

  void StructFromMotion::matchingImShow(cv::Mat& matchImage){

    cv::namedWindow("matches",CV_WINDOW_NORMAL);
    cv::resizeWindow("matches",800,400);
    cv::moveWindow("matches",0,0);
    cv::imshow("matches",matchImage);
    cv::waitKey(100);
  }

std::vector<cv::Point3d> StructFromMotion::triangulation(std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2,cv::Mat_<double>& projection1,cv::Mat_<double>& projection2,cv::Mat& inliers){

    std::vector<cv::Point2d> inlierPtsImage1, inlierPtsImage2;

    for (int i=0;i<inliers.rows;i++){

         inlierPtsImage1.push_back(cv::Point2d(points1[i].x,points1[i].y));
         inlierPtsImage2.push_back(cv::Point2d(points2[i].x,points2[i].y));
    }

    cv::Mat_<double> pointCloudOpenCV;
    std::vector<cv::Point3d> cloud;

    for(unsigned int n=0;n<inlierPtsImage1.size();n++){

    pointCloudOpenCV.push_back(StructFromMotion::LinearLSTriangulation4(inlierPtsImage1.at(n),projection1,
                                                            inlierPtsImage2.at(n),projection2));
  /*
    cv::Point3d u(inlierPtsImage1[n].x,inlierPtsImage1[n].y,1.0);
    cv::Mat_<double> matrixK = this->matrixK;
    cv::Mat_<double> invK = inverse(matrixK);

    cv::Mat_<double> um = invK * cv::Mat_<double>(u);

    u = um.at<cv::Point3d>(n);

    cv::Point3d u1(inlierPtsImage2[n].x,inlierPtsImage2[n].y,1.0);
    cv::Mat_<double> um1 = invK* cv::Mat_<double>(u1);
    u1 = um1.at<cv::Point3d>(n);
    //triangulate
    cv::Mat_<double> X = IterativeLinearLSTriangulation(u,this->projection1,u1,this->projection2);

  */
    cloud.push_back(cv::Point3d(inlierPtsImage1[n].x,inlierPtsImage1[n].y,cv::abs(pointCloudOpenCV(n))));
    }

    return cloud;
   }

void StructFromMotion::projection(const cv::Mat& relativeRotationCam,const cv::Mat& relativeTranslaCam, cv::Mat_<double>& projection1, cv::Mat_<double>& projection2){

      projection1 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));
      projection2 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));

      relativeRotationCam.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
      relativeTranslaCam.copyTo(projection2.colRange(3, 4));

      //projection2 --> crea una matríz homogénea 3x4 [R|t]      

      cv::Mat diag(cv::Mat::eye(3, 3, CV_64F)); // ---> Crea una matriz identidad
      diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));

      std::cout << "projection1 ciclo1: " << projection1 << std::endl;
      std::cout << "projection2 ciclo1: " << projection2 << std::endl;

  }

  void StructFromMotion::cameraPose(std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2,double& fx,double cx,double cy,cv::Mat& rot,cv::Mat& tra,cv::Mat& inliers,cv::Mat_<double>& essentialMatrix ){

     cv::recoverPose(essentialMatrix,points1, points2,
                               rot,tra,fx,cv::Point2d(cx,cy),inliers);
    }

cv::Mat StructFromMotion::inverse(cv::Mat& matrix){

    Eigen::MatrixXd invMatrix,invMatrixTranspose;
    Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,
                                    Eigen::Dynamic,
                                    Eigen::RowMajor> > eigenMatrix((double *)matrix.data,3,3);

    invMatrix = eigenMatrix.inverse();
    invMatrixTranspose = invMatrix.transpose();
    // create an OpenCV Mat header for the Eigen data:
    cv::Mat inv(invMatrixTranspose.rows(),
                                         invMatrixTranspose.cols(),
                                         CV_64FC1,invMatrixTranspose.data());

    return inv;
  }

cv::Mat_<double> StructFromMotion::findEssentialMatrix(std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2,cv::Mat_<double>& cameraMatrix){

     cv::Mat inli;
     cv::Mat_<double> matrixFundamental = cv::findFundamentalMat(points1,points2,cv::FM_RANSAC,0.1,0.99,inli);
     cv::Mat_<double> matrixE = cameraMatrix.t()*matrixFundamental*cameraMatrix;     

     return matrixE;

 }

     cv::Mat_<double> StructFromMotion::LinearLSTriangulation(cv::Point3d u,cv::Matx34d P,cv::Point3d u1,cv::Matx34d P1){
  //build A matrix
  cv::Matx43d A(u.x*P(2,0)-P(0,0),
                u.x*P(2,1)-P(0,1),
                u.x*P(2,2)-P(0,2),
                u.y*P(2,0)-P(1,0),
                u.y*P(2,1)-P(1,1),
                u.y*P(2,2)-P(1,2),
                u1.x*P1(2,0)-P1(0,0),
                u1.x*P1(2,1)-P1(0,1),
                u1.x*P1(2,2)-P1(0,2),
                u1.y*P1(2,0)-P1(1,0),
                u1.y*P1(2,1)-P1(1,1),
                u1.y*P1(2,2)-P1(1,2));

  //build B vector
  cv::Matx41d B(-(u.x*P(2,3)-P(0,3)),
                -(u.y*P(2,3)-P(1,3)),
                -(u1.x*P1(2,3)-P1(0,3)),
                -(u1.y*P1(2,3)-P1(1,3)));

  //solve for X
  cv::Mat_<double> X;
  cv::solve(A,B,X,cv::DECOMP_SVD);
  return X;
  }

  cv::Mat_<double> StructFromMotion::LinearLSTriangulation4(cv::Point2d u,cv::Matx34d P,cv::Point2d u1,cv::Matx34d P1){
  //build A matrix
  cv::Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
                u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
                u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
                u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2));

  //build B vector
  cv::Matx41d B(-(u.x*P(2,3)-P(0,3)),-(u.y*P(2,3)-P(1,3)),-(u1.x*P1(2,3)-P1(0,3)),-(u1.y*P1(2,3)-P1(1,3)));

  //solve for X
  cv::Mat_<double> X;
  cv::solve(A,B,X,cv::DECOMP_SVD);
  if(X.data<0){
      X*=-1;
    }else{
      X=X;
    }
  return X;
  }

  cv::Mat_<double> StructFromMotion::IterativeLinearLSTriangulation(cv::Point3d u,
                                              cv::Matx34d P,          //camera 1 matrix
                                              cv::Point3d u1,
                                              cv::Matx34d P1          //camera 2 matrix
                                              ) {
      double wi = 1, wi1 = 1;
      cv::Mat_<double> X(4,1);
      for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
          cv::Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
          X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;

          //recalculate weights
          double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
          double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);

          //breaking point
          if(fabsf(wi - p2x) <= 0.8 && fabsf(wi1 - p2x1) <= 0.8) break;

          wi = p2x;
          wi1 = p2x1;

          //reweight equations and solve
          cv::Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,
                    (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,
                    (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1,
                    (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                    );
          cv::Mat_<double> B = (cv::Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                            -(u.y*P(2,3)  -P(1,3))/wi,
                            -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                            -(u1.y*P1(2,3)    -P1(1,3))/wi1
                            );

          cv::solve(A,B,X_,cv::DECOMP_SVD);
          X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
      }
      return X;
  }

  void StructFromMotion::cameraPoseAcumulada(){

    cv::Mat rot,tra,inliers;
    /*
    cv::recoverPose(this->matrixETotal,points1, points2,
                               rot,tra,this->fx,cv::Point2d(this->cx,this->cy),inliers);

    this->relativeRotationCam = rot;
    this->relativeTranslaCam = tra;
*/
    }


  cv::Mat_<double> StructFromMotion::getCameraMatrix(){

    double fx = 800;
    double fy = 800;
    double cx = 400;
    double cy = 225;

    cv::Mat_<double> cameraMatrix = (cv::Mat_<double>(3,3) << fx,0,cx,
                                                     0,fy,cy,
                                                     0,0,1);
  /*
    // cv::Mat cameraMatrix;
     cv::Mat cameraDistCoeffs;
     cv::FileStorage fs("camera-calibration-data.xml", cv::FileStorage::READ);
    // fs["Camera_Matrix"] >> cameraMatrix;
     fs["Distortion_Coefficients"] >> cameraDistCoeffs;
     //cv::Matx33d cMatrix(cameraMatrix);
     std::vector<double> cMatrixCoef(cameraDistCoeffs);
     std::cout <<"Vector distortion coeff: "<< std::endl;
     std::cout << "[ ";
     for(size_t n=0;n<cMatrixCoef.size();n++){

         std::cout << cMatrixCoef.at(n)<< ",";
       }
     std::cout <<"]" << std::endl;
     */

    return cameraMatrix;

  }



