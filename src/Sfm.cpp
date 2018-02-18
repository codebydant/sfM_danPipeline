#include "../include/Sfm.h"

/********************************************
 PIPELINE
********************************************/

void StructFromMotion::recon(std::ifstream& file){

  // **(0) LECTURA DE IMAGENES
  cv::Mat img1, img2;  

  //int numImages = StructFromMotion::sizeTxtFile(file);

  std::string frame1,frame2;
  std::getline(file, frame1);
  std::cout << "---------------------------" << std::endl;
  std::cout << frame1 << std::endl;
  std::getline(file, frame2);
  std::cout << "---------------------------" << std::endl;
  std::cout << frame2 << std::endl;

  img1= cv::imread(frame1,CV_LOAD_IMAGE_COLOR);
  img2 = cv::imread(frame2,CV_LOAD_IMAGE_COLOR);

  // **(1) FEATURE EXTRACTION
  Keypoints keypoints1 = StructFromMotion::obtenerKeypoints(img1);
  Keypoints keypoints2 = StructFromMotion::obtenerKeypoints(img2);

  // **(2) FEATURE DESCRIPTION
  cv::Mat descriptors1,descriptors2;
  descriptors1 = StructFromMotion::obtenerDescriptors(img1,keypoints1);
  descriptors2 = StructFromMotion::obtenerDescriptors(img2,keypoints2);

  // **(3) FEATURE MATCHING
  MatchesVector good_matches = StructFromMotion::obtenerMatches(descriptors1,descriptors2);
  cv::Mat matchImage = StructFromMotion::imageMatching(img1,keypoints1,img2, keypoints2,good_matches);
  StructFromMotion::matchingImShow(matchImage);

  // **(4) KEYPOINTS 2F --> only pt.x and pt.y [x,y] without angle, point size, etc...
  Points2f leftPoints2F = StructFromMotion::keypoints2F(keypoints1,good_matches);
  Points2f rightPoints2F = StructFromMotion::keypoints2F(keypoints2,good_matches);

  Points2f leftPointsAligned;
  Points2f rightPointsAligned;

  if(good_matches.size() <= 0) {
      //points already aligned...
      leftPointsAligned = leftPoints2F;
      rightPointsAligned = rightPoints2F;
    }else {

        // **(4.1) ALIGNED POINTS
        StructFromMotion::AlignedPointsFromMatch(keypoints1,keypoints2,good_matches,
                                                 leftPointsAligned,rightPointsAligned);
     }

  // **(5) CAMERA MATRIX
  cv::Mat_<double> matrixK = StructFromMotion::getCameraMatrix();
  double f = matrixK.at<double>(0,0);
  double cx= matrixK.at<double>(0,2);
  double cy = matrixK.at<double>(1,2);

  // **(6) ESSENTIAL MATRIX
  cv::Mat mask;
  cv::Mat_<double> matrixE = StructFromMotion::findEssentialMatrix(leftPointsAligned,rightPointsAligned,matrixK,mask);

  // **(7) CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
  cv::Mat_<double> relativeRotationCam,relativeTranslaCam,inliers;
  StructFromMotion::cameraPose(leftPointsAligned,rightPointsAligned,f,cx,cy,relativeRotationCam,relativeTranslaCam,inliers,matrixE );

  // **(8) PROJECTION MATRIX
  cv::Mat_<double> projection1,projection2;
  if (! StructFromMotion::CheckCoherentRotation(relativeRotationCam)) {
      std::cout << "resulting rotation is not coherent\n" << std::endl;
      StructFromMotion::projection(relativeRotationCam,relativeTranslaCam, projection1,projection2);
      projection2 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));

  }else{
  StructFromMotion::projection(relativeRotationCam,relativeTranslaCam, projection1,projection2);
  }

  // **(9) IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)

  cv::Mat points1Homo,points2Homo;
  cv::undistortPoints(leftPointsAligned, points1Homo, matrixK, cv::Mat());
  cv::undistortPoints(rightPointsAligned, points2Homo, matrixK, cv::Mat());

  // **(10) TRIANGULATION
  cv::Mat cloud;
  cv::triangulatePoints(projection1,projection2,points1Homo,points2Homo,cloud);

  std::cout << "triangulate points, depth [z]:" << cloud << std::endl;

  /*

  std::vector<cv::Point3d> cloudVector;
  cloudVector = cloud.reshape(1,1);

  std::cout << "cloudVector size:" << cloudVector.size() << std::endl;
  std::vector<cv::Point3d> pointcloud;
 for(size_t n=0;n<points1Homo.rows;n++){

   pointcloud.push_back(cv::Point3d(points1Homo[n].x,points2Homo[n].y,cloudVector.at(n)));

  }
  */

  // **(11) CONVERTION CAMERA COORDINATE TO WORLD COORDINATE
  cv::Mat pointcloudWorld;
  cv::convertPointsFromHomogeneous(cloud.t(),pointcloudWorld);

  // cv::Mat tempImage2 = img2;
  // cv::Mat_<double> projectionTemp = projection2;
  // std::vector<cv::Point3d> pointcloud=cloud;
  // cv::Mat_<double> matrixE12 = matrixE;

  // **(12) POINTCLOUD VISUALIZER
  cv::Matx33d matrixCam = (cv::Matx33d)matrixK;
  StructFromMotion::visualizerPointCloud(matrixCam,img1,img2,relativeRotationCam,relativeTranslaCam,pointcloudWorld);

 /*    tempImage2 = img2;
       projectionTemp = projection2;
       matrixE12 = matrixETotal;
       pointcloud=pointcloud;
 */



}


/********************************************
 FUNCIONES
********************************************/

StructFromMotion::StructFromMotion(cv::Mat& img1,cv::Mat& img2):image1(img1),image2(img2){
    GaussianBlur(img1,img1, cv::Size(7,7),1.5,1.5);
    GaussianBlur(img2,img2, cv::Size(7,7),1.5,1.5);
}

void StructFromMotion::setConstructor(cv::Mat& img1,cv::Mat& img2){

  GaussianBlur(img1,img1, cv::Size(7,7),1.5,1.5);
  GaussianBlur(img2,img2, cv::Size(7,7),1.5,1.5); 
}

bool StructFromMotion::CheckCoherentRotation(cv::Mat_<double>& R) {

  if(fabsf(StructFromMotion::determinante(R))-1.0 > 1e-07) {
  std::cout << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;

  return false;
}
  return true;
}

int StructFromMotion::sizeTxtFile( std::ifstream& file){

  if (!file.is_open()) {
         std::cout << "There was a problem opening the file." << std::endl;
     }

  std::string cont;
  std::vector<std::string> textFile;

  while(file >> cont){

  std::string str;
  std::getline(file, str);
  textFile.push_back(str);

  }
  file.close();
  return textFile.size();

 }

Keypoints StructFromMotion::obtenerKeypoints (cv::Mat& image){

  Keypoints keypoints;
  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);
  ptrFeature2D->detect(image,keypoints);

  return keypoints;
}

Points2f StructFromMotion::keypoints2F(Keypoints& keypoints,MatchesVector& matches){

  Points2f points2F;

  for (MatchesVector::const_iterator it= matches.begin();it!= matches.end(); ++it){
  //for(int n=0;n<keypoints.size();n++){
  // Get the position of left keypoints
           float x= keypoints[it->queryIdx].pt.x;
           float y= keypoints[it->queryIdx].pt.y;
           points2F.push_back(cv::Point2f(x,y));

   }
   return points2F;
}

cv::Mat StructFromMotion::obtenerDescriptors (cv::Mat& image,Keypoints& keypoints){

  cv::Mat descriptor;
  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);
  ptrFeature2D->compute(image,keypoints,descriptor);

  return descriptor;
}

MatchesVector StructFromMotion::obtenerMatches(cv::Mat& descriptors1,cv::Mat& descriptors2){

  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);

  cv::Ptr<cv::DescriptorMatcher> matcherFlan = cv::DescriptorMatcher::create("FlannBased");
  MatchesVector matches12,matches21,buenosMatches;
  matcherFlan ->match(descriptors1,descriptors2,matches12);
  matcherFlan ->match(descriptors2,descriptors1,matches21);

  //CROSS-CHECK FILTER

  for (size_t i=0; i < matches12.size(); i++){
      cv::DMatch forward = matches12[i];
      cv::DMatch backward = matches21[forward.trainIdx];
      if(backward.trainIdx==forward.queryIdx){
          buenosMatches.push_back(forward);
      }
    }

return buenosMatches;

}

cv::Mat StructFromMotion::imageMatching(cv::Mat& img1,Keypoints& keypoints1,cv::Mat& img2, Keypoints& keypoints2,MatchesVector& matches){

  cv::Mat matchImage;
  cv::drawMatches(img1,keypoints1,img2,keypoints2,matches,matchImage,
                  cv::Scalar::all(-1),cv::Scalar::all(-1),std::vector<char>(),2);

  return matchImage;
}

void StructFromMotion::matchingImShow(cv::Mat& matchImage){

    cv::namedWindow("matches",CV_WINDOW_NORMAL);
    cv::resizeWindow("matches",800,400);
    cv::moveWindow("matches",0,0);
    cv::imshow("matches",matchImage);
    cv::waitKey(30);
  }

cv::Mat_<double> StructFromMotion::getCameraMatrix(){

    double fx = 1520.400000;
    double fy = 1525.900000;
    double cx = 302.320000;
    double cy = 246.870000;

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

void StructFromMotion::AlignedPointsFromMatch(Keypoints& left,
                            Keypoints& right, MatchesVector& matches, Points2f& featuresLeftAligned,Points2f& featuresRightAligned){

//align left and right point sets
for(size_t i=	0;i<matches.size();i++){
  //	queryIdx	is	the	"left"	image
  featuresLeftAligned.push_back(left[matches[i].queryIdx].pt);
  //trainIdx is the "right" image
  featuresRightAligned.push_back(right[matches[i].trainIdx].pt);
  }

}

cv::Mat_<double> StructFromMotion::findEssentialMatrix( Points2f& leftPoints,Points2f& rightPoints,cv::Mat_<double>& cameraMatrix,cv::Mat& mask){

     cv::Mat_<double> matrixFundamental = cv::findFundamentalMat(leftPoints,rightPoints,cv::FM_RANSAC,0.1,0.99,mask);
     cv::Mat_<double> matrixE = cameraMatrix.t()*matrixFundamental*cameraMatrix;

     return matrixE;

 }

void StructFromMotion::cameraPose(Points2f& points1,Points2f& points2,double& fx,double cx,double cy,cv::Mat& rot,cv::Mat& tra,cv::Mat& inliers,cv::Mat_<double>& essentialMatrix ){

     cv::recoverPose(essentialMatrix,points1, points2,
                               rot,tra,fx,cv::Point2d(cx,cy),inliers);
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

void StructFromMotion::visualizerPointCloud(cv::Matx33d& cameraMatrix,cv::Mat& img1,cv::Mat& img2,cv::Mat& cameraR,cv::Mat& cameraT,cv::Mat& pointcloud){

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
 // visualizer.showWidget("Camera1", cam1);
 // visualizer.showWidget("Camera2", cam2);

  cv::Affine3d pose(cameraR,cameraT);

//  visualizer.setWidgetPose("Camera2", pose);
 // visualizer.setWidgetPose("Point3D", pose);

  // visualization loop
  while(cv::waitKey(0) && !visualizer.wasStopped()){

    visualizer.spin();
  }
}

/*
std::vector<cv::Point3d> StructFromMotion::triangulation(std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2,cv::Mat_<double>& projection1,cv::Mat_<double>& projection2,cv::Mat& inliers,cv::Mat_<double>& cameraMatrix,std::vector<cv::po>& pointcloud){

    std::vector<cv::Point3d> inlierPtsImage1, inlierPtsImage2;


    for (int i=0;i<inliers.rows;i++){

         inlierPtsImage1.push_back(cv::Point3d(points1[i].x,points1[i].y,1));
         inlierPtsImage2.push_back(cv::Point3d(points2[i].x,points2[i].y,1));
    }

       cv::Mat_<double> X;
       std::vector<cv::Point3d> cloud;

       for(unsigned int n=0;n<inlierPtsImage1.size();n++){

       cv::Mat_<double> invK = inverse(cameraMatrix);
       cv::Mat_<double> um = invK * cv::Mat_<double>(inlierPtsImage1.at(n));
       inlierPtsImage1.at(n) = um.at<cv::Point3d>(n);

       cv::Mat_<double> um1 = invK* cv::Mat_<double>(inlierPtsImage2.at(n));
      inlierPtsImage2.at(n) = um1.at<cv::Point3d>(n);
       //triangulate
       cv::Mat_<double> X = LinearLSTriangulation(inlierPtsImage1.at(n),projection1,inlierPtsImage2.at(n),projection2);




           cv::triangulatePoints()

       X.push_back(StructFromMotion::LinearLSTriangulation(inlierPtsImage1.at(n),projection1,
                                                                   inlierPtsImage2.at(n),projection2));


       cloud.push_back(cv::Point3d(inlierPtsImage1[n].x,inlierPtsImage1[n].y,cv::abs(X(n))));
   }
       //calculate reprojection error
     //  cv::Mat_<double> xPt_img = cameraMatrix * projection2 * X;
    return cloud;
   }

    */

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

double StructFromMotion::determinante(cv::Mat& relativeRotationCam){


  Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,
                                  Eigen::Dynamic,
                                  Eigen::RowMajor> > eigenMatrix((double *)relativeRotationCam.data,3,3);

  Eigen::FullPivLU<Eigen::Matrix<double, Eigen::Dynamic,
                                         Eigen::Dynamic,
                                         Eigen::RowMajor>> eigenMatrixV2(eigenMatrix);

  double det = eigenMatrixV2.determinant();

  std::cout << "matrix: " << "\n" << relativeRotationCam << std::endl;
  std::cout << "matrix determinante: " << "\n"<< det << std::endl;

  return det;

}










