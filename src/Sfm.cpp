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
  Points2f keypoints1_2F = StructFromMotion::keypoints2F(keypoints1,good_matches);
  Points2f keypoints2_2F = StructFromMotion::keypoints2F(keypoints2,good_matches);

  Points2f leftPointsAligned;
  Points2f rightPointsAligned;

  // **(4.1) ALIGNED POINTS

  if(good_matches.size() <= 0) {

      //points already aligned...
      leftPointsAligned = keypoints1_2F;
      rightPointsAligned = keypoints2_2F;
    }else {

        StructFromMotion::AlignedPointsFromMatch(keypoints1,keypoints2,good_matches,
                                                leftPointsAligned,rightPointsAligned);
     }

  std::cout << "pts1 " << keypoints1.size() << " (orig pts " << leftPointsAligned.size() << ")" << std::endl;
  std::cout << "pts2 " << keypoints2.size() << " (orig pts " << rightPointsAligned.size() << ")" << std::endl;

  // **(5) CAMERA MATRIX
  cv::Mat_<double> matrixK = StructFromMotion::getCameraMatrix();
  double f = matrixK.at<double>(0,0);
  double cx= matrixK.at<double>(0,2);
  double cy = matrixK.at<double>(1,2);

  // **(6) ESSENTIAL MATRIX
  cv::Mat mask;
  cv::Mat_<double> matrixE = StructFromMotion::findEssentialMatrix(leftPointsAligned,
                                                                   rightPointsAligned,matrixK,mask);

  // **(7) CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
  cv::Mat_<double> relativeRotationCam,relativeTranslaCam,inliers;
  StructFromMotion::cameraPose(leftPointsAligned,rightPointsAligned,f,cx,cy,relativeRotationCam,relativeTranslaCam,
                               inliers,matrixE );

  // **(8) PROJECTION MATRIX
  cv::Mat_<double> projection1,projection2,projectionTemp;
  if (! StructFromMotion::CheckCoherentRotation(relativeRotationCam)) {
      std::cout << "resulting rotation is not coherent\n" << std::endl;
      StructFromMotion::projection(relativeRotationCam,relativeTranslaCam, projection1,projection2);
      projection2 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));

  }else{
  StructFromMotion::projection(relativeRotationCam,relativeTranslaCam, projection1,projection2);
  }

  // **(8.1) RANSAC THRESHOLD - INLIERS COUNT

  double minVal,maxVal;
  cv::minMaxIdx(leftPointsAligned,&minVal,&maxVal);

  cv::Mat matrixH(3,3,CV_32FC3);
  cv::Mat inliersMask;
  matrixH = cv::findHomography(leftPointsAligned,rightPointsAligned,cv::RANSAC,0.004 * maxVal,inliersMask);
  int numInliers = cv::countNonZero(inliersMask);
  std::cout << "numInliers betwen 2 views =" << numInliers<<std::endl;

  // **(9) IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)

  Points2f points1CC,points2CC;
  cv::undistortPoints(leftPointsAligned, points1CC, matrixK, cv::Mat());
  cv::undistortPoints(rightPointsAligned, points2CC, matrixK, cv::Mat());

  // **(9.1) HOMOGENEOUS COORDINATE VISUALIZER

  Points3d points1Homo,points2Homo;
  for(size_t n=0;n<points1CC.size();n++){

    points1Homo.push_back(cv::Point3d(points1CC[n].x,points1CC[n].y,1));
    points2Homo.push_back(cv::Point3d(points2CC[n].x,points2CC[n].y,1));

   }

  std::cout << "points1-pixels-coordinate:" << leftPointsAligned.at(0) << std::endl;
  std::cout << "points1-camera-coordinate:" << points1CC.at(0) << std::endl;
  std::cout << "points1-homogeneous-coordinate:" << points1Homo.at(0) << std::endl;

  // **(10) TRIANGULATION

  cv::Mat cloud(1,points1CC.size(),CV_32FC4);
  cv::triangulatePoints(projection1,projection2,points1CC,points2CC,cloud);

  // **(11) CONVERTION CAMERA COORDINATE TO WORLD COORDINATE

  Points3f  pointcloudWorld;
  cv::convertPointsFromHomogeneous(cloud.t(),pointcloudWorld);
  std::cout << "pointcloudWorld:"<<"\n" << pointcloudWorld.at(0) << std::endl;

  // **(12) POINTCLOUD 3D VECTOR

  Points3d pointcloud;
  for(unsigned int i=0; i<points1CC.size();i++) {

     pointcloud.push_back(pointcloudWorld[i]);
  }

  projectionTemp = projection2;

  // Get the MODEL INFO:
  // list with model 3D coordinates
  Points3d list_points3d_model = StructFromMotion::cloudPointsCoordinates(pointcloud);
  // list with descriptors of each 3D coordinate
  cv::Mat descriptors_model = descriptors2;




  // -- Step 1: Robust matching between model descriptors and scene descriptors
  std::vector<cv::DMatch> good_matches2;       // to obtain the model 3D points  in the scene
  std::vector<cv::KeyPoint> keypoints_scene;  // to obtain the 2D points of the scene

  std::getline(file, frame2);
  std::cout << "---------------------------" << std::endl;
  std::cout << frame2 << std::endl;

  img2= cv::imread(frame2,CV_LOAD_IMAGE_COLOR);

  // **(1) FEATURE EXTRACTION
  keypoints_scene = StructFromMotion::obtenerKeypoints(img2);


  // **(2) FEATURE DESCRIPTION
  cv::Mat descriptors3;
  descriptors3 = StructFromMotion::obtenerDescriptors(img2,keypoints_scene);


  // **(3) FEATURE MATCHING
  good_matches2 = StructFromMotion::obtenerMatches(descriptors_model,descriptors3);


  cv::Mat cameraDistCoeffs;
  cv::FileStorage fs("camera-calibration-data.xml", cv::FileStorage::READ);
  fs["Distortion_Coefficients"] >> cameraDistCoeffs;
  std::vector<double> cMatrixCoef(cameraDistCoeffs);

   std::vector<cv::Point2d> projected3D;
   cv::Mat rvec;
   cv::Rodrigues(relativeRotationCam,rvec);
   cv::projectPoints(pointcloud, rvec, relativeTranslaCam, matrixK, cameraDistCoeffs, projected3D);

   std::cout<<"projected3D" << projected3D << std::endl;

  // -- Step 2: Find out the 2D/3D correspondences
  std::vector<cv::Point3d> list_points3d_model_match; // container for the model 3D coordinates found in the scene
  std::vector<cv::Point2d> list_points2d_scene_match; // container for the model 2D coordinates found in the scene
  for(unsigned int match_index = 0; match_index < good_matches2.size(); ++match_index)
  {
      cv::Point3d point3d_model = list_points3d_model[ good_matches2[match_index].trainIdx ];   // 3D point from model
      cv::Point2f point2d_scene = keypoints_scene[ good_matches2[match_index].queryIdx ].pt;    // 2D point from the scene
      list_points3d_model_match.push_back(point3d_model);                                      // add 3D point
      list_points2d_scene_match.push_back(point2d_scene);                                      // add 2D point
  }

  list_points2d_scene_match=projected3D;

 cv::Mat Rot,Tra,inliers2;

 cv::solvePnPRansac(list_points3d_model_match,list_points2d_scene_match,matrixK,cMatrixCoef,Rot,Tra,false,100,0.004*maxVal,0.99,inliers2);

 std::cout<<"Rot" << Rot << std::endl;
  std::cout<<"Tra" << Tra << std::endl;


int x=1;



  if(x==1){

      // **(4) KEYPOINTS 2F --> only pt.x and pt.y [x,y] without angle, point size, etc...

      Points2f keypoints3_2F = StructFromMotion::keypoints2F(keypoints_scene,good_matches2);



      // **(4.1) ALIGNED POINTS



      if(good_matches2.size() <= 0) {

          //points already aligned...
          leftPointsAligned = keypoints2_2F;
          rightPointsAligned = keypoints3_2F;
        }else {

            StructFromMotion::AlignedPointsFromMatch(keypoints2,keypoints_scene,good_matches2,
                                                    leftPointsAligned,rightPointsAligned);
         }






      // **(8) PROJECTION MATRIX
      cv::Mat projection1,projection2;
      if (! StructFromMotion::CheckCoherentRotation(Rot)) {
          std::cout << "resulting rotation is not coherent\n" << std::endl;
          StructFromMotion::projection(Rot,Tra, projection1,projection2);
          projection2 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));

      }else{
      StructFromMotion::projection(Rot,Tra, projection1,projection2);
      projection1=projectionTemp;
      }

    }

       /*
      // **(8.1) RANSAC THRESHOLD - INLIERS COUNT

      double minVal,maxVal;
      cv::minMaxIdx(leftPointsAligned,&minVal,&maxVal);



      // **(9) IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)

      Points2f points2CC;
      cv::undistortPoints(rightPointsAligned, points2CC, matrixK, cv::Mat());

      // **(10) TRIANGULATION

      cv::Mat cloud(1,points1CC.size(),CV_32FC4);
      cv::triangulatePoints(projection1,projection2,points1CC,points2CC,cloud);

      // **(11) CONVERTION CAMERA COORDINATE TO WORLD COORDINATE

      Points3f  pointcloudWorld;
      cv::convertPointsFromHomogeneous(cloud.t(),pointcloudWorld);
      std::cout << "pointcloudWorld:"<<"\n" << pointcloudWorld.at(0) << std::endl;

      // **(12) POINTCLOUD 3D VECTOR


      for(unsigned int i=0; i<points1CC.size();i++) {

         pointcloud.push_back(pointcloudWorld[i]);
      }


    }

    */

  // **(12) POINTCLOUD VISUALIZER
  cv::Matx33d matrixCam = (cv::Matx33d)matrixK;
  StructFromMotion::visualizerPointCloud(matrixCam,img1,img2,relativeRotationCam,relativeTranslaCam,pointcloud);



  // cv::Mat tempImage2 = img2;
  // cv::Mat_<double> projectionTemp = projection2;
  // std::vector<cv::Point3d> pointcloud=cloud;
  // cv::Mat_<double> matrixE12 = matrixE;

 /*    tempImage2 = img2;
       projectionTemp = projection2;
       matrixE12 = matrixETotal;
       pointcloud=pointcloud;
 */



}


/********************************************
 FUNCTIONS
********************************************/

//===============================================
//CONSTRUCTOR
//===============================================

StructFromMotion::StructFromMotion(cv::Mat& img1,cv::Mat& img2):image1(img1),image2(img2){
    GaussianBlur(img1,img1, cv::Size(7,7),1.5,1.5);
    GaussianBlur(img2,img2, cv::Size(7,7),1.5,1.5);
}

//===============================================
//CARGA DE IMAGENES
//===============================================


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

//===============================================
//FEATURE DETECTION AND EXTRACTION
//===============================================

Keypoints StructFromMotion::obtenerKeypoints (cv::Mat& image){

  Keypoints keypoints;
  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);
  ptrFeature2D->detect(image,keypoints);

  return keypoints;
}

cv::Mat StructFromMotion::obtenerDescriptors (cv::Mat& image,Keypoints& keypoints){

  cv::Mat descriptor;
  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);
  ptrFeature2D->compute(image,keypoints,descriptor);

  return descriptor;
}

//===============================================
//FEATURE MATCHING
//===============================================

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

cv::Mat StructFromMotion::imageMatching(cv::Mat& img1,Keypoints& keypoints1,
                                        cv::Mat& img2, Keypoints& keypoints2,MatchesVector& matches){

  cv::Mat matchImage;
  cv::drawMatches(img1,keypoints1,img2,keypoints2,matches,matchImage,
                  cv::Scalar::all(-1),cv::Scalar::all(-1),std::vector<char>(),2);

  return matchImage;
}

void StructFromMotion::matchingImShow(cv::Mat& matchImage){

    cv::namedWindow("Matching",CV_WINDOW_NORMAL);
    cv::resizeWindow("Matching",800,400);
    cv::moveWindow("Matching",0,0);
    cv::imshow("Matching",matchImage);
    cv::waitKey(30);
  }

//===============================================
//CONVERTION KEYPOINTS TO POINTS2F
//===============================================

Points2f StructFromMotion::keypoints2F(Keypoints& keypoints,MatchesVector& matches){

  Points2f points2F;

  for (MatchesVector::const_iterator it= matches.begin();it!= matches.end(); ++it){

           float x= keypoints[it->queryIdx].pt.x;
           float y= keypoints[it->queryIdx].pt.y;
           points2F.push_back(cv::Point2f(x,y));
   }
   return points2F;
}

//===============================================
//CAMERA MATRIX
//===============================================

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

//===============================================
//ESSENTIAL MATRIX
//===============================================

cv::Mat_<double> StructFromMotion::findEssentialMatrix( Points2f& leftPoints,Points2f& rightPoints,
                                                        cv::Mat_<double>& cameraMatrix,cv::Mat& mask){

     cv::Mat_<double> matrixFundamental = cv::findFundamentalMat(leftPoints,rightPoints,
                                                                 cv::FM_RANSAC,0.1,0.99,mask);
     cv::Mat_<double> matrixE = cameraMatrix.t()*matrixFundamental*cameraMatrix;

     return matrixE;

}

//===============================================
//ROTATION AND TRASLATION MATRIX[R|t]
//===============================================

void StructFromMotion::cameraPose(Points2f& points1,Points2f& points2,double& fx,double cx,double cy,cv::Mat& rot,cv::Mat& tra,cv::Mat& inliers,cv::Mat_<double>& essentialMatrix ){

     cv::recoverPose(essentialMatrix,points1, points2,
                               rot,tra,fx,cv::Point2d(cx,cy),inliers);
}

//===============================================
//PROJECTION MATRIX
//===============================================

void StructFromMotion::projection(const cv::Mat& relativeRotationCam,const cv::Mat& relativeTranslaCam, cv::Mat& projection1, cv::Mat& projection2){

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

//----------------------------------------------
//FUNCTION CHECK ROTATION MATRIX (Must be det=1)
//----------------------------------------------

bool StructFromMotion::CheckCoherentRotation(cv::Mat& R){

  if(fabsf(StructFromMotion::determinante(R))-1.0 > 1e-07) {
  std::cout << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;

  return false;
  }
  return true;
}

//----------------------------------------------
//FUNCTION ALIGNED POINTS
//----------------------------------------------

void StructFromMotion::AlignedPointsFromMatch(Keypoints& left,Keypoints& right,MatchesVector& matches,
                                              Points2f& featuresLeftAligned,Points2f& featuresRightAligned){

    //align left and right point sets
    for(size_t i=0;i<matches.size();i++){

       //trainIdx is the "left" image
       featuresLeftAligned.push_back(left[matches[i].trainIdx].pt);
       //queryIdx is the "right" image
       featuresRightAligned.push_back(right[matches[i].queryIdx].pt);
     }
}


//===============================================
//FUNCTION CORRESPONDENCES 2D-3D
//===============================================

//===============================================
//POINTCLOUD VISUALIZER
//===============================================

void StructFromMotion::visualizerPointCloud(cv::Matx33d& cameraMatrix,cv::Mat& img1,cv::Mat& img2,cv::Mat& cameraR,cv::Mat& cameraT,Points3d& pointcloud){

  // Create a viz window
  cv::viz::Viz3d visualizer("Viz window");
  cv::viz::WCoordinateSystem ucs(0.3);

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
  //visualizer.showWidget("Camera1", cam1);
  //visualizer.showWidget("Camera2", cam2);

  cv::Affine3d pose(cameraR,cameraT);

  //visualizer.setWidgetPose("Camera2", pose);
  //visualizer.setWidgetPose("Point3D", pose);

  // visualization loop
  while(cv::waitKey(0) && !visualizer.wasStopped()){

    visualizer.spin();
}

 //===============================================
 //COLOR MAP TO POINTCLOUD VISUALIZER
 //===============================================

/*
  cv::viz::Viz3d myWindow("Point Cloud");

          int height = 480, width = 640;
          cv::Mat pCloud(height, width, CV_32FC3);

          float fx = 525.0f, // default
                    fy = 525.0f,
                    cx = 319.5f,
                    cy = 239.5f;

          cv::Mat colorImage;
          cv::Mat depth, depth_flt;

          colorImage = img1;
          depth = cv::imread("depth.png", -1);
          cv::imshow("rgb", colorImage);
          cv::imshow("depth", depth);

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
                                  pCloud.at<cv::Vec3f>(y, x) = cv::Vec3f(X, Y, Z);
                          }
                          else{
                                  //RGB-D Dataset
                                  pCloud.at<cv::Vec3f>(y, x) = cv::Vec3f(0.f, 0.f, 0.f);
                          }
                  }
          }

          cv::viz::WCloud wcloud(pointcloud, colorImage);
          myWindow.showWidget("CLOUD", wcloud);
          myWindow.spin();


*/






}

/*
    std::vector<cv::Point3d> inlierPtsImage1, inlierPtsImage2;
    for (int i=0;i<inliers.rows;i++){

         inlierPtsImage1.push_back(cv::Point3d(points1[i].x,points1[i].y,1));
         inlierPtsImage2.push_back(cv::Point3d(points2[i].x,points2[i].y,1));
    }    

*/

//----------------------------------------------
//INVERSE MATRIX-DETERMINANT FUNCTION EIGEN
//----------------------------------------------

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

Points3d StructFromMotion::cloudPointsCoordinates(const Points3d cloudpoint) {
	Points3d pointd3D_coordinates;
	for (unsigned int i=0; i<cloudpoint.size(); i++) {
		pointd3D_coordinates.push_back(cv::Point3d(cloudpoint[i].x,cloudpoint[i].y,cloudpoint[i].z));
	}
	return pointd3D_coordinates;
}










