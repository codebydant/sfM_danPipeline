#include "../include/Sfm.h"


/********************************************
                  PIPELINE
********************************************/

void StructFromMotion::recon(std::ifstream& file){

  // **(0) IMAGES LOAD
  StructFromMotion::imagesLOAD(file,images,totalImages,imagesPath);
  std::cout << "Total images = " << totalImages << std::endl;

  // **(1) FEATURE DETECTION AND EXTRACTION - ALL IMAGES
  StructFromMotion::loadFeatures(images,totalImages,featuresImages,imagesPath);
 // std::cout << "train descriptors = " << featuresImages[0].descriptors << std::endl;

  // **(2) FEATURE MATCHING - ALL IMAGES
  StructFromMotion::loadMatches(featuresImages, matchesImages,totalImages);

  for(size_t n=0;n<1;n++){

    cv::Mat matchImage;
    matchImage =StructFromMotion::imageMatching(images.at(n),featuresImages[n].kps,
                                              images.at(n+1),featuresImages[n+1].kps,
                                              matchesImages[n].goodMatches);
    StructFromMotion::matchingImShow(matchImage);
    }
  std::cout << "si nene" << std::endl;

  // **(3) ALIGNED POINTS - FIRST PAIR IMAGES
  Features aleft,aright;
  StructFromMotion::AlignedPoints(featuresImages[0],featuresImages[1],matchesImages[0]);

  std::cout << "si nene2" << std::endl;

  // **(4) CAMERA MATRIX
  cv::Mat_<double> matrixK = StructFromMotion::getCameraMatrix();
  double f = matrixK.at<double>(0,0);
  double cx= matrixK.at<double>(0,2);
  double cy = matrixK.at<double>(1,2);   

  // **(5) ESSENTIAL MATRIX
  cv::Mat mask;
  cv::Mat_<double> matrixE=    StructFromMotion::findEssentialMatrix(featuresImagesBetter[0].pt2D,featuresImagesBetter[1].pt2D,matrixK,mask);

  // **(6) CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
  cv::Mat relativeRotationCam,relativeTranslaCam;
  StructFromMotion::cameraPose(featuresImagesBetter[0].pt2D,featuresImagesBetter[1].pt2D,f,cx,cy,
                               relativeRotationCam,relativeTranslaCam,mask,matrixE);

  // **(7) PROJECTION MATRIX 
  if (! StructFromMotion::CheckCoherentRotation(relativeRotationCam)) {
      std::cout << "resulting rotation is not coherent\n" << std::endl;
      StructFromMotion::projection(relativeRotationCam,
                                   relativeTranslaCam, projectionMatrices.at(0),projectionMatrices.at(1));
      projectionMatrices.at(1) = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));

  }else{
  StructFromMotion::projection(relativeRotationCam,relativeTranslaCam,
                               projectionMatrices.at(0),projectionMatrices.at(1));
  }

  // **(8.1) RANSAC THRESHOLD - INLIERS COUNT
  double minVal,maxVal;
  cv::minMaxIdx(featuresImages[0].pt2D,&minVal,&maxVal);

  cv::Mat matrixH(3,3,CV_32FC3);
  cv::Mat inliersMask;
  matrixH = cv::findHomography(featuresImagesBetter[0].pt2D,featuresImagesBetter[1].pt2D,cv::RANSAC,0.004 * maxVal,inliersMask);
  int numInliers = cv::countNonZero(inliersMask);
  std::cout << "numInliers betwen 2 views =" << numInliers<<std::endl;

  // **(9) IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)
  Points2f points1CC,points2CC;
  cv::undistortPoints(featuresImagesBetter[0].pt2D, points1CC, matrixK, cv::Mat());
  cv::undistortPoints(featuresImagesBetter[1].pt2D, points2CC, matrixK, cv::Mat());

  // **(10) TRIANGULATION
  cv::Mat cloud(1,points1CC.size(),CV_32FC4);
  cv::triangulatePoints(projectionMatrices.at(0),projectionMatrices.at(1),points1CC,points2CC,cloud);

  // **(11) CONVERTION CAMERA COORDINATE TO WORLD COORDINATE
  Points3f  pointcloudWorld;
  cv::convertPointsFromHomogeneous(cloud.t(),pointcloudWorld);
  std::cout << "pointcloudWorld:"<<"\n" << pointcloudWorld.at(0) << std::endl;

  // **(12) POINTCLOUD VISUALIZER
  cv::Matx33d matrixCam = (cv::Matx33d)matrixK;
  StructFromMotion::visualizerPointCloud(matrixCam,images[0],images[1],relativeRotationCam,
                                         relativeTranslaCam,pointcloudWorld);


}

/********************************************
 FUNCTIONS
********************************************/

//===============================================
//CARGA DE IMAGENES
//===============================================

void StructFromMotion::imagesLOAD(std::ifstream& file,std::vector<cv::Mat>& imageSet,
                                  int& numImages,std::vector<std::string>& textFile){

  imageSet.clear();
  if (!file.is_open()) {
         std::cout << "There was a problem opening the file." << std::endl;
     }

  std::string str;
  while(file >> str){

      cv::Mat img   = cv::imread(str,CV_LOAD_IMAGE_COLOR);
      imageSet.push_back(img);
      textFile.push_back(str);
    }
   numImages = imageSet.size();
}

//===============================================
//FEATURE DETECTION AND EXTRACTION
//===============================================

Features StructFromMotion::obtenerFeatures(cv::Mat& image,std::string& path) {
    Features features;
    ptrFeature2D->detect(image,features.kps);
    ptrFeature2D->compute(image,features.kps,features.descriptors);
    keypoints2F(features.kps,features.pt2D);
    features.imagePath = path;
    return features;
}

//===============================================
//FUNCTION LOAD FEATURES
//===============================================

void StructFromMotion::loadFeatures(std::vector<cv::Mat>& imagesList,int& totalImages, std::vector<Features>& ftsVector,std::vector<std::string>& imagePath){

  ftsVector.clear();

  for(size_t n=0;n<imagesList.size();n++){

      ftsVector.push_back(StructFromMotion::obtenerFeatures(imagesList.at(n),imagePath.at(n)));
   }

}

//===============================================
//CONVERTION KEYPOINTS TO POINTS2D
//===============================================

void StructFromMotion::keypoints2F(Keypoints& keypoints, Points2d& points2D){

  points2D.clear();
  for(auto kps: keypoints){

         points2D.push_back(kps.pt);
   }
}

//===============================================
//FEATURE MATCHING
//===============================================

/*
 *MÉTODO 1 (CROSS-CHECK FILTER)
 */

Matches StructFromMotion::obtenerMatches(cv::Mat& descriptors1,cv::Mat& descriptors2){

  Matches matching;
  matcherFlan ->match(descriptors1,descriptors2,matching.matches12);
  matcherFlan ->match(descriptors2,descriptors1,matching.matches21);

  //CROSS-CHECK FILTER
  for (size_t i=0; i < matching.matches12.size(); i++){
      cv::DMatch forward = matching.matches12[i];
      cv::DMatch backward = matching.matches21[forward.trainIdx];
      if(backward.trainIdx==forward.queryIdx){
          matching.goodMatches.push_back(forward);
      }
   }
 // std::cout << "matches" << matching.matches12 << std::endl;

/*
*MÉTODO 2 (RATIO-TEST)
*/

  /*
  //initial matching between features
  std::vector<MatchesVector> initialMatching;
  double NN_MATCH_RATIO = 0.8f;

  //auto matcher = cv::DescriptorMatcher::create(2);
  //matcher->knnMatch(descriptors1,descriptors2, initialMatching, 2);
  //matcherFlan ->knnMatch(descriptors1,descriptors2,initialMatching,2);

  //prune the matching using the ratio test

  for(unsigned i = 0; i < initialMatching.size(); i++) {
      if(initialMatching[i][0].distance <= NN_MATCH_RATIO * initialMatching[i][1].distance) {
          matching.goodMatches.push_back(initialMatching[i][0]);
      }
  }
  */
  return matching;
}

//===============================================
//FUNCTION LOAD MATCHES
//===============================================

void StructFromMotion::loadMatches(std::vector<Features>& ftsVec,std::vector<Matches>& matchesImages,int& numImages){

  matchesImages.clear();

  for(size_t n=0;n<numImages-1;n++){

      matchesImages.push_back(StructFromMotion::obtenerMatches(ftsVec[n].descriptors,ftsVec[n+1].descriptors));
    }

  if (matchesImages.size() == 0){
    std::cout<< " --(!) No good matches found " << std::endl;
  }


}

//===============================================
//FUNCTION IMAGE MATCHING
//===============================================

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
    cv::waitKey(0);
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
                                                              0, 0 ,1);
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

cv::Mat_<double> StructFromMotion::findEssentialMatrix(Points2d& leftPoints,Points2d& rightPoints,
                                                        cv::Mat_<double>& cameraMatrix,cv::Mat& mask){

     cv::Mat_<double> matrixFundamental = cv::findFundamentalMat(leftPoints,rightPoints,
                                                                 cv::FM_RANSAC,0.1,0.99,mask);
     cv::Mat_<double> matrixE = cameraMatrix.t()*matrixFundamental*cameraMatrix;

     return matrixE;

}

//===============================================
//ROTATION AND TRASLATION MATRIX[R|t] - CAMERA POSE
//===============================================

void StructFromMotion::cameraPose(Points2d& points1,Points2d& points2,double& fx,double cx,double cy,cv::Mat& rot,cv::Mat& tra,cv::Mat& mask,cv::Mat_<double>& essentialMatrix ){

     cv::recoverPose(essentialMatrix,points1, points2,
                               rot,tra,fx,cv::Point2d(cx,cy),mask);
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

//===============================================
//FUNCTION CHECK ROTATION MATRIX (Must be det=1)
//===============================================

bool StructFromMotion::CheckCoherentRotation(cv::Mat& R){

   if(fabsf(StructFromMotion::determinante(R))-1.0 > 1e-07) {

      std::cout << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
      return false;
    }
    return true;
}

//===============================================
//FUNCTION ALIGNED POINTS
//===============================================

void StructFromMotion::AlignedPoints( Features& left, Features& right, Matches& matches){


      //align left and right point sets
      for(unsigned int i=0;i<matches.goodMatches.size();i++){

        matches.leftReference.kps.push_back(left.kps[matches.goodMatches[i].queryIdx]);
        matches.leftReference.descriptors.push_back(left.descriptors.row(matches.goodMatches[i].queryIdx));
        matches.leftReference.pt2D.push_back(left.kps[matches.goodMatches[i].queryIdx].pt);
        matches.queryIdx.push_back(matches.goodMatches[i].queryIdx);

        matches.rightReference.kps.push_back(right.kps[matches.goodMatches[i].trainIdx]);
        matches.rightReference.descriptors.push_back(right.descriptors.row(matches.goodMatches[i].trainIdx));
        matches.rightReference.pt2D.push_back(right.kps[matches.goodMatches[i].trainIdx].pt);
        matches.trainIdx.push_back(matches.goodMatches[i].trainIdx);

      }

      std::cout << "matches[i].queryIdx " << matches.goodMatches[0].queryIdx <<
                   "matches[i].trainIdx " << matches.goodMatches[0].trainIdx << std::endl;

    std::cout << "original pts1 size=" << left.pt2D.size() <<
                 " (pts1 new size=" << matches.leftReference.pt2D.size() << ")" << std::endl;
    std::cout << "original pts2 size=" << right.pt2D.size() <<
                 " (pts2 new size=" << matches.rightReference.pt2D.size() << ")" << std::endl;


}

void StructFromMotion::AlignedPointsFromMatch( Features& left, Features& right,
                             Matches& matches,Features& leftAligned,Features& rightAligned){

  std::vector<int> leftReference,rightReference;
  std::cout << "yeah1" <<std::endl;


  std::cout << "yeah3" <<std::endl;
}

//===============================================
//FUNCTION CORRESPONDENCES 2D-3D
//===============================================

//===============================================
//POINTCLOUD VISUALIZER
//===============================================

void StructFromMotion::visualizerPointCloud(cv::Matx33d& cameraMatrix,cv::Mat& img1,cv::Mat& img2,cv::Mat& cameraR,cv::Mat& cameraT,Points3f& pointcloud){

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

          cv::viz::WCloud wcloud(pCloud, colorImage);
          myWindow.showWidget("CLOUD", wcloud);
          myWindow.spin();

*/

}

//==============================================
//INVERSE MATRIX-DETERMINANT FUNCTION EIGEN
//==============================================

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

//==============================================
//PROJECTION 3D POINTCLOUD TO 2D POINTS
//==============================================

Points3d StructFromMotion::cloudPointsCoordinates(const Points3d cloudpoint) {

	Points3d pointd3D_coordinates;
	for (unsigned int i=0; i<cloudpoint.size(); i++) {
		pointd3D_coordinates.push_back(cv::Point3d(cloudpoint[i].x,cloudpoint[i].y,cloudpoint[i].z));
	}
	return pointd3D_coordinates;
}










