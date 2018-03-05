#include "../include/Sfm.h"


/********************************************
                  PIPELINE
********************************************/

void StructFromMotion::recon(std::ifstream& file){

  // **(0) IMAGES LOAD
  StructFromMotion::imagesLOAD(file);
  std::cout << "Total images = " << nImages.size() << std::endl;
  /*
  for(size_t n=0;n<nImages.size();n++){

      StructFromMotion::matchingImShow(nImages[n]);


    }
*/
  // **(1) FEATURE DETECTION AND EXTRACTION - ALL IMAGES
  StructFromMotion::extractFeatures();

  StructFromMotion::matchFeatures();
/*
  for(size_t n=0;n<nFeaturesMatches.size()-1;n++){
      cv::Mat outImg= StructFromMotion::imageMatching(nImages[n],nFeaturesImages[n].kps,
                                              nImages[n+1],nFeaturesImages[n+1].kps,nFeaturesMatches[n]);

      StructFromMotion::matchingImShow(outImg);


    }
*/

 StructFromMotion::baseTriangulation();

  cv::Matx33d matrixCam = (cv::Matx33d)matrixK.K;
  StructFromMotion::visualizerPointCloud(nReconstructionCloud);


}

/********************************************
 FUNCTIONS
********************************************/

//===============================================
//CARGA DE IMAGENES
//===============================================

void StructFromMotion::imagesLOAD(std::ifstream& file){

  nImages.clear();
  if (!file.is_open()) {
         std::cout << "There was a problem opening the file." << std::endl;
     }

  std::string str;
  while(file >> str){

      cv::Mat img   = cv::imread(str,CV_LOAD_IMAGE_COLOR);
      cv::Mat temp = img.clone();
      cv::Mat resize;
      cv::resize(temp,resize,cv::Size(),0.75,0.75);
      cv::GaussianBlur(resize,temp, cv::Size(3,3),0,0);
      nImages.push_back(temp);
      nImagesPath.push_back(str);
    }

}

//===============================================
//FEATURE DETECTION AND EXTRACTION
//===============================================

Features StructFromMotion::obtenerFeatures(const cv::Mat& image) {
    Features features;
    ptrFeature2D->detect(image,features.kps);
    ptrFeature2D->compute(image,features.kps,features.descriptors);
    keypoints2F(features.kps,features.pt2D);
    return features;
}

//===============================================
//FUNCTION LOAD FEATURES
//===============================================

void StructFromMotion::extractFeatures(){

  nFeaturesImages.resize(nImages.size());
  for(size_t n=0;n<nImages.size();n++){

      nFeaturesImages[n] =StructFromMotion::obtenerFeatures(nImages[n]);
   }

}

//===============================================
//CONVERTION KEYPOINTS TO POINTS2D
//===============================================

void StructFromMotion::keypoints2F(Keypoints& keypoints, Points2f& points2D){

  points2D.clear();
  for(const auto& kps: keypoints){

         points2D.push_back(kps.pt);
   }
}

//===============================================
//FEATURE MATCHING
//===============================================

/*
 *MÉTODO 1 (CROSS-CHECK FILTER)
 */

Matching StructFromMotion::obtenerMatches(const Features& left,const Features& right){

  Matching matches12,matches21,goodMatches;

  matcherFlan ->match(left.descriptors,right.descriptors,matches12);
  matcherFlan ->match(right.descriptors,left.descriptors,matches21);

  //CROSS-CHECK FILTER
  for (size_t i=0; i < matches12.size(); i++){
      cv::DMatch forward = matches12[i];
      cv::DMatch backward = matches21[forward.trainIdx];
      if(backward.trainIdx==forward.queryIdx){
          goodMatches.push_back(forward);
      }
   }
 // std::cout << "matches" << matching.matches12 << std::endl;

/*
*MÉTODO 2 (RATIO-TEST)
*/
  /*

  //initial matching between features
  std::vector<Matching> initialMatching;
  matcherFlan ->knnMatch(left.descriptors,right.descriptors,initialMatching,2);

  //prune the matching using the ratio test

  for(unsigned i = 0; i < initialMatching.size(); i++) {
      if(initialMatching[i][0].distance <= NN_MATCH_RATIO * initialMatching[i][1].distance) {
          goodMatches.push_back(initialMatching[i][0]);
      }
  }
*/
  return goodMatches;
}

//===============================================
//FUNCTION LOAD MATCHES
//===============================================

void StructFromMotion::matchFeatures(){

  const size_t numImages = nImages.size();
  nFeaturesMatches.resize(numImages);

  for (size_t i = 0; i < numImages - 1; i++) {

      nFeaturesMatches[i]=StructFromMotion::obtenerMatches(nFeaturesImages[i],
                                                           nFeaturesImages[i+1]);

      if (nFeaturesMatches[i].size() == 0){
        std::cout<< " --(!) No good matches found " << std::endl;
      }
  }
}



//===============================================
//FUNCTION IMAGE MATCHING
//===============================================

cv::Mat StructFromMotion::imageMatching(const cv::Mat& img1,const Keypoints& keypoints1,
                                        const cv::Mat& img2,const Keypoints& keypoints2,const Matching& matches){

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

void StructFromMotion::getCameraMatrix(CameraData& intrinsics){

    intrinsics.fx = 1520.400000;
    intrinsics.fy = 1525.900000;
    intrinsics.cx = 302.320000;
    intrinsics.cy = 246.870000;
    intrinsics.K = (cv::Mat_<double>(3,3) << intrinsics.fx,    0,           intrinsics.cx,
                                                  0,       intrinsics.fy,   intrinsics.cy,
                                                  0,           0 ,                 1);
    intrinsics.invK = StructFromMotion::inverse(intrinsics.K);
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
}

//===============================================
//ESSENTIAL MATRIX
//===============================================

cv::Mat StructFromMotion::findEssentialMatrix(const Points2f& leftPoints,const Points2f& rightPoints,
                                                       const cv::Mat& cameraMatrix,cv::Mat& mask){

     cv::Mat matrixFundamental = cv::findFundamentalMat(leftPoints,rightPoints,
                                                                 cv::FM_RANSAC,0.1,0.99,mask);
     cv::Mat matrixE = cameraMatrix.t()*matrixFundamental*cameraMatrix;

     return matrixE;

}

//===============================================
//ROTATION AND TRASLATION MATRIX[R|t] - CAMERA POSE
//===============================================

void StructFromMotion::cameraPose(const Points2f& points1,const Points2f& points2,const double& fx,const double cx,const double cy,cv::Mat& rot,cv::Mat& tra,const cv::Mat& mask,const cv::Mat_<double>& essentialMatrix ){

     cv::recoverPose(essentialMatrix,points1, points2,
                               rot,tra,fx,cv::Point2d(cx,cy),mask);
}

//===============================================
//PROJECTION MATRIX
//===============================================

void StructFromMotion::projection(cv::Mat& relativeRotationCam,const cv::Mat& relativeTranslaCam, std::vector<cv::Mat_<double>>& matricesProVec,bool& status){

   if(!StructFromMotion::CheckCoherentRotation(relativeRotationCam)) {
      std::cout << "resulting rotation is not coherent\n" << std::endl;
      status = false;
      cv::Mat_<double> projection1 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));
      cv::Mat_<double> projection2 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));
      matricesProVec.push_back(projection1);
      matricesProVec.push_back(projection2);
   }else{

      status = true;
      cv::Mat_<double> projection1 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));
      cv::Mat_<double> projection2 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));

      relativeRotationCam.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
      relativeTranslaCam.copyTo(projection2.colRange(3, 4));

      //projection2 --> crea una matríz homogénea 3x4 [R|t]

      cv::Mat diag(cv::Mat::eye(3, 3, CV_64F)); // ---> Crea una matriz identidad
      diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));

      matricesProVec.push_back(projection1);
      matricesProVec.push_back(projection2);

      std::cout << "projection1 ciclo: " << projection1 << std::endl;
      std::cout << "projection2 ciclo: " << projection2 << std::endl;
   }
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

void StructFromMotion::AlignedPointsFromMatch(const Features& left,const Features& right,const Matching& matches,Features& alignedL,Features& alignedR){

   std::vector<int> leftId,rightId;
   StructFromMotion::AlignedPoints(left,right,matches,alignedL,alignedR,leftId,rightId);

   std::cout << "original pts1 size=" << left.pt2D.size() <<
                " (pts1 new size=" << alignedL.pt2D.size() << ")" << std::endl;
   std::cout << "original pts2 size=" << right.pt2D.size() <<
               " (pts2 new size=" << alignedR.pt2D.size() << ")" << std::endl;
}

void StructFromMotion::AlignedPoints(const Features& left,const Features& right,const Matching& matches, Features& alignedL, Features& alignedR,std::vector<int>& idLeftOrigen,std::vector<int>& idRightOrigen){

      //align left and right point sets
      for(unsigned int i=0;i<matches.size();i++){

        //alignedL.pt2D.push_back(left.kps[matches[i].queryIdx].pt);
        alignedL.kps.push_back(left.kps[matches[i].queryIdx]);
        alignedL.descriptors.push_back(left.descriptors.row(matches[i].queryIdx));

        //alignedR.pt2D.push_back(right.kps[matches[i].trainIdx].pt);
        alignedR.kps.push_back(right.kps[matches[i].trainIdx]);
        alignedR.descriptors.push_back(right.descriptors.row(matches[i].trainIdx));

        idLeftOrigen.push_back(matches[i].queryIdx);
        idRightOrigen.push_back(matches[i].trainIdx);

      }

      StructFromMotion::keypoints2F(alignedL.kps,alignedL.pt2D);
      StructFromMotion::keypoints2F(alignedR.kps,alignedR.pt2D);
}


//===============================================
//FUNCTION CORRESPONDENCES 2D-3D
//===============================================

//===============================================
//POINTCLOUD VISUALIZER
//===============================================

void StructFromMotion::visualizerPointCloud(const std::vector<Point3D>& pointcloud){

  // Create a viz window
  cv::viz::Viz3d visualizer("Viz window");
  cv::viz::WCoordinateSystem ucs(1);

  Points3f cloud;
  for(size_t i=0;i<pointcloud.size();i++){
      cloud.push_back(pointcloud[i].pt);

    }

  //Create a virtual camera
 // cv::viz::WCameraPosition cam1(cameraMatrix, img1, 150,cv::viz::Color::white());
 // cam1.setRenderingProperty(cv::viz::LINE_WIDTH,2);
 // cv::viz::WCameraPosition cam2(cameraMatrix, img2, 150,cv::viz::Color::white());
 // cam2.setRenderingProperty(cv::viz::LINE_WIDTH,2);
  cv::viz::WCloud point3d(cloud, cv::viz::Color::green());
  point3d.setRenderingProperty(cv::viz::POINT_SIZE, 1.0);

  visualizer.setBackgroundColor(cv::viz::Color::black());
  visualizer.showWidget("Coordinate Widget", ucs);
  visualizer.showWidget("Point3D", point3d);
  //visualizer.showWidget("Camera1", cam1);
  //visualizer.showWidget("Camera2", cam2);

 // cv::Affine3d pose(cameraR,cameraT);

  //visualizer.setWidgetPose("Camera2", pose);
  //visualizer.setWidgetPose("Point3D", pose);

  // visualization loop
  while(cv::waitKey(0) && !visualizer.wasStopped()){

    visualizer.spin();
  }
}

 //===============================================
 //COLOR MAP TO POINTCLOUD VISUALIZER
 //===============================================




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
  return det;

}

//==============================================
//PROJECTION 3D POINTCLOUD TO 2D POINTS
//==============================================

std::map<float,ImagePair> StructFromMotion::best2Views(){

  int totalInliers;
  Matching matches;
  std::map<float,ImagePair> matchesWeigths;
  const size_t numImages = nImages.size();

  for(size_t i=0;i<numImages-1;i++){

      matches = StructFromMotion::obtenerMatches(nFeaturesImages[0],nFeaturesImages[i+1]);
      std::cout << "matches size1=" << matches.size() << std::endl;
      if(matches.size() < 30){

          std::cout << "No hay suficientes matches" << std::endl;
          matchesWeigths[1.0]={i,i+1};
       }

          totalInliers = StructFromMotion::findHomographyInliers(nFeaturesImages[0],nFeaturesImages[i+1],matches);
          float weight = (float)totalInliers/(float)matches.size();
          matchesWeigths[weight] = {0,i+1};

  }
  /*
  for(std::map<float,ImagePair>::iterator it=matchesWeigths.begin(); it!=matchesWeigths.end(); ++it){
        std::cout << "weight=" << it->first <<":" <<it->second.left <<"," << it->second.right << std::endl;
  }
  */


  return matchesWeigths;

}



int StructFromMotion::findHomographyInliers(const Features& f1,const Features& f2,const Matching& matches){

  Features alignedLeft,alignedRight;
  StructFromMotion::AlignedPointsFromMatch(f1,f2,matches,alignedLeft,alignedRight);

  double minVal,maxVal;
  cv::minMaxIdx(alignedLeft.pt2D,&minVal,&maxVal);

  cv::Mat matrixH(3,3,CV_32FC3);
  cv::Mat inliersMask;
  int numInliers;
  if(matches.size()>=4){

     matrixH = cv::findHomography(alignedLeft.pt2D,alignedRight.pt2D,
                                  cv::RANSAC,0.004 * maxVal,inliersMask);
     numInliers = cv::countNonZero(inliersMask);
  }

  if(matches.size()< 4 or matrixH.empty()){

      numInliers = 0;
  }

  return numInliers;
}

void StructFromMotion::baseTriangulation(){

  std::map<float,ImagePair> bestPairs = StructFromMotion::best2Views();

  std::map<float,ImagePair>::iterator it=bestPairs.begin();
  ImagePair imagePair= it->second;

  std::cout << "best pair:" << imagePair.left << "," << imagePair.right << std::endl;

  StructFromMotion::getCameraMatrix(matrixK);

  for (auto& imagePair : bestPairs) {

  size_t i = imagePair.second.left;
  size_t j = imagePair.second.right;

  Features alignedLeft;
  Features alignedRight;

  Matching matchingImagePair = StructFromMotion::obtenerMatches(nFeaturesImages[i],nFeaturesImages[j]);
  StructFromMotion::AlignedPointsFromMatch(nFeaturesImages[i],nFeaturesImages[j],matchingImagePair,alignedLeft,alignedRight);

  // **(5) ESSENTIAL MATRIX
  double focal = matrixK.K.at<float>(0, 0); //Note: assuming fx = fy
  cv::Point2d pp(matrixK.K.at<float>(0, 2), matrixK.K.at<float>(1, 2));
  cv::Mat mask;
  cv::Mat E=cv::findEssentialMat(alignedLeft.pt2D, alignedRight.pt2D, matrixK.K, cv::RANSAC, 0.999, 1.0, mask);

  Matching bestMatches;

   bestMatches.clear();
   for (size_t i = 0; i < mask.rows; i++) {
           if (mask.at<uchar>(i)) {
                   bestMatches.push_back(matchingImagePair[i]);
           }
   }

   cv::Mat outImg= StructFromMotion::imageMatching(nImages[i],nFeaturesImages[i].kps,
                                           nImages[j],nFeaturesImages[j].kps,bestMatches);

   cv::imshow("good matches", outImg);
   cv::waitKey(0);


   float poseInliersRatio = (float)bestMatches.size() / (float)matchingImagePair.size();
   const float POSE_INLIERS_MINIMAL_RATIO = 0.5;
   if (poseInliersRatio < POSE_INLIERS_MINIMAL_RATIO) {

               continue;
   }

   //StructFromMotion::matchingImShow(outImg);

   // **(6) CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
   cv::Mat R,T;
   cv::recoverPose(E,alignedLeft.pt2D, alignedRight.pt2D,R,T,matrixK.fx,
                   cv::Point2d(matrixK.cx,matrixK.cy),mask);

   cv::Matx34f Pleft  = cv::Matx34f::eye();
   cv::Matx34f Pright = cv::Matx34f::eye();

   Pright = cv::Matx34f(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), T.at<double>(0),
                        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), T.at<double>(1),
                        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), T.at<double>(2));

   std::cout << "rot" << R << std::endl;
   std::cout << "tra" << T << std::endl;
   std::cout << "pro2" << Pright << std::endl;

   std::vector<Point3D> pointcloud;

   //  reconstructionPointCloud = pointcloud;
   StructFromMotion::map2D3D(nFeaturesImages[i],nFeaturesImages[j],Pleft,Pright,bestMatches,matrixK,imagePair.second,pointcloud);

    nReconstructionCloud = pointcloud;
    break;

    }

}


void StructFromMotion::map2D3D(const Features& left,const Features& right,const cv::Matx34f& P1,const cv::Matx34f& P2,const Matching& matches,const CameraData& matrixK,const ImagePair imagePair,std::vector<Point3D>& pointcloud){

  std::vector<int> leftBackReference;
  std::vector<int> rightBackReference;
  Features alignedLeft;
  Features alignedRight;
  StructFromMotion::AlignedPoints(left,right,matches,alignedLeft,alignedRight,leftBackReference,rightBackReference);

  // NORMALIZE IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)
  cv::Mat normalizedLeftPts,normalizedRightPts;
  cv::undistortPoints(alignedLeft.pt2D, normalizedLeftPts, matrixK.K, cv::Mat());
  cv::undistortPoints(alignedRight.pt2D, normalizedRightPts, matrixK.K, cv::Mat());

  // TRIANGULATE POINTS
  cv::Mat pts3dHomogeneous;
  cv::triangulatePoints(P1,P2,normalizedLeftPts,normalizedRightPts,pts3dHomogeneous);

  // CONVERTION CAMERA COORDINATE - WORLD COORDINATE
  cv::Mat pts3d;
  cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(),pts3d);
  std::cout << "points3d: " << pts3d << std::endl;




  cv::Mat rvecLeft;
  cv::Rodrigues(P1.get_minor<3,3>(0,0),rvecLeft);
  cv::Mat tvecLeft(P1.get_minor<3,1>(0,3).t());

  Points2f projectedLeft(alignedLeft.pt2D.size());
  cv::projectPoints(pts3d,rvecLeft,tvecLeft,matrixK.K,cv::Mat(),projectedLeft);

  cv::Mat rvecRight;
  cv::Rodrigues(P2.get_minor<3,3>(0,0),rvecRight);
  cv::Mat tvecRight(P2.get_minor<3,1>(0,3).t());

  Points2f projectedRight(alignedRight.pt2D.size());
  cv::projectPoints(pts3d,rvecRight,tvecRight,matrixK.K,cv::Mat(),projectedRight);

 /*

*/

  for (size_t i = 0; i < pts3d.rows; i++) {
          //check if point reprojection error is small enough

          if (cv::norm(projectedLeft[i]  - alignedLeft.pt2D[i])  > 10 or
              cv::norm(projectedRight[i] - alignedRight.pt2D[i]) > 10){
              continue;
          }

          Point3D p;
          p.pt = cv::Point3f(pts3d.at<float>(i, 0),
                             pts3d.at<float>(i, 1),
                             pts3d.at<float>(i, 2));

          //use back reference to point to original features in images
          p.pt3D2D[imagePair.left]  = leftBackReference[i];
          p.pt3D2D[imagePair.right] = rightBackReference[i];

          pointcloud.push_back(p);
  }


}










