#include "../include/Sfm.h"

/********************************************
                  PIPELINE
********************************************/

int StructFromMotion::run_SFM(std::ifstream& file){

  // **(0) GET CAMERA MATRIX
  StructFromMotion::getCameraMatrix(matrixK);

  // **(0) IMAGES LOAD
  StructFromMotion::imagesLOAD(file);  
  nCameraPoses.resize(nImages.size());

  // **(1) FEATURE DETECTION AND EXTRACTION - ALL IMAGES
  StructFromMotion::extractFeatures();

  // **(2) FEATURE MATCHING
  StructFromMotion::matchFeatures();

  // **(3) BASE LINE TRIANGULATION
  bool success = StructFromMotion::baseTriangulation();

  StructFromMotion::addMoreViews();

  // **(4) ADD MORE VIEWS

  // **(5) VISUALIZER POINTCLOUD
  StructFromMotion::visualizerPointCloud(nReconstructionCloud);




}

/********************************************
 FUNCTIONS
********************************************/

//===============================================
//FUNCTION: IMAGES LOAD
//===============================================

void StructFromMotion::imagesLOAD(std::ifstream& file){

  std::cout << "Getting images..." << std::endl;

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
  std::cout << "IMAGES SET ==> OK "<< "\n"<< "Total images = "<< nImages.size() << std::endl;

}

//===============================================
//FUNCTION: OBTENER FEATURES
//===============================================

Features StructFromMotion::obtenerFeatures(const cv::Mat& image) {
    Features features;
    ptrFeature2D->detect(image,features.kps);
    ptrFeature2D->compute(image,features.kps,features.descriptors);
    keypoints2F(features.kps,features.pt2D);
    return features;
}

//===============================================
//FUNCTION: EXTRACT FEATURES
//===============================================

void StructFromMotion::extractFeatures(){

  std::cout << "Getting features from all images..." << std::endl;

  nFeaturesImages.resize(nImages.size());
  for(size_t n=0;n<nImages.size();n++){

      nFeaturesImages[n] =StructFromMotion::obtenerFeatures(nImages[n]);
   }

  std::cout << "FEATURES SET ==> OK" << std::endl;

}

//===============================================
//FUNCTION: KEYPOINTS TO POINTS2D
//===============================================

void StructFromMotion::keypoints2F(Keypoints& keypoints, Points2f& points2D){

  points2D.clear();
  for(const auto& kps: keypoints){

         points2D.push_back(kps.pt);
   }
}

//===============================================
//FUNCTION: FEATURE MATCHING
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
//FUNCTION: MATCHS FEATURES
//===============================================

void StructFromMotion::matchFeatures(){

  std::cout << "Getting matches from images pairs..." << std::endl;

  const size_t numImages = nImages.size();
  nFeaturesMatches.resize(numImages);

  for (size_t i = 0; i < numImages - 1; i++) {

      nFeaturesMatches[i]=StructFromMotion::obtenerMatches(nFeaturesImages[i],
                                                           nFeaturesImages[i+1]);

      if (nFeaturesMatches[i].size() == 0){
        std::cout<< " --(!) No good matches found " << std::endl;
      }
  }

  std::cout << "MATCHES FEATURES ==> OK" << std::endl;
}

//===============================================
//FUNCTION: IMAGE MATCHING
//===============================================

cv::Mat StructFromMotion::imageMatching(const cv::Mat& img1,const Keypoints& keypoints1,
                                        const cv::Mat& img2,const Keypoints& keypoints2,const Matching& matches){

  cv::Mat matchImage;
  cv::drawMatches(img1,keypoints1,img2,keypoints2,matches,matchImage,
                  cv::Scalar::all(-1),cv::Scalar::all(-1),std::vector<char>(),2);

  return matchImage;
}

//===============================================
//FUNCTION: IMAGE MATCHING
//===============================================

void StructFromMotion::matchingImShow(cv::Mat& matchImage){

    cv::namedWindow("Matching",CV_WINDOW_NORMAL);
    cv::resizeWindow("Matching",800,400);
    cv::moveWindow("Matching",0,0);
    cv::imshow("Matching",matchImage);
    cv::waitKey(70);
  }

//===============================================
//FUNCTION: GET CAMERA MATRIX
//===============================================

void StructFromMotion::getCameraMatrix(CameraData& intrinsics){

    std::cout << "Getting camera matrix..." << std::endl;
    intrinsics.fx = 1520.400000;
    intrinsics.fy = 1525.900000;
    intrinsics.cx = 302.320000;
    intrinsics.cy = 246.870000;
    intrinsics.K = (cv::Mat_<double>(3,3) << intrinsics.fx,    0,           intrinsics.cx,
                                                  0,       intrinsics.fy,   intrinsics.cy,
                                                  0,           0 ,                 1);
    intrinsics.invK = StructFromMotion::inverse(intrinsics.K);

    std::cout << "Camera matrix ==> OK" << std::endl;

    // cv::Mat cameraMatrix;
     cv::Mat cameraDistCoeffs;
     cv::FileStorage fs("camera-calibration-data.xml", cv::FileStorage::READ);
    // fs["Camera_Matrix"] >> cameraMatrix;
     fs["Distortion_Coefficients"] >> cameraDistCoeffs;
     //cv::Matx33d cMatrix(cameraMatrix);
     std::vector<double> cMatrixCoef(cameraDistCoeffs);
     std::cout <<"Vector distortion coeff: "<< std::endl;
     std::cout << "[ ";
     intrinsics.distCoef = cameraDistCoeffs;
     for(size_t n=0;n<cMatrixCoef.size();n++){

         std::cout << cMatrixCoef.at(n)<< ",";
       }
     std::cout <<"]" << std::endl;
    /* */


}

//===============================================
//FUNCTION: CHECK ROTATION MATRIX (Must be det=1)
//===============================================

bool StructFromMotion::CheckCoherentRotation(cv::Mat& R){

   if(fabsf(StructFromMotion::determinante(R))-1.0 > 1e-07) {

      std::cout << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
      return false;
    }
    return true;
}

//===============================================
//FUNCTION: ALIGNED POINTS
//===============================================

void StructFromMotion::AlignedPointsFromMatch(const Features& left,const Features& right,const Matching& matches,Features& alignedL,Features& alignedR){

   std::vector<int> leftId,rightId;
   StructFromMotion::AlignedPoints(left,right,matches,alignedL,alignedR,leftId,rightId);

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
//FUNCTION: POINTCLOUD VISUALIZER
//===============================================

void StructFromMotion::visualizerPointCloud(const std::vector<Point3D>& pointcloud){

  // Create a viz window
  cv::viz::Viz3d visualizer("Viz window");
  cv::viz::WCoordinateSystem ucs(1);

  Points3f cloud;
  for(size_t i=0;i<pointcloud.size();i++){
      cloud.push_back(pointcloud[i].pt);

    }

  cv::viz::WCloud point3d(cloud, cv::viz::Color::green());
  point3d.setRenderingProperty(cv::viz::POINT_SIZE, 1.0);

  visualizer.setBackgroundColor(cv::viz::Color::black());
  visualizer.showWidget("Coordinate Widget", ucs);
  visualizer.showWidget("Point3D", point3d);

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
//FUNCTION: BEST TWO VIEWS
//==============================================

std::map<float,ImagePair> StructFromMotion::best2Views(){

  int totalInliers;
  Matching matches;
  std::map<float,ImagePair> matchesWeigths;
  const size_t numImages = nImages.size();

  std::cout << "Getting best two views for baseline reconstruction..." << std::endl;

  for(size_t i=0;i<numImages-1;i++){

      matches = StructFromMotion::obtenerMatches(nFeaturesImages[0],nFeaturesImages[i+1]);

      if(matches.size() < 30){

          matchesWeigths[1.0]={i,i+1};
       }

       totalInliers = StructFromMotion::findHomographyInliers(nFeaturesImages[0],nFeaturesImages[i+1],matches);
       float weight = (float)totalInliers/(float)matches.size();
       matchesWeigths[weight] = {0,i+1};
  }

  std::cout << "BEST TWO VIEWS ==> OK" << std::endl;

  /*
  for(std::map<float,ImagePair>::iterator it=matchesWeigths.begin(); it!=matchesWeigths.end(); ++it){
        std::cout << "weight=" << it->first <<":" <<it->second.left <<"," << it->second.right << std::endl;
  }
  */
  return matchesWeigths;
}

//===============================================
//FUNCTION: FIND HOMOGRAPHY INLIERS
//===============================================

int StructFromMotion::findHomographyInliers(const Features& f1,const Features& f2,const Matching& matches){

  Features alignedLeft,alignedRight;
  StructFromMotion::AlignedPointsFromMatch(f1,f2,matches,alignedLeft,alignedRight);

  double minVal,maxVal;
  cv::minMaxIdx(alignedLeft.pt2D,&minVal,&maxVal);

  cv::Mat matrixH(3,3,CV_32FC3);
  cv::Mat inliersMask;
  int numInliers;
  if(matches.size()>=4){

     matrixH = cv::findHomography(alignedLeft.pt2D,alignedRight.pt2D,cv::RANSAC,0.004 * maxVal,inliersMask);
     numInliers = cv::countNonZero(inliersMask);
  }

  if(matches.size()< 4 or matrixH.empty()){

      numInliers = 0;
  }

  return numInliers;
}

//===============================================
//FUNCTION: BASE TRIANGULATION
//===============================================

bool StructFromMotion::baseTriangulation(){

  if (matrixK.K.empty()) {
         std::cerr << "Intrinsics matrix (K) must be initialized." << std::endl;
         return false;
  }

  bool success = true;
  std::map<float,ImagePair> bestPairs = StructFromMotion::best2Views();

  for (auto& imagePair : bestPairs) {

    size_t i = imagePair.second.left;
    size_t j = imagePair.second.right;

    Features alignedLeft;
    Features alignedRight;

    Matching matchingImagePair = StructFromMotion::obtenerMatches(nFeaturesImages[i],nFeaturesImages[j]);
    StructFromMotion::AlignedPointsFromMatch(nFeaturesImages[i],nFeaturesImages[j],matchingImagePair,
                                           alignedLeft,alignedRight);

    // ESSENTIAL MATRIX
    cv::Mat mask;
    cv::Mat E=cv::findEssentialMat(alignedLeft.pt2D, alignedRight.pt2D, matrixK.K, cv::RANSAC, 0.999, 1.0, mask);

    Matching bestMatches;

    bestMatches.clear();
    for (size_t i = 0; i < mask.rows; i++) {
           if (mask.at<uchar>(i)) {
                   bestMatches.push_back(matchingImagePair[i]);
           }
    }

    std::cout << "Showing matches between "<< "image:" << i << " and image:"<< j << std::endl;

   cv::Mat outImg= StructFromMotion::imageMatching(nImages[i],nFeaturesImages[i].kps,
                                           nImages[j],nFeaturesImages[j].kps,bestMatches);

   StructFromMotion::matchingImShow(outImg);

   float poseInliersRatio = (float)bestMatches.size()/(float)matchingImagePair.size();
   const float POSE_INLIERS_MINIMAL_RATIO = 0.5;
   if (poseInliersRatio < POSE_INLIERS_MINIMAL_RATIO) {

     continue;
   }

   // CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
   cv::Mat R,T;
   cv::recoverPose(E,alignedLeft.pt2D, alignedRight.pt2D,R,T,matrixK.fx,
                   cv::Point2d(matrixK.cx,matrixK.cy),mask);

   cv::Matx34f Pleft  = cv::Matx34f::eye();
   cv::Matx34f Pright = cv::Matx34f::eye();

   Pright = cv::Matx34f(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), T.at<double>(0),
                        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), T.at<double>(1),
                        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), T.at<double>(2));

   std::vector<Point3D> pointcloud;

   std::cout << "best pair:" << imagePair.second.left << "," << imagePair.second.right << std::endl;
   success = StructFromMotion::map2D3D(nFeaturesImages[i],nFeaturesImages[j],Pleft,Pright,bestMatches,matrixK,
                             imagePair.second,pointcloud);

   std::cout << "Base pointcloud ==> OK" << std::endl;

   nReconstructionCloud = pointcloud;
   nCameraPoses[i] = Pleft;
   nCameraPoses[j] = Pright;
   nDoneViews.insert(i);
   nDoneViews.insert(j);
   nGoodViews.insert(i);
   nGoodViews.insert(j);

   //adjustCurrentBundle();

   break;

   }
  cv::destroyAllWindows();
  return success;
}

//===============================================
//FUNCTION: MAPPING 2D TO 3D
//===============================================

bool StructFromMotion::map2D3D(const Features& left,const Features& right,const cv::Matx34f& P1,const cv::Matx34f& P2,const Matching& matches,const CameraData& matrixK,const ImagePair imagePair,std::vector<Point3D>& pointcloud){

  if(matrixK.K.empty() or matches.empty()){
      std::cerr << "It could not map 2D points-3D points, K is empty or matches vector is not valid" << std::endl;
      return false;
    }

  std::vector<int> leftBackReference;
  std::vector<int> rightBackReference;
  Features alignedLeft;
  Features alignedRight;
  StructFromMotion::AlignedPoints(left,right,matches,alignedLeft,alignedRight,leftBackReference,rightBackReference);

  // NORMALIZE IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)
  std::cout << "Normalizing points..." << std::endl;
  cv::Mat normalizedLeftPts,normalizedRightPts;
  cv::undistortPoints(alignedLeft.pt2D, normalizedLeftPts, matrixK.K, cv::Mat());
  cv::undistortPoints(alignedRight.pt2D, normalizedRightPts, matrixK.K, cv::Mat());
  std::cout << "POINTS NORMALIZED ==> OK" << std::endl;

  std::cout << "Triangulating points from --> " << "image: " << imagePair.left << " and image: "
            << imagePair.right << std::endl;

  // TRIANGULATE POINTS
  cv::Mat pts3dHomogeneous;
  cv::triangulatePoints(P1,P2,normalizedLeftPts,normalizedRightPts,pts3dHomogeneous);
  std::cout << "TRIANGULATE POINTS ==> OK" << std::endl;

  // CONVERTION CAMERA COORDINATE - WORLD COORDINATE
  std::cout << "Converting points to world coordinate..." << std::endl;
  cv::Mat pts3d;
  cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(),pts3d);
  std::cout << "POINTS WORLD COORDINATE ==> OK " << std::endl;

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

  std::cout << "Creating a pointcloud vector..." << std::endl;

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


  if(pointcloud.size() == 0){
      std::cerr << "ERROR: pointcloud size = " <<pointcloud.size()<< std::endl;
      return false;
    }

  std::cout << "POINTCLOUD VECTOR ==> OK" << std::endl;

  return true;
}

void StructFromMotion::addMoreViews(){

  while(nDoneViews.size() != 4) {

    std::cout << "Finding 2D-3D correspondences..." << std::endl;
    Images2D3DMatches matches2D3D = StructFromMotion::find2D3DMatches();
    std::cout << "BEST CORRESPONDENCES TO ADD ==> OK" << std::endl;

    size_t bestView;
    size_t bestNumMatches = 0;
    for(const auto& match2D3D : matches2D3D) {
         const size_t numMatches = match2D3D.second.pts2D.size();
         if (numMatches > bestNumMatches) {
             bestView       = match2D3D.first;
             bestNumMatches = numMatches;
         }
    }

    nDoneViews.insert(bestView);
    cv::Matx34f newCameraPose = cv::Matx34f::eye();

    StructFromMotion::findCameraPosePNP(matrixK,matches2D3D[bestView],newCameraPose);
    bool anyViewSuccess = false;
    nCameraPoses[bestView]=newCameraPose;

    for (const int goodView : nGoodViews) {
       //since match matrix is upper-triangular (non symmetric) - use lower index as left
       size_t leftViewIdx  = (goodView < bestView) ? goodView : bestView;
       size_t rightViewIdx = (goodView < bestView) ? bestView : goodView;

       //use the essential matrix recovery to prune the matches
       Matching matchingImagePair = StructFromMotion::obtenerMatches(nFeaturesImages[leftViewIdx],nFeaturesImages[rightViewIdx]);

       //triangulate the matching points
       std::vector<Point3D> pointcloud;
       bool success = StructFromMotion::map2D3D(nFeaturesImages[leftViewIdx],nFeaturesImages[rightViewIdx],nCameraPoses[leftViewIdx],nCameraPoses[rightViewIdx],matchingImagePair,matrixK, { leftViewIdx, rightViewIdx },pointcloud);

       if (success==true) {

           //add new points to the reconstruction
           StructFromMotion::mergeNewPointCloud(pointcloud);
           anyViewSuccess = true;
       } else {

         std::cerr << "Failed to triangulate " << leftViewIdx << " and " << rightViewIdx << std::endl;
      }
    }
nGoodViews.insert(bestView);
 }



}

Images2D3DMatches StructFromMotion::find2D3DMatches(){

  Images2D3DMatches matches;

  for (size_t viewIdx = 0; viewIdx < 4; viewIdx++) {
          if (nDoneViews.find(viewIdx) != nDoneViews.end()) {
              continue; //skip done views
          }

  Point3D2DMatch match2D3D;

  //scan all cloud 3D points
         for (const Point3D& cloudPoint : nReconstructionCloud) {
                 bool found2DPoint = false;

             //scan all originating views for that 3D point
             for (const auto& origViewAndPoint : cloudPoint.pt3D2D) {
                 //check for 2D-2D matching via the match matrix
                 const int originatingViewIndex        = origViewAndPoint.first;
                 const int originatingViewFeatureIndex = origViewAndPoint.second;

                 //match matrix is upper-triangular (not symmetric) so the left index must be the smaller one
                 const int leftViewIdx  = (originatingViewIndex < viewIdx) ? originatingViewIndex : viewIdx;
                 const int rightViewIdx = (originatingViewIndex < viewIdx) ? viewIdx : originatingViewIndex;

                 std::vector<cv::DMatch> newMatches = StructFromMotion::obtenerMatches(nFeaturesImages[leftViewIdx],nFeaturesImages[rightViewIdx]);

                 //scan all 2D-2D matches between originating view and new view
                 for (const cv::DMatch& m : newMatches) {
                     int matched2DPointInNewView = -1;
                     if (originatingViewIndex < viewIdx) { //originating view is 'left'
                         if (m.queryIdx == originatingViewFeatureIndex) {
                             matched2DPointInNewView = m.trainIdx;
                         }
                     } else {                              //originating view is 'right'
                         if (m.trainIdx == originatingViewFeatureIndex) {
                             matched2DPointInNewView = m.queryIdx;
                         }
                     }
                     if (matched2DPointInNewView >= 0) {
                         //This point is matched in the new view
                         const Features& newViewFeatures = nFeaturesImages[viewIdx];
                         match2D3D.pts2D.push_back(newViewFeatures.pt2D[matched2DPointInNewView]);
                         match2D3D.pts3D.push_back(cloudPoint.pt);
                         found2DPoint = true;
                         break;
                     }
                 }

                 if(found2DPoint){
                     break;
                }
           }
        }
        matches[viewIdx] = match2D3D;
        std::cout<<"found "<<match2D3D.pts3D.size() <<" 3d-2d point correspondences"<<std::endl;
    }

  return matches;

}

void StructFromMotion::findCameraPosePNP(const CameraData& matrixK,const Point3D2DMatch& match,
                                         cv::Matx34f& cameraPose){

  cv::Mat rvec, tvec;
  cv::Mat inliers;
  double RANSAC_THRESHOLD=10.0f;

  cv::solvePnPRansac(match.pts3D,match.pts2D,matrixK.K,matrixK.distCoef,rvec,tvec,
               false,100,RANSAC_THRESHOLD,0.99,inliers);

  cv::Mat rotMat;
  cv::Rodrigues(rvec, rotMat); //convert to a rotation matrix

std::cout << "Finding new camera pose..." << std::endl;
cameraPose = cv::Matx34f(rotMat.at<double>(0,0), rotMat.at<double>(0,1), rotMat.at<double>(0,2), tvec.at<double>(0),
                       rotMat.at<double>(1,0), rotMat.at<double>(1,1), rotMat.at<double>(1,2), tvec.at<double>(1),
                       rotMat.at<double>(2,0), rotMat.at<double>(2,1), rotMat.at<double>(2,2), tvec.at<double>(2));

 std::cout << "NEW CAMERA POSE ==> OK" << std::endl;

}

void StructFromMotion::mergeNewPointCloud(const std::vector<Point3D>& cloud){

  const size_t numImages = nImages.size();
  std::vector<Matching> mergeMatchMatrix;
  mergeMatchMatrix.resize(numImages);

  size_t newPoints = 0;
  size_t mergedPoints = 0;
  const float MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE = 0.01;
  const float MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE = 20.0;

  for (const Point3D& p : cloud) {
          const cv::Point3f newPoint = p.pt; //new 3D point

          bool foundAnyMatchingExistingViews = false;
          bool foundMatching3DPoint = false;

          for (Point3D& existingPoint : nReconstructionCloud) {
              if (cv::norm(existingPoint.pt - newPoint) < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE) {
                  //This point is very close to an existing 3D cloud point
                  foundMatching3DPoint = true;

                  //Look for common 2D features to confirm match
                  for (const auto& newKv : p.pt3D2D) {
                      //kv.first = new point's originating view
                      //kv.second = new point's view 2D feature index

                      for (const auto& existingKv : existingPoint.pt3D2D) {
                          //existingKv.first = existing point's originating view
                          //existingKv.second = existing point's view 2D feature index

                          bool foundMatchingFeature = false;

                          const bool newIsLeft = newKv.first < existingKv.first;
                          const int leftViewIdx         = (newIsLeft) ? newKv.first  : existingKv.first;
                          const int leftViewFeatureIdx  = (newIsLeft) ? newKv.second : existingKv.second;
                          const int rightViewIdx        = (newIsLeft) ? existingKv.first  : newKv.first;
                          const int rightViewFeatureIdx = (newIsLeft) ? existingKv.second : newKv.second;

                          const Matching& matching = StructFromMotion::obtenerMatches(nFeaturesImages[leftViewIdx],nFeaturesImages[rightViewIdx]);
                          for (const cv::DMatch& match : matching) {
                              if (match.queryIdx == leftViewFeatureIdx
                                  and match.trainIdx == rightViewFeatureIdx
                                  and match.distance < MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE) {

                                  mergeMatchMatrix[leftViewIdx].push_back(match);

                                  //Found a 2D feature match for the two 3D points - merge
                                  foundMatchingFeature = true;
                                  break;
                              }
                          }

                          if (foundMatchingFeature) {
                              //Add the new originating view, and feature index
                              existingPoint.pt3D2D[newKv.first] = newKv.second;

                              foundAnyMatchingExistingViews = true;

                          }
                      }
                  }



   }

  if (foundAnyMatchingExistingViews) {
                  mergedPoints++;
                  break; //Stop looking for more matching cloud points
              }
          }

          if (not foundAnyMatchingExistingViews and not foundMatching3DPoint) {
              //This point did not match any existing cloud points - add it as new.
              nReconstructionCloud.push_back(p);
              newPoints++;
          }
      }

}










