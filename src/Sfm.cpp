//***********************************************
//HEADERS
//***********************************************

#include "../include/Sfm.h"
#include <ctime>


/********************************************
                  PIPELINE
********************************************/

int StructFromMotion::run_SFM(std::ifstream& file){

  std::cout << "************************************************" << std::endl;
  std::cout << "              3D RECONSTRUCTION                 " << std::endl;
  std::cout << "************************************************" << std::endl;

  // **(0) GET CAMERA MATRIX


  // **(0) IMAGES LOAD
  StructFromMotion::imagesLOAD(file);  
  nCameraPoses.resize(nImages.size());
   StructFromMotion::getCameraMatrix(matrixK);


  // **(1) FEATURE DETECTION AND EXTRACTION - ALL IMAGES
 StructFromMotion::extractFeatures();

  // **(2) FEATURE MATCHING
  StructFromMotion::matchFeatures();
/*
  for(size_t i=0;i<nImages.size()-1;i++){

  cv::Mat out;
  out = StructFromMotion::imageMatching(nImages[i],nFeaturesImages[i].kps,nImages[i+1],nFeaturesImages[i+1].kps,nFeatureMatchMatrix[i][i+1]);
  StructFromMotion::imShow(out,"matching");
}
*/
  // **(3) BASE LINE TRIANGULATION
  bool success = StructFromMotion::baseTriangulation();

  // **(4) ADD MORE VIEWS
  StructFromMotion::addMoreViews();

  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

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
  std::cout << "IMAGES SET ==> [DONE] "<< "\n"<< "Total images = "<< nImages.size() << std::endl;

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

  std::cout << "FEATURES SET ==> [DONE]" << std::endl;
  std::cout << "Total features = " << nFeaturesImages.size() << std::endl;

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

  /*
  *(RATIO-TEST)
  */
  Matching goodMatches;

  //initial matching between features
  std::vector<Matching> initialMatching;
  matcherFlan ->knnMatch(left.descriptors,right.descriptors,initialMatching,2);

  //prune the matching using the ratio test

  for(unsigned i = 0; i < initialMatching.size(); i++) {
      if(initialMatching[i][0].distance <= NN_MATCH_RATIO * initialMatching[i][1].distance) {
          goodMatches.push_back(initialMatching[i][0]);
      }
  }

  return goodMatches;
}

//===============================================
//FUNCTION: COMPARE
//===============================================
bool compare(const MatchesforSort& matche1, const MatchesforSort& matche2){

  if(matche1.size<matche2.size){
      return true;
    }else{
      return false;
    }
}

//===============================================
//FUNCTION: CREATE MATCH MATRIX FROM FEATURES
//===============================================

void StructFromMotion::matchFeatures(){

  std::clock_t begin = clock();
  std::cout << "Getting matches from images pairs..." << std::endl;
  nFeatureMatchMatrix.resize(nImages.size(),std::vector<Matching>(nImages.size()));

  MatchesforSort matchesSize;
  const size_t numImg = nImages.size();

  //prepare image pairs to match concurrently
  std::vector<ImagePair> pairs;
  for (size_t i = 0; i < numImg; i++) {
     for (size_t j = i + 1; j < numImg; j++) {
          pairs.push_back({ i, j });
     }
  }

      std::vector<std::thread> threads;

      //find out how many threads are supported, and how many pairs each thread will work on
  const int numThreads = std::thread::hardware_concurrency() - 1;
  const int numPairsForThread = (numThreads > pairs.size()) ? 1 : (int)ceilf((float)(pairs.size()) / numThreads);

  for (size_t threadId = 0; threadId < MIN(numThreads, pairs.size()); threadId++) {
      threads.push_back(std::thread([&, threadId] {
           const int startingPair = numPairsForThread * threadId;

           for (int j = 0; j < numPairsForThread; j++) {
               const int pairId = startingPair + j;
               if (pairId >= pairs.size()) { //make sure threads don't overflow the pairs
                      break;
               }
               const ImagePair& pair = pairs[pairId];

    nFeatureMatchMatrix[pair.left][pair.right]=StructFromMotion::obtenerMatches(nFeaturesImages[pair.left],
                                                                             nFeaturesImages[pair.right]);

           }
       }));
  }

  //wait for threads to complete
  for (auto& t : threads) {
       t.join();
  }

  for(size_t i=0;i<numImg-1;i++) {
       for(size_t j=i+1;j<numImg;j++){

           matchesSize.size = nFeatureMatchMatrix[i][j].size();
           matchesSize.i = i;
           matchesSize.j = j;

           nMatchesSorted.push_back(matchesSize);
       }
  }

  std::sort(nMatchesSorted.begin(),nMatchesSorted.end(),compare);
  std::cout << "MATCHES FEATURES ==> [DONE]" << std::endl;
  std::cout << "Total matches = " << nFeatureMatchMatrix.size()*nFeatureMatchMatrix.capacity()
            << std::endl;

  std::clock_t end = clock();
  double elapsed_secs = double(end - begin)/ 1000000000.0;


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
//FUNCTION: MATCHING IMAGE SHOW
//===============================================

void StructFromMotion::imShow(const cv::Mat& matchImage, const std::string& str){

    cv::namedWindow(str,CV_WINDOW_NORMAL);
    cv::resizeWindow(str,800,400);
    cv::moveWindow(str,0,0);
    cv::imshow(str,matchImage);
    cv::waitKey(0);
  }

//===============================================
//FUNCTION: GET CAMERA MATRIX
//===============================================

void StructFromMotion::getCameraMatrix(CameraData& intrinsics){

    std::cout << "Getting camera matrix..." << std::endl;
/*
    intrinsics.K=(cv::Mat_<double>(3,3) << 2500,   0, nImages[0].cols / 2,
                                               0, 2500, nImages[0].rows / 2,
    0, 0, 1);
    */
    intrinsics.fx = 1520.400000;
    intrinsics.fy = 1525.900000;
    intrinsics.cx = 302.320000;
    intrinsics.cy = 246.870000;
    intrinsics.K = (cv::Mat_<double>(3,3) << intrinsics.fx,    0,           intrinsics.cx,
                                                  0,       intrinsics.fy,   intrinsics.cy,
                                                  0,           0 ,                 1);

    //intrinsics.invK = StructFromMotion::inverse(intrinsics.K);



     cv::Mat_<double> cameraMatrix;
     cv::Mat cameraDistCoeffs;
     cv::FileStorage fs("camera-calibration-data.xml", cv::FileStorage::READ);
     fs["Camera_Matrix"] >> cameraMatrix;
     fs["Distortion_Coefficients"] >> cameraDistCoeffs;
     //cv::Matx33d cMatrix(cameraMatrix);
     std::vector<double> cMatrixCoef(cameraDistCoeffs);
     std::cout <<"Vector distortion coeff: "<< std::endl;
     std::cout << "[ ";
     intrinsics.distCoef = cameraDistCoeffs;
     //intrinsics.K = cameraMatrix;
     /*
     intrinsics.fx = intrinsics.K.at<double>(0,0);
     intrinsics.fy = intrinsics.K.at<double>(1,1);
     intrinsics.cx = intrinsics.K.at<double>(0,2);
     intrinsics.cy = intrinsics.K.at<double>(1,2);
*/
     std::cout << "CAMERA MATRIX ==> [DONE]" << std::endl;
     std::cout << "matrix K:" << "\n" << intrinsics.K << std::endl;

     for(size_t n=0;n<cMatrixCoef.size();n++){

         std::cout << cMatrixCoef.at(n) << ",";
       }
     std::cout <<"]" << std::endl;

}

//===============================================
//FUNCTION: CHECK ROTATION MATRIX (Must be det=1)
//===============================================

bool StructFromMotion::CheckCoherentRotation(cv::Mat& R){

   if(fabsf(StructFromMotion::determinante(R))-1.0 > 1e-07) {

      std::cout << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
      return false;
    }else {

       return true;
    }
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

        alignedL.kps.push_back(left.kps[matches[i].queryIdx]);
        alignedL.descriptors.push_back(left.descriptors.row(matches[i].queryIdx));

        alignedR.kps.push_back(right.kps[matches[i].trainIdx]);
        alignedR.descriptors.push_back(right.descriptors.row(matches[i].trainIdx));

        idLeftOrigen.push_back(matches[i].queryIdx);
        idRightOrigen.push_back(matches[i].trainIdx);
      }

      StructFromMotion::keypoints2F(alignedL.kps,alignedL.pt2D);
      StructFromMotion::keypoints2F(alignedR.kps,alignedR.pt2D);
}

//===============================================
//FUNCTION: POINTCLOUD VISUALIZER
//===============================================

void StructFromMotion::visualizerPointCloud(const std::vector<Point3D>& pointcloud){

  // Create a viz window
  cv::viz::Viz3d visualizer("Viz window");
  cv::viz::WCoordinateSystem ucs(0.5);

  Points3f cloud;
  for(size_t i=0;i<pointcloud.size();i++){
      cloud.push_back(pointcloud[i].pt);
  }

  cv::viz::WCloud point3d(cloud, cv::viz::Color::green());
  point3d.setRenderingProperty(cv::viz::POINT_SIZE, 1.0);

  std::vector<cv::Affine3d> path;
  for(size_t i=0;i<nCameraPoses.size();i++){
    cv::Mat rot = cv::Mat(nCameraPoses[i].get_minor<3,3>(0,0));
    cv::Mat tra = cv::Mat(nCameraPoses[i].get_minor<3,1>(0,3));
    if (rot.empty()) {
        //This camera pose is empty, it was not used in the optimization
        continue;
    }else{
         path.push_back(cv::Affine3f(rot,tra));
      }

  }

  //cv::viz::WCameraPosition view(cv::Matx33d(matrixK.K),0.2,cv::viz::Color::green());
  //cv::viz::Camera cam(matrixK.fx,matrixK.fy,matrixK.cx,matrixK.cy,cv::Size(3,3));
  //cv::viz::WGrid grid((100,100),2,cv::viz::Color::white());
 // cv::viz::WText3D text("3D RECONSTRUCTION",cv::Point3d(1,1,0),0.1,true,cv::viz::Color::white());

  cv::viz::WTrajectory trajectory(path,cv::viz::WTrajectory::BOTH,0.07,cv::viz::Color::yellow());
  cv::viz::WTrajectoryFrustums frustums(path, cv::Matx33d(matrixK.K), 0.2, cv::viz::Color::yellow());
  frustums.setRenderingProperty(cv::viz::LINE_WIDTH,2);

  visualizer.setBackgroundColor(cv::viz::Color::black());
  visualizer.showWidget("Coordinate Widget", ucs);
  visualizer.showWidget("Point3D", point3d);
 // visualizer.showWidget("grid",grid);
 // visualizer.showWidget("text",text);
  visualizer.showWidget("trajectory",trajectory);
  visualizer.showWidget("frustums",frustums);

  // visualization loop
  while(cv::waitKey(0) && !visualizer.wasStopped()){

    visualizer.spin();
  }
}

//===============================================
//FUNCTION: BASE RECONSTRUCTION
//===============================================

bool StructFromMotion::baseTriangulation(){

  if (matrixK.K.empty()) {
         std::cerr << "Intrinsics matrix (K) must be initialized." << std::endl;
         return false;
  }

  std::cout << "Getting best two views for first reconstruction..." << std::endl;
  ImagePair pair = findBestPair();
  //ImagePair pair ={4,5};
  Features alignedLeft,alignedRight;
  StructFromMotion::AlignedPointsFromMatch(nFeaturesImages[pair.left],
                                           nFeaturesImages[pair.right],
                                           nFeatureMatchMatrix[pair.left][pair.right],
                                           alignedLeft,alignedRight);

  // ESSENTIAL MATRIX
  cv::Mat mask;
  cv::Mat E = cv::findEssentialMat(alignedLeft.pt2D, alignedRight.pt2D,
                                   matrixK.K,cv::RANSAC,0.999, 1.0,mask);

  Matching bestMatches;

  for (size_t i = 0; i < mask.rows; i++) {
     if(mask.at<uchar>(i)) {
           bestMatches.push_back(nFeatureMatchMatrix[pair.left][pair.right][i]);
         }
    }    

  std::cout << "best pair:" << " image:(" <<pair.left << ") and image:("<<pair.right <<")" << std::endl;
  std::cout << "Showing matches between "<< "image:" << pair.left << " and image:"
            << pair.right << "..."<< std::endl;

  cv::Mat outImg= StructFromMotion::imageMatching(nImages[pair.left],nFeaturesImages[pair.left].kps,
                                                  nImages[pair.right],nFeaturesImages[pair.right].kps,
                                                  bestMatches);

  nFeatureMatchMatrix[pair.left][pair.right] = bestMatches;

 // StructFromMotion::imShow(outImg,"Matching");
  std::cout << "Getting camera pose..." << std::endl;

  // CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
  cv::Mat R,T;
  cv::recoverPose(E,alignedLeft.pt2D, alignedRight.pt2D,R,T,matrixK.fx,
                   cv::Point2d(matrixK.cx,matrixK.cy),mask);

  bool status = StructFromMotion::CheckCoherentRotation(R);

  cv::Matx34f Pleft  = cv::Matx34f::eye();
  cv::Matx34f Pright = cv::Matx34f::eye();

  if(status == true){

      Pright = cv::Matx34f(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), T.at<double>(0),
                           R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), T.at<double>(1),
                           R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), T.at<double>(2));
   }else{
      Pright = cv::Matx34f(0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0);
   }

  std::cout << "CAMERA POSE ==> [DONE]" << std::endl;
  std::cout << "Projection1:" << "\n" << Pleft << std::endl;
  std::cout << "Projection2:" << "\n" << Pright << std::endl;
  std::cout << "Triangulating points..." << std::endl;

  std::vector<Point3D> pointcloud;

  bool success = StructFromMotion::triangulateViews(nFeaturesImages[pair.left],nFeaturesImages[pair.right],
                                                    Pleft,Pright,bestMatches,
                                                    matrixK,pair,pointcloud);

  if(success==true){

      std::cout << "Base pointcloud ==> [DONE]" << std::endl;

      nReconstructionCloud = pointcloud;
      nCameraPoses[pair.left] = Pleft;
      nCameraPoses[pair.right] = Pright;

      nDoneViews.insert(pair.left);
      nDoneViews.insert(pair.right);

     // adjustCurrentBundle() ;
      return true;

  }else{
      std::cerr << "Could not triangulate image:" << pair.left << " and image:"<< pair.right<< std::endl;
      return false;
  }
}

//===============================================
//FUNCTION: FIND BEST PAIR
//===============================================

ImagePair StructFromMotion::findBestPair(){

  ImagePair pairIdx;

  for(const MatchesforSort& weight:nMatchesSorted){
      if(weight.size<30){
          continue;
      }else{

        int N = StructFromMotion::findHomographyInliers(nFeaturesImages[weight.i],
                                                      nFeaturesImages[weight.j],
                                                      nFeatureMatchMatrix[weight.i][weight.j]);

        if(N < 60){
          continue;
        }else{

          pairIdx = {weight.i,weight.j};
          break;
        }
     }
  }
  return pairIdx;
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

  matrixH = cv::findHomography(alignedLeft.pt2D,alignedRight.pt2D,cv::RANSAC,
                               0.004 * maxVal,inliersMask);

  int numInliers = cv::countNonZero(inliersMask);

  if(matches.size()< 30 or matrixH.empty()){

      numInliers = 0;
  }else{

  return numInliers;

  }
}

//===============================================
//FUNCTION: TRIANGULATE VIEWS
//===============================================

bool StructFromMotion::triangulateViews(const Features& left,const Features& right,const cv::Matx34f& P1,const cv::Matx34f& P2,const Matching& matches,const CameraData& matrixK,const ImagePair& pair,std::vector<Point3D>& pointcloud){

  Features alignedLeft,alignedRight;
  std::vector<int> leftBackReference,rightBackReference;
  StructFromMotion::AlignedPoints(left,right,matches,alignedLeft,alignedRight,
                                  leftBackReference,rightBackReference);

  // NORMALIZE IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)
  std::cout << "Normalizing points..." << std::endl;
  cv::Mat normalizedLeftPts,normalizedRightPts;
  cv::undistortPoints(alignedLeft.pt2D, normalizedLeftPts, matrixK.K, cv::Mat());
  cv::undistortPoints(alignedRight.pt2D, normalizedRightPts, matrixK.K, cv::Mat());
  std::cout << "POINTS NORMALIZED ==> [DONE]" << std::endl;

  std::cout << "Triangulating points from --> " << "image: " << pair.left << " and image: "
            << pair.right << std::endl;

  // TRIANGULATE POINTS
  cv::Mat pts3dHomogeneous;
  cv::triangulatePoints(P1,P2,normalizedLeftPts,normalizedRightPts,pts3dHomogeneous);
  std::cout << "TRIANGULATE POINTS ==> [DONE]" << std::endl;

  // CONVERTION CAMERA COORDINATE - WORLD COORDINATE
  std::cout << "Converting points to world coordinate..." << std::endl;
  cv::Mat pts3d; 
  cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(),pts3d);
  std::cout << "POINTS WORLD COORDINATE ==> [DONE] " << std::endl;

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
          p.idxImage[pair.left]  = leftBackReference[i];
          p.idxImage[pair.right] = rightBackReference[i];

          pointcloud.push_back(p);
  }
  std::cout << "POINTCLOUD VECTOR ==> [DONE]" << std::endl;
  std::cout << "Pointcloud size=" << pointcloud.size() << std::endl;

  return true;
}

//===============================================
//FUNCTION: ADD MORE VIEWS
//===============================================

void StructFromMotion::addMoreViews(){

  while(nDoneViews.size() != 5){

    std::cout <<"\n"<< "===================================="<< std::endl;
    std::cout << "Adding more views..." << std::endl;
    std::cout << "Finding 2D-3D correspondences..." << std::endl;
    ImagePair pair;
    Pts3D2DPNP pts2D3D = StructFromMotion::find2D3DMatches(pair);

    size_t framePC = pair.left;
    size_t newFrame = pair.right;

    std::cout << "The new frame:" << newFrame << " is ready for been add" << std::endl;

    nDoneViews.insert(newFrame);

    Points2f pts2D_PNP=pts2D3D.pts2D;
    Points3f pts3D_PNP=pts2D3D.pts3D;

    cv::Matx34f newCameraPose = cv::Matx34f::eye();
    StructFromMotion::findCameraPosePNP(matrixK,pts3D_PNP,pts2D_PNP,newCameraPose);
    nCameraPoses[newFrame]=newCameraPose;
    std::cout << "Add frame:("<< newFrame << ")"<< " - New camera pose:"<< "\n"
              << newCameraPose << std::endl;

    std::vector<Point3D> pointcloud;
    std::cout << "Triangulating points..." << std::endl;

    Matching prunedMatch;

    size_t leftView,rightView;
    if(framePC<newFrame){
        leftView = framePC;
        rightView = newFrame;
    }else{
        leftView = newFrame;
        rightView = framePC;
    }

    Features alignedLeft,alignedRight;
    StructFromMotion::AlignedPointsFromMatch(nFeaturesImages[leftView],
                                             nFeaturesImages[rightView],
                                             nFeatureMatchMatrix[leftView][rightView],
                                             alignedLeft,alignedRight);

    // ESSENTIAL MATRIX
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(alignedLeft.pt2D, alignedRight.pt2D,
                                     matrixK.K,cv::RANSAC,0.999, 1.0,mask);

    for (size_t i = 0; i < mask.rows; i++) {
       if(mask.at<uchar>(i)) {
             prunedMatch.push_back(nFeatureMatchMatrix[leftView][rightView][i]);
           }
    }
    nFeatureMatchMatrix[leftView][rightView]=prunedMatch;

    bool success = StructFromMotion::triangulateViews(nFeaturesImages[leftView],nFeaturesImages[rightView],
                                   nCameraPoses[leftView],nCameraPoses[rightView],
                                   nFeatureMatchMatrix[leftView][rightView],matrixK,
                                   {leftView,rightView},pointcloud);

    std::cout << "New pointcloud ==> [DONE]" << std::endl;
    StructFromMotion::mergeNewPoints(pointcloud);
/*
    for(size_t i=0;i<pointcloud.size();i++){

       nReconstructionCloud.push_back(pointcloud[i]);
    }

*/
    //adjustCurrentBundle() ;
  }
  std::cout << "\n"<< "=============================== " << std::endl;
  std::cout << "Images processed = " << nDoneViews.size() << " of " << nImages.size() << std::endl;
}

//===============================================
//FUNCTION: FIND CORRESPONDENCES 2D-3D
//===============================================

Pts3D2DPNP StructFromMotion::find2D3DMatches(ImagePair& pair){

   Pts3D2DPNP matches2D3D;
   std::map<int,ImagePair> matchesFramesPC_NewFrame;
   const size_t totalImages = nImages.size();


   /*Cargar nuevos frames (empezando en orden desde el frame número cero)!!*/
   for(size_t newFrame = 0;newFrame< totalImages;newFrame++){
      std::cout << "Finding best frame to add...";

      /* Se verifica si el nuevo frame a adicionar ya fue agregado anteriormente a la nube de puntos*/
      /* 'nDoneViews' es un vector que contiene los frames ya adicionados*/
      /* Si la condición es verdadera, entonces el nuevo frame está en la nube de puntos,
       * no es necesario procesar!*/
      if(nDoneViews.count(newFrame)== 1){          
        continue; //Pase al siguiente frame
      }

      /* La nube de puntos se compone de varios frames, por lo que se debe verificar con cual de esos
       * frames, el nuevo frame tiene mejor coincidencias (número de matches)*/
      /* 'matchesFramesPC_NewFrame' es un vector que contiene el número de matches entre el frame de la
       * nube de puntos y el nuevo frame*/
      /* se ordena de menor a mayor número de matches*/
      for(size_t framePC:nDoneViews){

          if(framePC>newFrame){
            const size_t bestSizeMatches = nFeatureMatchMatrix[newFrame][framePC].size();
            matchesFramesPC_NewFrame[bestSizeMatches]={framePC,newFrame};
            continue;
          }else{
            const size_t bestSizeMatches = nFeatureMatchMatrix[framePC][newFrame].size();
            matchesFramesPC_NewFrame[bestSizeMatches]={framePC,newFrame};
            continue;
          }
       }

      std::map<int,ImagePair>::const_iterator pos = std::prev(matchesFramesPC_NewFrame.end());
      size_t matchSizes = pos->first;
      size_t left = pos->second.left;
      size_t right = pos->second.right;

      std::cout << "[DONE]" << std::endl;
      std::cout << "New frame to add: " << right << std::endl;
      std::cout << "Verifying if number of matches is enough...";
      if(matchSizes < 60){
         matchesFramesPC_NewFrame.clear();
         std::cout << "[X]" << std::endl;
         continue;
      }
      std::cout << "[OK]" << std::endl;
      std::cout << "Found "<< matchSizes << " matches between frame pointCloud:"
                    << left << " and new frame:" << right << std::endl;
      std::cout << "Finding points 3D of frame pointCloud that match with the new frame..";

      for(const Point3D& numPt3D : nReconstructionCloud){

         if(numPt3D.idxImage.count(left)==0){
               continue;
          }

          const size_t idxImage = left;
          std::map<int,int>::const_iterator idx = numPt3D.idxImage.find(left);
          const size_t idxPt2D =  idx->second;

          size_t leftView,rightView;

          if(idxImage<right){
              leftView = idxImage;
              rightView = right;
          }else{
              leftView = right;
              rightView= idxImage;
          }

          for (const cv::DMatch& DMatch : nFeatureMatchMatrix[leftView][rightView]) {
              size_t idxMatchPoint = -1;
              if(idxImage<right){

                  const cv::Point2f pt1= nFeaturesImages[idxImage].pt2D[DMatch.trainIdx];
                  const cv::Point2f pt2= nFeaturesImages[idxImage].pt2D[idxPt2D];

                  if(pt1 != pt2){
                      continue;
                  }else{
                      idxMatchPoint = DMatch.queryIdx;
                  }
                }else{

                  const cv::Point2f pt1= nFeaturesImages[idxImage].pt2D[DMatch.queryIdx];
                  const cv::Point2f pt2= nFeaturesImages[idxImage].pt2D[idxPt2D];

                  if(pt1 != pt2){
                      continue;
                  }else{
                      idxMatchPoint = DMatch.trainIdx;
                  }

                }

                if (idxMatchPoint >= 0) {
                  //This point is matched in the new view
                  const Features& newView = nFeaturesImages[right];
                  matches2D3D.pts2D.push_back(newView.pt2D[idxMatchPoint]);
                  matches2D3D.pts3D.push_back(numPt3D.pt);
                  break;
                }
          }
        }
      if(matches2D3D.pts2D.size()<10){
          std::cout << "\n"<< "Not found enough points for PnPRansac, found: "
                    << matches2D3D.pts2D.size() << std::endl;
          std::cout << "\n"<< "=============================== " << std::endl;
          matchesFramesPC_NewFrame.clear();
          continue;
        }else{
          pair = {left,right};
          std::cout << "[DONE]" << std::endl;
          std::cout << "Found: " << matches2D3D.pts2D.size() << " for PnPRansac Pt2D and "
                    << matches2D3D.pts3D.size() << " for PnPRansac Pt3D" << std::endl;
          break;
        }
     }
      return matches2D3D;
}

//===============================================
//FUNCTION: FIND CAMERA POSE PNP RANSAC
//===============================================

void StructFromMotion::findCameraPosePNP(const CameraData& matrixK,const std::vector<cv::Point3f>& pts3D,const std::vector<cv::Point2f>& pts2D,cv::Matx34f& P){

  cv::Mat rvec, T;
  cv::Mat inliers;
  double RANSAC_THRESHOLD=8.0f;

  std::cout << "Finding new camera pose..." << std::endl;

  cv::solvePnPRansac(pts3D,pts2D,matrixK.K,cv::Mat(),rvec,T,false,100,
                     RANSAC_THRESHOLD,0.99,inliers);

  cv::Mat R;
  cv::Rodrigues(rvec, R); //convert to a rotation matrix

  P = cv::Matx34f(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), T.at<double>(0),
                  R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), T.at<double>(1),
                  R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), T.at<double>(2));
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
  return det;
}

void StructFromMotion::adjustCurrentBundle() {
    adjustBundle(nReconstructionCloud,nCameraPoses,matrixK,nFeaturesImages);

}

void StructFromMotion::mergeNewPointCloud(const std::vector<Point3D>& cloud){

  const size_t numImages = nImages.size();
  const float MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE   = 0.2;

  for (const Point3D& p : cloud) {
      const cv::Point3f newPoint = p.pt; //new 3D point

      for (Point3D& existingPoint : nReconstructionCloud) {

         float error = cv::norm(existingPoint.pt - newPoint);

         if (error < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE) {
          //This point is very close to an existing 3D cloud point

             nReconstructionCloud.push_back(p);
           }else{
             continue;
           }
        }
    }
}


void StructFromMotion::mergeNewPoints(const std::vector<Point3D>& cloud) {
    const size_t numImages = nImages.size();
    std::vector<std::vector<Matching>> mergeMatchMatrix;
    mergeMatchMatrix.resize(numImages, std::vector<Matching>(numImages));

    size_t newPoints = 0;
    size_t mergedPoints = 0;
    const float MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE   = 0.01;
    const float MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE = 20.0;

    for (const Point3D& p : cloud) {
        const cv::Point3f newPoint = p.pt; //new 3D point

        bool foundAnyMatchingExistingViews = false;
        bool foundMatching3DPoint = false;
        for (const Point3D& existingPoint : nReconstructionCloud) {
            if (cv::norm(existingPoint.pt - newPoint) < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE) {
                //This point is very close to an existing 3D cloud point
                foundMatching3DPoint = true;

                //Look for common 2D features to confirm match
                for (const auto& newKv : p.idxImage) {
                    //kv.first = new point's originating view
                    //kv.second = new point's view 2D feature index

                    for (const auto& existingKv : existingPoint.idxImage) {
                        //existingKv.first = existing point's originating view
                        //existingKv.second = existing point's view 2D feature index

                        bool foundMatchingFeature = false;
                        const bool newIsLeft = newKv.first > existingKv.first;
                        const int leftViewIdx         = (newIsLeft) ? existingKv.first  :newKv.first ;
                        const int leftViewFeatureIdx  = (newIsLeft) ? existingKv.second: newKv.second ;
                        const int rightViewIdx        = (newIsLeft) ? newKv.first  : existingKv.first;
                        const int rightViewFeatureIdx = (newIsLeft) ? newKv.second: existingKv.second ;

                        const Matching& matching = nFeatureMatchMatrix[leftViewIdx][rightViewIdx];
                        for (const cv::DMatch& match : matching) {
                            if (match.queryIdx == leftViewFeatureIdx
                                and match.trainIdx == rightViewFeatureIdx
                                and match.distance < MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE) {

                               // mergeMatchMatrix[leftViewIdx][rightViewIdx].push_back(match);

                                //Found a 2D feature match for the two 3D points - merge
                                foundMatchingFeature = true;
                                break;
                            }
                        }

                        if (foundMatchingFeature) {
                            //Add the new originating view, and feature index
                            //existingPoint.idxImage[newKv.first] = newKv.second;

                            foundAnyMatchingExistingViews = true;
                        }
                    }
                }
            }
            if (foundAnyMatchingExistingViews) {
              //  mergedPoints++;
                break; //Stop looking for more matching cloud points
            }
        }

        if (not foundAnyMatchingExistingViews and not foundMatching3DPoint) {
            //This point did not match any existing cloud points - add it as new.
            nReconstructionCloud.push_back(p);
           // newPoints++;
        }

   }
}






