//***********************************************
//HEADERS
//***********************************************
#include "include/Sfm.h"

/********************************************
                  PIPELINE
********************************************/
bool StructFromMotion::run_SFM(){

  std::cout << "************************************************" << std::endl;
  std::cout << "              3D MAPPING                        " << std::endl;
  std::cout << "************************************************" << std::endl;

  if(nImages.size() <= 0) {
      std::cerr << "No images to work on." << std::endl;
      return false;
  }

  nCameraPoses.resize(nImages.size()); //Define a fixed size for vector(cv::Matx34f) camera poses

  // **(1) FEATURE DETECTION AND EXTRACTION - ALL IMAGES
  bool success = extractFeature();
  if(not success){
      std::cerr << "No could find features. corrupt images" << std::endl;
      return false;
  }

  // **(2) PRINT INPUT IMAGES
  for(unsigned int i=0;i<nImages.size();i++){
      cv::namedWindow("Input images",cv::WINDOW_NORMAL);
      cv::resizeWindow("Input images",nImages[i].cols,nImages[i].rows);
      cv::moveWindow("Input images",700,0);
      cv::imshow("Input images",nImages[i]);
      cv::waitKey(500);
  }
  cv::destroyWindow("Input images");

  // **(3) BASE RECONSTRUCTION
  success= baseTriangulation();
  if(not success){
      std::cerr << "No could find a good pair for initial reconstruction" << std::endl;
      return false;
  }

  // **(4) ADD MORE VIEWS
  success = addMoreViews();
  if(not success){
      std::cerr << "Could not add more views" << std::endl;
      return false;
  }

  saveCloudToPCD();
  PMVS2();

  Visualizer vs;
  vs.addPointCloudToPCL(nReconstructionCloud,nReconstructionCloudRGB);

  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

  return true;
}

/********************************************
 FUNCTIONS
********************************************/

//===============================================
//IMAGES LOAD
//===============================================
bool StructFromMotion::imagesLOAD(const std::string&  directoryPath){

  std::cout << "Getting images..." << std::flush;
  pathImages = directoryPath;
  boost::filesystem::path dirPath(directoryPath);

  if(not boost::filesystem::exists(dirPath) or not boost::filesystem::is_directory(dirPath)){
      std::cerr << "Cannot open directory: " << directoryPath << std::endl;
      return false;
  }

  for(boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(dirPath)){
      std::string extension = x.path().extension().string();
      boost::algorithm::to_lower(extension);
      if(extension == ".jpg" or extension == ".png"){
          nImagesPath.push_back(x.path().string());
      }
  }

  /*sort data set of images in vector*/
  std::sort(nImagesPath.begin(), nImagesPath.end());

  if(nImagesPath.size() <= 0){
       std::cerr << "Unable to find valid files in images directory (\"" << directoryPath << "\")."
                 << std::endl;
       return false;
  }

  std::cout << "Found " << nImagesPath.size() << " image files in directory." << std::endl;
  /*Read input images and save them in a images vector*/
  for(const std::string& imageFilename : nImagesPath){

      cv::Mat img   = cv::imread(imageFilename,cv::IMREAD_COLOR); //Read input image
      cv::Mat temp = img.clone(); //Copy image to temp variable
      cv::Mat resize1,resize2,GaussianBlur;
      cv::resize(temp,resize1,cv::Size(640,480),0.0,0.0); //Define a size of 640x480
      cv::resize(resize1,resize2,cv::Size(),0.75,0.75); //Define a size of 640(75%)x480(75%)
      cv::GaussianBlur(resize2,GaussianBlur, cv::Size(3,3),0,0); //Apply a filter gaussian for noise
      nImages.push_back(GaussianBlur); //Save input image in nImages vector

      if(nImages.back().empty()) {
          std::cerr << "[x]"<<"\n" <<"Unable to read image from file: " << imageFilename << std::endl;
          return false;
      }
  }

  if(nImages.size()<2){
      std::cerr << "Sorry. is not enough images, 6 minimum" << std::endl;
      return false;
      }

  return true;
}

//===============================================
//GET CAMERA MATRIX
//===============================================
bool StructFromMotion::getCameraMatrix(const std::string str){

    std::cout << "Getting camera matrix..." << std::endl;
    cv::Mat intrinsics;
    cv::Mat cameraDistCoeffs;

    /*Read camera calobration file*/
    cv::FileStorage fs(str, cv::FileStorage::READ);

    /*Get data from tags: Camera_Matrix and Distortion_Coefficients*/
    fs["Camera_Matrix"] >> intrinsics;
    fs["Distortion_Coefficients"] >> cameraDistCoeffs;

    if(intrinsics.empty() or intrinsics.at<float>(2,0) !=0){
        std::cerr << "Error: no found or invalid camera calibration file.xml" << std::endl;
        return false;
    }

    /*Fill local variables with input data*/
    cameraMatrix.K = intrinsics;                  //Matrix K (3x3)
    cameraMatrix.distCoef = cameraDistCoeffs;     //Distortion coefficients (1x5)
    cameraMatrix.invK = inverse(intrinsics);      //Inverse matrix K
    cameraMatrix.fx = intrinsics.at<float>(0,0);  //Focal length in x
    cameraMatrix.fy = intrinsics.at<float>(1,1);  //Focal length in y
    cameraMatrix.cx = intrinsics.at<float>(0,2);  //Center image in x
    cameraMatrix.cy = intrinsics.at<float>(1,2);  //Center image in y

    std::cout << "Camera matrix:" << "\n" << intrinsics << std::endl;
    std::cout <<"Distortion coefficients: "<< std::endl;
    std::cout << cameraDistCoeffs << std::endl;

    if(cameraMatrix.K.empty()){
        std::cerr << "Could not load local variables with camera calibration file data" << std::endl;
        return false;
    }else{
        return true;
    }
}

//===============================================
// Extract feature
//===============================================
bool StructFromMotion::extractFeature(){

  std::cout << "Getting features from all images..." << std::endl;
  nFeatureImages.resize(nImages.size());

  for(size_t n=0;n<nImages.size();n++){

      nFeatureImages[n] = getFeature(nImages[n]);
      Feature ft = nFeatureImages[n];
      std::cout << "Image:" << n << " --> " << ft.pt2D.size() << " keypoints" << std::endl;
   }

  if(nFeatureImages.empty()){return false;}
  return true;
}

//===============================================
//Get Feature
//===============================================
Feature StructFromMotion::getFeature(const cv::Mat& image) {
    Feature Feature;
    ptrFeature2D->detect(image,Feature.kps);
    ptrFeature2D->compute(image,Feature.kps,Feature.descriptors);
    keypoints2F(Feature.kps,Feature.pt2D);
    return Feature;
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
//FUNCTION: BASE RECONSTRUCTION
//===============================================
bool StructFromMotion::baseTriangulation(){

  std::map<float,ImagePair>  bestViews = findBestPair();

  for(std::pair<const float,ImagePair>& bestpair : bestViews){

      int queryImage = bestpair.second.left;
      int trainImage = bestpair.second.right;

      Matching bestMatch = getMatching(nFeatureImages[queryImage],nFeatureImages[trainImage]);

      std::cout << "Best pair:" << "["<< queryImage << "," << trainImage<<"]" << " has:"
                << bestMatch.size() << " matches" << " and " << bestpair.first << " inliers." << std::endl;

      Matching prunedMatching;
      cv::Matx34f Pleft  = cv::Matx34f::eye();
      cv::Matx34f Pright = cv::Matx34f::eye();

      std::cout << "Estimating camera pose with Essential Matrix..." << std::flush;
      bool success = getCameraPose(cameraMatrix,bestMatch,nFeatureImages[queryImage],
                                   nFeatureImages[trainImage],prunedMatching,Pleft,Pright);

      if(not success){
         std::cout << "[X]" << std::endl;
         std::cerr << "Failed. stereo view could not be obtained " << queryImage << "," << trainImage
                   << ", something wrong." << std::endl;
         continue;
      }

      std::cout << "Camera:" << queryImage << "\n" << Pleft << std::endl;
      std::cout << "Camera:" << trainImage <<"\n" << Pright << std::endl;
      std::cout << "Showing matches between "<< "image:" << queryImage << " and image:"
                << trainImage << std::endl;

      cv::Mat matchImage;
      cv::destroyWindow("Matching pairs");
      cv::drawMatches(nImages[queryImage],nFeatureImages[queryImage].kps,nImages[trainImage],
                      nFeatureImages[trainImage].kps,bestMatch,matchImage,
                      cv::Scalar::all(-1),cv::Scalar::all(-1),std::vector<char>(),2);
      cv::namedWindow("Best pair matching",cv::WINDOW_NORMAL);
      cv::resizeWindow("Best pair matching",matchImage.cols,matchImage.rows);
      cv::putText(matchImage, "Image " + std::to_string(queryImage) + "                        "+
                  "                     "+  "                     " +
                  "Image" + std::to_string(trainImage),
                  cv::Point(10,matchImage.rows-10),cv::FONT_ITALIC,0.5,cv::Scalar(0,255,0),1);
      cv::moveWindow("Best pair matching",700,0);
      cv::imshow("Best pair matching", matchImage);
      cv::waitKey(0);
      cv::destroyWindow("Best pair matching");

      std::vector<Point3D> pointcloud;
      std::vector<cv::Vec3b> cloudRGB;

      success = triangulateViews(nFeatureImages[queryImage],nFeatureImages[trainImage],
                                 Pleft,Pright,bestMatch,cameraMatrix,
                                 {queryImage,trainImage},pointcloud);

      if(not success){
          std::cerr << "Could not triangulate image:" << queryImage << " and image:"<< trainImage
                    << std::endl;
          continue;
      }

      nReconstructionCloud = pointcloud;
      GetRGBForPointCloud(pointcloud,cloudRGB);
      nReconstructionCloudRGB = cloudRGB;

      nCameraPoses[queryImage] = Pleft;
      nCameraPoses[trainImage] = Pright;

      nDoneViews.insert(queryImage);
      nDoneViews.insert(trainImage);

      nGoodViews.insert(queryImage);
      nGoodViews.insert(trainImage);

      saveCloudToPCD();
      break;
  }  
  adjustCurrentBundle();
  return true;
}

//===============================================
//BEST PAIR FOR BASELINE
//===============================================
std::map<float,ImagePair>  StructFromMotion::findBestPair(){

  std::cout << "Getting best two views for baseline..." << std::flush;
  std::map<float,ImagePair> numInliers;
  const size_t numImg = nImages.size();

  for(int queryImage=0;queryImage<numImg-1;queryImage++) {
     for(int trainImage=queryImage+1;trainImage<numImg;trainImage++){

        Matching correspondences,prunedMatching;
        correspondences = getMatching(nFeatureImages[queryImage],nFeatureImages[trainImage]);

        cv::Mat matchImage;
        cv::drawMatches(nImages[queryImage],nFeatureImages[queryImage].kps,nImages[trainImage],
                        nFeatureImages[trainImage].kps,correspondences,matchImage,
                        cv::Scalar::all(-1),cv::Scalar::all(-1),std::vector<char>(),2);

        cv::namedWindow("Matching pairs",cv::WINDOW_NORMAL);
        cv::resizeWindow("Matching pairs",matchImage.cols,matchImage.rows);
        cv::moveWindow("Matching pairs",700,0);
        cv::putText(matchImage, "Image" + std::to_string(queryImage)+ "                        "+
                    "                     " +  "                     " +
                    +" Image" + std::to_string(trainImage),
                     cv::Point(10,matchImage.rows-10),cv::FONT_ITALIC,0.5,cv::Scalar(0,255,0),1);
        cv::imshow("Matching pairs", matchImage);
        cv::waitKey(1);

        if(correspondences.size()<30) continue;
        int N = findHomographyInliers(nFeatureImages[queryImage],nFeatureImages[trainImage],
                                                   correspondences);

        if(N < 60) continue;

        Feature alignedQuery,alignedTrain;
        AlignedPointsFromMatch(nFeatureImages[queryImage],nFeatureImages[trainImage],correspondences,
                               alignedQuery,alignedTrain);

        // ESSENTIAL MATRIX
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(alignedQuery.pt2D, alignedTrain.pt2D,
                                         cameraMatrix.K,cv::RANSAC,0.999, 1.0,mask);

        for(size_t i = 0; i < mask.rows; i++) {
           if(mask.at<uchar>(i)) {
                 prunedMatching.push_back(correspondences[i]);
               }
          }

        float poseInliersRatio = (float)prunedMatching.size()/(float)correspondences.size();
        std::cout << "pair:" << "[" << queryImage << "," << trainImage << "]" << " has:" << poseInliersRatio
                  << " pose inliers ratio."<< std::endl;

        numInliers[poseInliersRatio]={queryImage,trainImage};
     }
  }
  return numInliers;
}

//===============================================
//FUNCTION: FEATURE MATCHING
//===============================================
Matching StructFromMotion::getMatching(const Feature& queryImage,const Feature& trainImage){

  /*Knn matching*/
  Matching goodMatches;
  std::vector<Matching> initialMatching;
  matcherFlan ->knnMatch(queryImage.descriptors,trainImage.descriptors,initialMatching,2);

  /*RATIO-TEST FILTER*/
  for(unsigned i = 0; i < initialMatching.size(); i++) {
      if(initialMatching[i][0].distance <= NN_MATCH_RATIO * initialMatching[i][1].distance) {
          goodMatches.push_back(initialMatching[i][0]);
      }
  }

  return goodMatches;
}

//===============================================
//FUNCTION: FIND HOMOGRAPHY INLIERS
//===============================================
int StructFromMotion::findHomographyInliers(const Feature& queryFeature,const Feature& trainFeature,const Matching& matches){

  Feature alignedQuery,alignedTrain;
  AlignedPointsFromMatch(queryFeature,trainFeature,matches,alignedQuery,alignedTrain);

  double minVal,maxVal;
  cv::minMaxIdx(alignedQuery.pt2D,&minVal,&maxVal);

  cv::Mat matrixH(3,3,CV_32FC3);
  cv::Mat inliersMask;

  matrixH = cv::findHomography(alignedQuery.pt2D,alignedTrain.pt2D,cv::RANSAC,
                               0.004 * maxVal,inliersMask);

  int numInliers = cv::countNonZero(inliersMask);
  return numInliers;
}

//===============================================
//FUNCTION: ALIGNED POINTS
//===============================================
void StructFromMotion::AlignedPointsFromMatch(const Feature& queryImg,const Feature& trainImg,const Matching& matches,Feature& alignedL,Feature& alignedR){

   std::vector<int> leftId,rightId;
   AlignedPoints(queryImg,trainImg,matches,alignedL,alignedR,leftId,rightId);  
}

void StructFromMotion::AlignedPoints(const Feature& queryImg,const Feature& trainImg,const Matching& matches, Feature& alignedL, Feature& alignedR,std::vector<int>& idLeftOrigen,std::vector<int>& idRightOrigen){

      //align left and right point sets
      for(unsigned int i=0;i<matches.size();i++){

        alignedL.pt2D.push_back(queryImg.pt2D[matches[i].queryIdx]);
        alignedR.pt2D.push_back(trainImg.pt2D[matches[i].trainIdx]);

        idLeftOrigen.push_back(matches[i].queryIdx);
        idRightOrigen.push_back(matches[i].trainIdx);
      }
}

bool StructFromMotion::getCameraPose(const CameraData& intrinsics,const Matching & matches,
                                     const Feature& left, const Feature& right, Matching& prunedMatch,
                                     cv::Matx34f& Pleft, cv::Matx34f& Pright){

  if (intrinsics.K.empty()) {

      std::cerr << "Intrinsics matrix (K) must be initialized." << std::endl;
      return false;
  }

  Feature alignedLeft,alignedRight;
  AlignedPointsFromMatch(left,right,matches,alignedLeft,alignedRight);

  // ESSENTIAL MATRIX
  cv::Mat mask;
  cv::Mat E = cv::findEssentialMat(alignedLeft.pt2D, alignedRight.pt2D,
                                   intrinsics.K,cv::RANSAC,0.999, 1.0,mask);

  // CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
  cv::Mat R,T;
  cv::Point2d pp = cv::Point2d(intrinsics.cx,intrinsics.cy);
  cv::recoverPose(E,alignedLeft.pt2D, alignedRight.pt2D,R,T,intrinsics.fx,pp,mask);

  bool success = CheckCoherentRotation(R);

  if(not success){
      Pright = cv::Matx34f(0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0);
  }

  //Rotational element in a 3x4 matrix
  const cv::Rect ROT(0, 0, 3, 3);
  //Translational element in a 3x4 matrix
  const cv::Rect TRA(3, 0, 1, 3);

  Pleft  = cv::Matx34f::eye();
  R.copyTo(cv::Mat(3, 4, CV_32FC1, Pright.val)(ROT));
  T.copyTo(cv::Mat(3, 4, CV_32FC1, Pright.val)(TRA));

  prunedMatch.clear();

  for(size_t i = 0; i < mask.rows; i++) {
     if(mask.at<uchar>(i)) prunedMatch.push_back(matches[i]);

  }
  return success;
}

//===============================================
//FUNCTION: CHECK ROTATION MATRIX (Must be det=1)
//===============================================
bool StructFromMotion::CheckCoherentRotation(cv::Mat& R){

   if(fabsf(determinante(R))-1.0 > 1e-07) {

      std::cout << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
      return false;
    }
    return true;
}

//===============================================
//FUNCTION: TRIANGULATE VIEWS
//===============================================
bool StructFromMotion::triangulateViews(const Feature& query,const Feature& train,const cv::Matx34f& P1,const cv::Matx34f& P2,const Matching& matches,const CameraData& matrixK,const ImagePair& pair,std::vector<Point3D>& pointcloud){

  std::cout << "** IMAGE COORDINATE - CAMERA COORDINATE CONVERTION **" << std::endl;

  pointcloud.clear();

  Feature alignedQuery,alignedTrain;
  std::vector<int> leftBackReference,rightBackReference;
  AlignedPoints(query,train,matches,alignedQuery,alignedTrain,
                                  leftBackReference,rightBackReference);

  // NORMALIZE IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)
  std::cout << "Normalizing points..." << std::flush;
  cv::Mat normalizedLeftPts,normalizedRightPts;
  cv::undistortPoints(alignedQuery.pt2D, normalizedLeftPts, matrixK.K,matrixK.distCoef);
  cv::undistortPoints(alignedTrain.pt2D, normalizedRightPts, matrixK.K, matrixK.distCoef);
  std::cout << "[DONE]" << std::endl;

  // TRIANGULATE POINTS
  std::cout << "Triangulating points..." << std::flush;
  cv::Mat pts3dHomogeneous;
  cv::triangulatePoints(P1,P2,normalizedLeftPts,normalizedRightPts,pts3dHomogeneous);
  std::cout << "[DONE]" << std::endl;

  std::cout << "** CAMERA COORDINATE - WORLD COORDINATE CONVERTION **" << std::endl;

  // CONVERTION CAMERA COORDINATE - WORLD COORDINATE
  std::cout << "Converting points to world coordinate..." << std::flush;
  cv::Mat pts3d;
  cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(),pts3d);
  std::cout << "[DONE]" << std::endl;

  cv::Mat rvecLeft;
  cv::Rodrigues(P1.get_minor<3,3>(0,0),rvecLeft);
  cv::Mat tvecLeft(P1.get_minor<3,1>(0,3));

  Points2f projectedLeft(alignedQuery.pt2D.size());
  cv::projectPoints(pts3d,rvecLeft,tvecLeft,matrixK.K,matrixK.distCoef,projectedLeft);

  cv::Mat rvecRight;
  cv::Rodrigues(P2.get_minor<3,3>(0,0),rvecRight);
  cv::Mat tvecRight(P2.get_minor<3,1>(0,3));

  Points2f projectedRight(alignedTrain.pt2D.size());
  cv::projectPoints(pts3d,rvecRight,tvecRight,matrixK.K,matrixK.distCoef,projectedRight);

  std::cout << "Creating a pointcloud vector..." << std::flush;
  const float MIN_REPROJECTION_ERROR = 10.0f; //Maximum 10-pixel allowed re-projection error

  for(int i = 0; i < pts3d.rows; i++){

      //check if point reprojection error is small enough

      const float queryError = cv::norm(projectedLeft[i]  - alignedQuery.pt2D[i]);
      const float trainError = cv::norm(projectedRight[i] - alignedTrain.pt2D[i]);

      if(MIN_REPROJECTION_ERROR < queryError or
         MIN_REPROJECTION_ERROR < trainError) continue;

          Point3D p;
          p.pt = cv::Point3f(pts3d.at<float>(i, 0),
                             pts3d.at<float>(i, 1),
                             pts3d.at<float>(i, 2));

          //use back reference to point to original Feature in images
          p.idxImage[pair.left]  = leftBackReference[i];
          p.idxImage[pair.right] = rightBackReference[i];
          p.pt2D[pair.left]=nFeatureImages[pair.left].pt2D[leftBackReference[i]];
          p.pt2D[pair.right]=nFeatureImages[pair.right].pt2D[rightBackReference[i]];

          pointcloud.push_back(p);
  }

  std::cout << "[DONE]" << std::endl;
  std::cout << "Pointcloud vector = " << pointcloud.size() << " 3D pts" << std::endl;
  return true;
}

//===============================================
//BUNDLE ADJUSTMENT
//===============================================
void StructFromMotion::adjustCurrentBundle() {

  std::cout << "Bundle adjuster..." << std::endl;
  BundleAdjustment::adjustBundle(nReconstructionCloud,nCameraPoses,cameraMatrix,nFeatureImages);

}

//===============================================
//FUNCTION: ADD MORE VIEWS
//===============================================
bool StructFromMotion::addMoreViews(){

  while(nDoneViews.size() != 5){

      std::cout <<"\n"<< "===================================="<< std::endl;
      std::cout << "ESTIMATING MORE CAMERAS PROJECTION..." << std::endl;

      std::vector<cv::Point3f> points3D;
      std::vector<cv::Point2f> points2D;

      int queryImage,trainImage;

      for(int NEW_FRAME = 0;NEW_FRAME<nImages.size();NEW_FRAME++){

          if(nDoneViews.count(NEW_FRAME)==1) continue; //Skip done views
          int LEFTVIEW,RIGHTVIEW;
          int bestNumMatches = 0;
          Matching bestMatch;

          for(int doneView : nDoneViews){

              if(NEW_FRAME < doneView){
                  queryImage = NEW_FRAME;
                  trainImage = doneView;
              }else{
                  queryImage = doneView;
                  trainImage = NEW_FRAME;
              }

              const Matching match = getMatching(nFeatureImages[queryImage],
                                                 nFeatureImages[trainImage]);

              int numMatches = match.size();
              if(numMatches > bestNumMatches) {
                 bestMatch       = match;           
                 bestNumMatches = numMatches;
                 LEFTVIEW = queryImage;
                 RIGHTVIEW = trainImage;
              }
              int p=7;
          }

          int oldView,newView;

          if(nDoneViews.count(LEFTVIEW)==1){
              oldView = LEFTVIEW;
              newView = RIGHTVIEW;
              std::cout << "Old view:" <<  LEFTVIEW << " new view:" << RIGHTVIEW << std::endl;
          }else{
              oldView = RIGHTVIEW;
              newView = LEFTVIEW;
              std::cout << "Old view:" <<  RIGHTVIEW << " new view:" << LEFTVIEW << std::endl;
          }

          std::cout << "Extracting 2d3d correspondences..." << std::endl;
          find2D3DMatches(LEFTVIEW,RIGHTVIEW,bestMatch,points3D,points2D);

          std::cout << "Estimating camera pose..." << std::endl;
          cv::Matx34f newCameraPose = cv::Matx34f::eye();
          bool success = findCameraPosePNP(cameraMatrix,points3D,points2D,newCameraPose);

          if(not success){
             continue;
          }                   

          std::cout << "Adding " << newView << " to existing "
                          << cv::Mat(std::vector<int>(nDoneViews.begin(), nDoneViews.end())).t() << std::endl;


          nCameraPoses[newView] = newCameraPose;
          std::vector<Point3D> new_triangulated;

          bool good_triangulation = triangulateViews(nFeatureImages[LEFTVIEW],nFeatureImages[RIGHTVIEW],
                                                     nCameraPoses[LEFTVIEW],nCameraPoses[RIGHTVIEW],
                                                     bestMatch,cameraMatrix,
                                                     {LEFTVIEW,RIGHTVIEW},new_triangulated);

          if(not good_triangulation){
            continue;
           }

          std::cout << "before triangulation: " << nReconstructionCloud.size() << std::endl;;
          mergeNewPoints(new_triangulated);         
          std::cout << "after " << nReconstructionCloud.size() << std::endl;

          nDoneViews.insert(newView);
          break;

          saveCloudToPCD();
      }

      adjustCurrentBundle() ;
      saveCloudToPCD();
 }

 std::cout << "\n"<< "=============================== " << std::endl;
 std::cout << "Images processed = " << nDoneViews.size() << " of " << nImages.size() << std::endl;
 std::cout << "PointCloud size = " << nReconstructionCloud.size() << " pts3D" << std::endl;

 return true;
}

//===============================================
//FUNCTION: FIND CORRESPONDENCES 2D-3D
//===============================================
void StructFromMotion::find2D3DMatches(const int& queryImage,const int& trainImage,const Matching& bestMatch,
                                       std::vector<cv::Point3f>& points3D,
                                       std::vector<cv::Point2f>& points2D){

     points3D.clear(); points2D.clear();
     int oldView,newView;

     //scan all cloud 3D points
     for(const Point3D& cloudPoint : nReconstructionCloud){

         bool found2DPoint = false;
         //scan all originating views for that 3D point
         for(const std::pair<const int,int>& origViewAndPoint : cloudPoint.idxImage){

             //check for 2D-2D matching
             const int originatingViewIndex      = origViewAndPoint.first;
             const int originatingViewFeatureIndex = origViewAndPoint.second;

             if(nDoneViews.count(queryImage) == 1){
                 oldView = queryImage;
                 newView = trainImage;
             }else{
                 oldView = trainImage;
                 newView = queryImage;
             }

             if(originatingViewIndex != oldView)continue;

             //scan all 2D-2D matches between originating view and new view
             for(const cv::DMatch& m : bestMatch){

                 int matched2DPointInNewView = -1;
                 if(originatingViewIndex < newView) { //originating view is 'left'

                     if(m.queryIdx == originatingViewFeatureIndex){
                         matched2DPointInNewView = m.trainIdx;
                     }
                 }else{ //originating view is 'right'

                     if(m.trainIdx == originatingViewFeatureIndex){
                         matched2DPointInNewView = m.queryIdx;
                     }
                 }
                 if(matched2DPointInNewView >= 0){

                     //This point is matched in the new view
                     const Feature& newViewFeatures = nFeatureImages[newView];
                     points2D.push_back(newViewFeatures.pt2D[matched2DPointInNewView]);
                     points3D.push_back(cloudPoint.pt);
                     found2DPoint = true;
                     break;
                  }
              }

              if(found2DPoint){

                  break;
              }
         }
    }

     std::cout << "Found: " << points3D.size() << " Pt3D and "
                  << points2D.size() << " Pt2D" << std::endl;

}







//==============================================
//INVERSE MATRIX-DETERMINANT FUNCTION EIGEN
//==============================================

cv::Mat StructFromMotion::inverse(cv::Mat& matrix){

    Eigen::MatrixXd invMatrix,invMatrixTranspose;
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic,
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















//===============================================
//FUNCTION: FIND CAMERA POSE PNP RANSAC
//===============================================

bool StructFromMotion::findCameraPosePNP(const CameraData& intrinsics,const std::vector<cv::Point3f>& pts3D,const std::vector<cv::Point2f>& pts2D,cv::Matx34f& P){

  if(pts3D.size() <= 7 || pts2D.size() <= 7 || pts3D.size() != pts2D.size()) {

      //something went wrong aligning 3D to 2D points..
      std::cerr << "couldn't find [enough] corresponding cloud points... (only " << pts3D.size() << ")"
                << std::endl;
      return false;
   }

  cv::Mat rvec, T;
  //cv::Mat inliers;
  std::vector<int> inliers;
  double minVal, maxVal;
  cv::minMaxIdx(pts2D, &minVal, &maxVal);
  //"solvePnPRansac"
  cv::solvePnPRansac(pts3D,pts2D,intrinsics.K,intrinsics.distCoef,rvec,T,true,1000,
                     0.006 * maxVal,0.99,inliers,CV_EPNP);

  std::vector<cv::Point2f> projected3D;
  cv::projectPoints(pts3D, rvec, T, intrinsics.K,intrinsics.distCoef, projected3D);

  if(inliers.size() == 0) { //get inliers
     for(int i = 0; i < projected3D.size(); i++) {
         if(cv::norm(projected3D[i] - pts2D[i]) < 10.0){
             inliers.push_back(i);

          }
     }
  }

  if(inliers.size() < (double)(pts2D.size()) / 5.0) {

      std::cerr << "not enough inliers to consider a good pose (" << inliers.size() << "/"
                << pts2D.size() << ")" << std::endl;
      return false;
  }

  if (cv::norm(T) > 200.0) {

      // this is bad...
      std::cerr << "estimated camera movement is too big, skip this camera\r\n";
      return false;
  }

  cv::Mat R;
  cv::Rodrigues(rvec, R);
  if(!CheckCoherentRotation(R)) {

     std::cerr << "rotation is incoherent. we should try a different base view..." << std::endl;
     return false;
  }

  std::cout << "found t = " << "\n"<< T << "\nR = \n" << R << std::endl;

  //Rotational element in a 3x4 matrix
  const cv::Rect ROT(0, 0, 3, 3);

  //Translational element in a 3x4 matrix
  const cv::Rect TRA(3, 0, 1, 3);

  R.copyTo(cv::Mat(3, 4, CV_32FC1, P.val)(ROT));
  T.copyTo(cv::Mat(3, 4, CV_32FC1, P.val)(TRA));

  return true;

}




void StructFromMotion::mergeNewPoints(const std::vector<Point3D>& newPointCloud) {

  std::cout << "Adding new points..." << std::flush;

  const float MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE   = 0.01;
  const float MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE = 20.0;

      size_t newPoints = 0;
      size_t mergedPoints = 0;

      for (const Point3D& p : newPointCloud) {
          const cv::Point3f newPoint = p.pt; //new 3D point

          bool foundAnyMatchingExistingViews = false;
          bool foundMatching3DPoint = false;
          for (Point3D& existingPoint : nReconstructionCloud) {
              if(cv::norm(existingPoint.pt - newPoint) < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE) {
                  //This point is very close to an existing 3D cloud point
                  foundMatching3DPoint = true;

                  //Look for common 2D features to confirm match
                  for(const std::pair<const int,int>& newKv : p.idxImage) {
                      //kv.first = new point's originating view
                      //kv.second = new point's view 2D feature index

                      for (const std::pair<const int,int>& existingKv : existingPoint.idxImage){
                          //existingKv.first = existing point's originating view
                          //existingKv.second = existing point's view 2D feature index

                          bool foundMatchingFeature = false;

                          const bool newIsLeft = newKv.first < existingKv.first;
                          const int leftViewIdx         = (newIsLeft) ? newKv.first  : existingKv.first;
                          const int leftViewFeatureIdx  = (newIsLeft) ? newKv.second : existingKv.second;
                          const int rightViewIdx        = (newIsLeft) ? existingKv.first  : newKv.first;
                          const int rightViewFeatureIdx = (newIsLeft) ? existingKv.second : newKv.second;

                          const Matching matching = getMatching(nFeatureImages[leftViewIdx],
                                                             nFeatureImages[rightViewIdx]);
                          for (const cv::DMatch& match : matching) {
                              if (match.queryIdx == leftViewFeatureIdx
                                  and match.trainIdx == rightViewFeatureIdx
                                  and match.distance < MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE) {

                                  //Found a 2D feature match for the two 3D points - merge
                                  foundMatchingFeature = true;
                                  break;
                              }
                          }

                          if(foundMatchingFeature) {
                              //Add the new originating view, and feature index
                              existingPoint.idxImage[newKv.first] = newKv.second;                              
                              foundAnyMatchingExistingViews = true;
                              break;

                          }
                      }
                  }
              }
              if(foundAnyMatchingExistingViews) {
                  mergedPoints++;
                  break; //Stop looking for more matching cloud points
              }
          }

          if(not foundAnyMatchingExistingViews and not foundMatching3DPoint) {
              //This point did not match any existing cloud points - add it as new.
              nReconstructionCloud.push_back(p);
              std::vector<Point3D> temp = nReconstructionCloud;
              GetRGBForPointCloud(temp,nReconstructionCloudRGB);
              newPoints++;
          }
      }

      std::cout << "[DONE]" << std::endl;
      std::cout << "New points:" << newPoints << std::endl;




/*
  const float ERROR_REFERENCE   = 0.01;

          for(const Point3D& newPoint3D : newPointCloud) {

               for(Point3D& existingPoint3D : nReconstructionCloud) {

                 double error_distance = cv::norm(existingPoint3D.pt - newPoint3D.pt);

                 bool foundAnyMatchingExistingViews = false;
                 bool foundMatchingFeature = false;

                 if(error_distance < ERROR_REFERENCE){

                     //This point is very close to an existing 3D cloud point

                     //Look for common 2D Feature to confirm match
                     for(const std::pair<const int,int>& newKv : newPoint3D.idxImage){
                         //kv.first = new point's originating view
                         //kv.second = new point's view 2D feature index

                         for(const std::pair<const int,int>& existingKv : existingPoint3D.idxImage){
                             //existingKv.first = existing point's originating view
                             //existingKv.second = existing point's view 2D feature index

                             const int leftViewIdx  = existingKv.first;
                             const int rightViewIdx = newKv.first;
                             const int leftViewFeatureIdx = existingKv.second;
                             const int rightViewFeatureIdx = newKv.second;

                             if(leftViewIdx == rightViewIdx
                                 and leftViewFeatureIdx == rightViewFeatureIdx){

                                     //Found a 2D feature match for the two 3D points - merge
                                     foundMatchingFeature = true;
                                     //Add the new originating view, and feature index
                                     for(const std::pair<const int,int>& newKv :newPoint3D.idxImage){
                                         existingPoint3D.idxImage[newKv.first] = newKv.second;
                                       }

                                     foundAnyMatchingExistingViews = true;
                                     break;
                               }
                           }
                    }


                 }

                if(not foundAnyMatchingExistingViews and not foundMatchingFeature) {
                         //This point did not match any existing cloud points - add it as new.
                         nReconstructionCloud.push_back(newPoint3D);
                         std::vector<Point3D> temp = nReconstructionCloud;
                         //GetRGBForPointCloud(temp,nReconstructionCloudRGB);
                         //pclVisualizer.addPointCloudToPCL(temp,nReconstructionCloudRGB);
                         break;
                 }


                 if(foundAnyMatchingExistingViews) {
                    continue; //Stop looking for more matching cloud points
                 }


                }

              }
*/
}




void StructFromMotion::saveCloudAndCamerasToPLY() {

     std::ofstream ofs("pointcloud.ply");
     std::cout << "Saving result reconstruction with prefix:" << " pointcloud.ply" << std::endl;

    //write PLY header
    ofs << "ply                 " << std::endl <<
           "format ascii 1.0    " << std::endl <<
           "element vertex " << nReconstructionCloud.size() << std::endl <<
           "property float x    " << std::endl <<
           "property float y    " << std::endl <<
           "property float z    " << std::endl <<
           "property uchar red  " << std::endl <<
           "property uchar green" << std::endl <<
           "property uchar blue " << std::endl <<
           "end_header          " << std::endl;

    for (const Point3D& p : nReconstructionCloud) {
        //get color from first originating view
                auto originatingView = p.idxImage.begin();
                const int viewIdx = originatingView->first;
                cv::Point2f p2d = nFeatureImages[viewIdx].pt2D[originatingView->second];
                cv::Vec3b pointColor = nImages[viewIdx].at<cv::Vec3b>(p2d);

                //write vertex
        ofs << p.pt.x              << " " <<
                   p.pt.y              << " " <<
                           p.pt.z              << " " <<
                           (int)pointColor(2) << " " <<
                           (int)pointColor(1) << " " <<
                           (int)pointColor(0) << " " << std::endl;
    }

    std::cout << "Saved " << nReconstructionCloud.size () << " data points to pointcloud.ply" << std::endl;
    ofs.close();
    std::ofstream ofsc("cameras.ply");

    //write PLY header
    ofsc << "ply                 " << std::endl <<
           "format ascii 1.0    " << std::endl <<
           "element vertex " << (nCameraPoses.size() * 4) << std::endl <<
           "property float x    " << std::endl <<
           "property float y    " << std::endl <<
           "property float z    " << std::endl <<
           "element edge " << (nCameraPoses.size() * 3) << std::endl <<
           "property int vertex1" << std::endl <<
           "property int vertex2" << std::endl <<
           "property uchar red  " << std::endl <<
           "property uchar green" << std::endl <<
           "property uchar blue " << std::endl <<
           "end_header          " << std::endl;

    //save cameras polygons..
    for (const cv::Matx34f& pose : nCameraPoses) {
        cv::Point3d c(pose(0, 3), pose(1, 3), pose(2, 3));
        cv::Point3d cx = c + cv::Point3d(pose(0, 0), pose(1, 0), pose(2, 0)) * 0.2;
        cv::Point3d cy = c + cv::Point3d(pose(0, 1), pose(1, 1), pose(2, 1)) * 0.2;
        cv::Point3d cz = c + cv::Point3d(pose(0, 2), pose(1, 2), pose(2, 2)) * 0.2;

        ofsc << c.x  << " " << c.y  << " " << c.z  << std::endl;
        ofsc << cx.x << " " << cx.y << " " << cx.z << std::endl;
        ofsc << cy.x << " " << cy.y << " " << cy.z << std::endl;
        ofsc << cz.x << " " << cz.y << " " << cz.z << std::endl;
    }

    for (size_t i = 0; i < nCameraPoses.size(); i++) {
        ofsc << (i * 4 + 0) << " " <<
                (i * 4 + 1) << " " <<
                "255 0 0" << std::endl;
        ofsc << (i * 4 + 0) << " " <<
                (i * 4 + 2) << " " <<
                "0 255 0" << std::endl;
        ofsc << (i * 4 + 0) << " " <<
                (i * 4 + 3) << " " <<
                "0 0 255" << std::endl;
    }
}

void StructFromMotion::saveCloudToPCD(){
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  std::cout << "Saving result reconstruction with prefix:" << " pointcloud.pcd" << std::endl;

   for(size_t i = 0; i < nReconstructionCloud.size(); ++i){
       Point3D pt3d = nReconstructionCloud[i];
       cv::Vec3b rgbv(255,255,255);
       pcl::PointXYZRGB pclp;
       pclp.x  = pt3d.pt.x;
       pclp.y  = pt3d.pt.y;
       pclp.z  = pt3d.pt.z;
       rgbv = nReconstructionCloudRGB[i];

       // RGB color, needs to be represented as an integer
       uint32_t rgb = ((uint32_t)rgbv[2] << 16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);
       pclp.rgb = *reinterpret_cast<float*>(&rgb);
       cloud.points.push_back(pclp);
    }

    cloud.width = (uint32_t) cloud.points.size(); // number of points
    cloud.height = 1;	// a list, one row of data
    cloud.header.frame_id ="map";
    cloud.is_dense = false;

   pcl::io::savePCDFileASCII ("pointcloud.pcd", cloud);
   std::cout << "Saved " << cloud.points.size () << " data points to pointcloud.pcd" << std::endl;

}

void StructFromMotion::GetRGBForPointCloud(std::vector<Point3D>& _pcloud,
                                           std::vector<cv::Vec3b>& RGBforCloud){

    RGBforCloud.resize(_pcloud.size());
     for (unsigned int i = 0; i < _pcloud.size(); i++) {
             unsigned int good_view = 0;
             std::vector<cv::Vec3b> point_colors;
             for (; good_view < nImages.size(); good_view++) {

                        if(_pcloud[i].idxImage[good_view] != -1) {
                                int pt_idx = _pcloud[i].idxImage[good_view];
                                if(pt_idx >= nFeatureImages[good_view].pt2D.size()) {
                                   std::cerr << "BUG: point id:" << pt_idx
                                           << " should not exist for img #" << good_view
                                            << " which has only " << nFeatureImages[good_view].pt2D.size()
                                            << std::endl;
                                        continue;
                                 }
                                 cv::Point _pt = nFeatureImages[good_view].pt2D[pt_idx];
                                assert(good_view < nImages.size() && _pt.x < nImages[good_view].cols && _pt.y < nImages[good_view].rows);

					point_colors.push_back(nImages[good_view].at<cv::Vec3b>(_pt));
				}
			}

			cv::Scalar res_color = cv::mean(point_colors);
			RGBforCloud[i] = (cv::Vec3b(res_color[0], res_color[1], res_color[2])); //bgr2rgb
			if(good_view == nImages.size()){ //nothing found.. put red dot
			   RGBforCloud[i] = (cv::Vec3b(res_color[0], res_color[1], res_color[2]));

                }
       }

}

Matching StructFromMotion::matchingFor2D3D(Feature& feature1,Feature& feature2){

  cv::Ptr<cv::DescriptorMatcher> matcherFlan = cv::DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<cv::DMatch> matches12,matches21,buenosMatches;
  matcherFlan ->match(feature1.descriptors,feature2.descriptors,matches12);
  matcherFlan ->match(feature2.descriptors,feature1.descriptors,matches21);

  for (size_t i=0; i < matches12.size(); i++){
      cv::DMatch forward = matches12[i];
      cv::DMatch backward = matches21[forward.trainIdx];
      if(backward.trainIdx==forward.queryIdx){
          buenosMatches.push_back(forward);
      }
    }

return buenosMatches;

}

void StructFromMotion::createPointsTxt(){

  std::cout << "Saving points for bundle with prefix:" << "points.txt" << std::endl;
  std::ofstream ofs("points.txt");
  for(const Point3D& pt3d : nReconstructionCloud) {

      ofs << pt3d.pt.x << " " << pt3d.pt.y << " " <<  pt3d.pt.z << " ";
      int cont = 1;

      for(const std::pair<const int,int> originatingView : pt3d.idxImage){

          if(cont ==1){
              ofs << std::tuple_size<decltype(originatingView)>::value << " ";
          }

          const int viewIdx = originatingView.first;
          const cv::Point2f p2d = nFeatureImages[viewIdx].pt2D[originatingView.second];

          ofs << viewIdx << " " << p2d.x << " " << p2d.y << " ";
          cont +=1;
        }
      ofs << std::endl;
  }
 ofs.close();
}

void StructFromMotion::PMVS2(){

  /*FOLDERS FOR PMVS2*/
  std::cout << "Creating folders for PMVS2..." << std::endl;
  std::system("mkdir -p denseCloud/visualize");
  std::system("mkdir -p denseCloud/txt");
  std::system("mkdir -p denseCloud/models");
  std::cout << "Created: \nvisualize" << "\n" << "txt" << "\n" << "models" << std::endl;

  /*OPTIONS CONFIGURATION FILE FOR PMVS2*/
  std::cout << "Creating options file for PMVS2..." << std::endl;
  ofstream option("denseCloud/options.txt");
  option << "timages  -1 " << 0 << " " << (nImages.size()-1) << std::endl;
  option << "oimages 0" << std::endl;
  option << "level 1" << std::endl;
  option.close();
  std::cout << "Created: options.txt" << std::endl;

  /*CAMERA POSES AND IMAGES INPUT FOR PMVS2*/
  std::cout << "Saving camera poses for PMVS2..." << std::endl;
  std::cout << "Saving camera images for PMVS2..." << std::endl;
  for(int i=0; i<nCameraPoses.size(); i++) {

      /*
      cv::Matx33f R = pose.get_minor<3, 3>(0, 0);
      Eigen::Map<Eigen::Matrix3f> R_eigen(R.val);
      Eigen::Quaternionf q(R_eigen);
      */

      char str[256];
      boost::filesystem::directory_entry x(nImagesPath[i]);
      std::string extension = x.path().extension().string();
      boost::algorithm::to_lower(extension);
      if(extension == ".jpg"){
          std::sprintf(str, "cp -f %s denseCloud/visualize/%04d.jpg", nImagesPath[i].c_str(), (int)i);
          std::system(str);
          cv::imwrite(str, nImages[i]);
      }else if(extension == ".png"){
          std::sprintf(str, "cp -f %s denseCloud/visualize/%04d.png", nImagesPath[i].c_str(), (int)i);
          std::system(str);
          cv::imwrite(str, nImages[i]);
      }else{
          std::cerr << "Sorry. Only .jpg and .png is supported." << std::endl;
          std::exit(-1);
      }

      std::sprintf(str, "denseCloud/txt/%04d.txt", (int)i);
      ofstream ofs(str);
      cv::Matx34f pose = nCameraPoses[i];

      //K*P
      pose = (cv::Matx33f)cameraMatrix.K*pose;

      ofs << "CONTOUR" << std::endl;
      ofs << pose(0,0) << " " << pose(0,1) << " " << pose(0,2) << " " << pose(0,3) << "\n"
          << pose(1,0) << " " << pose(1,1) << " " << pose(1,2) << " " << pose(1,3) << "\n"
          << pose(2,0) << " " << pose(2,1) << " " << pose(2,2) << " " << pose(2,3) << std::endl;

      ofs << std::endl;
      ofs.close();
  } 

  std::cout << "Camera poses saved." << "\n" << "Camera images saved." << std::endl;
  std::cout << "You can now run pmvs2 on the results: PATH_TO_PMVS_BINARY/pmvs2 options.txt"
            << std::endl;
}




