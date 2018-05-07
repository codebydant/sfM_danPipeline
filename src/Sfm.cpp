//***********************************************
//HEADERS
//***********************************************

#include "include/Sfm.h"

/********************************************
                  PIPELINE
********************************************/
void StructFromMotion::pipeLineSFM(){

  std::cout << "************************************************" << std::endl;
  std::cout << "              3D RECONSTRUCTION                 " << std::endl;
  std::cout << "************************************************" << std::endl;

  if(nImages.size() <= 0) {
      std::cerr << "No images to work on." << std::endl;
      std::exit(-1);
  }

  nCameraPoses.resize(nImages.size());
  bool success = false;

  // **(1) FEATURE DETECTION AND EXTRACTION - ALL IMAGES
  success = extractFeature();
  if(not success){
      std::cerr << "No could find features. corrupt images" << std::endl;
      std::exit(-1);
  }

  // **(2) PRINT INPUT IMAGES
  for(unsigned int i=0;i<nImages.size();i++){
      cv::namedWindow("Input images");
      cv::resizeWindow("Input images",nImages[i].rows,nImages[i].cols);
      cv::moveWindow("Input images",700,0);
      cv::imshow("Input images",nImages[i]);
      cv::waitKey(100);

  }
  cv::destroyWindow("Input images");

  // **(3) BASE RECONSTRUCTION
  success= baseTriangulation();
  if(not success){
      std::cerr << "No could find a good pair for initial reconstruction" << std::endl;
      std::exit(-1);
  }

  // **(4) ADD MORE VIEWS
  addMoreViews();

  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

 //saveCloudAndCamerasToPLY("temple");
  saveCloudToPCD();

}

/********************************************
 FUNCTIONS
********************************************/

//===============================================
//MULTITHREADING FUNCTION
//===============================================

void StructFromMotion::run_SFM (){

   std::thread first([&] {pclVisualizer.showPCLVisualizer(); });
   std::thread second([&] {pipeLineSFM(); });

   //synchronize threads:
   first.join();     // pauses until first finishes
   second.join();    // pauses until second finishes

}

//===============================================
//PCL VISUALIZER
//===============================================

void StructFromMotion::loadVisualizer(){

   pcl::visualization::PCLVisualizer viewer=pcl::visualization::PCLVisualizer("3D Reconstruction",true);
   viewer.setPosition(0,0);
   viewer.setSize(800,600);

   viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
   viewer.resetCamera();
   viewer.setCameraPosition(0,0,7,0,0,0);

   while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed

       viewer.addCoordinateSystem (1.0, "cloud", 0);
       viewer.removeAllPointClouds();
       pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL (new pcl::PointCloud<pcl::PointXYZ> ());
//cloudPCL.reset(new pcl::PointCloud<pcl::PointXYZ> ());


       // Fill in the cloud data
       cloudPCL->width    = nReconstructionCloud.size();
       cloudPCL->height   = 1;
       cloudPCL->is_dense = false;
       cloudPCL->points.resize(cloudPCL->width * cloudPCL->height);

       for (size_t i = 0; i < cloudPCL->points.size (); ++i){
          Point3D pt3d = nReconstructionCloud[i];
          cloudPCL->points[i].x = pt3d.pt.x;
          cloudPCL->points[i].y = pt3d.pt.y;
          cloudPCL->points[i].z = pt3d.pt.z;
       }

       // Define R,G,B colors for the point cloud
       pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloudPCL, 255, 255, 255);
       //We add the point cloud to the viewer and pass the color handler

       viewer.addPointCloud (cloudPCL, cloud_color, "original_cloud");
       viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");

       viewer.spinOnce(500);
   }
}

//===============================================
//IMAGES LOAD
//===============================================

bool StructFromMotion::imagesLOAD(const std::string&  directoryPath){

  std::cout << "Getting images..." << std::flush;
  boost::filesystem::path dirPath(directoryPath);

      if (not boost::filesystem::exists(dirPath) or not boost::filesystem::is_directory(dirPath)) {
          std::cerr << "Cannot open directory: " << directoryPath << std::endl;
          return false;
      }


      for(boost::filesystem::directory_entry& x : boost::filesystem::directory_iterator(dirPath)) {
          std::string extension = x.path().extension().string();
          boost::algorithm::to_lower(extension);
          if (extension == ".jpg" or extension == ".png") {

              nImagesPath.push_back(x.path().string());
          }
      }

      std::sort(nImagesPath.begin(), nImagesPath.end());

      if (nImagesPath.size() <= 0) {
          std::cerr << "Unable to find valid files in images directory (\"" << directoryPath << "\")." << std::endl;
          return false;
      }else{

         std::cout << "Found " << nImagesPath.size() << " image files in directory." << std::endl;
      }

      for (auto& imageFilename : nImagesPath) {
          cv::Mat img   = cv::imread(imageFilename,cv::IMREAD_COLOR);
          cv::Mat temp = img.clone();
          cv::Mat resize;
          cv::resize(temp,resize,cv::Size(),0.75,0.75);
          cv::GaussianBlur(resize,temp, cv::Size(3,3),0,0);
          nImages.push_back(temp);

          if (nImages.back().empty()) {
              std::cerr << "[x]"<<"\n" <<"Unable to read image from file: " << imageFilename << std::endl;
              return false;
          }
      }

  return true;  
}

//===============================================
//FUNCTION: OBTENER Feature
//===============================================

Feature StructFromMotion::getFeature(const cv::Mat& image) {
    Feature Feature;
    ptrFeature2D->detect(image,Feature.kps);
    ptrFeature2D->compute(image,Feature.kps,Feature.descriptors);
    keypoints2F(Feature.kps,Feature.pt2D);
    return Feature;
}

//===============================================
//FUNCTION: EXTRACT Feature
//===============================================

bool StructFromMotion::extractFeature(){

  std::cout << "Getting Feature from all images..." << std::endl;
  nFeatureImages.resize(nImages.size());
  for(size_t n=0;n<nImages.size();n++){

      nFeatureImages[n] = getFeature(nImages[n]);
   }  
  std::cout << "Total Feature = " << nFeatureImages.size() << std::endl;
  if(nFeatureImages.empty()){
      return false;
    }else{
      return true;
    }
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

Matching StructFromMotion::getMatching(const Feature& left,const Feature& right){

  /*
  *(RATIO-TEST)
  */
  Matching goodMatches;

  //initial matching between Feature
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

    cv::namedWindow(str,cv::WINDOW_NORMAL);    
    cv::resizeWindow(str,800,400);
    cv::moveWindow(str,0,0);
    cv::imshow(str,matchImage);
    cv::waitKey(0);
}

//===============================================
//FUNCTION: GET CAMERA MATRIX
//===============================================

void StructFromMotion::getCameraMatrix(const std::string str){

    std::cout << "Getting camera matrix..." << std::flush;

    cv::Mat intrinsics;
    cv::Mat cameraDistCoeffs;
    cv::FileStorage fs(str, cv::FileStorage::READ);
    fs["Camera_Matrix"] >> intrinsics;
    fs["Distortion_Coefficients"] >> cameraDistCoeffs;

    std::cout << "[DONE]" << std::endl;

    if(intrinsics.empty()){
        std::cerr << "Error: no found or invalid camera calibration file.xml" << std::endl;
        std::exit(-1);
    }

    cv::Matx33f cMatrix(intrinsics);
    std::vector<double> coefVec(cameraDistCoeffs);

    cameraMatrix.K = cv::Mat_<float>(intrinsics);
    cameraMatrix.distCoef = cv::Mat_<float>::zeros(1, 4);
    cameraMatrix.invK = inverse(cameraMatrix.K);
    cameraMatrix.K3x3 = cMatrix;
    cameraMatrix.distCoefVec = coefVec;
    cameraMatrix.fx = cameraMatrix.K.at<float>(0,0);
    cameraMatrix.fy = cameraMatrix.K.at<float>(1,1);
    cameraMatrix.cx = cameraMatrix.K.at<float>(0,2);
    cameraMatrix.cy = cameraMatrix.K.at<float>(1,2);

    std::cout << "Camera matrix:" << "\n" << cameraMatrix.K << std::endl;
    std::cout <<"Distortion coefficients: "<< std::endl;
    std::cout << "[";

    for(size_t n=0;n<cameraMatrix.distCoefVec.size();n++){

         std::cout << cameraMatrix.distCoefVec.at(n);

         if(n<cameraMatrix.distCoefVec.size()-1){
             std::cout << ",";
           }else{
             continue;
           }
       }
    std::cout <<"]" << std::endl;
}

//===============================================
//FUNCTION: CHECK ROTATION MATRIX (Must be det=1)
//===============================================

bool StructFromMotion::CheckCoherentRotation(cv::Mat& R){

   if(fabsf(determinante(R))-1.0 > 1e-07) {

      std::cout << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
      return false;
    }else {

       return true;
    }
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
//FUNCTION: ALIGNED POINTS
//===============================================

void StructFromMotion::AlignedPointsFromMatch(const Feature& left,const Feature& right,const Matching& matches,Feature& alignedL,Feature& alignedR){

   std::vector<int> leftId,rightId;
   AlignedPoints(left,right,matches,alignedL,alignedR,leftId,rightId);

}

void StructFromMotion::AlignedPoints(const Feature& left,const Feature& right,const Matching& matches, Feature& alignedL, Feature& alignedR,std::vector<int>& idLeftOrigen,std::vector<int>& idRightOrigen){

      //align left and right point sets
      for(unsigned int i=0;i<matches.size();i++){

        alignedL.kps.push_back(left.kps[matches[i].queryIdx]);
        alignedL.descriptors.push_back(left.descriptors.row(matches[i].queryIdx));
        alignedL.pt2D.push_back(left.pt2D[matches[i].queryIdx]);

        alignedR.kps.push_back(right.kps[matches[i].trainIdx]);
        alignedR.descriptors.push_back(right.descriptors.row(matches[i].trainIdx));
        alignedR.pt2D.push_back(right.pt2D[matches[i].trainIdx]);

        idLeftOrigen.push_back(matches[i].queryIdx);
        idRightOrigen.push_back(matches[i].trainIdx);
      }
}

//===============================================
//FUNCTION: BASE RECONSTRUCTION
//===============================================

bool StructFromMotion::baseTriangulation(){

  std::cout << "Getting best two views for first reconstruction..." << std::flush;
  std::map<int,ImagePair> bestViews = findBestPair();  
  std::cout << "[DONE]" << std::endl;

  for(std::pair<const int,ImagePair>& pair : bestViews){

      int leftImage = pair.second.left;
      int rightImage = pair.second.right;

      std::cout << "best pair:" << " image:(" << leftImage << ") and image:("
                << rightImage << ")" << std::endl;

      Matching prunedMatching,correspondences;
      cv::Matx34f Pleft  = cv::Matx34f::eye();
      cv::Matx34f Pright = cv::Matx34f::eye();

      std::cout << "best pair:" << " image:(" <<leftImage << ") and image:("<<rightImage <<")" << std::endl;

      std::cout << "Getting camera pose..." << std::flush;
      correspondences = getMatching(nFeatureImages[leftImage],nFeatureImages[rightImage]);

      bool success = getCameraPose(cameraMatrix,correspondences,
                                                nFeatureImages[leftImage],nFeatureImages[rightImage],
                                                prunedMatching, Pleft, Pright);
      std::cout << "[DONE]" << std::endl;

      if(not success) {

         std::cerr << "stereo view could not be obtained " << leftImage << "," << rightImage
                   << ", go to next pair" << std::endl;
         continue;
      }

      std::cout << "Showing matches between "<< "image:" << leftImage << " and image:"
                << rightImage << std::flush;

      cv::Mat outImg = imageMatching(nImages[leftImage],nFeatureImages[leftImage].kps,
                                     nImages[rightImage],nFeatureImages[rightImage].kps,prunedMatching);
      cv::resize(outImg, outImg, cv::Size(), 0.5, 0.5);
      cv::namedWindow("Best pair matching");
      cv::resizeWindow("Best pair matching",outImg.rows,outImg.cols);
      cv::putText(outImg, "Image " + std::to_string(leftImage) + "-Image" + std::to_string(rightImage),
                  cv::Point(10,50),cv::FONT_ITALIC,0.5,cv::Scalar(0,255,0),1);
      cv::moveWindow("Best pair matching",700,250);
      cv::imshow("Best pair matching", outImg);
      cv::waitKey(5000);
      cv::destroyWindow("Matching pairs");
      cv::destroyWindow("Best pair matching");

      // StructFromMotion::imShow(outImg,"Matching");

      std::cout << " [DONE]"<<std::endl;
      std::vector<Point3D> pointcloud;

      success = triangulateViews(nFeatureImages[leftImage],nFeatureImages[rightImage],
                                 Pleft,Pright,prunedMatching,cameraMatrix,pair.second,pointcloud);

      if(not success){

          std::cerr << "Could not triangulate image:" << leftImage << " and image:"<< rightImage << std::endl;
          continue;
      }

      nReconstructionCloud = pointcloud;
      nCameraPoses[leftImage] = Pleft;
      nCameraPoses[rightImage] = Pright;

      nDoneViews.insert(leftImage);
      nDoneViews.insert(rightImage);

      GetRGBForPointCloud(nReconstructionCloud,cloudRGB);
      pclVisualizer.addPointCloudToPCL(nReconstructionCloud,cloudRGB);


      break;
  }
  //adjustCurrentBundle() ;
  return true;
}

//===============================================
//FUNCTION: FIND BEST PAIR
//===============================================

std::map<int,ImagePair> StructFromMotion::findBestPair(){

  std::map<int,ImagePair> numInliers; 
  const size_t numImg = nImages.size();

  for(size_t i=0;i<numImg-1;i++) {

     for(size_t j=i+1;j<numImg;j++){

        Matching correspondences;
        correspondences = getMatching(nFeatureImages[i],nFeatureImages[j]);
        cv::Mat outImg = imageMatching(nImages[i],nFeatureImages[i].kps,
                                       nImages[j],nFeatureImages[j].kps,correspondences);
        cv::Mat temp;
        cv::resize(outImg, temp, cv::Size(), 0.5, 0.5);
        cv::namedWindow("Matching pairs");
        cv::resizeWindow("Matching pairs",temp.rows,temp.cols);
        cv::moveWindow("Matching pairs",700,0);
        cv::imshow("Matching pairs", temp);
        cv::waitKey(1);
        if(correspondences.size()<30){
            continue;
        }else{
            int N = findHomographyInliers(nFeatureImages[i],nFeatureImages[j],
                                                            correspondences);

            if(N < 60 or N>200){
              continue;
            }else{

              numInliers[N]={i,j};

            }
        }
     }

  }
  return numInliers;
}


//===============================================
//FUNCTION: FIND HOMOGRAPHY INLIERS
//===============================================

int StructFromMotion::findHomographyInliers(const Feature& f1,const Feature& f2,const Matching& matches){

  Feature alignedLeft,alignedRight;
  AlignedPointsFromMatch(f1,f2,matches,alignedLeft,alignedRight);

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

bool StructFromMotion::triangulateViews(const Feature& left,const Feature& right,const cv::Matx34f& P1,const cv::Matx34f& P2,const Matching& matches,const CameraData& matrixK,const ImagePair& pair,std::vector<Point3D>& pointcloud){

  std::cout << "** IMAGE COORDINATE - CAMERA COORDINATE CONVERTION **" << std::endl;

  Feature alignedLeft,alignedRight;
  std::vector<int> leftBackReference,rightBackReference;
  AlignedPoints(left,right,matches,alignedLeft,alignedRight,
                                  leftBackReference,rightBackReference);

  // NORMALIZE IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)
  std::cout << "Normalizing points..." << std::flush;
  cv::Mat normalizedLeftPts,normalizedRightPts; 
  cv::undistortPoints(alignedLeft.pt2D, normalizedLeftPts, matrixK.K, cv::Mat());
  cv::undistortPoints(alignedRight.pt2D, normalizedRightPts, matrixK.K, cv::Mat());
  std::cout << "[DONE]" << std::endl;

  // TRIANGULATE POINTS
  std::cout << "Triangulating points..." << std::flush;
  cv::Mat pts3dHomogeneous;
  cv::triangulatePoints(P1,P2,normalizedLeftPts,normalizedRightPts,pts3dHomogeneous);
  std::cout << "[DONE]" << std::endl;
  std::cout << "Points triangulate from --> " << "image: " << pair.left << " and image: "
            << pair.right << std::endl;

  std::cout << "** CAMERA COORDINATE - WORLD COORDINATE CONVERTION **" << std::endl;

  // CONVERTION CAMERA COORDINATE - WORLD COORDINATE
  std::cout << "Converting points to world coordinate..." << std::flush;
  cv::Mat pts3d;
  cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(),pts3d);
  std::cout << "[DONE]" << std::endl;

  cv::Mat rvecLeft;
  cv::Rodrigues(P1.get_minor<3,3>(0,0),rvecLeft);
  cv::Mat tvecLeft(P1.get_minor<3,1>(0,3));

  Points2f projectedLeft(alignedLeft.pt2D.size());
  cv::projectPoints(pts3d,rvecLeft,tvecLeft,matrixK.K,cv::Mat(),projectedLeft);

  cv::Mat rvecRight;
  cv::Rodrigues(P2.get_minor<3,3>(0,0),rvecRight);
  cv::Mat tvecRight(P2.get_minor<3,1>(0,3));

  Points2f projectedRight(alignedRight.pt2D.size());
  cv::projectPoints(pts3d,rvecRight,tvecRight,matrixK.K,cv::Mat(),projectedRight);

  std::cout << "Creating a pointcloud vector..." << std::flush;

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

          //use back reference to point to original Feature in images
          p.idxImage[pair.left]  = leftBackReference[i];
          p.idxImage[pair.right] = rightBackReference[i];
          p.id = i;

          pointcloud.push_back(p);
  }

  std::cout << "Pointcloud size = " << pointcloud.size() << std::endl;

  return true;
}

//===============================================
//FUNCTION: ADD MORE VIEWS
//===============================================

void StructFromMotion::addMoreViews(){

  while(nDoneViews.size() != 10){

      std::cout <<"\n"<< "===================================="<< std::endl;
      std::cout << "Adding more views..." << std::endl;
      std::cout << "Finding 2D-3D correspondences..." << std::endl;
      ImagePair pair;
      Pts3D2DPNP pts2D3D = find2D3DMatches(pair);

      size_t leftView = pair.left;
      size_t rightView = pair.right;
      std::cout << "The new frame:" << rightView << " is ready for been add" << std::endl;

      nDoneViews.insert(rightView);
      std::cout << "Add frame:("<< rightView << ")"<< std::endl;

      Points2f pts2D_PNP=pts2D3D.pts2D;
      Points3f pts3D_PNP=pts2D3D.pts3D;

      std::cout << "Finding new camera pose..." << std::flush;

      cv::Matx34f newCameraPose = cv::Matx34f::eye();
      findCameraPosePNP(cameraMatrix,pts3D_PNP,pts2D_PNP,newCameraPose);
      std::cout << "[DONE]" << std::endl;
      std::cout << "Add frame:("<< rightView << ")"<< " - New camera pose:"<< "\n"
                << newCameraPose << std::endl;

      nCameraPoses[rightView]=newCameraPose;

      std::vector<Point3D> pointcloud;
      std::cout << "Triangulating points..." << std::flush;

      bool success;
      const Matching newMatch = getMatching(nFeatureImages[leftView],nFeatureImages[rightView]);
      Matching prunedMatching;
      cv::Matx34f Pleft=  cv::Matx34f::eye();
      cv::Matx34f Pright = cv::Matx34f::eye();

      StructFromMotion::getCameraPose(cameraMatrix,newMatch,nFeatureImages[leftView],
                                      nFeatureImages[rightView],prunedMatching,Pleft,Pright);

   success = triangulateViews(nFeatureImages[leftView],nFeatureImages[rightView],
                                                nCameraPoses[leftView],nCameraPoses[rightView],
                                                prunedMatching,cameraMatrix,
                                                {leftView,rightView},pointcloud);

   std::cout << "[DONE]" << std::endl;
   std::cout << "Adding new pointcloud..." << std::flush;
   mergeNewPoints(pointcloud);
   std::cout << "[DONE]" << std::endl;


 }

adjustCurrentBundle() ;

 std::cout << "\n"<< "=============================== " << std::endl;
 std::cout << "Images processed = " << nDoneViews.size() << " of " << nImages.size() << std::endl;
 std::cout << "PointCloud size =" << nReconstructionCloud.size() << std::endl;
}

//===============================================
//FUNCTION: FIND CORRESPONDENCES 2D-3D
//===============================================

Pts3D2DPNP StructFromMotion::find2D3DMatches(ImagePair& pair){

  Pts3D2DPNP matches2D3D;
     std::map<int,ImagePair> matchesSizes;

     //Buscar si el frame N está en la nube de puntos
     for(size_t newFrame=0;newFrame<nImages.size();newFrame++){

         std::cout << "Finding best frame to add!" << std::endl;

        if(nDoneViews.count(newFrame)== 1){          
          std::cerr << "Frame:" << newFrame << " is already add" << std::endl;
          continue; //Pase al siguiente frame
        }

        for(size_t framePC:nDoneViews){

            const Matching Match = getMatching(nFeatureImages[framePC],nFeatureImages[newFrame]);
            const size_t bestSizeMatches = Match.size();
            matchesSizes[bestSizeMatches]={framePC,newFrame};
            continue;
        }

        std::map<int,ImagePair>::const_iterator pos = std::prev(matchesSizes.end());
        const size_t bestMatchSize = pos->first;

        size_t left = pos->second.left;
        size_t right = pos->second.right;

        std::cout << "New frame to add: " << right << std::endl;
        std::cout << "Verifying if number of matches is enough..." << std::flush;

        if(bestMatchSize < 60){
           matchesSizes.clear();
           std::cerr << "[X]" << std::endl;
           continue;
        }
        std::cout << "[OK]" << std::endl;
        std::cout << "Found "<< bestMatchSize << " matches between frame:"
                            << left << " and new frame:" << right << std::endl;
        std::cout << "Finding 2D points of new frame that match with POINTCLOUD!" << std::endl;
        pair={left,right};

        std::set<int> nDonePts;
        const Matching bestMatch = getMatching(nFeatureImages[left],nFeatureImages[right]);

        for(const cv::DMatch& match_index : bestMatch){

          for(Point3D numPt3D : nReconstructionCloud){

             if(nDonePts.count(numPt3D.id) == 1){
                 continue;
             }             

             if(numPt3D.idxImage.count(left)==0){
                continue;
             }


             if(match_index.queryIdx != numPt3D.idxImage[left]){
                continue;
             }

             matches2D3D.pts3D.push_back(numPt3D.pt);
             matches2D3D.pts2D.push_back(nFeatureImages[right].pt2D[match_index.trainIdx]);
             nDonePts.insert(numPt3D.id);
             break;

         }//End for-(vector point3D comparison)
       }//End for-(best matches vector comparison)

       if(matches2D3D.pts2D.size()< 50){

           std::cerr << "Not found enough points for PnPRansac, found: "
                     << matches2D3D.pts2D.size() << std::endl;
           std::cout << "\n"<< "=============================== " << std::endl;
           matchesSizes.clear();
           continue;
       }

        std::cout << "Found: " << matches2D3D.pts2D.size() << " Pt2D and "
                     << matches2D3D.pts3D.size() << " Pt3D" << std::endl;
         break;

    }//End for-(NewFrames)

   return matches2D3D;
}

//===============================================
//FUNCTION: FIND CAMERA POSE PNP RANSAC
//===============================================

void StructFromMotion::findCameraPosePNP(const CameraData& intrinsics,const std::vector<cv::Point3f>& pts3D,const std::vector<cv::Point2f>& pts2D,cv::Matx34f& P){

  cv::Mat rvec, T;
  cv::Mat inliers;
  double RANSAC_THRESHOLD=0.99f;

  cv::solvePnPRansac(pts3D,pts2D,intrinsics.K,intrinsics.distCoef,rvec,T,false,100,
                     RANSAC_THRESHOLD,0.99,inliers);

  cv::Mat R;
  cv::Rodrigues(rvec, R); //convert to a rotation matrix

  //Rotational element in a 3x4 matrix
  const cv::Rect ROT(0, 0, 3, 3);

  //Translational element in a 3x4 matrix
  const cv::Rect TRA(3, 0, 1, 3);

  R.copyTo(cv::Mat(3, 4, CV_32FC1, P.val)(ROT));
  T.copyTo(cv::Mat(3, 4, CV_32FC1, P.val)(TRA));

}


void StructFromMotion::adjustCurrentBundle() {
    //adjustBundle(nReconstructionCloud,nCameraPoses,cameraMatrix,nFeatureImages);

}

void StructFromMotion::mergeNewPoints(const std::vector<Point3D>& newPointCloud) {

  const float ERROR_DISTANCE   = 0.01;

      for (const Point3D& p : newPointCloud) {
          const cv::Point3f newPoint = p.pt; //new 3D point

          bool foundAnyMatchingExistingViews = false;
          bool foundMatching3DPoint = false;
          for (Point3D& existingPoint : nReconstructionCloud) {
              if (cv::norm(existingPoint.pt - newPoint) < ERROR_DISTANCE) {
                  //This point is very close to an existing 3D cloud point
                  foundMatching3DPoint = true;

                  //Look for common 2D Feature to confirm match
                  for (const auto& newKv : p.idxImage) {
                      //kv.first = new point's originating view
                      //kv.second = new point's view 2D feature index

                      for (const auto& existingKv : existingPoint.idxImage) {
                          //existingKv.first = existing point's originating view
                          //existingKv.second = existing point's view 2D feature index

                          bool foundMatchingFeature = false;
                          const int leftViewIdx  = newKv.first;
                          const int rightViewIdx = existingKv.first;
                          const int leftViewFeatureIdx = newKv.second;
                          const int rightViewFeatureIdx = existingKv.second;


                          if (leftViewIdx == rightViewIdx
                              and leftViewFeatureIdx == rightViewFeatureIdx) {

                                  //Found a 2D feature match for the two 3D points - merge
                                  foundMatchingFeature = true;
                                  break;

                          }

                          if (foundMatchingFeature) {
                              //Add the new originating view, and feature index
                              existingPoint.idxImage[newKv.first] = newKv.second;

                              foundAnyMatchingExistingViews = true;

                          }
                      }
                  }
              }
              if (foundAnyMatchingExistingViews) {
                 break; //Stop looking for more matching cloud points
              }
          }

          if (not foundAnyMatchingExistingViews and not foundMatching3DPoint) {
              //This point did not match any existing cloud points - add it as new.
              nReconstructionCloud.push_back(p);
              GetRGBForPointCloud(nReconstructionCloud,cloudRGB);
              pclVisualizer.addPointCloudToPCL(nReconstructionCloud,cloudRGB);

          }
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
  cv::recoverPose(E,alignedLeft.pt2D, alignedRight.pt2D,R,T,intrinsics.fx,
                   cv::Point2d(intrinsics.cx,intrinsics.cy),mask);

  bool status = CheckCoherentRotation(R);

  Pleft  = cv::Matx34f::eye();

  if(status == true){

      Pright = cv::Matx34f(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), T.at<double>(0),
                           R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), T.at<double>(1),
                           R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), T.at<double>(2));
   }else{
      Pright = cv::Matx34f(0, 0, 0, 0,
                           0, 0, 0, 0,
                           0, 0, 0, 0);
   }

  std::cout << "Projection1:" << "\n" << Pleft << std::endl;
  std::cout << "Projection2:" << "\n" << Pright << std::endl;

  prunedMatch.clear();

  for (size_t i = 0; i < mask.rows; i++) {
     if(mask.at<uchar>(i)) {
           prunedMatch.push_back(matches[i]);
         }
    }

  return true;
}


void StructFromMotion::saveCloudAndCamerasToPLY(const std::string& prefix) {

     std::ofstream ofs(prefix + "_points.ply");
     std::cout << "Saving result reconstruction with prefix..." << prefix + "_points.ply" << std::endl;

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

    ofs.close();

    std::ofstream ofsc(prefix + "_cameras.ply");

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
    for (const auto& pose : nCameraPoses) {
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
  pcl::PointCloud<pcl::PointXYZ> cloud;

   // Fill in the cloud data
   cloud.width    = nReconstructionCloud.size();
   cloud.height   = 1;
   cloud.is_dense = false;
   cloud.points.resize(cloud.width * cloud.height);

   for (size_t i = 0; i < cloud.points.size (); ++i)
   {
       Point3D pt3d = nReconstructionCloud[i];
     cloud.points[i].x = pt3d.pt.x;
     cloud.points[i].y = pt3d.pt.y;
     cloud.points[i].z = pt3d.pt.z;
   }

   pcl::io::savePCDFileASCII ("temple.pcd", cloud);
   std::cout << "Saved " << cloud.points.size () << " data points to temple.pcd" << std::endl;


}



void StructFromMotion::meshingPointCloud(){
/*
  pcl::visualization::PCLVisualizer viewer=pcl::visualization::PCLVisualizer("Meshing",true);
  viewer.setPosition(0,0);
  viewer.setSize(800,600);

  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZ> ());

  // Fill in the cloud data
  cloudPCL->width    = nReconstructionCloud.size();
  cloudPCL->height   = 1;
  cloudPCL->is_dense = false;
  cloudPCL->points.resize(cloudPCL->width * cloudPCL->height);

  for (size_t i = 0; i < cloudPCL->points.size (); ++i){
     Point3D pt3d = nReconstructionCloud[i];
     cloudPCL->points[i].x = pt3d.pt.x;
     cloudPCL->points[i].y = pt3d.pt.y;
     cloudPCL->points[i].z = pt3d.pt.z;
  }

  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloudPCL, 255, 255, 255);
  //We add the point cloud to the viewer and pass the color handler


  // Normal estimation*
   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
   pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
   tree->setInputCloud (cloudPCL);
   n.setInputCloud (cloudPCL);
   n.setSearchMethod (tree);
   n.setKSearch (20);
   n.compute (*normals);
   //* normals should not contain the point normals + surface curvatures

   // Concatenate the XYZ and normal fields*
   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
   pcl::concatenateFields (*cloudPCL, *normals, *cloud_with_normals);
   //*cloud_with_normals = cloudPCL + normals;

   // Create search tree*
   pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
   tree2->setInputCloud (cloud_with_normals);

   // Initialize objects
   pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
   pcl::PolygonMesh triangles;

   // Set the maximum distance between connected points (maximum edge length)
   gp3.setSearchRadius (50);

   // Set typical values for the parameters
   gp3.setMu (2.5);
   gp3.setMaximumNearestNeighbors (100);
   gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
   gp3.setMinimumAngle(M_PI/18); // 10 degrees
   gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
   gp3.setNormalConsistency(false);

   // Get result
   gp3.setInputCloud (cloud_with_normals);
   gp3.setSearchMethod (tree2);
   gp3.reconstruct (triangles);

   // Additional vertex information
   std::vector<int> parts = gp3.getPartIDs();
   std::vector<int> states = gp3.getPointStates();


    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloudPCL,normals,10,0.05,"norm");
    viewer.addPolygonMesh(triangles,"meshing");

   while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed



       viewer.spin();
   }
*/
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

