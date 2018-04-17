//***********************************************
//HEADERS
//***********************************************

#include "include/Sfm.h"

/********************************************
                  PIPELINE
********************************************/

int StructFromMotion::run_SFM(std::ifstream& file){

  std::cout << "************************************************" << std::endl;
  std::cout << "              3D RECONSTRUCTION                 " << std::endl;
  std::cout << "************************************************" << std::endl;

  bool success = false;
  // **(0) GET CAMERA MATRIX
  success = StructFromMotion::getCameraMatrix();

  // **(1) IMAGES LOAD
  success = StructFromMotion::imagesLOAD(file);

  // **(2) FEATURE DETECTION AND EXTRACTION - ALL IMAGES
  success = StructFromMotion::extractFeatures();

  // **(3) BASE RECONSTRUCTION
  success= StructFromMotion::baseTriangulation();

  // **(4) ADD MORE VIEWS
  StructFromMotion::addMoreViews();

  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

  StructFromMotion::saveCloudAndCamerasToPLY("templePointCloud");

  // **(5) VISUALIZER POINTCLOUD
  StructFromMotion::visualizerPointCloud(nReconstructionCloud);

}

/********************************************
 FUNCTIONS
********************************************/

//===============================================
//FUNCTION: IMAGES LOAD
//===============================================

bool StructFromMotion::imagesLOAD(std::ifstream& file){

  std::cout << "Getting images..." << std::flush;

  nImages.clear();
  if (!file.is_open()) {
         std::cout << "There was a problem opening the file. No images loaded!" << std::endl;
         return false;
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
  nCameraPoses.resize(nImages.size());
  std::cout << "[DONE] "<< "\n"<< "Total images = "<< nImages.size() << std::endl;
  return true;
}

//===============================================
//FUNCTION: OBTENER FEATURES
//===============================================

Features StructFromMotion::getFeatures(const cv::Mat& image) {
    Features features;
    ptrFeature2D->detect(image,features.kps);
    ptrFeature2D->compute(image,features.kps,features.descriptors);
    keypoints2F(features.kps,features.pt2D);
    return features;
}

//===============================================
//FUNCTION: EXTRACT FEATURES
//===============================================

bool StructFromMotion::extractFeatures(){

  std::cout << "Getting features from all images..." << std::flush;
  nFeaturesImages.resize(nImages.size());
  for(size_t n=0;n<nImages.size();n++){

      nFeaturesImages[n] =StructFromMotion::getFeatures(nImages[n]);
   }

  std::cout << "[DONE]" << std::endl;
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

Matching StructFromMotion::getMatching(const Features& left,const Features& right){

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

bool StructFromMotion::getCameraMatrix(){

    std::cout << "Getting camera matrix..." << std::flush;
    cameraMatrix.K = (cv::Mat_<float>(3,3) << 1520.400000,    0,           302.320000,
                                                  0,    1525.900000,     246.870000,
                                                  0,        0 ,              1);
    cameraMatrix.fx = cameraMatrix.K.at<float>(0,0);
    cameraMatrix.fy = cameraMatrix.K.at<float>(1,1);
    cameraMatrix.cx = cameraMatrix.K.at<float>(0,2);
    cameraMatrix.cy = cameraMatrix.K.at<float>(1,2);
    //intrinsics.invK = StructFromMotion::inverse(intrinsics.K);

    std::cout << "[DONE]" << std::endl;
    std::cout << "matrix K:" << "\n" << cameraMatrix.K << std::endl;

     cv::Mat intrinsics;
     cv::Mat cameraDistCoeffs;
     cv::FileStorage fs("camera-calibration-data.xml", cv::FileStorage::READ);
     fs["Camera_Matrix"] >> intrinsics;
     fs["Distortion_Coefficients"] >> cameraDistCoeffs;
     //cv::Matx33d cMatrix(cameraMatrix);
     std::vector<double> cMatrixCoef(cameraDistCoeffs);
     //cameraMatrix.K = intrinsics;
     cameraMatrix.distCoef = cv::Mat_<float>::zeros(1, 4);

     /*
     std::cout <<"Vector distortion coeff: "<< std::endl;
     std::cout << "[ ";
     intrinsics.distCoef = cameraDistCoeffs;
     for(size_t n=0;n<cMatrixCoef.size();n++){

         std::cout << cMatrixCoef.at(n) << ",";
       }
     std::cout <<"]" << std::endl;
     */
     if(cameraMatrix.K.empty()){
         std::cout << "Error: no found or invalid camera calibration file.xml" << std::endl;
         std::exit(-1);
     }
     return true;
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

void StructFromMotion::AlignedPointsFromMatch(const Features& left,const Features& right,const Matching& matches,Features& alignedL,Features& alignedR){

   std::vector<int> leftId,rightId;
   StructFromMotion::AlignedPoints(left,right,matches,alignedL,alignedR,leftId,rightId);

}

void StructFromMotion::AlignedPoints(const Features& left,const Features& right,const Matching& matches, Features& alignedL, Features& alignedR,std::vector<int>& idLeftOrigen,std::vector<int>& idRightOrigen){

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

     // StructFromMotion::keypoints2F(alignedL.kps,alignedL.pt2D);
     // StructFromMotion::keypoints2F(alignedR.kps,alignedR.pt2D);
}


//===============================================
//FUNCTION: POINTCLOUD VISUALIZER
//===============================================

void StructFromMotion::visualizerPointCloud(const std::vector<Point3D>& pointcloud){

  std::cout << "visualizing pointcloud..." << std::flush;
  cv::viz::Viz3d visualizer("Viz window");
  cv::viz::WCoordinateSystem ucs(0.5);

  Points3f cloud;
  for(size_t i=0;i<pointcloud.size();i++){
      cloud.push_back(pointcloud[i].pt);
  }

  cv::viz::WCloud point3d(cloud, cv::viz::Color::green());
  point3d.setRenderingProperty(cv::viz::POINT_SIZE, 1.0);

  std::vector<cv::Affine3f> path;
  for(size_t i=0;i<nCameraPoses.size();i++){
    cv::Mat rot = cv::Mat(nCameraPoses[i].get_minor<3,3>(0,0));
    cv::Mat tra =cv::Mat(nCameraPoses[i].get_minor<3,1>(0,3));

    path.push_back(cv::Affine3f(rot,tra));
  }

  std::vector<cv::Scalar> rgb;
  cv::Mat rvecRight;
  cv::Rodrigues(nCameraPoses[5].get_minor<3,3>(0,0),rvecRight);
  cv::Mat tvecRight(nCameraPoses[5].get_minor<3,1>(0,3));

  Points2f projectedPoints(nFeaturesImages[5].pt2D.size());
  cv::projectPoints(cloud,rvecRight,tvecRight,cameraMatrix.K,cv::Mat(),projectedPoints);
  cv::Mat img = nImages[5];
/*
  for(size_t i=0;i<cloud.size();i++){

      for(cv::Point2f& pt2D : projectedPoints){
            for(size_t i=0;i<img.rows;i++){
                for(size_t j=0;j<img.cols;j++){

                   cv::Point2f imageCoordinate = cv::Point2f(i,j);

                   if(imageCoordinate==pt2D){
                       rgb.push_back(cv::Scalar(img.at<cv::Vec3b>(i,j)[0],
                                                img.at<cv::Vec3b>(i,j)[1],
                                                img.at<cv::Vec3b>(i,j)[2]));
                       break;
                     }else{
                       continue;
                     }

               }
          }
     }
   }
   */
  std::cout << "[DONE]"  << std::endl;
  std::cout << "pointcloud RGB=" << rgb.size() << std::endl;
  cv::viz::WTrajectory trajectory(path,cv::viz::WTrajectory::BOTH,0.07,cv::viz::Color::yellow());
  cv::viz::WTrajectoryFrustums fustrum(path, cv::Matx33f(cameraMatrix.K), 0.2, cv::viz::Color::yellow());

  fustrum.setRenderingProperty(cv::viz::LINE_WIDTH,2);
  trajectory.setRenderingProperty(cv::viz::REPRESENTATION_WIREFRAME,2);

  visualizer.setBackgroundColor(cv::viz::Color::black());
  visualizer.showWidget("Coordinate Widget", ucs);
  visualizer.showWidget("Point3D", point3d);
  visualizer.showWidget("trajectory",trajectory);
  visualizer.showWidget("fus",fustrum);


  // visualization loop
  while(cv::waitKey(0) && !visualizer.wasStopped()){

    visualizer.spin();
  }
}

//===============================================
//FUNCTION: BASE RECONSTRUCTION
//===============================================

bool StructFromMotion::baseTriangulation(){

  std::cout << "Getting best two views for first reconstruction..." << std::flush;
  std::map<int,ImagePair> bestViews = findBestPair();
  std::map<int,ImagePair>::const_iterator pos = bestViews.begin();
  ImagePair pair = {pos->second.left,pos->second.right};
  std::cout << "[DONE]" << std::endl;

  std::cout << "best pair:" << " image:(" <<pair.left << ") and image:("<<pair.right <<")" << std::endl;

  Matching prunedMatching,correspondences;
  cv::Matx34f Pleft  = cv::Matx34f::eye();
  cv::Matx34f Pright = cv::Matx34f::eye();

  std::cout << "Getting camera pose..." << std::flush;

  size_t leftView = pair.left;
  size_t rightView = pair.right;
  bool success = false;

  correspondences = getMatching(nFeaturesImages[leftView],nFeaturesImages[rightView]);

  success = StructFromMotion::getCameraPose(cameraMatrix,correspondences,
                                            nFeaturesImages[leftView],nFeaturesImages[rightView],
                                            prunedMatching, Pleft, Pright);
  std::cout << "[DONE]" << std::endl;

  if(not success) {

     std::cerr << "stereo view could not be obtained " << leftView << "," << rightView
               << ", go to next pair" << std::endl;
  }

  std::cout << "Showing matches between "<< "image:" << leftView << " and image:"
            << rightView << std::flush;

  cv::Mat outImg= StructFromMotion::imageMatching(nImages[leftView],nFeaturesImages[leftView].kps,
                                                  nImages[rightView],nFeaturesImages[rightView].kps,
                                                  prunedMatching);

  // StructFromMotion::imShow(outImg,"Matching");

  std::cout << " [DONE]"<<std::endl;
  std::vector<Point3D> pointcloud;

  success = StructFromMotion::triangulateViews(nFeaturesImages[leftView],nFeaturesImages[rightView],
                                               Pleft,Pright,prunedMatching,cameraMatrix,pair,pointcloud);

  if(not success){

      std::cerr << "Could not triangulate image:" << leftView << " and image:"<< rightView<< std::endl;
      return false;
  }

  nReconstructionCloud = pointcloud;
  nCameraPoses[leftView] = Pleft;
  nCameraPoses[rightView] = Pright;

  nDoneViews.insert(leftView);
  nDoneViews.insert(rightView);

 // adjustCurrentBundle() ;
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
        correspondences = StructFromMotion::getMatching(nFeaturesImages[i],nFeaturesImages[j]);
        if(correspondences.size()<30){
            continue;
        }else{
            int N = StructFromMotion::findHomographyInliers(nFeaturesImages[i],nFeaturesImages[j],
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

  std::cout << "** IMAGE COORDINATE - CAMERA COORDINATE CONVERTION **" << std::endl;

  Features alignedLeft,alignedRight;
  std::vector<int> leftBackReference,rightBackReference;
  StructFromMotion::AlignedPoints(left,right,matches,alignedLeft,alignedRight,
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

          //use back reference to point to original features in images
          p.idxImage[pair.left]  = leftBackReference[i];
          p.idxImage[pair.right] = rightBackReference[i];
          p.id = i;

          pointcloud.push_back(p);
  }

  std::cout << "[DONE]" << std::endl;
  std::cout << "Pointcloud size = " << pointcloud.size() << std::endl;

  return true;
}

//===============================================
//FUNCTION: ADD MORE VIEWS
//===============================================

void StructFromMotion::addMoreViews(){

  while(nDoneViews.size() != 3){

      std::cout <<"\n"<< "===================================="<< std::endl;
      std::cout << "Adding more views..." << std::endl;
      std::cout << "Finding 2D-3D correspondences..." << std::endl;
      ImagePair pair;
      Pts3D2DPNP pts2D3D = StructFromMotion::find2D3DMatches(pair);

      size_t leftView = pair.left;
      size_t rightView = pair.right;
      std::cout << "The new frame:" << rightView << " is ready for been add" << std::endl;

      nDoneViews.insert(rightView);
      std::cout << "Add frame:("<< rightView << ")"<< std::endl;

      Points2f pts2D_PNP=pts2D3D.pts2D;
      Points3f pts3D_PNP=pts2D3D.pts3D;

      std::cout << "Finding new camera pose..." << std::flush;

      cv::Matx34f newCameraPose = cv::Matx34f::eye();
      StructFromMotion::findCameraPosePNP(cameraMatrix,pts3D_PNP,pts2D_PNP,newCameraPose);
      std::cout << "[DONE]" << std::endl;
      std::cout << "Add frame:("<< rightView << ")"<< " - New camera pose:"<< "\n"
                << newCameraPose << std::endl;

      nCameraPoses[rightView]=newCameraPose;

      std::vector<Point3D> pointcloud;
      std::cout << "Triangulating points..." << std::flush;

      bool success;
      const Matching newMatch = getMatching(nFeaturesImages[leftView],nFeaturesImages[rightView]);

   success = StructFromMotion::triangulateViews(nFeaturesImages[leftView],nFeaturesImages[rightView],
                                                nCameraPoses[leftView],nCameraPoses[rightView],
                                                newMatch,cameraMatrix,
                                                {leftView,rightView},pointcloud);

   std::cout << "[DONE]" << std::endl;
   StructFromMotion::mergeNewPoints(pointcloud);

   //adjustCurrentBundle() ;
 }

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
     for(size_t newFrame = 0;newFrame< nImages.size();newFrame++){

         std::cout << "Finding best frame to add..." << std::flush;
        //Si es verdadero, entonces el nuevo frame está en la nube de puntos, no es necesario procesar!

        if(nDoneViews.count(newFrame)== 1){
          std::cout << "[X]" << std::endl;
          std::cout << "Frame:" << newFrame << " is already add." << std::endl;
          continue; //Pase al siguiente frame
        }

        for(size_t framePC:nDoneViews){

            const Matching Match = getMatching(nFeaturesImages[framePC],nFeaturesImages[newFrame]);
            const size_t bestSizeMatches = Match.size();
            matchesSizes[bestSizeMatches]={framePC,newFrame};
            continue;
        }

        std::map<int,ImagePair>::const_iterator pos = std::prev(matchesSizes.end());
        const size_t bestMatchSize = pos->first;

        size_t left = pos->second.left;
        size_t right = pos->second.right;

        std::cout << "[OK]" << std::endl;
        std::cout << "New frame to add: " << right << std::endl;
        std::cout << "Verifying if number of matches is enough..." << std::flush;

        if(bestMatchSize < 60){
           matchesSizes.clear();
           std::cout << "[X]" << std::endl;
           continue;
        }
        std::cout << "[OK]" << std::endl;
        std::cout << "Found "<< bestMatchSize << " matches between frame:"
                            << left << " and new frame:" << right << std::endl;
        std::cout << "Finding 2D points of new frame that match with POINTCLOUD..." << std::flush;
        pair={left,right};

        std::set<int> nDonePts;
        const Matching bestMatch = getMatching(nFeaturesImages[left],nFeaturesImages[right]);

        for(const cv::DMatch& match_index : bestMatch){

          for(Point3D numPt3D : nReconstructionCloud){
/*
             if(nDonePts.count(numPt3D.id) == 1){
                 continue;
             }
             */
/*
             if(numPt3D.idxImage.count(left)==0){
                continue;
             }
             */

             if(match_index.queryIdx != numPt3D.idxImage[left]){
                continue;
             }

             matches2D3D.pts3D.push_back(numPt3D.pt);
             matches2D3D.pts2D.push_back(nFeaturesImages[right].pt2D[match_index.trainIdx]);
             nDonePts.insert(numPt3D.id);
             break;

         }//End for-(vector point3D comparison)
       }//End for-(best matches vector comparison)

       if(matches2D3D.pts2D.size()< 70){
           std::cout << "[X]" << std::endl;
           std::cout << "Not found enough points for PnPRansac, found: "
                     << matches2D3D.pts2D.size() << std::endl;
           std::cout << "\n"<< "=============================== " << std::endl;
           matchesSizes.clear();
           continue;
       }else{

           std::cout << "[OK]" << std::endl;
           std::cout << "Found: " << matches2D3D.pts2D.size() << " Pt2D and "
                     << matches2D3D.pts3D.size() << " Pt3D" << std::endl;
         break;
       }
    }//End for-(NewFrames)

   return matches2D3D;
}

//===============================================
//FUNCTION: FIND CAMERA POSE PNP RANSAC
//===============================================

void StructFromMotion::findCameraPosePNP(const CameraData& intrinsics,const std::vector<cv::Point3f>& pts3D,const std::vector<cv::Point2f>& pts2D,cv::Matx34f& P){

  cv::Mat rvec, T;
  cv::Mat inliers;
  double RANSAC_THRESHOLD=10.0f;

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
    adjustBundle(nReconstructionCloud,nCameraPoses,cameraMatrix,nFeaturesImages);

}

void StructFromMotion::mergeNewPoints(const std::vector<Point3D>& newPointCloud) {

    const float DISTANCE_ERROR = 0.1;

    for (const Point3D& pt3D : newPointCloud) {

        bool foundMatchingFeature = false;
        bool foundMatching3DPoint = false;
        for (const Point3D& existingPoint3D : nReconstructionCloud) {
            if (cv::norm(existingPoint3D.pt - pt3D.pt) < DISTANCE_ERROR) {
                //This point is very close to an existing 3D cloud point
                foundMatching3DPoint = true;

                //Look for common 2D features to confirm match
                for (std::pair<const int,int> idxNewPoint3D : pt3D.idxImage) {
                    //idxNewPoint3D.first = image index --> originating
                    //idxNewPoint3D.second = feature 2D index

                    for (std::pair<const int,int> idxExistingPoint3D : existingPoint3D.idxImage) {
                        //idxExistingPoint3D.first = existing point's originating view
                        //idxExistingPoint3D.second = existing point's view 2D feature index
                        if(idxNewPoint3D.first == idxExistingPoint3D.first and
                           idxNewPoint3D.second==idxExistingPoint3D.second){

                            foundMatchingFeature = true;
                            break;
                        }else{
                            continue;
                        }
                    }
                    if(foundMatchingFeature){
                        break;
                    }
                }
            }//end if(cv::norm)
            if (not foundMatchingFeature and not foundMatching3DPoint) {
                //This point did not match any existing cloud points - add it as new.
                nReconstructionCloud.push_back(pt3D);
                break;
            }
        }       
   }
}

bool StructFromMotion::getCameraPose(const CameraData& intrinsics,const Matching & matches,
                                     const Features& left, const Features& right, Matching& prunedMatch,
                                     cv::Matx34f& Pleft, cv::Matx34f& Pright){

  if (intrinsics.K.empty()) {
         std::cerr << "Intrinsics matrix (K) must be initialized." << std::endl;
         return false;
  }

  Features alignedLeft,alignedRight;
  StructFromMotion::AlignedPointsFromMatch(left,right,matches,alignedLeft,alignedRight);

  // ESSENTIAL MATRIX
  cv::Mat mask;
  cv::Mat E = cv::findEssentialMat(alignedLeft.pt2D, alignedRight.pt2D,
                                   intrinsics.K,cv::RANSAC,0.999, 1.0,mask);

  // CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
  cv::Mat R,T;
  cv::recoverPose(E,alignedLeft.pt2D, alignedRight.pt2D,R,T,intrinsics.fx,
                   cv::Point2d(intrinsics.cx,intrinsics.cy),mask);

  bool status = StructFromMotion::CheckCoherentRotation(R);

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

     ofstream ofs(prefix + "_points.ply");
     std::cout << "Saving result reconstruction with prefix..." << prefix + "_points.ply" << std::flush;

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
                cv::Point2f p2d = nFeaturesImages[viewIdx].pt2D[originatingView->second];
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

    ofstream ofsc(prefix + "_cameras.ply");

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

        ofsc << c.x  << " " << c.y  << " " << c.z  << endl;
        ofsc << cx.x << " " << cx.y << " " << cx.z << endl;
        ofsc << cy.x << " " << cy.y << " " << cy.z << endl;
        ofsc << cz.x << " " << cz.y << " " << cz.z << endl;
    }

    const int camVertexStartIndex = nReconstructionCloud.size();

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
    std::cout << "[DONE]" << std::endl;
}




