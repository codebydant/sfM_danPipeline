//***********************************************
//HEADERS
//***********************************************

#include "../include/Sfm.h"

/********************************************
                  PIPELINE
********************************************/

int StructFromMotion::run_SFM(std::ifstream& file){

  std::cout << "************************************************" << std::endl;
  std::cout << "              3D RECONSTRUCTION                 " << std::endl;
  std::cout << "************************************************" << std::endl;

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

  // **(4) ADD MORE VIEWS
  StructFromMotion::addMoreViews();  

  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

  // **(5) VISUALIZER POINTCLOUD
  StructFromMotion::visualizerPointCloud(nReconstructionCloud,nReconstructionCloud2);

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

  std::cout << "Getting matches from images pairs..." << std::endl;
  nFeatureMatchMatrix.resize(nImages.size(),std::vector<Matching>(nImages.size()));

  MatchesforSort matchesSize;
  const size_t numImg = nImages.size();

  for(size_t i=0;i<numImg-1;i++) {
    for(size_t j=i+1;j<numImg;j++){

      nFeatureMatchMatrix[i][j]=StructFromMotion::obtenerMatches(nFeaturesImages[i],
                                                                 nFeaturesImages[j]);

      matchesSize.size = nFeatureMatchMatrix[i][j].size();
      matchesSize.i = i;
      matchesSize.j = j;

      nMatchesSorted.push_back(matchesSize);
     }
  }

  std::sort(nMatchesSorted.begin(),nMatchesSorted.end(),compare);
  std::cout << "MATCHES FEATURES ==> OK" << std::endl;
  std::cout << "Total matches = " << nFeatureMatchMatrix.size()*nFeatureMatchMatrix.capacity() << std::endl;
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
    intrinsics.fx = 1520.400000;
    intrinsics.fy = 1525.900000;
    intrinsics.cx = 302.320000;
    intrinsics.cy = 246.870000;
    intrinsics.K = (cv::Mat_<double>(3,3) << intrinsics.fx,    0,           intrinsics.cx,
                                                  0,       intrinsics.fy,   intrinsics.cy,
                                                  0,           0 ,                 1);
    intrinsics.invK = StructFromMotion::inverse(intrinsics.K);

    std::cout << "CAMERA MATRIX ==> OK" << std::endl;
    std::cout << "matrix K:" << "\n" << intrinsics.K << std::endl;

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
//FUNCTION: POINTCLOUD VISUALIZER
//===============================================

void StructFromMotion::visualizerPointCloud(const std::vector<Point3D>& pointcloud,const std::vector<Point3D>& pointcloud2){

  // Create a viz window
  cv::viz::Viz3d visualizer("Viz window");
  cv::viz::WCoordinateSystem ucs(1);

  Points3f cloud,cloud2;
  for(size_t i=0;i<pointcloud.size();i++){
      cloud.push_back(pointcloud[i].pt);

  }
/*
  for(size_t i=0;i<pointcloud2.size();i++){
      cloud2.push_back(pointcloud2[i].pt);

  }
  */

  cv::viz::WCloud point3d(cloud, cv::viz::Color::green());
  point3d.setRenderingProperty(cv::viz::POINT_SIZE, 1.0);

 // cv::viz::WCloud point3d2(cloud2, cv::viz::Color::red());
//  point3d.setRenderingProperty(cv::viz::POINT_SIZE, 1.0);

  visualizer.setBackgroundColor(cv::viz::Color::black());
  visualizer.showWidget("Coordinate Widget", ucs);
  visualizer.showWidget("Point3D", point3d);
 // visualizer.showWidget("Point3D2", point3d2);

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

//  StructFromMotion::imShow(outImg,"Matching");
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

  std::cout << "CAMERA POSE ==> OK" << std::endl;
  std::cout << "Projection1:" << "\n" << Pleft << std::endl;
  std::cout << "Projection2:" << "\n" << Pright << std::endl;
  std::cout << "Triangulating points..." << std::endl;

  std::vector<Point3D> pointcloud;

  bool success = StructFromMotion::triangulateViews(nFeaturesImages[pair.left],nFeaturesImages[pair.right],
                                           Pleft,Pright,bestMatches,
                                           matrixK,pair,pointcloud);

  if(success==true){

      std::cout << "Base pointcloud ==> OK" << std::endl;

      nReconstructionCloud = pointcloud;
      nCameraPoses[pair.left] = Pleft;
      nCameraPoses[pair.right] = Pright;

      nDoneViews.insert(pair.left);
      nDoneViews.insert(pair.right);
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

  for(size_t i =0;i<nMatchesSorted.size();i++){

     if(nMatchesSorted[i].size < 30){
         continue;
      }

     size_t left = nMatchesSorted[i].i;
     size_t right = nMatchesSorted[i].j;

     int N = StructFromMotion::findHomographyInliers(nFeaturesImages[left],nFeaturesImages[right],
                                                     nFeatureMatchMatrix[left][right]);

     if(N < 60){
         continue;
     }else{

      pairIdx = {left,right};
      break;
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

  if(matrixK.K.empty()){
      std::cerr << "It could not map 2D points-3D points, K is empty or matches vector is not valid" << std::endl;
      return false;
  }

  Features alignedLeft,alignedRight;
  std::vector<int> leftBackReference;
  std::vector<int> rightBackReference;
  StructFromMotion::AlignedPoints(left,right,matches,alignedLeft,alignedRight,leftBackReference,rightBackReference);

  // NORMALIZE IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)
  std::cout << "Normalizing points..." << std::endl;
  cv::Mat normalizedLeftPts,normalizedRightPts;
  cv::undistortPoints(alignedLeft.pt2D, normalizedLeftPts, matrixK.K, cv::Mat());
  cv::undistortPoints(alignedRight.pt2D, normalizedRightPts, matrixK.K, cv::Mat());
  std::cout << "POINTS NORMALIZED ==> OK" << std::endl;

  std::cout << "Triangulating points from --> " << "image: " << pair.left << " and image: "
            << pair.right << std::endl;

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

          if (cv::norm(projectedLeft[i]  - alignedLeft.pt2D[i])  > 50 or
              cv::norm(projectedRight[i] - alignedRight.pt2D[i]) > 50){
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
  std::cout << "POINTCLOUD VECTOR ==> OK" << std::endl;
  std::cout << "Pointcloud size=" << pointcloud.size() << std::endl;

  return true;
}

//===============================================
//FUNCTION: ADD MORE VIEWS
//===============================================

void StructFromMotion::addMoreViews(){

  for(size_t newFrame = 0;newFrame < 6;newFrame++){

    const size_t status = nDoneViews.count(newFrame);

    if(status==1){

       continue;
    }

    std::cout <<"\n"<< "===================================="<< std::endl;
    std::cout << "Adding more views..." << std::endl;
    std::cout << "Finding 2D-3D correspondences..." << std::endl;
    size_t bestFrame;
    Pts3D2DPNP pts2D3D = StructFromMotion::find2D3DMatches(newFrame,bestFrame);

    if(pts2D3D.pts2D.empty() or pts2D3D.pts3D.empty()){
       continue;
      }

    std::cout << "Match 3D-2D ==> image:" << bestFrame << " and image:" << newFrame << std::endl;

    nDoneViews.insert(newFrame);
    std::cout << "Add frame:("<< newFrame << ")"<< std::endl;

    Points2f pts2D_PNP=pts2D3D.pts2D;
    Points3f pts3D_PNP=pts2D3D.pts3D;

    Matching matchNewFrame_PC = nFeatureMatchMatrix[bestFrame][newFrame];

    if(matchNewFrame_PC.size()==0){
        continue;
      }

    cv::Matx34f newCameraPose = cv::Matx34f::eye();
    StructFromMotion::findCameraPosePNP(matrixK,pts3D_PNP,pts2D_PNP,newCameraPose);
    std::cout << "New camera pose:"<< "\n" << newCameraPose << std::endl;

    nCameraPoses[newFrame]=newCameraPose;

    std::vector<Point3D> pointcloud;
    ImagePair pair = {bestFrame,newFrame};
    std::cout << "Triangulating points..." << std::endl;

    bool success = StructFromMotion::triangulateViews(nFeaturesImages[bestFrame],nFeaturesImages[newFrame],
                                                      nCameraPoses[bestFrame],nCameraPoses[newFrame],
                                                      nFeatureMatchMatrix[bestFrame][newFrame],
                                                      matrixK,pair,pointcloud);

    std::cout << "New pointcloud ==> OK" << std::endl;

    for(size_t i=0;i<pointcloud.size();i++){

       nReconstructionCloud.push_back(pointcloud[i]);
    }
  }  
  std::cout << "\n"<< "=============================== " << std::endl;
  std::cout << "Images processed = " << nDoneViews.size() << " of " << nImages.size() << std::endl;


}

//===============================================
//FUNCTION: FIND CORRESPONDENCES 2D-3D
//===============================================

Pts3D2DPNP StructFromMotion::find2D3DMatches(const size_t& newFrame,size_t& bestFrame){

  const size_t numImg = nImages.size();
  std::map<int,ImagePair> matcheIdx;

  for(size_t frameN=0;frameN<numImg;frameN++){

      const size_t status = nDoneViews.count(frameN);

      if(status==0){

          continue;
       }

      else{

          if(frameN>newFrame){
              continue;
            }

          const size_t bestSizeMatches = nFeatureMatchMatrix[frameN][newFrame].size();
          ImagePair pair;
          pair.left = frameN;
          pair.right = newFrame;
          matcheIdx[bestSizeMatches]=pair;

          continue;
       }
   }

  Pts3D2DPNP matches2D3D;

  if(matcheIdx.empty()){

      return matches2D3D;
  }

  std::map<int,ImagePair>::const_iterator pos = std::prev(matcheIdx.end());
  std::cout<<"found "<<pos->first <<" 3d-2d point correspondences"<<std::endl;
  bestFrame = pos->second.left;

  const size_t pts2DBestFrame = nFeaturesImages[bestFrame].pt2D.size();
  std::set<int> nDonePts;

   for(size_t idxPC=0; idxPC<pts2DBestFrame;idxPC++){

       for(size_t orgIdx=0;orgIdx<nReconstructionCloud.size();orgIdx++){

          Point3D pt = nReconstructionCloud[idxPC];
          const size_t status = pt.idxImage.count(bestFrame);

          if(status==0){
              continue;
            }

          const size_t status2 = nDonePts.count(orgIdx);
          if(status2 == 1){
              continue;
            }

          if(pt.idxImage[bestFrame]!=orgIdx){
              continue;
            }

          matches2D3D.pts2D.push_back(nFeaturesImages[bestFrame].pt2D[orgIdx]);
          nDonePts.insert(orgIdx);
          break;


         }
     }
   /*

          if(pt3d.idxImgLeft){

              matches2D3D.pts3D.push_back(pt3d.pt);
              matches2D3D.pts2D.push_back(nFeaturesImages[newFrame].kps[idx.trainIdx].pt);

            }
          continue;
        }
    }
*/

  return matches2D3D;
}

//===============================================
//FUNCTION: FIND CAMERA POSE PNP RANSAC
//===============================================

void StructFromMotion::findCameraPosePNP(const CameraData& matrixK,const std::vector<cv::Point3f>& pts3D,const std::vector<cv::Point2f>& pts2D,cv::Matx34f& P){

  cv::Mat rvec, T;
  cv::Mat inliers;
  double RANSAC_THRESHOLD=10.0f;

  std::cout << "Finding new camera pose..." << std::endl;

  cv::solvePnPRansac(pts3D,pts2D,matrixK.K,matrixK.distCoef,rvec,T,false,100,
                     RANSAC_THRESHOLD,0.99,inliers);

  cv::Mat R;
  cv::Rodrigues(rvec, R); //convert to a rotation matrix

  cv::Mat projection2(3, 4, CV_64F,cv::Scalar(0));
  R.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
  T.copyTo(projection2.colRange(3, 4));
  P=projection2;

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








