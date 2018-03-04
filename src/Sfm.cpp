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

  // **(2) FEATURE MATCHING - ALL IMAGES
  StructFromMotion::loadMatches(featuresImages, matchesImages,totalImages);
  std::cout << "primer match=" << matchesImages[0].goodMatches.size() << std::endl;


  //for(size_t n=0;n<1;n++){
   //}

  // **(3) BEST IMAGE PAIR
  std::map<float,idImagePair> bestPair;
  StructFromMotion::best2Views(featuresImages,bestPair,pts2DAligVec);
  std::map<float,idImagePair>::iterator it=bestPair.begin();
  idImagePair p= it->second;
  std::cout << "i=" << p.i << "\n" << "j=" << p.j << std::endl;

  // **(4) CAMERA MATRIX
  StructFromMotion::getCameraMatrix(matrixK);

  // **(5) ESSENTIAL MATRIX
  cv::Mat mask;
  cv::Mat_<double> matrixE=StructFromMotion::findEssentialMatrix(pts2DAligVec[0].Pt2D_left,
                                                                 pts2DAligVec[0].Pt2D_right,matrixK.K,mask);

  // **(6) CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
  cv::Mat relativeRotationCam,relativeTranslaCam;
  StructFromMotion::cameraPose(pts2DAligVec[0].Pt2D_left,pts2DAligVec[0].Pt2D_right,
                               matrixK.fx,matrixK.cx,matrixK.cy,relativeRotationCam,
                               relativeTranslaCam,mask,matrixE);

  // **(7) PROJECTION MATRIX 
  bool status;
  StructFromMotion::projection(relativeRotationCam,relativeTranslaCam,projectionMatrices,status);
  PointCloud cloud;
  StructFromMotion::map2D3D(projectionMatrices,pts2DAligVec,matrixK,status, cloud);

  // **(12) POINTCLOUD VISUALIZER
  cv::Matx33d matrixCam = (cv::Matx33d)matrixK.K;
  StructFromMotion::visualizerPointCloud(matrixCam,images[11],images[12],relativeRotationCam,
                                         relativeTranslaCam,cloud.pt3D);
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

Features StructFromMotion::obtenerFeatures(const cv::Mat& image,const std::string& path) {
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

void StructFromMotion::keypoints2F(Keypoints& keypoints, Points2f& points2D){

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

Matches StructFromMotion::obtenerMatches(const cv::Mat& descriptors1,const cv::Mat& descriptors2){

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

void StructFromMotion::loadMatches(const std::vector<Features>& ftsVec,std::vector<Matches>& matchesImages,int& numImages){

  matchesImages.clear();

  for(size_t n=0;n<numImages-1;n++){

      matchesImages.push_back(StructFromMotion::obtenerMatches(ftsVec[n].descriptors,ftsVec[n+1].descriptors));

    }

  if (matchesImages.size() == 0){
    std::cout<< " --(!) No good matches found " << std::endl;
  }

}

void StructFromMotion::guardarIdx(Matches& matches,Pt2DAligned& pt){

  for(unsigned int i=0;i<matches.goodMatches.size();i++){

    pt.trainIdx.push_back(matches.goodMatches[i].trainIdx);
    pt.queryIdx.push_back(matches.goodMatches[i].queryIdx);

  }

}

//===============================================
//FUNCTION IMAGE MATCHING
//===============================================

cv::Mat StructFromMotion::imageMatching(const cv::Mat& img1,const Keypoints& keypoints1,
                                        const cv::Mat& img2,const Keypoints& keypoints2,const MatchesVector& matches){

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

cv::Mat_<double> StructFromMotion::findEssentialMatrix(const Points2f& leftPoints,const Points2f& rightPoints,
                                                        cv::Mat_<double>& cameraMatrix,cv::Mat& mask){

     cv::Mat_<double> matrixFundamental = cv::findFundamentalMat(leftPoints,rightPoints,
                                                                 cv::FM_RANSAC,0.1,0.99,mask);
     cv::Mat_<double> matrixE = cameraMatrix.t()*matrixFundamental*cameraMatrix;

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

void StructFromMotion::AlignedPoints(const Features& left,const Features& right,const Matches& matches, std::vector<Pt2DAligned>& pts2dAlig){

      Pt2DAligned pts;
      StructFromMotion::AlignedPoints(left,right,matches,pts);
      pts2dAlig.push_back(pts);


     std::cout << "original pts1 size=" << left.pt2D.size() <<
                " (pts1 new size=" << pts.Pt2D_left.size() << ")" << std::endl;
    std::cout << "original pts2 size=" << right.pt2D.size() <<
               " (pts2 new size=" << pts.Pt2D_right.size() << ")" << std::endl;
}

void StructFromMotion::AlignedPoints(const Features& left,const Features& right,const Matches& matches, Pt2DAligned& pts2dAlig){

      //align left and right point sets
      for(unsigned int i=0;i<matches.goodMatches.size();i++){

        pts2dAlig.Pt2D_left.push_back(left.kps[matches.goodMatches[i].queryIdx].pt);
        pts2dAlig.Pt2D_right.push_back(right.kps[matches.goodMatches[i].trainIdx].pt);
      }

      //StructFromMotion::guardarIdx(matches,pts2dAlig);
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


void StructFromMotion::best2Views(std::vector<Features>& fts,std::map<float,idImagePair>& bestPair,std::vector<Pt2DAligned>& pts2dAlig){

  int totalInliers;
  Pt2DAligned pts;
  Matches matchImg1N;
  int temp;
  for(size_t i=0;i<fts.size()-1;i++){

      matchImg1N = StructFromMotion::obtenerMatches(fts[0].descriptors,fts[i+1].descriptors);
      //std::cout << "matches size1=" << matchImg1N.goodMatches.size() << std::endl;
      if(matchImg1N.goodMatches.size() <= 47 && matchImg1N.goodMatches.size() > 30){

          totalInliers = StructFromMotion::findHomographyInliers(fts.at(0),fts.at(i+1),matchImg1N,pts);
          float weight = (float)totalInliers/(float)matchImg1N.goodMatches.size();
          bestPair[weight] = {0,i+1};
          temp = i+1;

       /*   cv::Mat matchImage;
          matchImage =StructFromMotion::imageMatching(images.at(0),featuresImages[0].kps,
                                                    images.at(i+1),featuresImages[i+1].kps,
                                                    matchImg1N.goodMatches);
          StructFromMotion::matchingImShow(matchImage); */

      }else if(matchImg1N.goodMatches.size() < 30){

          std::cout << "No hay suficientes matches para escoger el mejor par de imágenes" << std::endl;
          temp = i+1;
      }
  }

  pts2dAlig.push_back(pts);

  for(std::map<float,idImagePair>::iterator it=bestPair.begin(); it!=bestPair.end(); ++it){
      std::cout << "weight=" << it->first << '\n';
  }

}


int StructFromMotion::findHomographyInliers(const Features& f1,const Features& f2,const Matches& matches,Pt2DAligned& ptsAligned){

  StructFromMotion::AlignedPoints(f1,f2,matches,ptsAligned);
  double minVal,maxVal;
  cv::minMaxIdx(ptsAligned.Pt2D_left,&minVal,&maxVal);

  cv::Mat matrixH(3,3,CV_32FC3);
  cv::Mat inliersMask;
  int numInliers;
  if(matches.goodMatches.size()>=4){

     matrixH = cv::findHomography(ptsAligned.Pt2D_left,ptsAligned.Pt2D_right,
                                  cv::RANSAC,0.004 * maxVal,inliersMask);
     numInliers = cv::countNonZero(inliersMask);
  }else if(matches.goodMatches.size()<4 or matrixH.empty()){

      numInliers = 0;
  }

  return numInliers;
}

void StructFromMotion::map2D3D(std::vector<cv::Mat_<double>>& ProjectionVector,Features& left,Features& right,Matches& matches,CameraData& matrixK,bool& status, Point3D2DMatch& pointcloud){


  for(unsigned int i=0;i<matches.goodMatches.size();i++){

    pts2dAlig.Pt2D_left.push_back(left.kps[matches.goodMatches[i].queryIdx].pt);
    pts2dAlig.Pt2D_right.push_back(right.kps[matches.goodMatches[i].trainIdx].pt);
  }


  // NORMALIZE IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)
  Points2f points1CC,points2CC;
  if(status == true){

      cv::undistortPoints(pts[0].Pt2D_left, points1CC, matrixK.K, cv::Mat());
      cv::undistortPoints(pts[0].Pt2D_right, points2CC, matrixK.K, cv::Mat());
  }else{

      std::cout << "La matriz de rotación no es adecuada" << std::endl;
  }

  // TRIANGULATE POINTS
  cv::Mat cloud(1,points1CC.size(),CV_32FC4);
  cv::triangulatePoints(ProjectionVector[0],ProjectionVector[1],points1CC,points2CC,cloud);

  // CONVERTION CAMERA COORDINATE - WORLD COORDINATE
  cv::Mat pts3d;
  cv::convertPointsFromHomogeneous(cloud.t(),pts3d);

  cv::Mat rvecLeft,tvecLeft;
  cv::Rodrigues(cv::Matx34f(ProjectionVector[0]).get_minor<3,3>(0,0),rvecLeft);
  tvecLeft = cv::Mat(cv::Matx34f(ProjectionVector[0]).get_minor<3,1>(0,3).t());

  Points2f projectedLeft(pts[0].Pt2D_left.size());
  cv::projectPoints(cloud,rvecLeft,tvecLeft,matrixK.K,cv::Mat(),projectedLeft);

  cv::Mat rvecRight,tvecRight;
  cv::Rodrigues(cv::Matx34f(ProjectionVector[1]).get_minor<3,3>(0,0),rvecRight);
  tvecRight = cv::Mat(cv::Matx34f(ProjectionVector[1]).get_minor<3,1>(0,3).t());

  Points2f projectedRight(pts[0].Pt2D_right.size());
  cv::projectPoints(cloud,rvecRight,tvecRight,matrixK.K,cv::Mat(),projectedRight);

  for (size_t i = 0; i < cloud.rows; i++) {
          //check if point reprojection error is small enough
          if (cv::norm(projectedLeft[i]  - pts[0].Pt2D_left[i])  > 10 or
              cv::norm(projectedRight[i] - pts[0].Pt2D_right[i]) > 10){
              continue;
          }

          Point3D p;
          p.pt = cv::Point3f(cloud.at<float>(i, 0),
                             cloud.at<float>(i, 1),
                             cloud.at<float>(i, 2));

          //use back reference to point to original features in images
          p.pt3D2D[imagePair.left]  = leftBackReference [i];
          p.pt3D2D[imagePair.right] = rightBackReference[i];

          pointCloud.push_back(p);
  }

  /*
  for(size_t i=0;i<pointcloud.pt3D.rows;i++){
      const double error = cv::norm(projPts.at<Points2f>(i)- imgPts[i]);
    }

    */
}










