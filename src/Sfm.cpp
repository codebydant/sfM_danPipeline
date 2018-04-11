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

  // **(0) GET CAMERA MATRIX
  StructFromMotion::getCameraMatrix(matrixK);

  // **(0) IMAGES LOAD
  StructFromMotion::imagesLOAD(file);  
  nCameraPoses.resize(nImages.size());


  // **(1) FEATURE DETECTION AND EXTRACTION - ALL IMAGES
  StructFromMotion::extractFeatures();

  // **(2) FEATURE MATCHING
  std::cout << "Getting matches from images pairs...";
  StructFromMotion::matchFeatures();

  for(size_t n=0;n<nImages.size()-1;n++){


  cv::Mat out;
  out = StructFromMotion::imageMatching(nImages[n],nFeaturesImages[n].kps,nImages[n+1],nFeaturesImages[n+1].kps,nFeatureMatchMatrix[n][n+1]);


 StructFromMotion::imShow(out,"matching");
}

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

  std::cout << "Getting images...";

  nImages.clear();
  if (!file.is_open()) {
         std::cout << "There was a problem opening the file. No images loaded!" << std::endl;
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
  std::cout << "[DONE] "<< "\n"<< "Total images = "<< nImages.size() << std::endl;
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

  std::cout << "Getting features from all images...";

  nFeaturesImages.resize(nImages.size());
  for(size_t n=0;n<nImages.size();n++){

      nFeaturesImages[n] =StructFromMotion::obtenerFeatures(nImages[n]);
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
    std::cout << "[DONE]" << std::endl;
    std::cout << "Total matches = " << nFeatureMatchMatrix.size()*nFeatureMatchMatrix.capacity()
              << std::endl;
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

    std::cout << "Getting camera matrix...";
    intrinsics.K = (cv::Mat_<float>(3,3) << 1520.400000,    0,           302.320000,
                                                  0,    1525.900000,     246.870000,
                                                  0,        0 ,              1);
    intrinsics.fx = intrinsics.K.at<float>(0,0);
    intrinsics.fy = intrinsics.K.at<float>(1,1);
    intrinsics.cx = intrinsics.K.at<float>(0,2);
    intrinsics.cy = intrinsics.K.at<float>(1,2);
    //intrinsics.invK = StructFromMotion::inverse(intrinsics.K);

    std::cout << "[DONE]" << std::endl;
    std::cout << "matrix K:" << "\n" << intrinsics.K << std::endl;

    // cv::Mat cameraMatrix;
     cv::Mat cameraDistCoeffs;
     cv::FileStorage fs("camera-calibration-data.xml", cv::FileStorage::READ);
    // fs["Camera_Matrix"] >> cameraMatrix;
     fs["Distortion_Coefficients"] >> cameraDistCoeffs;
     //cv::Matx33d cMatrix(cameraMatrix);
     std::vector<double> cMatrixCoef(cameraDistCoeffs);
     intrinsics.distCoef = cv::Mat_<float>::zeros(1, 4);

     /*
     std::cout <<"Vector distortion coeff: "<< std::endl;
     std::cout << "[ ";
     intrinsics.distCoef = cameraDistCoeffs;
     for(size_t n=0;n<cMatrixCoef.size();n++){

         std::cout << cMatrixCoef.at(n) << ",";
       }
     std::cout <<"]" << std::endl;
     */

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

  int p=8;

vtkSmartPointer<vtkPolyData> cloud = vtkSmartPointer<vtkPolyData>::New();
vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

for(int n=0;n<nReconstructionCloud.size();n++){
    cv::Point3f p = nReconstructionCloud[n].pt;
    pts->InsertNextPoint(p.x,p.y,p.z);
  }
cloud->SetPoints(pts);

vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
    vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexFilter->SetInputData(cloud);
  vertexFilter->Update();

  vtkSmartPointer<vtkPolyData> polydata =
     vtkSmartPointer<vtkPolyData>::New();
   polydata->ShallowCopy(vertexFilter->GetOutput());



    // Visualize
vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

  //mapper->SetInput(cloud);

  mapper->SetInputData(polydata);

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
     actor->GetProperty()->SetColor(0.0, 1.0, 0.0);
     actor->GetProperty()->SetPointSize(1);


    vtkSmartPointer<vtkRenderer> renderer =  vtkSmartPointer<vtkRenderer>::New();
     renderer->AddActor(actor);
     renderer->SetBackground(0.0, 0.0, 0.0);
      // Zoom in a little by accessing the camera and invoking its "Zoom" method.
      renderer->ResetCamera();


      // The render window is the actual GUI window
      // that appears on the computer screen
      vtkSmartPointer<vtkRenderWindow> renderWindow =
        vtkSmartPointer<vtkRenderWindow>::New();
      renderWindow->SetSize(800, 600);
      renderWindow->AddRenderer(renderer);

      // The render window interactor captures mouse events
      // and will perform appropriate camera or actor manipulation
      // depending on the nature of the events.
      vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
      renderWindowInteractor->SetRenderWindow(renderWindow);

      // This starts the event loop and as a side effect causes an initial render.
      renderWindowInteractor->Start();

}



















//===============================================
//FUNCTION: BASE RECONSTRUCTION
//===============================================

bool StructFromMotion::baseTriangulation(){

  std::cout << "Getting best two views for first reconstruction...";
  std::map<int,ImagePair> bestViews = findBestPair();

  std::map<int,ImagePair>::const_iterator pos = bestViews.begin();
  ImagePair pair = {pos->second.left,pos->second.right};
  std::cout << "[DONE]" << std::endl;


  std::cout << "best pair:" << " image:(" <<pair.left << ") and image:("<<pair.right <<")" << std::endl;

  Matching prunedMatching;
  cv::Matx34f Pleft  = cv::Matx34f::eye();
  cv::Matx34f Pright = cv::Matx34f::eye();

  std::cout << "Getting camera pose...";


    size_t left = pair.right;
    size_t right = pair.left;

  bool success = StructFromMotion::getCameraPose(matrixK,nFeatureMatchMatrix[pair.left][pair.right],
                                                 nFeaturesImages[pair.left], nFeaturesImages[pair.right],
                                                 prunedMatching, Pleft, Pright);

  if(not success) {

     std::cerr << "stereo view could not be obtained " << pair.left << "," <<pair.right
               << ", go to next pair" << std::endl << std::flush;
  }

  std::cout << "Showing matches between "<< "image:" << pair.left << " and image:"
            << pair.right;

  cv::Mat outImg= StructFromMotion::imageMatching(nImages[pair.left],nFeaturesImages[pair.left].kps,
                                                  nImages[pair.right],nFeaturesImages[pair.right].kps,
                                                  prunedMatching);

  // StructFromMotion::imShow(outImg,"Matching");

  nFeatureMatchMatrix[pair.left][pair.right] = prunedMatching;

  std::cout << " [DONE]"<<std::endl;


  std::vector<Point3D> pointcloud;

  success = StructFromMotion::triangulateViews(nFeaturesImages[pair.left],nFeaturesImages[pair.right],
                                               Pleft,Pright,nFeatureMatchMatrix[pair.left][pair.right],
                                               matrixK,pair,pointcloud);

  if(success==true){

      nReconstructionCloud = pointcloud;
      nCameraPoses[pair.left] = Pleft;
      nCameraPoses[pair.right] = Pright;

      nDoneViews.insert(pair.left);
      nDoneViews.insert(pair.right);

      //adjustCurrentBundle() ;
      return true;

  }else{
      std::cerr << "Could not triangulate image:" << pair.left << " and image:"<< pair.right<< std::endl;
      return false;
  }
}

//===============================================
//FUNCTION: FIND BEST PAIR
//===============================================

std::map<int,ImagePair> StructFromMotion::findBestPair(){

  std::map<int,ImagePair> numInliers;

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

            numInliers[N]={weight.i,weight.j};
            continue;
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
  std::cout << "Normalizing points...";
  cv::Mat normalizedLeftPts,normalizedRightPts;
  cv::undistortPoints(alignedLeft.pt2D, normalizedLeftPts, matrixK.K, cv::Mat());
  cv::undistortPoints(alignedRight.pt2D, normalizedRightPts, matrixK.K, cv::Mat());
  std::cout << "[DONE]" << std::endl;

  // TRIANGULATE POINTS
  std::cout << "Triangulating points...";
  cv::Mat pts3dHomogeneous;
  cv::triangulatePoints(P1,P2,normalizedLeftPts,normalizedRightPts,pts3dHomogeneous);
  std::cout << "[DONE]" << std::endl;
  std::cout << "Points triangulate from --> " << "image: " << pair.left << " and image: "
            << pair.right << std::endl;

  std::cout << "** CAMERA COORDINATE - WORLD COORDINATE CONVERTION **" << std::endl;

  // CONVERTION CAMERA COORDINATE - WORLD COORDINATE
  std::cout << "Converting points to world coordinate...";
  cv::Mat pts3d;
  cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(),pts3d);
  std::cout << "[DONE]" << std::endl;

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

  std::cout << "Creating a pointcloud vector...";

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
  std::cout << "Pointcloud size = " << pointcloud.size() << "."<< std::endl;

  return true;
}

//===============================================
//FUNCTION: ADD MORE VIEWS
//===============================================

void StructFromMotion::addMoreViews(){

  std::cout << "PointCloud size init =" << nReconstructionCloud.size() << std::endl;

  while(nDoneViews.size() != 3){

    std::cout <<"\n"<< "===================================="<< std::endl;
    std::cout << "Adding more views..." << std::endl;
    std::cout << "Finding 2D-3D correspondences..." << std::endl;
    ImagePair pair;
    Pts3D2DPNP pts2D3D = StructFromMotion::find2D3DMatches(pair);

    size_t numFrame = pair.left;
    size_t bestFrame = pair.right;

    std::cout << "The new frame:" << bestFrame << " is ready for been add" << std::endl;

    nDoneViews.insert(bestFrame);
    std::cout << "Add frame:("<< bestFrame << ")"<< std::endl;

    Points2f pts2D_PNP=pts2D3D.pts2D;
    Points3f pts3D_PNP=pts2D3D.pts3D;

    cv::Matx34f newCameraPose = cv::Matx34f::eye();
    StructFromMotion::findCameraPosePNP(matrixK,pts3D_PNP,pts2D_PNP,newCameraPose);
    std::cout << "Add frame:("<< bestFrame << ")"<< " - New camera pose:"<< "\n"
   << newCameraPose << std::endl;

    nCameraPoses[bestFrame]=newCameraPose;

    Matching prunedMatch,matchInv;
    cv::Matx34f Pleft  = cv::Matx34f::eye();
    cv::Matx34f Pright = cv::Matx34f::eye();

    size_t left,right;
    if(numFrame>bestFrame){
        left = bestFrame;
        right = numFrame;

        matchInv = nFeatureMatchMatrix[left][right];

        for(size_t id=0;id<matchInv.size();id++){

            cv::DMatch p = matchInv[id];
            std::swap(p.queryIdx,p.trainIdx);
            matchInv[id]=p;
          }


    }else{
        left = numFrame;
        right = bestFrame;
        matchInv = nFeatureMatchMatrix[left][right];
    }

/*
    bool success = StructFromMotion::getCameraPose(matrixK,matchInv,
                                                   nFeaturesImages[left], nFeaturesImages[right],
                                                   prunedMatch, Pleft, Pright);


*/

    Features alignedLeft,alignedRight;
    StructFromMotion::AlignedPointsFromMatch(nFeaturesImages[pair.left],
                                             nFeaturesImages[pair.right],
                                             matchInv,
                                             alignedLeft,alignedRight);

    // ESSENTIAL MATRIX
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(alignedLeft.pt2D, alignedRight.pt2D,
                                     matrixK.K,cv::RANSAC,0.999, 1.0,mask);

    for (size_t i = 0; i < mask.rows; i++) {
       if(mask.at<uchar>(i)) {
             prunedMatch.push_back(matchInv[i]);
           }
    }

    nFeatureMatchMatrix[pair.left][pair.right]=prunedMatch;

    std::vector<Point3D> pointcloud;
    std::cout << "Triangulating points..." << std::endl;

   bool success = StructFromMotion::triangulateViews(nFeaturesImages[pair.left],nFeaturesImages[pair.right],
                                   nCameraPoses[pair.left],nCameraPoses[pair.right],
                                   prunedMatch,matrixK,
                                   {pair.left,pair.right},pointcloud);

    std::cout << "New pointcloud ==> [DONE]" << std::endl;
    StructFromMotion::mergeNewPoints(pointcloud);
/*
    for(size_t i=0;i<pointcloud.size();i++){

       nReconstructionCloud.push_back(pointcloud[i]);
    }
*/

   // adjustCurrentBundle() ;
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

       std::cout << "Finding best frame to add...";
      //Si es verdadero, entonces el nuevo frame está en la nube de puntos, no es necesario procesar!

      if(nDoneViews.count(newFrame)== 1){
        std::cout << "Frame:" << newFrame << " is already add." << std::endl;
        continue; //Pase al siguiente frame
      }

      for(size_t framePC:nDoneViews){

          if(framePC>newFrame){
                const size_t bestSizeMatches = nFeatureMatchMatrix[newFrame][framePC].size();
                matchesSizes[bestSizeMatches]={framePC,newFrame};
                continue;
          }else{

            const size_t bestSizeMatches = nFeatureMatchMatrix[framePC][newFrame].size();
            matchesSizes[bestSizeMatches]={framePC,newFrame};
            continue;
          }
      }

      std::map<int,ImagePair>::const_iterator pos = std::prev(matchesSizes.end());
      const size_t bestMatchSize = pos->first;

      size_t left = pos->second.left;
      size_t right = pos->second.right;

      std::cout << "[DONE]" << std::endl;
      std::cout << "New frame to add: " << right << std::endl;
      std::cout << "Verifying if number of matches is enough...";

      if(bestMatchSize < 60){
         matchesSizes.clear();
         std::cout << "[X]" << std::endl;
         continue;
      }
      std::cout << "[OK]" << std::endl;
      std::cout << "Found "<< bestMatchSize << " matches between frame pointCloud:"
                          << left << " and new frame:" << right << std::endl;
      std::cout << "Finding points 3D of frame pointCloud that match with the new frame..";
      pair={left,right};

      std::set<int> nDonePts;
      Matching bestMatch;

      if(left > right){
          bestMatch = nFeatureMatchMatrix[right][left];
      }else{
          bestMatch = nFeatureMatchMatrix[left][right];
      }

      for(const cv::DMatch& DMatchBestMatches : bestMatch){

        for(Point3D numPt3D : nReconstructionCloud){

           if(nDonePts.count(numPt3D.id) == 1){
               continue;
           }

           if(numPt3D.idxImage.count(left)==0){
              continue;
           }

           int matchedPoint = -1;

           if(left>right){
              if(DMatchBestMatches.trainIdx==numPt3D.idxImage[left]){
                  matchedPoint = DMatchBestMatches.queryIdx;
              }else{
                  continue;
              }

           }else{
              if(DMatchBestMatches.queryIdx==numPt3D.idxImage[left]){
                   matchedPoint = DMatchBestMatches.trainIdx;
              }else{
                   continue;
              }
          }

          matches2D3D.pts3D.push_back(numPt3D.pt);
          matches2D3D.pts2D.push_back(nFeaturesImages[right].pt2D[matchedPoint]);
          nDonePts.insert(numPt3D.id);
          break;
       }//End for-(vector point3D comparison)
     }//End for-(best matches vector comparison)

     if(matches2D3D.pts2D.size()< 80){
         std::cout << "\n"<< "Not found enough points for PnPRansac, found: "
                          << matches2D3D.pts2D.size() << std::endl;
         std::cout << "\n"<< "=============================== " << std::endl;
          matchesSizes.clear();
          continue;
     }else{

         std::cout << "[DONE]" << std::endl;
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

void StructFromMotion::findCameraPosePNP(const CameraData& matrixK,const std::vector<cv::Point3f>& pts3D,const std::vector<cv::Point2f>& pts2D,cv::Matx34f& P){

  cv::Mat rvec, T;
  cv::Mat inliers;
  double RANSAC_THRESHOLD=10.0f;

  std::cout << "Finding new camera pose..." << std::endl;

  cv::solvePnPRansac(pts3D,pts2D,matrixK.K,cv::Mat(),rvec,T,false,100,
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
    //adjustBundle(nReconstructionCloud,nCameraPoses,matrixK,nFeaturesImages);

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
    const float MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE = 30.0;

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
  double focal = matrixK.K.at<float>(0, 0); //Note: assuming fx = fy
  cv::Point2d pp(matrixK.K.at<float>(0, 2), matrixK.K.at<float>(1, 2));
  cv::Mat E = cv::findEssentialMat(alignedLeft.pt2D, alignedRight.pt2D,
                                   focal,pp,cv::RANSAC,0.999, 1.0,mask);

  // CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
  cv::Mat R,T;
  cv::recoverPose(E,alignedLeft.pt2D, alignedRight.pt2D,R,T,matrixK.fx,
                   cv::Point2d(matrixK.cx,matrixK.cy),mask);

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

  std::cout << "[DONE]" << std::endl;
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





