//***********************************************
//HEADERS
//***********************************************
#include "include/Sfm.h"

/********************************************
                  PIPELINE
********************************************/
bool StructFromMotion::map3D(){

  std::cout << "************************************************" << std::endl;
  std::cout << "              3D MAPPING                        " << std::endl;
  std::cout << "************************************************" << std::endl;

  if(nImages.size() <= 0) {
      std::cout << "No images to work on." << std::endl;
      return false;
  }

  // **(1) FEATURE DETECTION AND EXTRACTION - ALL IMAGES
  extractFeature();

  // **(3) BASE RECONSTRUCTION
  bool success = baseReconstruction();
  if(not success){
      std::cerr << "No could find a good pair for initial reconstruction" << std::endl;
      return false;
  }

  // **(6) VISUALIZER 3D MAPPING
  /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  fromPoint3DToPCLCloud(nReconstructionCloud,cloud);
  pcl::visualization::CloudViewer viewer("MAP3D");
  viewer.showCloud(cloud,"cloudSFM");

  while(!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
  }

*/
  // **(4) ADD MORE VIEWS
  success = addMoreViews();
  if(not success){
      std::cerr << "Could not add more views" << std::endl;
      return false;
  }

  // **(5) PMVS2
  PMVS2();

  // **(6) VISUALIZER 3D MAPPING
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  fromPoint3DToPCLCloud(nReconstructionCloud,cloud);
  pcl::visualization::CloudViewer viewer("MAP3D");
  viewer.showCloud(cloud,"cloudSFM");

  std::cout << "Press q to continue --> [DENSE PROCESS]..." << std::endl;
  while(!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
  }

  // **(7) DENSIFIYING POINTCLOUD
  int dont_care;
  dont_care = std::system("../../programs/pmvs2 denseCloud/ options.txt");
  if(dont_care > 0){
    std::cout << "Failed. ./pmvs2 no found" << std::endl;
    std::exit(-1);
  }

  // **(8) CONVERT PLY TO PCD
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPLY (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PLYReader ply;
  ply.read("denseCloud/models/options.txt.ply",*cloudPLY);
  if(cloudPLY->size()<=0){
      std::cout << "Could not densify cloud --> ply file is empty. ** PMVS2 failed." << std::endl;
      return false;
  }

  std::cout << "Densify proccess --> [OK]" << std::endl;
  std::cout << "Saving file with prefix --> MAP3D.pcd" << std::endl;
  pcl::io::savePCDFile("MAP3D.pcd",*cloudPLY);
  cloudPCL = cloudPLY;

  // **(9) VISUALIZER DENSE 3D MAPPING
  std::cout << "Visualizing MAP3D.pcd..." << std::endl;

  pcl::visualization::CloudViewer viewer2("DENSE MAP3D");
  viewer2.showCloud(cloudPLY,"denseCloud");

  while(!viewer2.wasStopped ()) { // Display the visualiser until 'q' key is pressed
  }

  std::cout << "Press q to continue --> [MESH CLOUD]..." << std::endl;
  // **(10) MESHING POINTCLOUD
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("MAP3D.pcd",*cloudXYZ);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PolygonMesh mesh;

  cloudPointFilter(cloudXYZ,filterCloud);
  removePoints(cloudXYZ,filterCloud);
  create_mesh(cloudXYZ,mesh);
  vizualizeMesh(mesh);

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
      //cv::COLOR_BGR2GRAY
      cv::Mat image   = cv::imread(imageFilename); //Read input image
      cv::Mat image_copy = image.clone(); //Copy image to temp variable
      cv::Mat resize;
      if(image_copy.rows > 480 and image_copy.cols > 640){
          cv::resize(image_copy,resize,cv::Size(),0.60,0.60); //Define a size of 640x480

          nImages.push_back(resize); //Save input image in nImages vector

          if(nImages.back().empty()) {
              std::cerr << "[x]"<<"\n" <<"Unable to read image from file: " << imageFilename << std::endl;
              return false;
          }
      }else{
          nImages.push_back(image_copy); //Save input image in nImages vector

          if(nImages.back().empty()) {
              std::cerr << "[x]"<<"\n" <<"Unable to read image from file: " << imageFilename << std::endl;
              return false;
          }
      }
  }

  if(nImages.size()<2){
      std::cerr << "Sorry. is not enough images, 6 minimum" << std::endl;
      return false;
  }

  for(unsigned int i = 0; i < nImages.size(); i++){

      mColorImages.push_back(cv::Mat_<cv::Vec3b>());
      if(!nImages[i].empty()){
          if(nImages[i].type() == CV_8UC1){

              cv::cvtColor(nImages[i], mColorImages[i], CV_GRAY2BGR);
          }else if(nImages[i].type() == CV_32FC3 || nImages[i].type() == CV_64FC3){

              nImages[i].convertTo(mColorImages[i], CV_8UC3, 255.0);
          }else{

              nImages[i].copyTo(mColorImages[i]);
          }
      }

      mGrayImages.push_back(cv::Mat());
      cv::cvtColor(mColorImages[i], mGrayImages[i], CV_BGR2GRAY);
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

    /*Read camera calibration file*/
    cv::FileStorage fs(str, cv::FileStorage::READ);

    /*Get data from tags: Camera_Matrix and Distortion_Coefficients*/
    fs["Camera_Matrix"] >> intrinsics;
    fs["Distortion_Coefficients"] >> cameraDistCoeffs;

    if(intrinsics.empty() or intrinsics.at<double>(2,0) !=0){
        std::cerr << "Error: no found or invalid camera calibration file.xml" << std::endl;
        return false;
    }

    double fx = intrinsics.at<double>(0,0);
    double fy = intrinsics.at<double>(1,1);
    double cx = intrinsics.at<double>(0,2);
    double cy = intrinsics.at<double>(1,2);

    cv::Mat_<double> cam_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                                             0, fy, cy,
                                                             0,  0,  1);

    double k1 = cameraDistCoeffs.at<double>(0,0);
    double k2 = cameraDistCoeffs.at<double>(0,1);
    double k3 = cameraDistCoeffs.at<double>(0,2);
    double p1 = cameraDistCoeffs.at<double>(0,3);
    double p2 = cameraDistCoeffs.at<double>(0,4);

    cv::Mat_<double> distortionC = (cv::Mat_<double>(1, 5) << k1, k2, k3, p1, p2);

    /*Fill local variables with input data*/
    cameraMatrix.K = cam_matrix;                  //Matrix K (3x3)
    cameraMatrix.distCoef = distortionC;     //Distortion coefficients (1x5)

    std::cout << "Camera matrix:" << "\n" << cameraMatrix.K << std::endl;
    std::cout <<"Distortion coefficients: "<< std::endl;
    std::cout << cameraMatrix.distCoef << std::endl;

    if(cameraMatrix.K.empty()){
        std::cout << "Could not load local variables with camera calibration file data" << std::endl;
        return false;
    }

    return true;
}

//===============================================
// Extract feature
//===============================================
void StructFromMotion::extractFeature(){

  std::cout << "Extracting features from all images..." << std::endl;
  nCameraPoses.resize(mGrayImages.size());
  imagesKeypoints.resize(mGrayImages.size(),std::vector<cv::KeyPoint>());
  imagesDescriptors.resize(mGrayImages.size(),cv::Mat());
  imagesPts2D.resize(mGrayImages.size(),std::vector<cv::Point2d>());

  if(detector == 1){
      std::cout << "No detector choose. Using default:" << "SIFT(Scale-Invariant Feature Transform) detector."
                << "\n" << "Parameters:" << "\n" << "nFeatures = 0\n" << "nOctaveLayers = 3\n"
                << "contrastThreshold = 0.04\n" << "edgeThreshold = 10\n" << "sigma = 1.6"  << std::endl;
  }else if(detector == 2){
      std::cout << "AKAZE detector choose." << std::endl;
  }else if(detector == 3){
      std::cout << "ORB detector choose." << std::endl;
  }

  std::cout << "*-- Features --*" << std::endl;
  cv::namedWindow("Image kps",CV_WINDOW_NORMAL);
  cv::resizeWindow("Image kps",640,480);
  cv::moveWindow("Image kps",0,0);

  for(size_t n=0;n<mGrayImages.size();n++){

      const cv::Mat image = mGrayImages.at(n);
      getFeature(image,n);
      std::cout << "Image:" << n << " --> " << imagesKeypoints.at(n).size() << " kps" << std::endl;
      cv::Mat imageKps;
      cv::drawKeypoints(image,imagesKeypoints.at(n),imageKps,cv::Scalar::all(-1),0);
      cv::rectangle(imageKps,cv::Point(10,imageKps.rows-25),cv::Point(imageKps.cols/8,
                                                                      imageKps.rows-5),
                    cv::Scalar(0,255,0),CV_FILLED);
      cv::putText(imageKps, "Image" + std::to_string(n),
                  cv::Point(10,imageKps.rows-11),cv::FONT_ITALIC,imageKps.cols/imageKps.rows,cv::Scalar(0,0,0),2);
      cv::imshow("Image kps",imageKps);
      cv::waitKey(100);

   }
  cv::destroyAllWindows();
  //cv::destroyWindow("Image kps");
}

//===============================================
//Get Feature
//===============================================
void StructFromMotion::getFeature(const cv::Mat& image,const int& numImage){

  while(true){

    if(detector == 1){

      int nfeatures=0;
      int nOctaveLayers=3;
      double contrastThreshold=0.04;
      double edgeThreshold=10;
      double sigma=1.6;

      cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(nfeatures,nOctaveLayers,
                                                                  contrastThreshold,edgeThreshold,sigma);

      std::vector<cv::KeyPoint> kps;
      cv::Mat descriptors;
      sift->detectAndCompute(image,cv::noArray(),kps,descriptors,false);

      std::vector<cv::Point2d> points2d;
      keypointstoPoints(kps,points2d);

      imagesKeypoints[numImage]= kps;
      imagesDescriptors[numImage]=descriptors;
      imagesPts2D[numImage]=points2d;

      break;

    }else if(detector == 2){

        int descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
        int descriptor_size = 0;
        int descriptor_channels = 3;
        float threshold = 0.001f;
        int nOctaves = 4;
        int nOctaveLayers = 4;
        int diffusivity = cv::KAZE::DIFF_PM_G2;

        cv::Ptr<cv::AKAZE> akaze= cv::AKAZE::create(descriptor_type,descriptor_size,
                                                       descriptor_channels,threshold,nOctaves,
                                                       nOctaveLayers,diffusivity);

        std::vector<cv::KeyPoint> kps;
        cv::Mat descriptors;
        akaze->detectAndCompute(image,cv::noArray(),kps,descriptors,false);

        std::vector<cv::Point2d> points2d;
        keypointstoPoints(kps,points2d);

        imagesKeypoints[numImage]= kps;
        imagesDescriptors[numImage]=descriptors;
        imagesPts2D[numImage]=points2d;

        break;

    }else if(detector == 3){

        int nfeatures = 500;
        float scaleFactor = 1.2f;
        int nlevels = 8;
        int edgeThreshold = 31;
        int firstLevel = 0;
        int WTA_K = 2;
        int scoreType = cv::ORB::HARRIS_SCORE;
        int patchSize = 31;
        int fastThreshold = 20;

        cv::Ptr<cv::ORB> detector= cv::ORB::create(nfeatures,scaleFactor,nlevels,edgeThreshold,firstLevel,
                                                   WTA_K,scoreType,patchSize,fastThreshold);

        std::vector<cv::KeyPoint> kps;
        cv::Mat descriptors;
        detector->detectAndCompute(image,cv::noArray(),kps,descriptors,false);

        std::vector<cv::Point2d> points2d;
        keypointstoPoints(kps,points2d);

        imagesKeypoints[numImage]= kps;
        imagesDescriptors[numImage]=descriptors;
        imagesPts2D[numImage]=points2d;

        break;

    }else{

        std::cout << "No feature detector choose." << std::endl;
        break;
    }
  }
}

//===============================================
//FUNCTION: KEYPOINTS TO POINTS2D
//===============================================
void StructFromMotion::keypointstoPoints(Keypoints& keypoints, Points2d& points2D){

  points2D.clear();
  for(const cv::KeyPoint& kp: keypoints){
         points2D.push_back(kp.pt);
   }
}

//===============================================
//FUNCTION: BASE RECONSTRUCTION
//===============================================
bool StructFromMotion::baseReconstruction(){

  std::map<float,std::pair<int,int>> bestViews = findBestPair();
  if(bestViews.size() <= 0){
     std::cout << "Could not obtain a good pair for baseline reconstruction." << std::endl;
     return false;
  }

  cv::namedWindow("Best pair matching",CV_WINDOW_NORMAL);
  cv::resizeWindow("Best pair matching",640*2,480);
  cv::moveWindow("Best pair matching",0,0);

  for(std::pair<const float,std::pair<int,int>>& bestpair : bestViews){

      int queryImage = bestpair.second.first;
      int trainImage = bestpair.second.second;

      Matching bestMatch;
      getMatching(queryImage,trainImage,&bestMatch);
      //MatchFeatures(queryImage,trainImage,&bestMatch);

      std::cout << "Best pair:" << "["<< queryImage << "," << trainImage<<"]" << " has:"
                << bestMatch.size() << " matches" << " and " << bestpair.first << " inliers." << std::endl;

      cv::Matx34d Pleft  = cv::Matx34d::eye();
      cv::Matx34d Pright = cv::Matx34d::eye();

      std::cout << "Estimating camera pose with Essential Matrix..." << std::endl;
      bool success = getCameraPose(cameraMatrix,queryImage,trainImage,bestMatch,imagesPts2D.at(queryImage),
                                   imagesPts2D.at(trainImage),Pleft,Pright);

      if(not success){      
         std::cerr << "Failed. stereo view could not be obtained " << queryImage << "," << trainImage
                   << ", something wrong." << std::endl;
         continue;
      }

      std::cout << "Camera:" << queryImage << "\n" << Pleft << std::endl;
      std::cout << "Camera:" << trainImage <<"\n" << Pright << std::endl;
      std::cout << "Showing matches between "<< "image:" << queryImage << " and image:"
                << trainImage << std::endl;

      cv::Mat matchImage;
      cv::drawMatches(mGrayImages.at(queryImage),imagesKeypoints.at(queryImage),mGrayImages.at(trainImage),
                      imagesKeypoints.at(trainImage),bestMatch,matchImage,
                      cv::Scalar::all(-1),cv::Scalar::all(-1),std::vector<char>(),2);
      cv::rectangle(matchImage,cv::Point(10,matchImage.rows-25),cv::Point(80,matchImage.rows-5),
                    cv::Scalar(0,255,0),CV_FILLED);
      cv::putText(matchImage, "Image" + std::to_string(queryImage),
                  cv::Point(10,matchImage.rows-11),cv::FONT_ITALIC,0.5,cv::Scalar(0,0,0),2);
      cv::rectangle(matchImage,cv::Point(670,matchImage.rows-25),cv::Point(740,matchImage.rows-5),
                    cv::Scalar(0,255,0),CV_FILLED);
      cv::putText(matchImage, "Image" + std::to_string(trainImage),
                  cv::Point(670,matchImage.rows-11),cv::FONT_ITALIC,0.5,cv::Scalar(0,0,0),2);
      cv::imshow("Best pair matching", matchImage);
      cv::waitKey(0);
      cv::destroyAllWindows();

      std::vector<Point3D> pointcloud;

      success = triangulateViews(imagesPts2D.at(queryImage),imagesPts2D.at(trainImage),
                                 Pleft,Pright,bestMatch,cameraMatrix,
                                 std::make_pair(queryImage,trainImage),pointcloud);

      if(not success){
          std::cerr << "Could not triangulate image:" << queryImage << " and image:"<< trainImage
                    << std::endl;
          continue;
      }

      nReconstructionCloud = pointcloud;

      nCameraPoses[queryImage] = Pleft;
      nCameraPoses[trainImage] = Pright;

      nDoneViews.insert(queryImage);
      nDoneViews.insert(trainImage);

      nGoodViews.insert(queryImage);
      nGoodViews.insert(trainImage);

      break;
  }  

  //adjustCurrentBundle();
  return true;
}

//===============================================
//BEST PAIR FOR BASELINE
//===============================================
std::map<float,std::pair<int,int>>  StructFromMotion::findBestPair(){

  std::cout << "Getting best two views for baseline..." << std::endl;
  std::map<float,std::pair<int,int>> numInliers;
  const size_t numImg = nImages.size();

  cv::namedWindow("Matching pairs",CV_WINDOW_NORMAL);
  cv::resizeWindow("Matching pairs",640*2,480);
  cv::moveWindow("Matching pairs",0,0);

  auto start = std::chrono::high_resolution_clock::now();

  for(int queryImage=0;queryImage<numImg-1;queryImage++) {
     for(int trainImage=queryImage+1;trainImage<numImg;trainImage++){

        Matching correspondences;
        getMatching(queryImage,trainImage,&correspondences);
        //MatchFeatures(queryImage,trainImage,&correspondences);

        cv::Mat matchImage;
        cv::drawMatches(mGrayImages[queryImage],imagesKeypoints.at(queryImage),mGrayImages[trainImage],
                        imagesKeypoints.at(trainImage),correspondences,matchImage,
                        cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 255));
        cv::rectangle(matchImage,cv::Point(10,matchImage.rows-25),cv::Point(80,matchImage.rows-5),
                      cv::Scalar(0,255,0),CV_FILLED);
        cv::putText(matchImage, "Image" + std::to_string(queryImage),
                    cv::Point(10,matchImage.rows-11),cv::FONT_ITALIC,0.5,cv::Scalar(0,0,0),2);
        cv::rectangle(matchImage,cv::Point(670,matchImage.rows-25),cv::Point(740,matchImage.rows-5),
                      cv::Scalar(0,255,0),CV_FILLED);
        cv::putText(matchImage, "Image" + std::to_string(trainImage),
                    cv::Point(670,matchImage.rows-11),cv::FONT_ITALIC,0.5,cv::Scalar(0,0,0),2);
        cv::imshow("Matching pairs", matchImage);
        cv::waitKey(100);

        if(correspondences.size() < 120 ) continue;

        Points2d alignedLeft,alignedRight;
        AlignedPointsFromMatch(imagesPts2D.at(queryImage),imagesPts2D.at(trainImage),
                               correspondences,alignedLeft,alignedRight);

        // ESSENTIAL MATRIX
        cv::Mat mask;
        cv::Mat cam_matrix = cv::Mat(cameraMatrix.K);
        cv::Mat E = cv::findEssentialMat(alignedLeft, alignedRight,
                                         cam_matrix,CV_RANSAC,0.999, 1.0,mask);

        int numHomInli = findHomographyInliers(queryImage,trainImage,correspondences);

        std::vector<cv::DMatch> prunedMatch;

        for(unsigned i = 0; i < mask.rows; i++) {
            if(mask.at<uchar>(i)) {
                prunedMatch.push_back(correspondences[i]);
            }
        }

       // float percent = (float)(((float)numHomInli) / ((float)correspondences.size()) * 100.0);

        //std::cout << "image:" << queryImage << " and image:" << trainImage << " has " << numHomInli
          //        << " inliers" << std::endl;

        //if(numHomInli < 60) continue;


        float poseInliersRatio = (float)prunedMatch.size() / (float)correspondences.size();

        //if(poseInliersRatio>0.01 or poseInliersRatio<0.005) continue;

        std::cout << "pair:" << "[" << queryImage << "," << trainImage << "]" << " has:" << correspondences.size() << " matches " << numHomInli << " inliers and " <<poseInliersRatio << " pose inliers ratio."<< std::endl;

        numInliers[poseInliersRatio]=std::make_pair(queryImage,trainImage);
     }
  }


  cv::destroyAllWindows();
   auto end = std::chrono::high_resolution_clock::now();
   auto difference = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();

   if(difference >=60){
       std::cout << "Time for matching: " << difference/60 << " minutes" << std::endl;
       return numInliers;
   }

   std::cout << "Seconds matching: " << difference << " seconds" << std::endl;
  return numInliers;
}

//===============================================
//FUNCTION: FEATURE MATCHING
//===============================================
void StructFromMotion::getMatching(const int& idx_query,const int& idx_train,Matching* goodMatches){

  /*Knn matching*/
  cv::BFMatcher* matcher = new cv::BFMatcher(cv::NORM_L2, false);
  //std::vector<cv::KeyPoint> query_kps = imagesKeypoints.at(idx_query);
  //std::vector<cv::KeyPoint> train_kps = imagesKeypoints.at(idx_train);
  std::vector<std::vector<cv::DMatch>> knnMatches;
  cv::Mat query_descriptor = imagesDescriptors.at(idx_query);
  cv::Mat train_descriptor = imagesDescriptors.at(idx_train);
  matcher->knnMatch(query_descriptor,train_descriptor,knnMatches,2);

  /*RATIO-TEST FILTER*/

  for(unsigned i = 0; i < knnMatches.size(); i++) {
      if(knnMatches[i][0].distance <= NN_MATCH_RATIO * knnMatches[i][1].distance) {
          goodMatches->push_back(knnMatches[i][0]);
      }
  }
}

void StructFromMotion::prunedMatchingWithHomography(const int& idx_query, const int& idx_train,
                                                    const Matching& goodMatches,Matching* prunedMatch){

  std::vector<cv::KeyPoint> matched1,matched2;
  std::vector<cv::KeyPoint> query_kps = imagesKeypoints.at(idx_query);
  std::vector<cv::KeyPoint> train_kps = imagesKeypoints.at(idx_train);

  for(unsigned i = 0; i < goodMatches.size(); i++) {
      matched1.push_back(query_kps[goodMatches[i].queryIdx]);
      matched2.push_back(train_kps[goodMatches[i].trainIdx]);
  }

  std::vector<cv::Point2d> query_points,train_points;
  keypointstoPoints(matched1,query_points);
  keypointstoPoints(matched2,train_points);

  /*RANSAC FILTER*/
  const double ransac_thresh = 2.5;
  cv::Mat inliers_mask, homography;
  //std::vector<cv::KeyPoint> inliers1, inliers2;

  if(matched1.size() >= 4){
     homography = cv::findHomography(query_points,train_points,CV_RANSAC, ransac_thresh, inliers_mask);
  }

  std::cout << "Homography inliers mask:" << inliers_mask.rows << " inliers" <<std::endl;

  for(unsigned i = 0; i < inliers_mask.rows; i++) {
      if(inliers_mask.at<uchar>(i)) {
          //int new_i = static_cast<int>(inliers1.size());
          //inliers1.push_back(matched1[i]);
          //inliers2.push_back(matched2[i]);
          //goodMatches->push_back(cv::DMatch(new_i, new_i, 0));
          prunedMatch->push_back(goodMatches[i]);
      }
  }

  /*
  imagesKeypoints.at(idx_query).clear();
  imagesKeypoints.at(idx_train).clear();
  imagesKeypoints.at(idx_query) = inliers1;
  imagesKeypoints.at(idx_train) = inliers2;
  */
  /*
  cv::Mat res;
  cv::drawMatches(mGrayImages.at(idx_query), inliers1, mGrayImages.at(idx_train), inliers2,
              goodMatches, res,
              cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 0));
  cv::imshow("Good matches",res);
  cv::waitKey(0);
  */

}

//===============================================
//FUNCTION: FIND HOMOGRAPHY INLIERS
//===============================================
int StructFromMotion::findHomographyInliers(const int& idx_query,const int& idx_train,const Matching& matches){

  std::vector<cv::Point2d> query_points;
  std::vector<cv::Point2d> train_points;
  AlignedPointsFromMatch(imagesPts2D.at(idx_query),imagesPts2D.at(idx_train),matches,query_points,train_points);

  double minVal,maxVal;
  cv::minMaxIdx(query_points,&minVal,&maxVal);

  cv::Mat matrixH(3,3,CV_32FC3);
  cv::Mat inliersMask;
  //0.004 * maxVal

  if(matches.size()>=4){
       matrixH = cv::findHomography(query_points,train_points,CV_RANSAC,0.004 * maxVal,inliersMask);
  }

  if(matches.size() < 4 || matrixH.empty()) {
          return 0;
  }

  return cv::countNonZero(inliersMask);
}

//===============================================
//FUNCTION: ALIGNED POINTS
//===============================================
void StructFromMotion::AlignedPointsFromMatch(const Points2d& queryImg,const Points2d& trainImg,const Matching& matches,Points2d& alignedL,Points2d& alignedR){

   std::vector<int> leftId,rightId;
   AlignedPoints(queryImg,trainImg,matches,alignedL,alignedR,leftId,rightId);  
}

void StructFromMotion::AlignedPoints(const Points2d& queryImg,const Points2d& trainImg,const Matching& matches, Points2d& alignedL, Points2d& alignedR,std::vector<int>& idLeftOrigen,std::vector<int>& idRightOrigen){

      //align left and right point sets
      for(unsigned int i=0;i<matches.size();i++){

        alignedL.push_back(queryImg[matches[i].queryIdx]);
        alignedR.push_back(trainImg[matches[i].trainIdx]);

        idLeftOrigen.push_back(matches[i].queryIdx);
        idRightOrigen.push_back(matches[i].trainIdx);
      }
}

bool StructFromMotion::getCameraPose(const Intrinsics& intrinsics,const int& idx_query,const int& idx_train,
                                     const Matching & matches,
                                     const Points2d& left, const Points2d& right,
                                     cv::Matx34d& Pleft, cv::Matx34d& Pright){

  if (intrinsics.K.empty()) {

      std::cerr << "Intrinsics matrix (K) must be initialized." << std::endl;
      return false;
  }

  Matching prunedMatches;
  prunedMatchingWithHomography(idx_query,idx_train,matches,&prunedMatches);

  std::cout << "pruned matches:" << prunedMatches.size() << std::endl;

  Points2d alignedLeft,alignedRight;
  AlignedPointsFromMatch(left,right,matches,alignedLeft,alignedRight);

  std::cout << "aligned: " << alignedLeft.size() << " and " << alignedRight.size() << std::endl;

  if(alignedLeft.size()<=7 or alignedRight.size()<=7){
      std::cout << "Sorry. not enough points for findEssentialMat function. matches size is "
                << prunedMatches.size() << std::endl;
      return false;
  }

  // ESSENTIAL MATRIX
  cv::Mat mask;
  cv::Mat cam_matrix = cv::Mat(intrinsics.K);
  cv::Mat E = cv::findEssentialMat(alignedLeft, alignedRight,
                                   cam_matrix,CV_RANSAC,0.999, 1.0,mask);

  std::cout << "Essential matrix:\n" << E << std::endl;

  // CAMERA POSE -> Rotation and Traslation (MOTION ESTIMATION)
  cv::Mat R,T;
  double fx = intrinsics.K.at<double>(0,0);
  double cx = intrinsics.K.at<double>(0,2);
  double cy = intrinsics.K.at<double>(1,2);
  cv::Point2d pp = cv::Point2d(cx,cy);

  cv::recoverPose(E,alignedLeft, alignedRight,R,T,fx,pp,mask);

  bool success = CheckCoherentRotation(R);

  std::cout << "R:\n" << R << std::endl;
  std::cout << "T:\n" << T << std::endl;

  if(not success){

      std::cerr << "Bad rotation." << std::endl;
      return false;
  }

  Pright = cv::Matx34d(R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),T.at<double>(0),
                       R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),T.at<double>(1),
                       R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),T.at<double>(2));

  Pleft  = cv::Matx34d::eye();

  std::cout << "Pright:\n" << Pright << std::endl;

  /*
  //Rotational element in a 3x4 matrix
  const cv::Rect ROT(0, 0, 3, 3);
  //Translational element in a 3x4 matrix
  const cv::Rect TRA(3, 0, 1, 3);
  R.copyTo(cv::Mat(3, 4, CV_32FC1, Pright.val)(ROT));
  T.copyTo(cv::Mat(3, 4, CV_32FC1, Pright.val)(TRA));
  */

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
bool StructFromMotion::triangulateViews(const Points2d& query,const Points2d& train,const cv::Matx34d& P1,const cv::Matx34d& P2,const Matching& matches,const Intrinsics& matrixK,const std::pair<int,int>& pair,std::vector<Point3D>& pointcloud){

  std::cout << "-----------------------------------------------" << std::endl;
  std::cout << "Triangulating image:" << pair.first << " and image:" << pair.second << std::endl;
  std::cout << "** IMAGE COORDINATE - CAMERA COORDINATE CONVERTION **" << std::endl;

  pointcloud.clear();

  Points2d alignedQuery,alignedTrain;
  std::vector<int> leftBackReference,rightBackReference;
  AlignedPoints(query,train,matches,alignedQuery,alignedTrain,
                                  leftBackReference,rightBackReference);

  // NORMALIZE IMAGE COORDINATE TO CAMERA COORDINATE (pixels --> metric)
  std::cout << "Normalizing points..." << std::endl;
  cv::Mat normalizedLeftPts,normalizedRightPts;
  cv::undistortPoints(alignedQuery, normalizedLeftPts, matrixK.K,matrixK.distCoef);
  cv::undistortPoints(alignedTrain, normalizedRightPts, matrixK.K, matrixK.distCoef);

  // TRIANGULATE POINTS
  std::cout << "Triangulating points..." << std::endl;
  cv::Mat pts3dHomogeneous;
  cv::triangulatePoints(P1,P2,normalizedLeftPts,normalizedRightPts,pts3dHomogeneous);

  std::cout << "** CAMERA COORDINATE - WORLD COORDINATE CONVERTION **" << std::endl;

  // CONVERTION CAMERA COORDINATE - WORLD COORDINATE
  std::cout << "Converting points to world coordinate..." << std::endl;
  cv::Mat pts3d;
  cv::convertPointsFromHomogeneous(pts3dHomogeneous.t(),pts3d);

  cv::Mat rvecLeft;
  cv::Rodrigues(P1.get_minor<3,3>(0,0),rvecLeft);
  cv::Mat tvecLeft(P1.get_minor<3,1>(0,3).t());

  Points2d projectedLeft(alignedQuery.size());
  cv::projectPoints(pts3d,rvecLeft,tvecLeft,matrixK.K,matrixK.distCoef,projectedLeft);

  cv::Mat rvecRight;
  cv::Rodrigues(P2.get_minor<3,3>(0,0),rvecRight);
  cv::Mat tvecRight(P2.get_minor<3,1>(0,3).t());

  Points2d projectedRight(alignedTrain.size());
  cv::projectPoints(pts3d,rvecRight,tvecRight,matrixK.K,matrixK.distCoef,projectedRight);

  std::cout << "Creating a pointcloud vector..." << std::endl;
  const float MIN_REPROJECTION_ERROR = 6.0; //Maximum 10-pixel allowed re-projection error

  for(int i = 0; i < pts3d.rows; i++){

      //check if point reprojection error is small enough

      const float queryError = cv::norm(projectedLeft[i]  - alignedQuery[i]);
      const float trainError = cv::norm(projectedRight[i] - alignedTrain[i]);

      if(MIN_REPROJECTION_ERROR < queryError or
         MIN_REPROJECTION_ERROR < trainError) continue;

          Point3D p;
          p.pt = cv::Point3d(pts3d.at<double>(i, 0),
                             pts3d.at<double>(i, 1),
                             pts3d.at<double>(i, 2));

          //use back reference to point to original Feature in images
          p.idxImage[pair.first]  = leftBackReference[i];
          p.idxImage[pair.second] = rightBackReference[i];
          p.pt2D[pair.first]=imagesPts2D.at(pair.first).at(leftBackReference[i]);
          p.pt2D[pair.second]=imagesPts2D.at(pair.second).at(rightBackReference[i]);

          pointcloud.push_back(p);
  }

  std::cout << "New triangulated points: " << pointcloud.size() << " 3d pts" << std::endl;
  return true;
}

//===============================================
//BUNDLE ADJUSTMENT
//===============================================
void StructFromMotion::adjustCurrentBundle() {

  std::cout << "Bundle adjuster..." << std::endl;
  //BundleAdjustment::adjustBundle(nReconstructionCloud,nCameraPoses,cameraMatrix,nFeatureImages);

}

//===============================================
//FUNCTION: ADD MORE VIEWS
//===============================================
bool StructFromMotion::addMoreViews(){

      std::vector<cv::Point3d> points3D;
      std::vector<cv::Point2d> points2D;

      while(nDoneViews.size() != mGrayImages.size()){

      std::set<int> newFrames;
      for(int newViewstoAdd:nDoneViews){

               int i;
               int j;

               if(newViewstoAdd==0){
                   i=newViewstoAdd;
                   j=std::abs(newViewstoAdd+1);
                 }else if(newViewstoAdd==nImages.size()){
                   i=std::abs(newViewstoAdd-1);
                   j=newViewstoAdd;
                 }else{
                   i=std::abs(newViewstoAdd -1);
                   j=std::abs(newViewstoAdd+1);
                 }

               if(nDoneViews.count(i)== 1){
                   if(nDoneViews.count(j)== 1){
                       continue;
                     }else{
                        newFrames.insert(j);
                     }
               }else{
                   newFrames.insert(i);
                   if(nDoneViews.count(j)== 1){
                       continue;
                    }else{
                       newFrames.insert(j);
                    }
               }
      }

     for(int NEW_VIEW : newFrames){
      //for(int NEW_VIEW = 0;NEW_VIEW<nImages.size();NEW_VIEW++){

          //if(nDoneViews.count(NEW_VIEW)==1) continue; //Skip done views
          if(nDoneViews.find(NEW_VIEW)!=nDoneViews.end())continue;

          std::cout <<"\n"<< "===================================="<< std::endl;
          std::cout << "ESTIMATING MORE CAMERAS PROJECTION..." << std::endl;
          std::cout << "Extracting 2d3d correspondences..." << std::endl;
          std::cout << "Possible view:" << " image --> " << NEW_VIEW << std::endl;

          Matching bestMatches;
          int DONE_VIEW;
          find2D3DMatches(NEW_VIEW,points3D,points2D,bestMatches,DONE_VIEW);
          std::cout << "Adding " << NEW_VIEW << " to existing "
                    << cv::Mat(std::vector<int>(nDoneViews.begin(), nDoneViews.end())).t() << std::endl;
          nDoneViews.insert(NEW_VIEW);

          std::cout << "Estimating camera pose..." << std::endl;
          cv::Matx34d newCameraPose = cv::Matx34d::eye();
          bool success = findCameraPosePNP(cameraMatrix,points3D,points2D,newCameraPose);

          if(not success){
             std::cout << "Failed. Could not get a good pose estimation. skip view" << std::endl;
             continue;
          }                   

          nCameraPoses[NEW_VIEW] = newCameraPose;

          std::vector<Point3D> new_triangulated;

          for(int good_view : nGoodViews){

              int queryImage,trainImage;

              if(NEW_VIEW < good_view){
                  queryImage = NEW_VIEW;
                  trainImage = good_view;
              }else{
                  queryImage = good_view;
                  trainImage = NEW_VIEW;
              }

              Matching matches;
              getMatching(queryImage,trainImage,&matches);

              bool good_triangulation = triangulateViews(imagesPts2D.at(queryImage),imagesPts2D.at(trainImage),
                                                         nCameraPoses[queryImage],nCameraPoses[trainImage],
                                                         matches,cameraMatrix,
                                                         std::make_pair(queryImage,trainImage),new_triangulated);

              if(not good_triangulation){
                continue;
              }

              std::cout << "Before triangulation: " << nReconstructionCloud.size() << std::endl;;
              mergeNewPoints(new_triangulated);
              std::cout << "After triangulation: " << nReconstructionCloud.size() << std::endl;

              //break
          }

          nGoodViews.insert(NEW_VIEW);
          adjustCurrentBundle();
      }

     continue;
 }
 std::cout << "\n"<< "=============================== " << std::endl;
 std::cout << "Images processed = " << nDoneViews.size() << " of " << nImages.size() << std::endl;
 std::cout << "PointCloud size = " << nReconstructionCloud.size() << " pts3D" << std::endl;

 return true;
}

//===============================================
//FUNCTION: FIND CORRESPONDENCES 2D-3D
//===============================================
void StructFromMotion::find2D3DMatches(const int& NEW_VIEW,
                                       std::vector<cv::Point3d>& points3D,
                                       std::vector<cv::Point2d>& points2D,Matching& bestMatches,int& DONEVIEW){

     points3D.clear(); points2D.clear();
     int queryImage,trainImage;
     int bestNumMatches = 0;
     Matching bestMatch;

     for(int doneView : nDoneViews){

         if(NEW_VIEW < doneView){
             queryImage = NEW_VIEW;
             trainImage = doneView;
         }else{
             queryImage = doneView;
             trainImage = NEW_VIEW;
         }

         Matching match;
         getMatching(queryImage,trainImage,&match);
         //const Matching match = matchingFor2D3D(queryImage,trainImage);
         //Matching match;
         //MatchFeatures(queryImage,trainImage,&match);

         int numMatches = match.size();
         if(numMatches > bestNumMatches) {
            bestMatch       = match;
            bestNumMatches = numMatches;
            DONEVIEW = doneView;
         }
     }

     bestMatches = bestMatch;

     //scan all cloud 3D points
     for(const Point3D& cloudPoint : nReconstructionCloud){

         bool found2DPoint = false;
         //scan all originating views for that 3D point
         for(const std::pair<const int,int>& origViewAndPoint : cloudPoint.idxImage){

             //check for 2D-2D matching
             const int originatingViewIndex      = origViewAndPoint.first;
             const int originatingViewFeatureIndex = origViewAndPoint.second;

             if(originatingViewIndex != DONEVIEW)continue;

             //scan all 2D-2D matches between originating view and new view
             for(const cv::DMatch& m : bestMatch){

                 int matched2DPointInNewView = -1;
                 if(originatingViewIndex < NEW_VIEW) { //originating view is 'left'

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
                     const std::vector<cv::Point2d> newViewFeatures = imagesPts2D[NEW_VIEW];
                     points2D.push_back(newViewFeatures.at(matched2DPointInNewView));
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

bool StructFromMotion::findCameraPosePNP(const Intrinsics& intrinsics,const std::vector<cv::Point3d>& pts3D,const std::vector<cv::Point2d>& pts2D,cv::Matx34d& P){

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

  std::vector<cv::Point2d> projected3D;
  cv::projectPoints(pts3D, rvec, T, intrinsics.K,intrinsics.distCoef, projected3D);

  if(inliers.size() == 0) { //get inliers
     for(int i = 0; i < projected3D.size(); i++) {
         if(cv::norm(projected3D[i] - pts2D[i]) < 8.0){
             inliers.push_back(i);

          }
     }
  }
/*
  if(inliers.size() < (double)(pts2D.size()) / 5.0) {

      std::cerr << "not enough inliers to consider a good pose (" << inliers.size() << "/"
                << pts2D.size() << ")" << std::endl;
      return false;
  }
*/
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

  P = cv::Matx34d(R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),T.at<double>(0),
                  R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),T.at<double>(1),
                  R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),T.at<double>(2));

  /*
  //Rotational element in a 3x4 matrix
  const cv::Rect ROT(0, 0, 3, 3);

  //Translational element in a 3x4 matrix
  const cv::Rect TRA(3, 0, 1, 3);

  R.copyTo(cv::Mat(3, 4, CV_32FC1, P.val)(ROT));
  T.copyTo(cv::Mat(3, 4, CV_32FC1, P.val)(TRA));
  */
  std::cout << "new P:\n" << P << std::endl;

  return true;

}

void StructFromMotion::mergeNewPoints(const std::vector<Point3D>& newPointCloud) {

  std::cout << "Adding new points..." << std::endl;

  const float MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE   = 0.01;
//  const float MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE = 20.0;

      size_t newPoints = 0;
 //     size_t mergedPoints = 0;

      for(const Point3D& p : newPointCloud) {
          const cv::Point3d newPoint = p.pt; //new 3D point

          bool foundAnyMatchingExistingViews = false;
          bool foundMatching3DPoint = false;
          for(Point3D& existingPoint : nReconstructionCloud) {
              if(cv::norm(existingPoint.pt - newPoint) < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE){
                  //This point is very close to an existing 3D cloud point
                  foundMatching3DPoint = true;
                  break;

               }
          }

          if(not foundAnyMatchingExistingViews and not foundMatching3DPoint) {
              //This point did not match any existing cloud points - add it as new.
              nReconstructionCloud.push_back(p);
              newPoints++;
          }
      }

      std::cout << "New points:" << newPoints << std::endl;
}

void StructFromMotion::PMVS2(){

  /*FOLDERS FOR PMVS2*/
  std::cout << "Creating folders for PMVS2..." << std::endl;
  int dont_care;
  dont_care = std::system("mkdir -p denseCloud/visualize");
  dont_care = std::system("mkdir -p denseCloud/txt");
  dont_care = std::system("mkdir -p denseCloud/models");
  std::cout << "Created: \nfolder:visualize" << "\n" << "folder:txt" << "\n" << "folder:models" << std::endl;

  /*OPTIONS CONFIGURATION FILE FOR PMVS2*/
  std::cout << "Creating options file for PMVS2..." << std::endl;
  ofstream option("denseCloud/options.txt");
  option << "minImageNum 5" << std::endl;
  option << "CPU 4" << std::endl;
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

      std::sprintf(str, "cp -f %s denseCloud/visualize/%04d.jpg", nImagesPath[i].c_str(), (int)i);
      dont_care = std::system(str);
      cv::imwrite(str, nImages[i]);

      std::sprintf(str, "denseCloud/txt/%04d.txt", (int)i);
      ofstream ofs(str);
      cv::Matx34d pose = nCameraPoses[i];

      //K*P
      pose = (cv::Matx33d)cameraMatrix.K*pose;

      ofs << "CONTOUR" << std::endl;
      ofs << pose(0,0) << " " << pose(0,1) << " " << pose(0,2) << " " << pose(0,3) << "\n"
          << pose(1,0) << " " << pose(1,1) << " " << pose(1,2) << " " << pose(1,3) << "\n"
          << pose(2,0) << " " << pose(2,1) << " " << pose(2,2) << " " << pose(2,3) << std::endl;

      ofs << std::endl;
      ofs.close();
  } 
  std::cout << "Camera poses saved." << "\n" << "Camera images saved." << std::endl; 
}

void StructFromMotion::fromPoint3DToPCLCloud(const std::vector<Point3D> &input_cloud,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPCL){

  cloudPCL.reset(new pcl::PointCloud<pcl::PointXYZ> ());
  for(size_t i = 0; i < input_cloud.size(); ++i){
      Point3D pt3d = input_cloud[i];
      pcl::PointXYZ pclp;
      pclp.x  = pt3d.pt.x;
      pclp.y  = pt3d.pt.y;
      pclp.z  = pt3d.pt.z;
      cloudPCL->push_back(pclp);
   }
   cloudPCL->width = (uint32_t) cloudPCL->points.size(); // number of points
   cloudPCL->height = 1;	// a list, one row of data
   cloudPCL->header.frame_id ="map";
   cloudPCL->is_dense = false;
}

void StructFromMotion::cloudPointFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &filterCloud){
  //Removes points where values of selected field are out of range.

  pcl::PassThrough<pcl::PointXYZ> pass_through;
  pass_through.setInputCloud (cloud);
  pass_through.setFilterFieldName ("x");
  pass_through.setFilterLimits (0.003, 0.83);
  pass_through.filter (*filterCloud);
}

void StructFromMotion::removePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr &filterCloud){
  // Removes all points with less than a given
  // number of neighbors within a radius

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_removal;
  radius_outlier_removal.setInputCloud(cloud);
  radius_outlier_removal.setRadiusSearch (0.07);
  radius_outlier_removal.setMinNeighborsInRadius (150);
  radius_outlier_removal.filter (*filterCloud);

}

void StructFromMotion::create_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PolygonMesh &mesh){

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNumberOfThreads (8);
  // ne.setInputCloud (cloud_smoothed);
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  ne.setKSearch (10); //20
  //ne.setRadiusSearch (0.5); // no funciona
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
  ne.compute(*cloud_normals);

  for(std::size_t i = 0; i < cloud_normals->size (); ++i){
    cloud_normals->points[i].normal_x *= -1;
    cloud_normals->points[i].normal_y *= -1;
    cloud_normals->points[i].normal_z *= -1;
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::concatenateFields (*cloud, *cloud_normals, *cloud_smoothed_normals);//x

  pcl::Poisson<pcl::PointNormal> poisson;

  poisson.setDepth (7);//9
  poisson.setInputCloud (cloud_smoothed_normals);
  poisson.setPointWeight(4);//4
  //poisson.setDegree(5);
  poisson.setSamplesPerNode(1.5);//1.5
  poisson.setScale(1.1);//1.1
  poisson.setIsoDivide(8);//8
  poisson.setConfidence(1);
  poisson.setManifold(0);
  poisson.setOutputPolygons(0);
  poisson.setSolverDivide(8);//8
  poisson.reconstruct(mesh);

}

void StructFromMotion::vizualizeMesh(pcl::PolygonMesh &mesh){

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("MAP3D MESH"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPolygonMesh(mesh,"meshes",0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  std::cout << "Press q to finish 3D mapping and start segmentation process..." << std::endl;
  while (!viewer->wasStopped ()){
      viewer->spin();
  }
}

void StructFromMotion::MatchFeatures(int idx_i, int idx_j, std::vector<cv::DMatch>* matches) {


  std::vector<cv::Point2f> i_pts;
  keypointstoPoints2F(imagesKeypoints.at(idx_i),i_pts);
		//keypoints2F(nFeatureImages[idx_i].kps, nFeatureImages[i_pts].pt2D);

		std::vector<cv::Point2f> j_pts(i_pts.size());

		// making sure images are grayscale
		cv::Mat prevgray, gray;
		if(nImages[idx_i].channels() == 3) {
			cv::cvtColor(nImages[idx_i], prevgray, CV_RGB2GRAY);
			cv::cvtColor(nImages[idx_j], gray, CV_RGB2GRAY);
		}else {
			prevgray = nImages[idx_i];
			gray = nImages[idx_j];
		}

		std::vector<uchar> vstatus(i_pts.size());
		std::vector<float> verror(i_pts.size());
		cv::calcOpticalFlowPyrLK(prevgray, gray, i_pts, j_pts, vstatus, verror);

		double thresh = 1.0;
		std::vector<cv::Point2f> to_find;
		std::vector<int> to_find_back_idx;
		for(unsigned int i = 0; i < vstatus.size(); i++){
		    if(vstatus[i] && verror[i] < 12.0){
			to_find_back_idx.push_back(i);
			to_find.push_back(j_pts[i]);
		     }else{
			vstatus[i] = 0;
		     }
		}

		std::set<int> found_in_imgpts_j;
		cv::Mat to_find_flat = cv::Mat(to_find).reshape(1, to_find.size());

		std::vector<cv::Point2f> j_pts_to_find ;
		keypointstoPoints2F(imagesKeypoints.at(idx_j),j_pts_to_find);
		//keypoints2F(imgpts[idx_j], j_pts_to_find);
		cv::Mat j_pts_flat = cv::Mat(j_pts_to_find).reshape(1, j_pts_to_find.size());

		std::vector<std::vector<cv::DMatch> > knn_matches;
		cv::FlannBasedMatcher matcher;
		//cv::BFMatcher matcher(CV_L2);
		matcher.radiusMatch(to_find_flat, j_pts_flat, knn_matches, 2.0f);
		//Prune

		for(int i = 0; i < knn_matches.size(); i++){
		    cv::DMatch _m;

		    if(knn_matches[i].size() == 1){
			_m = knn_matches[i][0];
		     }else if(knn_matches[i].size()>1){
			if(knn_matches[i][0].distance / knn_matches[i][1].distance < 0.7){
			    _m = knn_matches[i][0];
			}else{
			    continue; // did not pass ratio test
			}
		     }else{
			continue; // no match
		     }
		     if(found_in_imgpts_j.find(_m.trainIdx) == found_in_imgpts_j.end()){ // prevent duplicates
			 _m.queryIdx = to_find_back_idx[_m.queryIdx]; //back to original indexing of points for <i_idx>
			 matches->push_back(_m);
			 found_in_imgpts_j.insert(_m.trainIdx);
			}

		}

		//std::cout << "pruned " << matches->size() << " / " << knn_matches.size() << " matches" << std::endl;



}

void StructFromMotion::keypointstoPoints2F(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& points2D){

  points2D.clear();
  for(const cv::KeyPoint& kp: keypoints){
         points2D.push_back(kp.pt);
   }
}





