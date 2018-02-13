#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/viz.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "opencv2/viz/viz3d.hpp"
#include <eigen3/Eigen/Dense>

class StructFromMotion{

  cv::Mat image1, image2;
  std::vector<cv::KeyPoint> keypoints1,keypoints2,keypoints;
  std::vector<cv::DMatch> matches,good_matches;
  cv::Mat descriptors1,descriptors2;
  std::vector<cv::Point2f> points1,points2,points2F;
  cv::Mat matrixE12,matrixE23,matrixETotal;
  cv::Mat inliers;
  cv::Mat relativeRotationCam, relativeTranslaCam;

  std::vector<cv::Point3d> cloud;
  cv::Mat projection1, projection2, projection_result;

  double fx = 800;
  double fy = 800;
  double cx = 400;
  double cy = 225;

  int contador=1;
  int contador2=1;

public:

  StructFromMotion();

  StructFromMotion(cv::Mat img1,cv::Mat img2):image1(img1),image2(img2){
    GaussianBlur(this->image1,this->image1, cv::Size(7,7),1.5,1.5);
    GaussianBlur(this->image2,this->image2, cv::Size(7,7),1.5,1.5);
  }

  void setConstructor(cv::Mat& img1,cv::Mat& img2){

    GaussianBlur(img1,img1, cv::Size(7,7),1.5,1.5);
    GaussianBlur(img2,img2, cv::Size(7,7),1.5,1.5);
    this->image1 = img1;
    this->image2 = img2;    
    this->matrixE23 = findEssentialMatrix();

    cv::Mat matrixTotal;
    matrixTotal =  this->matrixE12*this->matrixE23;
   // std::cout << "matrixE12 anterior: " << matrixE12 << std::endl;
    this->matrixE12 = matrixTotal;
    this->matrixETotal = matrixTotal;
   // std::cout << "matrixE12 actual: " << matrixE12 << std::endl;
   // std::cout << "matrixE23 " << matrixE23 << std::endl;
  }

  void initTriangulation(){

    triangulation();
  }

void cameraPoseAcumulada(){

  cv::Mat rot,tra,inliers;
  cv::recoverPose(this->matrixETotal,this->points1, this->points2,
                             rot,tra,this->fx,cv::Point2d(this->cx,this->cy),inliers);

  this->relativeRotationCam = rot;
  this->relativeTranslaCam = tra;

  }


cv::Mat imageMatching(){
  cv::Mat matchImage;

  cv::drawMatches(this->image1,
                  this->keypoints1,
                  this->image2,this->keypoints2,
                  obtenerMatches(),matchImage,
                  cv::Scalar::all(-1),cv::Scalar::all(-1),std::vector<char>(),2);

  return matchImage;
}

void keypoints2F(){

std::vector<cv::DMatch> matches = obtenerMatches();
for (std::vector<cv::DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it){
//for(int n=0;n<keypoints.size();n++){
      // Get the position of left keypoints
                   float x= this->keypoints1[it->queryIdx].pt.x;
                   float y= this->keypoints1[it->queryIdx].pt.y;
                   this->points1.push_back(cv::Point2f(x,y));
                   float x2= this->keypoints2[it->queryIdx].pt.x;
                   float y2= this->keypoints2[it->queryIdx].pt.y;
                   this->points2.push_back(cv::Point2f(x2,y2));

            }

}

std::vector<cv::KeyPoint>obtenerKeypoints (cv::Mat& image){

  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);
  ptrFeature2D->detect(image,this->keypoints);

  return this->keypoints;
}

std::vector<cv::DMatch> obtenerMatches(){

  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);

  this->keypoints1=obtenerKeypoints(image1);
  this->keypoints2=obtenerKeypoints(image2);
  ptrFeature2D->compute(image1,keypoints1,this->descriptors1);
  ptrFeature2D->compute(image2,keypoints2,this->descriptors2);
  float minRatio = 1.0/1.5f;
  cv::FlannBasedMatcher matcher;

  matcher.match(descriptors1,descriptors2,matches);

  good_matches = thresholdGoodMatches(descriptors1,matches);

  // 1st image is the destination image and the 2nd image is the src image

  std::vector<cv::Point2f> obj; //1st image
  std::vector<cv::Point2f> obj2;//2nd image

  for(size_t i=0;i<good_matches.size();i++)
  {
      obj.push_back(keypoints1[good_matches[i].queryIdx].pt);
      obj2.push_back(keypoints2[good_matches[i].trainIdx].pt);
  }

  // Find homography matrix and get inliers mask
  std::vector<unsigned char> inliersMask(keypoints1.size());
  cv::Mat homography = cv::findHomography(obj,obj2,cv::RANSAC,0, inliersMask);
  std::vector<cv::DMatch> inliers;
  for (size_t i=0; i<inliersMask.size(); i++) {
  if (inliersMask[i])
  inliers.push_back(good_matches[i]);
  }
  good_matches.swap(inliers);

return good_matches;

}

std::vector<cv::DMatch> thresholdGoodMatches(cv::Mat& descriptors,std::vector<cv::DMatch>& matches) {

        double max_dist=0.8;
        double min_dist = 1.0f/1.5f;


        for(int i=0;i<descriptors.rows;i++){

           double dist = matches[i].distance;
           if(dist<min_dist) {
                           min_dist = dist;

                      }if(dist > max_dist){
                          dist = max_dist;
                      }
        }

      for(int i=0;i<matches.size();i++){

        if(matches[i].distance <= max_dist*0.8){

              this->good_matches.push_back(matches[i]);
         }
      }

      return this->good_matches;
    }


cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d u,
                                            cv::Matx34d P,          //camera 1 matrix
                                            cv::Point3d u1,
                                            cv::Matx34d P1          //camera 2 matrix
                                            ) {
    double wi = 1, wi1 = 1;
    cv::Mat_<double> X(4,1);
    for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
        cv::Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;

        //recalculate weights
        double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
        double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);

        //breaking point
        if(fabsf(wi - p2x) <= 0.8 && fabsf(wi1 - p2x1) <= 0.8) break;

        wi = p2x;
        wi1 = p2x1;

        //reweight equations and solve
        cv::Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,
                  (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,
                  (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1,
                  (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        cv::Mat_<double> B = (cv::Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          -(u.y*P(2,3)  -P(1,3))/wi,
                          -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          -(u1.y*P1(2,3)    -P1(1,3))/wi1
                          );

        cv::solve(A,B,X_,cv::DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X_(3) = 1.0;
    }
    return X;
}

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
/*
   Eigen::MatrixXd invMatrixK,invMatrixKTranspose;
   Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,
                                   Eigen::Dynamic,
                                   Eigen::RowMajor> > eigenMatrixK((double *)cameraMatrix.data,3,3);

   invMatrixK = eigenMatrixK.inverse();
   invMatrixKTranspose = invMatrixK.transpose();
   // create an OpenCV Mat header for the Eigen data:
   cv::Mat invMatrixKOpenCV(invMatrixKTranspose.rows(),
                                        invMatrixKTranspose.cols(),
                                        CV_64FC1,invMatrixKTranspose.data());


*/

cv::Mat_<double> LinearLSTriangulation4(cv::Point2d u,cv::Matx34d P,cv::Point2d u1,cv::Matx34d P1){
//build A matrix
cv::Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2));

//build B vector
cv::Matx41d B(-(u.x*P(2,3)-P(0,3)),-(u.y*P(2,3)-P(1,3)),-(u1.x*P1(2,3)-P1(0,3)),-(u1.y*P1(2,3)-P1(1,3)));

//solve for X
cv::Mat_<double> X;
cv::solve(A,B,X,cv::DECOMP_SVD);
if(X.data<0){
    X*=-1;
  }else{
    X=X;
  }
return X;
}

cv::Mat_<double> LinearLSTriangulation(cv::Point3d u,cv::Matx34d P,cv::Point3d u1,cv::Matx34d P1){
//build A matrix
cv::Matx43d A(u.x*P(2,0)-P(0,0),
              u.x*P(2,1)-P(0,1),
              u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),
              u.y*P(2,1)-P(1,1),
              u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0),
              u1.x*P1(2,1)-P1(0,1),
              u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0),
              u1.y*P1(2,1)-P1(1,1),
              u1.y*P1(2,2)-P1(1,2));

//build B vector
cv::Matx41d B(-(u.x*P(2,3)-P(0,3)),
              -(u.y*P(2,3)-P(1,3)),
              -(u1.x*P1(2,3)-P1(0,3)),
              -(u1.y*P1(2,3)-P1(1,3)));

//solve for X
cv::Mat_<double> X;
cv::solve(A,B,X,cv::DECOMP_SVD);
return X;
}

cv::Mat_<double> TriangulatePoints2(const std::vector<cv::KeyPoint>& pt_set1,
                         const std::vector<cv::KeyPoint>& pt_set2,const cv::Mat&Kinv,
                         const cv::Matx34d& P,const cv::Matx34d& P1,
                         const cv::Mat& K,std::vector<cv::Point3d>& pointcloud){

        std::vector<double> reproj_error;
        cv::Mat_<double> X ;
        for (unsigned int i=0; i<pt_set1.size(); i++) {
        //convert to normalized homogeneous coordinates
           cv::Point2f kp = pt_set1[i].pt;
           cv::Point3d u(kp.x,kp.y,1.0);
           cv::Mat_<double> um = Kinv * cv::Mat_<double>(u);
           u = um.at<cv::Point3d>(0);
           cv::Point2f kp1 = pt_set2[i].pt;
           cv::Point3d u1(kp1.x,kp1.y,1.0);
           cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1);
           u1 = um1.at<cv::Point3d>(0);
           //triangulate
            X = LinearLSTriangulation(u,P,u1,P1);

           }
        return X;

}

cv::Mat findEssentialMatrix(){

   cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << fx,0,cx,
                                                    0,fy,cy,
                                                    0,0,1);

   keypoints2F();
   cv::Mat inli;

   cv::Mat matrixFundamental = cv::findFundamentalMat(this->points1,
                                                      this->points2,cv::FM_RANSAC,0.1,
                                                      0.99,inli);

   cv::Mat matrixE = cameraMatrix.t()*matrixFundamental*cameraMatrix;
   this->inliers=inli;

   if(this->matrixE12.empty()){

       this->matrixE12 = matrixE;
      // std::cout << "matrix E12 primer ciclo: " << matrixE12 << std::endl;
   }else{

     this->inliers=inli;
   }

   return matrixE;

   }

cv::Mat inverse(cv::Mat& matrix){

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

cv::Mat getCameraT(){

  return this->relativeTranslaCam;
}

cv::Mat getCameraR(){

  return this->relativeRotationCam;
}


void cameraPose(){

  cv::Mat rot,tra,inliers;
  cv::recoverPose(findEssentialMatrix(),this->points1, this->points2,
                             rot,tra,this->fx,cv::Point2d(this->cx,this->cy),inliers);

  this->relativeRotationCam = rot;
  this->relativeTranslaCam = tra;

  }

void projection(){

    this->projection1 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));
    this->projection2 = cv::Mat(3, 4, CV_64F,cv::Scalar(0.0));

    if(this->contador >= 2){
      cameraPoseAcumulada();
    }else{
      cameraPose();
    }
    this->contador +=1;

    this->relativeRotationCam.copyTo(this->projection2(cv::Rect(0, 0, 3, 3)));
    this->relativeTranslaCam.copyTo(this->projection2.colRange(3, 4));

    //projection2 --> crea una matríz homogénea 3x4 [R|t]

    if(this->projection_result.empty()){

        cv::Mat diag(cv::Mat::eye(3, 3, CV_64F)); // ---> Crea una matriz identidad
        diag.copyTo(this->projection1(cv::Rect(0, 0, 3, 3)));

        std::cout << "projection1 ciclo1: " << projection1 << std::endl;
        std::cout << "projection2 ciclo1: " << projection2 << std::endl;

    }else{

        this->projection1 = this->projection_result;
        std::cout << "projection1 ciclo2+: " << projection1 << std::endl;
        std::cout << "\n" << std::endl;
        std::cout << "projection2 ciclo2+: " << projection2 << std::endl;
    }

    this->projection_result = projection2;

}

std::vector<cv::Point3d> triangulation(){

  projection();
   std::vector<cv::Point2d> inlierPtsImage1, inlierPtsImage2;

  for (int i=0;i<this->inliers.rows;i++){

       inlierPtsImage1.push_back(cv::Point2d(this->points1[i].x,this->points1[i].y));
       inlierPtsImage2.push_back(cv::Point2d(this->points2[i].x,this->points2[i].y));
  }

  std::vector<cv::Point3d> in1,in2;

  for (int i=0;i<this->inliers.rows;i++){

       in1.push_back(cv::Point3d(this->points1[i].x,this->points1[i].y,1));
       in2.push_back(cv::Point3d(this->points2[i].x,this->points2[i].y,1));
  }
  cv::Mat_<double> pointCloudOpenCV;

  for(unsigned int n=0;n<inlierPtsImage1.size();n++){

  pointCloudOpenCV.push_back(LinearLSTriangulation4(inlierPtsImage1.at(n),
                                                          this->projection1,
                                                          inlierPtsImage2.at(n),
                                                          this->projection2));

  this->cloud.push_back(cv::Point3d(inlierPtsImage1[n].x,
                              inlierPtsImage1[n].y,
                              pointCloudOpenCV(n)));
  }
  return this->cloud;
 }

void visualizerPointCloud(){

// Create a viz window
cv::viz::Viz3d visualizer("Viz window");

cv::viz::WCoordinateSystem ucs(100);

//Create a virtual camera

//cv::viz::WCameraPosition cam1(matrixK, image1, 150,cv::viz::Color::white());
//cam1.setRenderingProperty(cv::viz::LINE_WIDTH,2);

cv::viz::WCloud point3d(this->cloud, cv::viz::Color::green());

point3d.setRenderingProperty(cv::viz::POINT_SIZE, 1.0);

visualizer.setBackgroundColor(cv::viz::Color::black());
visualizer.showWidget("Coordinate Widget", ucs);
visualizer.showWidget("Point3D", point3d);
//        visualizer.showWidget("Camera1", cam1);
//      visualizer.showWidget("Camera2", cam2);

cv::Affine3d pose(getCameraR(),getCameraT());

//visualizer.setWidgetPose("Camera2", pose);
//visualizer.setWidgetPose("Point3D", pose);

// visualization loop
while(cv::waitKey(0) && !visualizer.wasStopped()){

    visualizer.spin();
}

}

void matchingImShow(){
cv::namedWindow("matches",CV_WINDOW_NORMAL);
cv::resizeWindow("matches",600,300);
cv::moveWindow("matches",0,0);
cv::imshow("matches",imageMatching());
cv::waitKey(30);
}

};//Fin clase


