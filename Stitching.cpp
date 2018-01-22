/*********************************
           HEADERS
**********************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/viz.hpp>
#include <eigen3/Eigen/Dense>
#include <string>


/*********************************
        FUNCIONES
**********************************/

int frameNum(cv::VideoCapture &inputVideo);
void help(char** av);
cv::Mat cargarFrame(cv::VideoCapture &inputVideo);
cv::Mat obtenerMatrixH(cv::Mat& image1,cv::Mat& image2);
cv::Mat imageStitching(cv::Mat& image1,cv::Mat& image2, cv::Mat& invMatrixH);
cv::Mat imageMatching(cv::Mat& image1,cv::Mat& image2);
cv::Mat detMatrixH(cv::Mat& invMatrixH12,cv::Mat& invMatrixH23);
std::vector<cv::Point2f> keypoints2F(std::vector<cv::KeyPoint>& keypoints,std::vector<cv::DMatch>& matches);
std::vector<cv::KeyPoint> obtenerKeypoints(cv::Mat& image);
std::vector<cv::DMatch> obtenerMatches(cv::Mat& image1,
                                       cv::Mat& image2,
                                       std::vector<cv::KeyPoint>& keypoints1,
                                       std::vector<cv::KeyPoint>& keypoints2);
std::vector<cv::DMatch> thresholdGoodMatches(cv::Mat& descriptors,std::vector<cv::DMatch>& matches);
void dibujarKeypoints(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,cv::Mat& outImage);

cv::Mat result;

/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/


int main(int argc, char **argv ){


    cv::CommandLineParser parser(argc, argv, "{help h||}{@input||}");
    if (parser.has("help")){
          help(argv);
          return 0;
      }
    std::string arg = parser.get<std::string>("@input");
      if (arg.empty()){
          help(argv);
          return 1;
      }
    cv::VideoCapture inputVideo(arg);

    if (!inputVideo.isOpened()) {//if this fails, try to open as a video camera, through the use of an integer param
        inputVideo.open(atoi(arg.c_str()));

      }
    if (!inputVideo.isOpened()) {
            std::cout << "Failed to open the video device, video file or image sequence!\n" << std::endl;
            help(argv);
            return 1;
          }

   int framenu = frameNum(inputVideo);

   std::cout << "*******************************" << std::endl;
   std::cout << "El video: "<< "" << "contiene en total: " << framenu <<" frames." <<std::endl;
   std::cout << "*******************************" << std::endl;

   cv::Mat image1,image2;
   cv::Mat matchImage;
   cv::Mat image1Keypoints,image2Keypoints;
   cv::Mat temp_result,temp_img2;
   cv::Mat matrixH12,matrixH23,matrixHTotal;

   int x=1;
   int offsetx = 100;
   int offsety = 100;

   double frame_per_second;
   cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);   
   std::vector<cv::Point2f>points1,points2;   
   int contador1 = 1;
   int contador2 = 2;


   for(int n=0;n<framenu;n++){

          if (x==1){

              image1=cargarFrame(inputVideo); //Frame1
              image2=cargarFrame(inputVideo);//Frame2

              cv::warpAffine(image1, image1, trans_mat, cv::Size(2*image1.cols, 2*image1.rows),cv::INTER_LINEAR);

              matrixH12 = obtenerMatrixH(image1,image2);

              /**************************************************
                  CÁLCULO DE LA MATRIZ INVERSA DE LA MATRIZ H
              ***************************************************/

              Eigen::MatrixXd invMatrixH12,invMatrixHTranspose12;

              Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,
                                              Eigen::Dynamic,
                                              Eigen::RowMajor> > eigenMatrixH12 ((double *)matrixH12.data,3, 3);
              // Eigen handles the conversion from row major to column major
              invMatrixH12 = eigenMatrixH12.inverse();
              invMatrixHTranspose12 = invMatrixH12.transpose();
              // create an OpenCV Mat header for the Eigen data:
              cv::Mat invMatrixH12_OpenCV(invMatrixHTranspose12.rows(),
                                          invMatrixHTranspose12.cols(),
                                          CV_64FC1, invMatrixHTranspose12.data());

              /**************************************************
               STITCHING - MATCHING CON LA INVERSA DE LA MATRIZ H
              ***************************************************/

              std::vector<cv::KeyPoint> keypoints1 = obtenerKeypoints(image1);
              std::vector<cv::KeyPoint> keypoints2 = obtenerKeypoints(image2);

              std::vector<cv::DMatch> matches = obtenerMatches(image1,image2, keypoints1,keypoints2);       

              dibujarKeypoints(image1,keypoints1,image1Keypoints);
              dibujarKeypoints(image2,keypoints2,image2Keypoints);

              cv::Mat imageKeypoints(image1Keypoints,cv::Rect(0,0,image2Keypoints.rows,image2Keypoints.cols));
              image2Keypoints.copyTo(imageKeypoints);

              points1= keypoints2F( keypoints1, matches);
              points2= keypoints2F( keypoints2, matches);

              result = imageStitching(image1,image2,invMatrixH12_OpenCV);
              matchImage= imageMatching(image1,image2);

              temp_result = result;
              temp_img2 = image2;


              /**************************************************
                        IMPRIMIENDO EN PANTALLA
              ***************************************************/

              std::cout << "*******************************" << std::endl;
              std::printf("Frames %i y %i", contador1,contador2);std::cout << "\n";
              std::printf("Number of feature points (%i): %lu", contador1,keypoints1.size());std::cout << "\n";
              std::printf("Number of feature points (%i): %lu", contador2,keypoints2.size() );std::cout << "\n";
              std::printf("Number of points (%i): %lu", contador1,points1.size());std::cout << "\n";
              std::printf("Number of points (%i): %lu", contador2,points2.size());std::cout << "\n";
              std::printf("Number of matches: %lu", matches.size()); std::cout << "\n";
              std::cout << "*******************************" << std::endl;
/*
              //std::cout <<"Matrix H: "<< "\n" << matrixH12 << std::endl;
              //std::cout <<"Matrix H inversa: "<< "\n" << invMatrixH12_OpenCV << std::endl;
*/
              cv::namedWindow( "Keypoints ", CV_WINDOW_NORMAL  );
              cv::resizeWindow  ("Keypoints ",600,300);
              cv::moveWindow("Keypoints ",0,0);
              cv::imshow("Keypoints ",imageKeypoints);

              cv::namedWindow( "Matching ", CV_WINDOW_NORMAL  );
              cv::resizeWindow  ("Matching ",600,300);
              cv::moveWindow("Matching ",0,400);
              cv::imshow("Matching ",matchImage);

              cv::namedWindow( "Stitching ", CV_WINDOW_NORMAL );
              cv::resizeWindow  ("Stitching ",800,500);
              cv::moveWindow("Stitching ",700,0);
              cv::imshow("Stitching ",result);

              x = 3;
              contador1 +=1;
              contador2 +=1;

              cv::waitKey(50);

          }else if(x==2){

              frame_per_second= inputVideo.get(CV_CAP_PROP_FPS );
              std::cout <<"Frames por segundo: " << frame_per_second << std::endl;
              inputVideo.set(CV_CAP_PROP_FPS,frame_per_second/1000);
              x=3;

          }else if(x==3){

              image1 = temp_result;//Frame result 1+2
              image2=cargarFrame(inputVideo); //Frame num3           

              matrixH23= obtenerMatrixH(temp_img2,image2); //Homogr-frame2 y frame3
              matrixHTotal=detMatrixH(matrixH12,matrixH23);

              Eigen::MatrixXd invMatrixHTotal,invMatrixHTranspose2;
              Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,
                                              Eigen::Dynamic,
                                              Eigen::RowMajor> > eigenMatrixH23((double *)matrixHTotal.data,3,3);

              invMatrixHTotal = eigenMatrixH23.inverse();
              invMatrixHTranspose2 = invMatrixHTotal.transpose();
              // create an OpenCV Mat header for the Eigen data:
              cv::Mat invMatrixHTotal13(invMatrixHTranspose2.rows(),
                                     invMatrixHTranspose2.cols(),
                                     CV_64FC1,invMatrixHTranspose2.data());

              /**************************************************
               STITCHING - MATCHING CON LA INVERSA DE LA MATRIZ H
              ***************************************************/

              std::vector<cv::KeyPoint> keypoints1 = obtenerKeypoints(image1);
              std::vector<cv::KeyPoint> keypoints2 = obtenerKeypoints(image2);         

              std::vector<cv::DMatch> matches = obtenerMatches(image1,image2, keypoints1,keypoints2);              

              points1= keypoints2F( keypoints1, matches);
              points2= keypoints2F( keypoints2, matches);    

              result = imageStitching(image1,image2,invMatrixHTotal13);
              matchImage= imageMatching(image1,image2);

              dibujarKeypoints(image1,keypoints1,image1Keypoints);
              dibujarKeypoints(image2,keypoints2,image2Keypoints);

              cv::Mat imageKeypoints(image1Keypoints,cv::Rect(0,0,image2Keypoints.rows,image2Keypoints.cols));
              image2Keypoints.copyTo(imageKeypoints);

              temp_result = result;
              temp_img2 = image2;
              matrixH12=matrixHTotal;


              /**************************************************
                        IMPRIMIENDO EN PANTALLA
              ***************************************************/
              std::cout << "*******************************" << std::endl;
              std::printf("Frames %i y %i", contador1,contador2);std::cout << "\n";
              std::printf("Number of feature points (%i): %lu", contador1,keypoints1.size());std::cout << "\n";
              std::printf("Number of feature points (%i): %lu", contador2,keypoints2.size() );std::cout << "\n";
              std::printf("Number of points (%i): %lu", contador1,points1.size());std::cout << "\n";
              std::printf("Number of points (%i): %lu", contador2,points2.size());std::cout << "\n";
              std::printf("Number of matches: %lu", matches.size()); std::cout << "\n";
              std::cout << "*******************************" << std::endl;
              /*
              //std::cout <<"Matrix H: "<< "\n" << matrixHTotal << std::endl;
              //std::cout <<"Matrix H inversa: "<< "\n" << invMatrixHTotal13 << std::endl;
              */

              cv::namedWindow( "Keypoints ", CV_WINDOW_NORMAL  );
              cv::resizeWindow  ("Keypoints ",600,300);
              cv::imshow("Keypoints ",imageKeypoints);

              cv::namedWindow( "Matching ", CV_WINDOW_NORMAL  );
              cv::resizeWindow  ("Matching ",600,300);
              cv::imshow("Matching ",matchImage);

              cv::namedWindow( "Stitching ", CV_WINDOW_NORMAL );
              cv::resizeWindow  ("Stitching ",800,500);
              cv::imshow("Stitching ",result);

              x=3;
              contador1 +=1;
              contador2 +=1;
              cv::waitKey(50);


              }//end if(x==3)

     }//end for-loop

   cvWaitKey(0);
   return 0;

  }//end main function


/*********************************
        FUNCIONES-CUERPO
**********************************/
cv::Mat detMatrixH(cv::Mat& invMatrixH12,cv::Mat& invMatrixH23){
cv::Mat invMatrixH_n_esima;
   invMatrixH_n_esima = invMatrixH12*invMatrixH23;

  return invMatrixH_n_esima;
}

int frameNum(cv::VideoCapture &inputVideo){

  int framesNumber=0;
  framesNumber = static_cast<int>(inputVideo.get(CV_CAP_PROP_FRAME_COUNT));
  return framesNumber;

}

cv::Mat cargarFrame(cv::VideoCapture &inputVideo){

   cv::Mat frame,image,image_resized;
   inputVideo.read(frame);
   /*
   if(frame.channels()==3 || frame.channels()==1){
      cv::cvtColor(frame,frame, cv::COLOR_BGR2GRAY);
    }
   */
   image = frame.clone();
   cv::resize(image,image_resized,cv::Size(),0.75,0.75,cv::INTER_LINEAR);
   GaussianBlur(image_resized,image, cv::Size(7,7),1.5,1.5);   
   /*
   std::ostringstream strs;
   strs << position;
   std::string str2 = strs.str();
   std::string str1= "Frame";

   str1 +=str2;
   cv::putText(image,str1, cv::Point(50,500), cv::FONT_HERSHEY_SIMPLEX ,1.0, 0, 4);
   */
   return image;
}

cv::Mat imageMatching(cv::Mat& image1,cv::Mat& image2){
  cv::Mat descriptors1,descriptors2,matchImage;
  std::vector<cv::KeyPoint> keypoints1,keypoints2;

  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(2000.0);
  cv::FlannBasedMatcher matcher;

  std::vector<cv::DMatch> matches,good_matches; 

  ptrFeature2D->detect(image1,keypoints1);
  ptrFeature2D->detect(image2,keypoints2);

  ptrFeature2D->compute(image1,keypoints1,descriptors1);
  ptrFeature2D->compute(image2,keypoints2,descriptors2);

  matcher.match(descriptors1,descriptors2,matches);
  good_matches = thresholdGoodMatches(descriptors1,matches);

  cv::drawMatches(image1,keypoints1,image2,keypoints2,good_matches,matchImage,cv::Scalar::all(-1),
  cv::Scalar::all(-1),std::vector<char>(),2);

  return matchImage;
}

cv::Mat obtenerMatrixH(cv::Mat& image1,cv::Mat& image2){

  cv::Mat descriptors1,descriptors2;
  std::vector<cv::KeyPoint> keypoints1,keypoints2;

  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(2000.0);

  ptrFeature2D->detect(image1,keypoints1);
  ptrFeature2D->detect(image2,keypoints2);

  ptrFeature2D->compute(image1,keypoints1,descriptors1);
  ptrFeature2D->compute(image2,keypoints2,descriptors2);

  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> matches,good_matches;
  matcher.match(descriptors1,descriptors2,matches);

  good_matches = thresholdGoodMatches(descriptors1,matches);

  cv::Mat matchImage;

  cv::drawMatches(image1,keypoints1,image2,keypoints2,good_matches,matchImage,cv::Scalar::all(-1),
  cv::Scalar::all(-1),std::vector<char>(),2);

  // 1st image is the destination image and the 2nd image is the src image

  std::vector<cv::Point2f> obj; //1st image
  std::vector<cv::Point2f> obj2;//2nd image

  for(size_t i=0;i<good_matches.size();i++)
  {
      obj.push_back(keypoints1[good_matches[i].queryIdx].pt);
      obj2.push_back(keypoints2[good_matches[i].trainIdx].pt);
  }

  cv::Mat matrixH(3,3,CV_32FC3);
  matrixH = cv::findHomography(obj,obj2,cv::RANSAC,1);

  return matrixH;
}

cv::Mat imageStitching(cv::Mat& image1,cv::Mat& image2,  cv::Mat& invMatrixH){

  cv::Mat result;
  cv::warpPerspective(image2,result,invMatrixH,cv::Size(image1.cols,image1.rows),cv::INTER_NEAREST );

    for (int i = 0; i < image1.cols; i++){
          for (int j = 0; j < image1.rows; j++) {

		cv::Vec3b color_im1 = image1.at<cv::Vec3b>(cv::Point(i, j));
		cv::Vec3b color_im2 = result.at<cv::Vec3b>(cv::Point(i, j));
		if (cv::norm(color_im1) == 0){
			image1.at<cv::Vec3b>(cv::Point(i, j)) = color_im2;
			}
	   }
	}

  return image1;

}

void help(char** av){
  std::cout << "----------------------------------------------- \n"
               "The program create a panorama from a video file, image sequence or\ncamera connected to your computer."
            << std::endl
            << "Usage:\n" << av[0] << " <video file, image sequence or device number>" << std::endl
            << "'esc' --> for quit the program" <<
               "\n -----------------------------------------------"<< std::endl;

}

std::vector<cv::KeyPoint> obtenerKeypoints(cv::Mat& image){

  std::vector<cv::KeyPoint> keypoints;
  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(2500.0);
  ptrFeature2D->detect(image,keypoints);

  return keypoints;
}

std::vector<cv::DMatch> obtenerMatches(cv::Mat& image1,cv::Mat& image2,
                       std::vector<cv::KeyPoint>& keypoints1,std::vector<cv::KeyPoint>& keypoints2){

  cv::Mat descriptors1,descriptors2;
  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(2500.0);

  ptrFeature2D->compute(image1,keypoints1,descriptors1);
  ptrFeature2D->compute(image2,keypoints2,descriptors2);

  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> matches,good_matches;
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

  return good_matches;
}

std::vector<cv::Point2f> keypoints2F(std::vector<cv::KeyPoint>& keypoints,std::vector<cv::DMatch>& matches){

  std::vector<cv::Point2f> points;
  for (std::vector<cv::DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it){

      // Get the position of left keypoints
                   float x= keypoints[it->queryIdx].pt.x;
                   float y= keypoints[it->queryIdx].pt.y;
                   points.push_back(cv::Point2f(x,y));                 
            }
  return points;
}

std::vector<cv::DMatch> thresholdGoodMatches(cv::Mat& descriptors,std::vector<cv::DMatch>& matches) {

        double max_dist=0.421081;
        double min_dist = 0.019635;
        std::vector<cv::DMatch> good_matches;
        for(int i=0;i<descriptors.rows;i++){

           double dist = matches[i].distance;
           if(dist<min_dist) {
                min_dist = dist;

           }if(dist > max_dist){
               max_dist = dist;
           }
        }

      for(int i=0;i<descriptors.rows;i++){

        if(matches[i].distance <= 12*min_dist){

              good_matches.push_back(matches[i]);
         }
      }
      return good_matches;
    }

void dibujarKeypoints(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,cv::Mat& outImage){

  cv::drawKeypoints(image,keypoints,outImage,cv::Scalar(0,255,255));

}







