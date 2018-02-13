cv::Mat imageStitching(cv::Mat& image1,cv::Mat& image2, cv::Mat& invMatrixH);
cv::Mat obtenerMatrixH(cv::Mat& image1,cv::Mat& image2);
static void help();
int frameNum(cv::VideoCapture &inputVideo);
cv::Mat cargarFrame(cv::VideoCapture &inputVideo);

cv::Mat imageStitching(cv::Mat& image1,cv::Mat& image2,  cv::Mat& invMatrixH){

  cv::Mat result;
  cv::warpPerspective(image2,result,invMatrixH,cv::Size(image1.cols,image1.rows),  cv::INTER_CUBIC);

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

  std::cout << "Matrix H: "<< "\n" << matrixH << std::endl;

  return matrixH;
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
   cv::resize(image,image_resized,cv::Size(),0.75,0.75);
   GaussianBlur(image_resized,image, cv::Size(7,7),1.5,1.5);
   double position = inputVideo.get(CV_CAP_PROP_POS_FRAMES );

   std::ostringstream strs;
   strs << position;
   std::string str2 = strs.str();
   std::string str1= "Frame";

   str1 +=str2;
   cv::putText(image,str1, cv::Point(50,500), cv::FONT_HERSHEY_SIMPLEX ,1.0, 0, 4);
   return image;
}

static void help(){
   std::cout <<  "------------------------------------------------------------------------------------"
                 "This program shows the multiview reconstruction capabilities using the "
                 "OpenCV Library."
                 "It reconstruct a scene from a set of 2D images"
                 "Usage:"
                 "3D_recons <path_to_file> <f> <cx> <cy>"
                 "where: path_to_file is the file absolute path into your system which contains"
                 "the list of images to use for reconstruction."
                 "f  is the focal lenght in pixels. "
                 "cx is the image principal point x coordinates in pixels. "
                 "cy is the image principal point y coordinates in pixels."
            "------------------------------------------------------------------------------------" <<std::endl;
}


