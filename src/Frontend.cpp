/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2020
 *      Author: sleutene
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>

#include <brisk/brisk.h>

#include <arp/Frontend.hpp>

#ifndef CV_AA
#define CV_AA cv::LINE_AA // maintains backward compatibility with older OpenCV
#endif

#ifndef CV_BGR2GRAY
#define CV_BGR2GRAY cv::COLOR_BGR2GRAY // maintains backward compatibility with older OpenCV
#endif

namespace arp {

Frontend::Frontend(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2) :
  camera_(imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV,
          arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2))
{
  camera_.initialiseUndistortMaps();

  // also save for OpenCV RANSAC later
  cameraMatrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
  cameraMatrix_.at<double>(0,0) = focalLengthU;
  cameraMatrix_.at<double>(1,1) = focalLengthV;
  cameraMatrix_.at<double>(0,2) = imageCenterU;
  cameraMatrix_.at<double>(1,2) = imageCenterV;
  cameraMatrix_.at<double>(2,2) = 1.0;
  distCoeffs_ = cv::Mat::zeros(1, 4, CV_64FC1);
  distCoeffs_.at<double>(0) = k1;
  distCoeffs_.at<double>(1) = k2;
  distCoeffs_.at<double>(2) = p1;
  distCoeffs_.at<double>(3) = p2;
  
  // BRISK detector and descriptor
  detector_.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(15, 4, 50, 500)); //  10, 0, 100, 2000
  extractor_.reset(new brisk::BriskDescriptorExtractor(true, false));
  
  // leverage camera-aware BRISK (caution: needs the *_new* maps...)
  cv::Mat rays = cv::Mat(imageHeight, imageWidth, CV_32FC3);
  cv::Mat imageJacobians = cv::Mat(imageHeight, imageWidth, CV_32FC(6));
  for (int v=0; v<imageHeight; ++v) {
    for (int u=0; u<imageWidth; ++u) {
      Eigen::Vector3d ray;
      Eigen::Matrix<double, 2, 3> jacobian;
      if(camera_.backProject(Eigen::Vector2d(u,v), &ray)) {
        ray.normalize();
      } else {
        ray.setZero();
      }
      rays.at<cv::Vec3f>(v,u) = cv::Vec3f(ray[0],ray[1],ray[2]);
      Eigen::Vector2d pt;
      if(camera_.project(ray, &pt, &jacobian)
         ==cameras::ProjectionStatus::Successful) {
        cv::Vec6f j;
        j[0]=jacobian(0,0);
        j[1]=jacobian(0,1);
        j[2]=jacobian(0,2);
        j[3]=jacobian(1,0);
        j[4]=jacobian(1,1);
        j[5]=jacobian(1,2);
        imageJacobians.at<cv::Vec6f>(v,u) = j;
      }
    }
  }
  std::static_pointer_cast<cv::BriskDescriptorExtractor>(extractor_)->setCameraProperties(rays, imageJacobians, (focalLengthU + focalLengthV) / 2);
}

bool  Frontend::loadMap(std::string path) {
  std::ifstream mapfile(path);
  if(!mapfile.good()) {
    return false;
  }
  
  // read each line
  std::string line;
  std::set<uint64_t> lmIds;
  uint64_t poseId = 0;
  LandmarkVec landmarks;
  while (std::getline(mapfile, line)) {

    // Convert to stringstream
    std::stringstream ss(line);
    
    if(0==line.compare(0, 7,"frame: ")) {
      // store previous set into map
      landmarks_[poseId] = landmarks;
      // get pose id:
      std::stringstream frameSs(line.substr(7,line.size()-1));
      frameSs >> poseId;
      if(!frameSs.eof()) {
        std::string covisStr;
        frameSs >> covisStr; // comma
        frameSs >> covisStr;
        if(0==covisStr.compare("covisibilities:")) {
          while(!frameSs.eof()) {
            uint64_t covisId;
            frameSs >> covisId;
            covisibilities_[poseId].insert(covisId);
          }
        }
      }
      // move to filling next set of landmarks
      landmarks.clear();
    } else {
      if(poseId>0) {
        Landmark landmark;
      
        // get keypoint idx
        size_t keypointIdx;
        std::string keypointIdxString;
        std::getline(ss, keypointIdxString, ',');
        std::stringstream(keypointIdxString) >> keypointIdx;
        
        // get landmark id
        uint64_t landmarkId;
        std::string landmarkIdString;
        std::getline(ss, landmarkIdString, ',');
        std::stringstream(landmarkIdString) >> landmarkId;
        landmark.landmarkId = landmarkId;
        
        // read 3d position
        for(int i=0; i<3; ++i) {
          std::string coordString;
          std::getline(ss, coordString, ',');
          double coord;
          std::stringstream(coordString) >> coord;
          landmark.point[i] = coord;
        }

        // Get descriptor
        std::string descriptorstring;
        std::getline(ss, descriptorstring);
        landmark.descriptor = cv::Mat(1,48,CV_8UC1);
        for(int col=0; col<48; ++col) {
          uint32_t byte;
          std::stringstream(descriptorstring.substr(2*col,2)) >> std::hex >> byte;
          landmark.descriptor.at<uchar>(0,col) = byte;
        }
        lmIds.insert(landmarkId);
        landmarks.push_back(landmark);
      }      
    } 
  }
  if(poseId>0) {
    // store into map
    landmarks_[poseId] = landmarks;
  }
  std::cout << "loaded " << lmIds.size() << " landmarks from " << landmarks_.size() << " poses." << std::endl;
  return lmIds.size() > 0;
}

bool Frontend::loadDBoW2Voc(std::string path) {
  std::cout << "Loading DBoW2 vocabulary from " << path << std::endl;
  dBowVocabulary_.load(path);
  // Hand over vocabulary to dataset. false = do not use direct index:
  dBowDatabase_.setVocabulary(dBowVocabulary_, false, 0);
  std::cout << "loaded DBoW2 vocabulary with " << dBowVocabulary_.size() << " words." << std::endl;
  return true; 
}

int Frontend::detectAndDescribe(
    const cv::Mat& grayscaleImage, const Eigen::Vector3d& extractionDirection,
    std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const {

  // run BRISK detector
  detector_->detect(grayscaleImage, keypoints);

  // run BRISK descriptor extractor
  // orient the keypoints according to the extraction direction:
  Eigen::Vector3d ep;
  Eigen::Vector2d reprojection;
  Eigen::Matrix<double, 2, 3> Jacobian;
  Eigen::Vector2d eg_projected;
  for (size_t k = 0; k < keypoints.size(); ++k) {
    cv::KeyPoint& ckp = keypoints[k];
    const Eigen::Vector2d kp(ckp.pt.x, ckp.pt.y);
    // project ray
    camera_.backProject(kp, &ep);
    // obtain image Jacobian
    camera_.project(ep+extractionDirection.normalized()*0.001, &reprojection);
    // multiply with gravity direction
    eg_projected = reprojection-kp;
    double angle = atan2(eg_projected[1], eg_projected[0]);
    // set
    ckp.angle = angle / M_PI * 180.0;
  }
  extractor_->compute(grayscaleImage, keypoints, descriptors);

  return keypoints.size();
}

bool Frontend::ransac(const std::vector<cv::Point3d>& worldPoints, 
                      const std::vector<cv::Point2d>& imagePoints, 
                      kinematics::Transformation & T_CW, std::vector<int>& inliers) const {
  if(worldPoints.size() != imagePoints.size()) {
    return false;
  }
  if(worldPoints.size()<5) {
    return false; // not realiabl enough
  }

  inliers.clear();
  cv::Mat rvec, tvec;
  bool ransacSuccess = cv::solvePnPRansac(
      worldPoints, imagePoints, cameraMatrix_, distCoeffs_,
      rvec, tvec, false, 100, 5.0, 0.99, inliers, cv::SOLVEPNP_EPNP);	

  // set pose
  cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Rodrigues(rvec, R);
  Eigen::Matrix4d T_CW_mat = Eigen::Matrix4d::Identity();
  for(int i=0; i<3; i++) {
    T_CW_mat(i,3) = tvec.at<double>(i);
    for(int j=0; j<3; j++) {
      T_CW_mat(i,j) = R.at<double>(i,j);
    }
  }
  T_CW = kinematics::Transformation(T_CW_mat);

  return ransacSuccess && (double(inliers.size())/double(imagePoints.size()) > 0.7);
}

std::vector<DBoW2::FBrisk::TDescriptor> convertMatToTDescriptor(const cv::Mat& descriptors){
  cv::Size s = descriptors.size();
  auto rows = s.height;
  auto cols = s.width;

  std::vector<DBoW2::FBrisk::TDescriptor> descriptorsvec;
  for (int i = 0; i<rows; i++){
    uchar* keypointDescriptor = descriptors.data + i*48;
    DBoW2::FBrisk::TDescriptor feature(48);
    for (int i = 0; i<48; i++)
      feature[i] = keypointDescriptor[i];
    descriptorsvec.push_back(feature);
  }

  return descriptorsvec;
}


bool Frontend::detectFrames(
  const std::set<uint64_t>& possibleFrames,
  std::vector<cv::KeyPoint>& keypoints,
  cv::Mat& descriptors,
  DetectionVec & detections,
  const kinematics::Transformation * T_CW)
{
  int numberOfMatchesForActiveFrame = 0;

  // match to map:
  for(auto id : possibleFrames) { // go through all poses
    int numberOfMatches = 0 ;

    
    for(const auto& lm : landmarks_.at(id)) { // go through all landmarks seen from this pose

      Eigen::Vector2d imagePoint;
      if (T_CW != nullptr){
        // transform landmark into the camera frame using the pose prior
        Eigen::Vector3d transformedLandmark = *T_CW * lm.point;

        // project the transformed landmark into the image frame
        if (camera_.project(transformedLandmark, &imagePoint) != arp::cameras::ProjectionStatus::Successful) {
            continue; // skip if the projection is invalid
        }
      }
      
      arp::Detection best_detection;
      float best_dist = 60.0;
      for(size_t k = 0; k < keypoints.size(); ++k) { // go through all keypoints in the frame
        uchar* keypointDescriptor = descriptors.data + k*48; // descriptors are 48 bytes long
        const float dist = brisk::Hamming::PopcntofXORed(
              keypointDescriptor, lm.descriptor.data, 3); // compute desc. distance: 3 for 3x128bit (=48 bytes)
              
        // check if a match and process accordingly
        if (dist < best_dist) {
          const cv::KeyPoint& cvKeypoint = keypoints[k];
          Eigen::Vector2d keypoint(cvKeypoint.pt.x, cvKeypoint.pt.y);
                    
          if (T_CW != nullptr) {
            // check pixel distance from projected image point to keypoint
            double pixelDistance = (keypoint - imagePoint).norm();
            if (pixelDistance > 2) {
              continue; // skip if the pixel distance is too large
            }
          }
          
          best_detection = arp::Detection {keypoint, lm.point, lm.landmarkId};
          best_dist = dist;
        }
      }

      if (best_dist >= 60.0)
        continue;
      else{
        if (std::find(detections.begin(), detections.end(), best_detection) == detections.end()){
          detections.push_back(best_detection);
          numberOfMatches++;
        }
      }

    }

    // find the keyframe with highest number of matches
    if (numberOfMatchesForActiveFrame < numberOfMatches)
    {
      numberOfMatchesForActiveFrame = numberOfMatches;
      activeFrameId_ = id;
    }
  } 

  return numberOfMatchesForActiveFrame > 0;
}

struct Point3dComparator {
    bool operator()(const cv::Point3d& a, const cv::Point3d& b) const {
        // Compare x, then y, then z
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    }
};

bool Frontend::detectAndMatch(const cv::Mat& image, const Eigen::Vector3d & extractionDirection, 
                              DetectionVec & detections, kinematics::Transformation & T_CW, 
                              cv::Mat & visualisationImage, bool needsReInitialisation)
{
  
  detections.clear(); // make sure empty
  // to gray:
  cv::Mat grayScale;
  cv::cvtColor(image, grayScale, CV_BGR2GRAY);

  // run BRISK detector and descriptor extractor:
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  detectAndDescribe(grayScale, extractionDirection, keypoints, descriptors);
  
  bool isDetectSuccess = false;
  
  if (!needsReInitialisation){
    std::set<uint64_t> covisibilities = covisibilities_.at(activeFrameId_);
    covisibilities.insert(activeFrameId_);

    isDetectSuccess = detectFrames(covisibilities, keypoints, descriptors, detections, &T_CW);  
  }

  if(needsReInitialisation || !isDetectSuccess){
    // get poses
    auto features = convertMatToTDescriptor(descriptors);
    DBoW2::QueryResults results;
    int numPosesToMatch = 5;
    dBowDatabase_.query(features, results, numPosesToMatch);

    std::set<uint64_t> possibleFrames;
    for(const DBoW2::Result& res : results) {
      possibleFrames.insert(landmarks_vec_.at(res.Id));
    }
    
    isDetectSuccess = detectFrames(possibleFrames, keypoints, descriptors, detections,
                                   needsReInitialisation ? nullptr : &T_CW);
  }


  // run RANSAC (to remove outliers and get pose T_CW estimate)
  std::vector<cv::Point3d> worldPoints;
  std::vector<cv::Point2d> imagePoints;
  for (const auto& detection : detections) {
      worldPoints.emplace_back(detection.landmark.x(), detection.landmark.y(), detection.landmark.z());
      imagePoints.emplace_back(detection.keypoint.x(), detection.keypoint.y());
  }

  std::vector<int> inliers;
  bool isRansacSuccess = ransac(worldPoints, imagePoints, T_CW, inliers);

  // filter out outliers
  if (isRansacSuccess){
    DetectionVec filteredDetections;
    for (int index : inliers) {
        filteredDetections.push_back(detections[index]);
    }
    detections = std::move(filteredDetections);
  }

  // visualise by painting stuff into visualisationImage:
  visualisationImage = image.clone();
  
  // all the landmarks
  for(const auto& point : imagePoints) {
    cv::circle(visualisationImage,
                 point,
                 3, cv::Scalar(255,0,0), -1, CV_AA); // blue
  }
  
  // detected landmarks 
  for(const auto& detection : detections) {
    cv::circle(visualisationImage,
                 cv::Point2d(detection.keypoint.x(), detection.keypoint.y()),
                 3, cv::Scalar(0,255,0), -1, CV_AA); // red
  }

  return isRansacSuccess;
}


void Frontend::buildDBoWDatabase()
{
  landmarks_vec_.clear();

  for(const auto& lms : landmarks_) { // go through all poses
    std::vector<DBoW2::FBrisk::TDescriptor> features;

    for(const auto& lm : lms.second) {
      auto feat = convertMatToTDescriptor(lm.descriptor)[0];
      features.push_back(feat);
    }

    landmarks_vec_.push_back(lms.first);
    dBowDatabase_.add(features);
  }
}

}  // namespace arp

