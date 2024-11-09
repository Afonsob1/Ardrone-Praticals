#include <memory>
#include <unistd.h>
#include <stdlib.h>
#include <chrono>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <arp/Autopilot.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // -- for later use
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
};

/// @brief inicialize camera with camera parameters
/// @return true on success
arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> initCamera(ros::NodeHandle& nh){
  double fu=0, fv=0, cu=0, cv=0, k1=0, k2=0, p1=0, p2=0;
  bool success = true;
  
  auto getParam = [&nh, &success](const std::string& param_name, double& param_value) {
      if (!nh.getParam(param_name, param_value)) {
          ROS_WARN("Failed to get param '%s'", param_name.c_str());
          success = false;
      }
  };

  getParam("arp_node/fu", fu);
  getParam("arp_node/fv", fv);
  getParam("arp_node/cu", cu);
  getParam("arp_node/cv", cv);
  getParam("arp_node/k1", k1);
  getParam("arp_node/k2", k2);
  getParam("arp_node/p1", p1);
  getParam("arp_node/p2", p2);

  ROS_INFO("Camera parameters:");
  ROS_INFO("fu: %f, fv: %f ", fu, fv);
  ROS_INFO("cu: %f, cv: %f ", cu, cv);
  ROS_INFO("k1: %f, k2: %f ", k1, k2);
  ROS_INFO("p1: %f, p2: %f ", p1, p2);
  arp::cameras::RadialTangentialDistortion distortion(k1,k2,p1,p2);

  return arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>(
        640, 360, fu, fv, cu, cv, distortion);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // setup inputs
  Subscriber subscriber;
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

  // set up autopilot
  arp::Autopilot autopilot(nh);

  // setup rendering
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, 640, 360, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;

  // init camera
  auto camera = initCamera(nh);
  camera.initialiseUndistortMaps();
  bool undistorted = false;
  auto last_time = std::chrono::steady_clock::now();

  cv::Mat image;
  auto droneStatus = autopilot.droneStatus();
  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }

    // render image, if there is a new one available
    if(subscriber.getLastImage(image)) {
      if (undistorted){
        cv::Mat undistortedImg;
        camera.undistortImage(image, undistortedImg);
        image = undistortedImg;
      } 

      cv::putText(image, arp::Autopilot::getDroneStatusString(droneStatus), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
      cv::putText(image, std::to_string((int)autopilot.droneBattery()) + "%", cv::Point(560, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);

      if (droneStatus == arp::Autopilot::DroneStatus::Inited || droneStatus == arp::Autopilot::DroneStatus::Landed) {
        cv::putText(image, "T: Launch", cv::Point(240, 350), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
      }

      if (droneStatus == arp::Autopilot::DroneStatus::Flying || droneStatus == arp::Autopilot::DroneStatus::Hovering || droneStatus == arp::Autopilot::DroneStatus::Flying2) {
        cv::putText(image, "ESC", cv::Point(300, 350), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
        cv::putText(image, "  W  ", cv::Point(10, 320), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
        cv::putText(image, "A S D", cv::Point(10, 350), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
        cv::putText(image, "   UP   ", cv::Point(480, 320), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
        cv::putText(image, "LF DW RT", cv::Point(480, 350), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
      }

      cv::putText(image, (undistorted?" Undistorted":"Raw Camera"), cv::Point(220, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
      cv::putText(image, "Press C to Change", cv::Point(230, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

      
      // https://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
      // I'm using SDL_TEXTUREACCESS_STREAMING because it's for a video player, you should
      // pick whatever suits you most: https://wiki.libsdl.org/SDL_TextureAccess
      // remember to pick the right SDL_PIXELFORMAT_* !
      texture = SDL_CreateTexture(
          renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, image.cols, image.rows);
      SDL_UpdateTexture(texture, NULL, (void*)image.data, image.step1());
      SDL_RenderClear(renderer);
      SDL_RenderCopy(renderer, texture, NULL, NULL);
      SDL_RenderPresent(renderer);
      // cleanup (only after you're done displaying. you can repeatedly call UpdateTexture without destroying it)
      SDL_DestroyTexture(texture);
    }

    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(NULL);

    // check states!
    droneStatus = autopilot.droneStatus();
    // command
    if (state[SDL_SCANCODE_ESCAPE]) {
      std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus;
      bool success = autopilot.estopReset();
      if(success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_T]) {
      std::cout << "Taking off...                          status=" << droneStatus;
      bool success = autopilot.takeoff();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    if (state[SDL_SCANCODE_L]) {
      std::cout << "Going to land...                       status=" << droneStatus;
      bool success = autopilot.land();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_C]) {
      std::cout << "Requesting flattrim calibration...     status=" << droneStatus;
      bool success = autopilot.flattrimCalibrate();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    // check keyboard to move the drone
    double forward = 0;
    double left = 0;
    double up = 0;
    double rotateLeft = 0;

    if (state[SDL_SCANCODE_UP]){
      forward += 1;
    }
    if (state[SDL_SCANCODE_DOWN]){
      forward += -1;
    }
    if (state[SDL_SCANCODE_LEFT]){
      left += 1;
    }
    if (state[SDL_SCANCODE_RIGHT]){
      left += -1;
    }
    if (state[SDL_SCANCODE_W]){
      up += 1;
    }
    if (state[SDL_SCANCODE_S]){
      up += -1;
    }
    if (state[SDL_SCANCODE_A]){
      rotateLeft += 1;
    }
    if (state[SDL_SCANCODE_D]){
      rotateLeft += -1;
    }

    if (state[SDL_SCANCODE_C]){
      auto current_time = std::chrono::steady_clock::now();
      auto time_passed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time);
      
      if(time_passed.count() >= 200){ 
        // only process the key, if pass more than 200 ms

        undistorted = !undistorted;
        std::cout << "Undistorted: " << (undistorted?"On":"Off") << std::endl;
        last_time = current_time;
      }

    }

    autopilot.manualMove(forward, left, up, rotateLeft);

  }

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

