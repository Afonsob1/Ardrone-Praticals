#include <memory>
#include <unistd.h>
#include <stdlib.h>
#include <chrono>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <ros/package.h>

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

#include <arp/ViEkf.hpp>
#include <arp/VisualInertialTracker.hpp>
#include <arp/Frontend.hpp>
#include <arp/StatePublisher.hpp>
#include <arp/Planner.hpp>

#include <Eigen/Core>

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  arp::VisualInertialTracker* tracker;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!tracker) return;
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    tracker->addImage(timeMicroseconds, lastImage_);
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
    if (!tracker) return;
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d angular(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    tracker->addImuMeasurement(timeMicroseconds, angular, acc );

    
  }


 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
  
};

/// @brief get camera parameters
/// @return true on success
bool getCameraParameters(ros::NodeHandle& nh, double& fu, double&fv, double& cu, double& cv, double& k1, double& k2, double& p1, double&p2, double& mapFocalLength){
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
  getParam("arp_node/mapFocalLength", mapFocalLength);

  ROS_INFO("Camera parameters:");
  ROS_INFO("fu: %f, fv: %f ", fu, fv);
  ROS_INFO("cu: %f, cv: %f ", cu, cv);
  ROS_INFO("k1: %f, k2: %f ", k1, k2);
  ROS_INFO("p1: %f, p2: %f ", p1, p2);
  ROS_INFO("mapFocalLength: %f", mapFocalLength);
  return success;
}

bool getBriskParameters(ros::NodeHandle& nh, double& uniformityRadius, int& octaves, double& absoluteThreshold, int& maxNumKpt){
  bool success = true;
  
 if (!nh.getParam("arp_node/uniformityRadius", uniformityRadius)) {
    ROS_WARN("Failed to get param 'uniformityRadius'");
    success = false;
  }
  if (!nh.getParam("arp_node/octaves", octaves)) {
    ROS_WARN("Failed to get param 'octaves'");
    success = false;
  }
  if (!nh.getParam("arp_node/absoluteThreshold", absoluteThreshold)) {
    ROS_WARN("Failed to get param 'absoluteThreshold'");
    success = false;
  }
  if (!nh.getParam("arp_node/maxNumKpt", maxNumKpt)) {
    ROS_WARN("Failed to get param 'maxNumKpt'");
    success = false;
  }


  ROS_INFO("BRISK parameters:");
  ROS_INFO("uniformityRadius: %f, octaves: %d ", uniformityRadius, octaves);
  ROS_INFO("absoluteThreshold: %f, maxNumKpt: %d ", absoluteThreshold, maxNumKpt);
  return success;
}

bool getPlanAndFlyParameters(ros::NodeHandle& nh, double& THRESHOLD_MIDDLE_WAYPOINTS, double& THRESHOLD_LANDING, double& FLYING_HEIGHT, double& LAND_HEIGHT)
{
  bool success = true;
  
  if (!nh.getParam("arp_node//THRESHOLD_MIDDLE_WAYPOINTS", THRESHOLD_MIDDLE_WAYPOINTS)) {
    ROS_WARN("Failed to get param 'THRESHOLD_MIDDLE_WAYPOINTS'");
    success = false;
  }
  if (!nh.getParam("arp_node//THRESHOLD_LANDING", THRESHOLD_LANDING)) {
    ROS_WARN("Failed to get param 'THRESHOLD_LANDING'");
    success = false;
  }
  if (!nh.getParam("arp_node//FLYING_HEIGHT", FLYING_HEIGHT)) {
    ROS_WARN("Failed to get param 'FLYING_HEIGHT'");
    success = false;
  }
  if (!nh.getParam("arp_node//LAND_HEIGHT", LAND_HEIGHT)) {
    ROS_WARN("Failed to get param 'LAND_HEIGHT'");
    success = false;
  }

  ROS_INFO("Plan and Fly Challenge parameters:");
  ROS_INFO("THRESHOLD_MIDDLE_WAYPOINTS: %f, THRESHOLD_LANDING: %f ", THRESHOLD_MIDDLE_WAYPOINTS, THRESHOLD_LANDING);
  ROS_INFO("FLYING_HEIGHT: %f, LAND_HEIGHT: %f ", FLYING_HEIGHT, LAND_HEIGHT);
  return success;
}

void planAndFlyChallenge(
  arp::Autopilot& autopilot,
  arp::ViEkf& viEkf,
  arp::Planner& planner,
  Eigen::Vector3d & goal,
  bool& flyChallenge,
  double& THRESHOLD_MIDDLE_WAYPOINTS,
  double& THRESHOLD_LANDING,
  double& FLYING_HEIGHT,
  double& LAND_HEIGHT)
{

      // find start position
      uint64_t timestamp;
      arp::kinematics::RobotState currentState;
      viEkf.getState(timestamp, currentState);

      Eigen::Vector3d start(currentState.t_WS[0], currentState.t_WS[1], FLYING_HEIGHT);

      goal[2] = FLYING_HEIGHT;
      
      // call planner and set waypoints
      std::vector<Eigen::Vector3d> challengePath = planner.plan_path(start, goal);
      std::cout << "Path Length: " << challengePath.size() << std::endl;
      if (challengePath.size() == 0){
        std::cout << "No Path Found" << std::endl;
        return;
      }
      std::cout << "Creating Waypoints" << std::endl;

      // create waypoints dequeue
      std::deque<arp::Autopilot::Waypoint> waypoints;

      auto& last_wp = challengePath[0];

      // path to Point B
      for (int i = 1; i < challengePath.size(); i++){
        auto p = challengePath[i];
        
        arp::Autopilot::Waypoint wp;
        wp.x = p[0];
        wp.y = p[1];
        wp.z = p[2];

        // calculate yaw angle between two points
        double dx = p[0] - last_wp[0];
        double dy = p[1] - last_wp[1];

        wp.yaw = atan2(dy, dx);
        wp.posTolerance = THRESHOLD_MIDDLE_WAYPOINTS;
        wp.land = false;
        waypoints.push_back(wp);
        last_wp = p;
      }
      waypoints.back().land = true;
      waypoints.back().posTolerance = THRESHOLD_LANDING;
      waypoints.back().z = LAND_HEIGHT;

      // Path back to point A
      for (int i = challengePath.size()-2; i >= 0; i--){
        auto p = challengePath[i];
        
        arp::Autopilot::Waypoint wp;
        wp.x = p[0];
        wp.y = p[1];
        wp.z = p[2];

        // calculate yaw angle between two points
        double dx = p[0] - last_wp[0];
        double dy = p[1] - last_wp[1];

        wp.yaw = atan2(dy, dx);
        wp.posTolerance = THRESHOLD_MIDDLE_WAYPOINTS;
        wp.land = false;
        waypoints.push_back(wp);
        last_wp = p;
      }
      waypoints.back().land = true;
      waypoints.back().posTolerance = THRESHOLD_LANDING;
      waypoints.back().z = LAND_HEIGHT;

      autopilot.flyPath(waypoints);

      flyChallenge = true;
      autopilot.setAutomatic();
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


  // read camera params
  double fu=0, fv=0, cu=0, cv=0, k1=0, k2=0, p1=0, p2=0, mapFocalLength=0.0;
  getCameraParameters(nh, fu, fv, cu, cv, k1, k2, p1, p2, mapFocalLength);

  // init camera
  auto camera = arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>(
        640, 360, fu, fv, cu, cv, arp::cameras::RadialTangentialDistortion(k1,k2,p1,p2));
  camera.initialiseUndistortMaps();
  bool undistorted = false;
  
  double uniformityRadius, absoluteThreshold;
  int octaves, maxNumKpt;
  getBriskParameters(nh, uniformityRadius, octaves, absoluteThreshold, maxNumKpt);

  double THRESHOLD_MIDDLE_WAYPOINTS, THRESHOLD_LANDING, FLYING_HEIGHT, LAND_HEIGHT;
  getPlanAndFlyParameters(nh, THRESHOLD_MIDDLE_WAYPOINTS, THRESHOLD_LANDING, FLYING_HEIGHT, LAND_HEIGHT);

  // set up frontend -- use parameters as loaded in previous practical
  arp::Frontend frontend(640, 360, fu, fv, cu, cv, k1, k2, p1, p2,
                          uniformityRadius, octaves, absoluteThreshold, maxNumKpt, mapFocalLength);

  // load map
  std::string path = ros::package::getPath("ardrone_practicals");
  std::string mapFile;
  if(!nh.getParam("arp_node/map", mapFile))
    ROS_FATAL("error loading parameter");
  std::string mapPath = path+"/maps/"+mapFile;
  if(!frontend.loadMap(mapPath))
    ROS_FATAL_STREAM("could not load map from " << mapPath << " !");
  
  // load DBoW2 vocabulary
  std::string vocPath = path+"/maps/small_voc.yml.gz";
  if(!frontend.loadDBoW2Voc(vocPath))
    ROS_FATAL_STREAM("could not load DBoW2 voc. from " << vocPath << " !");

  // build database
  frontend.buildDBoWDatabase();

  // state publisher -- provided for rviz visualisation of drone pose:
  arp::StatePublisher pubState(nh);

  // set up EKF
  arp::ViEkf viEkf;
  Eigen::Matrix4d T_SC_mat;
  std::vector<double> T_SC_array;
  if(!nh.getParam("arp_node/T_SC", T_SC_array))
    ROS_FATAL("error loading parameter");
  T_SC_mat <<
    T_SC_array[0], T_SC_array[1], T_SC_array[2], T_SC_array[3],
    T_SC_array[4], T_SC_array[5], T_SC_array[6], T_SC_array[7],
    T_SC_array[8], T_SC_array[9], T_SC_array[10], T_SC_array[11],
    T_SC_array[12], T_SC_array[13], T_SC_array[14], T_SC_array[15];
  arp::kinematics::Transformation T_SC(T_SC_mat);
  viEkf.setCameraExtrinsics(T_SC);
  viEkf.setCameraIntrinsics(frontend.camera());
  
  // set up visual-inertial tracking
  arp::VisualInertialTracker visualInertialTracker;
  visualInertialTracker.setFrontend(frontend);
  visualInertialTracker.setEstimator(viEkf);
  subscriber.tracker = &visualInertialTracker;
  
  // set up visualisation: publish poses to topic ardrone/vi_ekf_pose
  visualInertialTracker.setVisualisationCallback(std::bind(
        &arp::StatePublisher::publish, &pubState, std::placeholders::_1,
        std::placeholders::_2));

  // tell estimator to call autopilot's controller
  visualInertialTracker.setControllerCallback(
        std::bind(&arp::Autopilot::controllerCallback, &autopilot,
        std::placeholders::_1, std::placeholders::_2));

  // // start rviz markers 
  // arp::InteractiveMarkerServer markerServer(autopilot);
  // markerServer.activate(0,0,0,0);

  auto last_time = std::chrono::steady_clock::now();
  bool flyChallenge = false;

  // Load map for planner
  std::string occupancymapFile;
  if (!nh.getParam("arp_node/occupancymap", occupancymapFile))
    ROS_FATAL("error loading occupancymap parameter");
  occupancymapFile = path+"/maps/" +occupancymapFile;
  arp::Planner planner = arp::Planner(occupancymapFile);

  // Get goal from parameters
  Eigen::Vector3d goal(0,0,0);
  std::vector<double> pointB;
  if (!nh.getParam("arp_node/pointB", pointB))
    ROS_FATAL("error loading pointB parameter");
  goal << pointB[0], pointB[1], pointB[2];

  cv::Mat image;
  auto droneStatus = autopilot.droneStatus();

  // challenge start time
  double challenge_start;

  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }

    // render image, if there is a new one available
    if(visualInertialTracker.getLastVisualisationImage(image)) {
      if (undistorted){
        cv::Mat undistortedImg;
        camera.undistortImage(image, undistortedImg);
        image = undistortedImg;
      } 

      cv::putText(image, arp::Autopilot::getDroneStatusString(droneStatus), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
      cv::putText(image, std::to_string((int)autopilot.droneBattery()) + "%", cv::Point(590, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
      cv::putText(image, (autopilot.isAutomatic() ? "Auto" : "Manual"), cv::Point(570, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
      cv::putText(image, (undistorted ? "Undistorted" : "Raw Camera"), cv::Point(260, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
      cv::putText(image, "Press C to Change", cv::Point(260, 40), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 2);

      if (droneStatus == arp::Autopilot::DroneStatus::Inited || droneStatus == arp::Autopilot::DroneStatus::Landed) {
        cv::putText(image, "T: Launch", cv::Point(270, 350), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
      }

      if (droneStatus == arp::Autopilot::DroneStatus::Flying || droneStatus == arp::Autopilot::DroneStatus::Hovering || droneStatus == arp::Autopilot::DroneStatus::Flying2) {
        cv::putText(image, "ESC", cv::Point(300, 350), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
        cv::putText(image, "  W  ", cv::Point(10, 330), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
        cv::putText(image, "A S D", cv::Point(10, 350), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
        cv::putText(image, "   UP   ", cv::Point(540, 330), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
        cv::putText(image, "LF DW RT", cv::Point(540, 350), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
      }
      
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

    if ((droneStatus == arp::Autopilot::Flying ||
      droneStatus == arp::Autopilot::Hovering ||
      droneStatus == arp::Autopilot::Flying2 ) && flyChallenge) {
        std::cout << "Waypoints Left: " << autopilot.waypointsLeft() << std::endl;
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

    // Start challenge mode
    if (state[SDL_SCANCODE_P] && !flyChallenge) {
      if (autopilot.waypointsLeft() == 0) {
        if (droneStatus == arp::Autopilot::Flying || 
          droneStatus == arp::Autopilot::Hovering || 
          droneStatus == arp::Autopilot::Flying2) {
          std::cout << "Drone has to be landed to start challenge" << std::endl;
        } else {
          std::cout << "Starting Challenge Mode" << std::endl;
          challenge_start = ros::Time::now().toSec();
          planAndFlyChallenge(autopilot, viEkf, planner, goal, flyChallenge, THRESHOLD_MIDDLE_WAYPOINTS, THRESHOLD_LANDING, FLYING_HEIGHT, LAND_HEIGHT);
        }
      } else {
        if (droneStatus == arp::Autopilot::Flying || 
          droneStatus == arp::Autopilot::Hovering || 
          droneStatus == arp::Autopilot::Flying2) {
          std::cout << "Resume challenge" << std::endl;
          flyChallenge = true;
          autopilot.setAutomatic();
        } else {
          std::cout << "Drone has to be flying to resume challenge" << std::endl;
        }
      }
    }

    if (flyChallenge && autopilot.waypointsLeft() == 0) {
      auto current_time = ros::Time::now().toSec();
      auto time_passed = (current_time - challenge_start);
      std::cout << "Challenge completed in " << time_passed << " s" << std::endl;
      flyChallenge = false;
      autopilot.setManual();
    }

    // enable automatic mode
    // if (state[SDL_SCANCODE_RCTRL]){
    //   double x, y, z, yaw;
    //   autopilot.getPoseReference(x, y, z, yaw);
    //   //markerServer.activate(x, y, z, yaw);
    //   autopilot.setAutomatic();
    // }


    // enable manual mode
    if (state[SDL_SCANCODE_SPACE]){
      //markerServer.deactivate();
      flyChallenge = false;
      autopilot.setManual();
    }

    if (!autopilot.isAutomatic()) {
      autopilot.manualMove(forward, left, up, rotateLeft);
    }
  }

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}
