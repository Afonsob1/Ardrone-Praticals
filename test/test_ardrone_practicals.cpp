// Bring in my package's API, which is what I'm testing
#include "arp/cameras/PinholeCamera.hpp"
#include "arp/cameras/RadialTangentialDistortion.hpp"
// Bring in gtest
#include <gtest/gtest.h>

#include <iostream>

// Test the projection and unprojection
TEST(PinholeCamera, projectBackProject)
{
  
  // create an arbitrary camera model
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> pinholeCamera = 
      arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

  // create a random visible point in the camera coordinate frame C
  auto point_C = pinholeCamera.createRandomVisiblePoint();


  // project
  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> pointJacobian;
  pinholeCamera.project(point_C,&imagePoint, &pointJacobian);

  // numeric diference
  double delta = 0.00001;
  Eigen::Vector3d temp(point_C);
  temp.x() = point_C.x() + delta;

  Eigen::Vector2d tempResult;

  pinholeCamera.project(temp,&tempResult);

  tempResult = (tempResult-imagePoint)/delta;
  EXPECT_TRUE( fabs( tempResult.x() -  pointJacobian(0,0)) < 0.01 );
  EXPECT_TRUE( fabs( tempResult.y() -  pointJacobian(1,0)) < 0.01 );

  // numeric difference y
  temp = point_C;
  temp.y() = point_C.y() + delta;
  pinholeCamera.project(temp,&tempResult);

  tempResult = (tempResult-imagePoint)/delta;
  EXPECT_TRUE( fabs( tempResult.x() -  pointJacobian(0,1)) < 0.01 );
  EXPECT_TRUE( fabs( tempResult.y() -  pointJacobian(1,1)) < 0.01 );

  // numeric difference z
  temp = point_C;
  temp.z() = point_C.z() + delta;
  pinholeCamera.project(temp,&tempResult);

  tempResult = (tempResult-imagePoint)/delta;
  EXPECT_TRUE( fabs( tempResult.x() -  pointJacobian(0,2)) < 0.01 );
  EXPECT_TRUE( fabs( tempResult.y() -  pointJacobian(1,2)) < 0.01 );

  
  // backProject
  Eigen::Vector3d ray_C;
  pinholeCamera.backProject(imagePoint,&ray_C);


  // now they should align:
  EXPECT_TRUE(fabs(ray_C.normalized().transpose()*point_C.normalized()-1.0)<1.0e-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

