#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 64
#define MAX_SPEED 1.27
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  DistanceSensor *ds[2];
  
  char dsNames[2][10] = {"ds_right", "ds_left"};
  for (int i = 0; i < 2; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  
  GPS *gp;
  gp=robot->getGPS("universal");
  gp-> enable(TIME_STEP);
  
  InertialUnit *iu;
  iu=robot->getInertialUnit("inert");
  iu-> enable(TIME_STEP);
  
  Motor *lr;
  lr=robot->getMotor("straight");
  
  Motor *rm;
  rm=robot->getMotor("RO");
  
  Camera *cm;
  cm=robot->getCamera("EYE");
  cm->enable(TIME_STEP);
  cm->recognitionEnable(TIME_STEP); 
  
  Motor *Tyres[4];
  char Tyres_names[4][8] = {"Tyre1", "Tyre2", "Tyre3", "Tyre4"};
  for (int i = 0; i < 4; i++) {
    Tyres[i] = robot->getMotor(Tyres_names[i]);
    Tyres[i]->setPosition(INFINITY);
    Tyres[i]->setVelocity(0.0);
  }
  int avoidObstacleCounter = 0;
  while (robot->step(TIME_STEP) != -1) {
    double leftSpeed = MAX_SPEED;
    double rightSpeed = MAX_SPEED;
    double straight = 0.0; 
    double rotate = 0.0; 
    if (avoidObstacleCounter > 0) {
      avoidObstacleCounter--;
      leftSpeed = MAX_SPEED;
      rightSpeed = -MAX_SPEED;
    } else { // read sensors
      for (int i = 0; i < 2; i++) {
        if (ds[i]->getValue() < 950.0)
          avoidObstacleCounter = 100;
      }
    }
    Tyres[0]->setVelocity(leftSpeed);
    Tyres[1]->setVelocity(rightSpeed);
    Tyres[2]->setVelocity(leftSpeed);
    Tyres[3]->setVelocity(rightSpeed);
    if (avoidObstacleCounter==0 && straight<0.19){
    straight += 0.005;
    } else if (avoidObstacleCounter==0 && straight>0){
    straight += -0.005;
    }else {
    straight+=0;
    }
    lr->setPosition(straight);
    
    if (avoidObstacleCounter==0 && rotate<1.57){
    rotate += 0.005;
    } else if (avoidObstacleCounter==0 && rotate>0){
    rotate += -0.005;
    }else {
    rotate +=0;
    }
    rm->setPosition(rotate);
    std::cout<<avoidObstacleCounter<<std::endl;
    std::cout<<"X : "<<gp->getValues()[0]<<std::endl;
    std::cout<<"Y : "<<gp->getValues()[1]<<std::endl;
    std::cout<<"Z : "<<gp->getValues()[2]<<std::endl;
    std::cout<<"########################"<<std::endl;
    std::cout<<"Angle X : "<<iu->getRollPitchYaw()[0]<<std::endl;
    std::cout<<"Angle Y : "<<iu->getRollPitchYaw()[1]<<std::endl;
    std::cout<<"Angle Z : "<<iu->getRollPitchYaw()[2]<<std::endl;
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}