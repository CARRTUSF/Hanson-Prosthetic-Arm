#include <iostream>
#include <armadillo>
#include "Kinematics.h"
#include "dynamixel_sdk.h"
#include "Motors.h"
#include <stdlib.h>
#include <cmath>
#include <time.h>

#define _USE_MATH_DEFINES

using namespace std;
using namespace arma;
using namespace dynamixel;


int main(int argc, char** argv)
  {

    srand(time(NULL));

    cout << "Testing out kinematics: " << endl;
    mat Ti, Tf;
  
  // load from file
  Ti.load("Ti.txt");
  Tf.load("Tfnew.txt");

  Ti.print("Initial Transform");

  Tf.print("Final Transform");

  Kinematics calculation;

  //vec A = randu<vec>(5);
  //mat B = diagmat(A);
  //B.print();
  
  /**/
  vector<double> theta = calculation.initialStart(Ti, Tf);
  /*
  for (int i = 0; i < 7; i++) {
      cout << theta.at(i) << " ";
  }
  
  for (int i = 0; i < 120; i++) {
      theta = calculation.updateTheta(i, theta[0], theta[1], theta[2], theta[3], theta[4], theta[5], theta[6]);

  }*/
  

      int p1 = rand() % 1024;
      int p2 = rand() % 1000;
      int p3 = rand() % 200 + 500;
      int p4 = rand() % 400 + 1000;
      int p5 = rand() % 600 + 3000;
  
  Motors dynamixelMotors;

  for (int i = 0; i < 3; i++) {
      dynamixelMotors.moveOne(2500, 7);
      dynamixelMotors.moveOne(4000, 7);
      //dynamixelMotors.moveTogether(0, 2000, 500, 0, 0, 2400, 3000);
      //dynamixelMotors.moveTogether(0, 2000, 500, 4095, 2048, 2400, 3000);
  }
     // dynamixelMotors.moveOne(2048, 4);
  vector <int> angle = dynamixelMotors.getAllMotorPosition();
  for (int x : angle) {
      cout << x*((double)45/512.0) << " \n";
  }
  
  
  //cout << "This is the motor position" <<dynamixelMotors.getOneMotorPosition(4) * ((double)45 / 512);
  //dynamixelMotors.moveTogether(0, 500, 500, 1000, 3300);

  //dynamixelMotors.moveOne(1400, 4);
  //dynamixelMotors.moveOne(0, 2);
  //dynamixelMotors.moveOne(3300, 5);
  //dynamixelMotors.moveOne(700, 3);

 /*
 for (int i = 0; i < n; i++) {
     vector<int> theta = calculation.updateTheta(theta inside here)
     //convert theta to position
     dynamixelMotors.moveTogether(theta value);
     pos = dynamixelMotors.getMotors();
     //convert back to theta 
  }

  */

  system("pause");
  return 0;
  }

int convertPosition(int theta) {
    return (int)((double)45 / 512) * theta;
}

int convertTheta(int pos) {
    return (int)((double)512 / 45) * pos;
}

