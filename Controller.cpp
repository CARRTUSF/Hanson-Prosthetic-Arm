#include <iostream>
#include <armadillo>
#include "Kinematics.h"
#include "dynamixel_sdk.h"
#include "Motors.h"
#include <stdlib.h>
#include <cmath>
#include <time.h>
#include <thread>;
#include <fstream>

#define _USE_MATH_DEFINES

using namespace std;
using namespace arma;
using namespace dynamixel;

int convertPosition(double theta);
double convertTheta(int pos);
void readIMU();

bool stop = false;
string imuOutput;

int main(int argc, char** argv)
{

    cout << "Testing out kinematics: " << endl;
    mat Ti, Tf;

    // load from file
    
    Ti.load("C:/Users/ttruo/MagicLeap/mlsdk/v0.24.1/Ti.txt");
    Tf.load("C:/Users/ttruo/MagicLeap/mlsdk/v0.24.1/Tf.txt");

    Ti.print("Initial Transform");

    Tf.print("Final Transform");

    Kinematics calculation;
    Motors dynamixelMotors;

   // for (int i = 1; i <= 7; i++)
     //   dynamixelMotors.connect(i);

    

    dynamixelMotors.setPWN(1, 600);
    dynamixelMotors.setPWN(2, 400);
    dynamixelMotors.setPWN(3, 840);
    dynamixelMotors.setPWN(4, 840);
    dynamixelMotors.setPWN(5, 840);
    dynamixelMotors.setPWN(6, 840);
    dynamixelMotors.setPWN(7, 840);
    /*
    2412
3301
3329
2399
4998
1
1434
*/
   // dynamixelMotors.moveOne(2400, 1);
    //dynamixelMotors.moveTogether(0, 3300, 3300, 2400, 5000, 0, 1400);

    vector <int> offset = dynamixelMotors.getAllMotorPosition();

    ofstream file;
    file.open("motorPosition.txt", 'w');
    for (int x : offset) {
        file << x << endl;
    }

    file.close();

  //  dynamixelMotors.moveOne(3300, 1);

//    dynamixelMotors.moveOne(offset[0] + 1000, 1);
  //  dynamixelMotors.moveOne(offset[0], 1);
   

    //system("pause");

    vector<double> theta;

    cout << endl;
    for (int x : offset) {
        cout << x << " ";
    }
    cout << endl;
    cout << endl;
    for (int x : offset) {
        theta.push_back(convertTheta(x));
    }
    cout << endl;
    system("pause");
    vector <int> orgPos = offset;
    vector<double> offsetTheta;
    for (int x : orgPos) {
        offsetTheta.push_back(convertTheta(x));
    }
    //vec A = randu<vec>(5);
    //mat B = diagmat(A);
    //B.print();

    
    theta = calculation.initialStart(Ti, Tf, theta);
    cout << endl;
    for (int i = 0; i < 7; i++) {
        cout << theta[i] << " ";
    }
    cout << endl;
    system("pause");
    for (int i = 0; i < 90; i++) {
        vector<int> motorTheta;
        for (int j = 0; j < 7; j++) {
            if (j == 4 || j == 5)
                motorTheta.push_back(convertPosition(theta[i] * 3));
            else if (j == 0)
                motorTheta.push_back(convertPosition(theta[i]));
            motorTheta.push_back(convertPosition(theta[i] * 2));
        }
        
        //imu stuff needs to be inserted here
        //read file with a filtering thing
        dynamixelMotors.moveOne(offset[0] + motorTheta[0], 1);
        dynamixelMotors.moveTogether(0, offset[1] + motorTheta[1], offset[2] + motorTheta[2], offset[3] + motorTheta[3], offset[4] + motorTheta[4], offset[5] + motorTheta[5], offset[6] + motorTheta[6]);

        offset = dynamixelMotors.getAllMotorPosition();
        for (int j = 0; j < 7; j++) {
            if (j == 4 || j == 5)
                theta[i] = convertTheta(offset[i]) / 3.0 - offsetTheta[i];
            else if (j == 0)
                theta[i] = convertTheta(offset[i]) - offsetTheta[i];
            theta[i] = convertTheta(offset[i]) / 2.0 - offsetTheta[i];
        }

        theta = calculation.updateTheta(i, theta[0], theta[1], theta[2], theta[3], theta[4], theta[5], theta[6]);
        cout << endl;
        for (int i = 0; i < 7; i++) {
            cout << theta[i] << " ";
        }
        cout << endl;

    }

    dynamixelMotors.backToNomial(orgPos);
    for (int i = 1; i<= 7; i++)
        dynamixelMotors.disconnect(i);
    /*
    stop = true;

    
    
    Motors dynamixelMotors;
    dynamixelMotors.setPWN(1, 600);
    dynamixelMotors.setPWN(2, 400);
    dynamixelMotors.setPWN(3, 840);
    dynamixelMotors.setPWN(4, 840);
    dynamixelMotors.setPWN(5, 840);
    dynamixelMotors.setPWN(6, 840);
    dynamixelMotors.setPWN(7, 840);
    for (int i = 1; i < 8; i++) {
        cout << dynamixelMotors.readPWN(i) << endl;
    }
   // dynamixelMotors.moveOne(2300, 1);
   // dynamixelMotors.moveOne(3300, 1);
    //dynamixelMotors.moveTogether(3300, 4000, 1750, 1500, 2500, 2800, 3980);
    
    vector<int> offset = dynamixelMotors.getAllMotorPosition();
   
   // dynamixelMotors.moveOne(2000, 1);
  //  for (int i = 2300; i < 3300; i++) {
    //    dynamixelMotors.moveOne(i, 1);
    //}
        dynamixelMotors.moveTogether(3300, 4000, 1750, 1500, 2500, 2800, 3980);
      //  dynamixelMotors.moveTogether(offset[0] + convertTheta(90), offset[1] + convertTheta(90), offset[2] + convertTheta(90), offset[3] + convertTheta(90), offset[4], offset[5], offset[6]);
       // dynamixelMotors.moveOne(3300, 1);
        //}
        for (int i = 0; i < 2; i++) {
            dynamixelMotors.moveOne(2500, 5);
            dynamixelMotors.moveOne(3000, 5);
        }
        //dynamixelMotors.moveTogether(2300, 2600, 1250, 3380, 1890, 2800, 3980);
        //dynamixelMotors.moveOne(2300, 1);
        dynamixelMotors.backToNomial(offset);
    */
    system("pause");
    return 0;
}

int convertPosition(double theta) {
    return floor(theta * 11.375);
}

double convertTheta(int pos) {
    return (double)pos/ 11.375;
}

void readIMU() {

    ifstream file("IMU_Output.txt");

    while (!stop) {
        getline(file, imuOutput);
    }

    file.close();
}