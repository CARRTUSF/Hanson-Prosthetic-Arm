#ifndef _KINEMATICS_H
#define _KINEMATICS_H
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <armadillo>
#include <iostream>

using namespace std;
using namespace arma;

class Kinematics
{
public:
	
	Kinematics();
	void findDHTable(double TH1, double TH2, double TH3, double TH4, double TH5, double TH6, double TH7);
	void forwardKinematics(double TH1, double TH2, double TH3, double TH4, double TH5, double TH6, double TH7);
	void getJacobian(double TH1, double TH2, double TH3, double TH4, double TH5, double TH6, double TH7);
	void getTrajetory(mat roti, mat rotf, mat Pi, mat Pf, double dt, double t);
	vector<double> initialStart(mat Ti, mat Tf, vector<double> readInAngle);
	vector<double> updateTheta(int i, double TH1, double TH2, double TH3, double TH4, double TH5, double TH6, double TH7);



private:
	mat dhTable;
	
	mat Jacobian;
	mat Trajetory;

	
	
	mat dHo = zeros<mat>(7, 1);


	//transformation from origin
	mat T0_1;
	mat T0_2;
	mat T0_3;
	mat T0_4;
	mat T0_5;
	mat T0_6;
	mat T0_7;
	//end effector is the 8th link 
	mat T0_8;

	//transformation from link to link
	mat T1_2;
	mat T2_3;
	mat T3_4;
	mat T4_5;
	mat T5_6;
	mat T6_7;
	//end effector 
	mat T7_8;

	//position from origin to link
	mat P0_1;
	mat P0_2;
	mat P0_3;
	mat P0_4;
	mat P0_5;
	mat P0_6;
	mat P0_7;
	//end effector 
	mat P0_8;

	//position from link to link
	mat P1_2, P2_3, P3_4, P4_5, P5_6, P6_7,  P7_8;

	//transformation
	mat Ti, Tf;

	mat J0Inv, J0;

	double DetJ0;

	cube Tarm;

};

#endif KINEMATICS_H


