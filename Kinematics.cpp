#include "Kinematics.h"
#include <iomanip>

Kinematics::Kinematics() {

}

void Kinematics::findDHTable(double TH1, double TH2, double TH3, double TH4, double TH5, double TH6, double TH7) {
	
	dhTable << 0 << 0 << 7.63 << TH1 + M_PI / 2 << endr
		<< M_PI / 2 << 0 << 0.521 << TH2 + M_PI / 2 << endr
		<< M_PI / 2 << 0 << -25.498 << TH3 - M_PI / 2 << endr
		<< -M_PI / 2 << 0.296092 << 0 << TH4 << endr
		<< -M_PI / 2 << 0.35 << 23.00216103 << TH5 - M_PI / 2 << endr
		<< -M_PI / 2 << 0 << 0 << TH6 - M_PI / 2 << endr
		<< M_PI / 2 << 0.5 << 0 << TH7<< endr
		<< 0 << 5 << 0 << 0 << endr;
	
	//dhTable.print("DH Table");

}

void Kinematics::forwardKinematics(double TH1, double TH2, double TH3, double TH4, double TH5, double TH6, double TH7) {

	findDHTable(TH1, TH2, TH3, TH4, TH5, TH6, TH7);

	T0_1 << cos(dhTable(0,3)) << -sin(dhTable(0,3)) << 0 << dhTable(0,1) << endr
		<< sin(dhTable(0,3))*cos(dhTable(0,0)) << cos(dhTable(0,3))*cos(dhTable(0,0)) << -sin(dhTable(0,0)) << -sin(dhTable(0,0))*dhTable(0,2) << endr
		<< sin(dhTable(0,3))*sin(dhTable(0,0)) << cos(dhTable(0,3))*sin(dhTable(0,0)) << cos(dhTable(0,0)) << cos(dhTable(0,0))*dhTable(0,2) << endr
		<< 0 << 0 << 0 << 1 << endr;

	//T0_1.print("T0_1");

	T1_2 << cos(dhTable(1,3)) << -sin(dhTable(1,3)) << 0 << dhTable(1,1) << endr
		<< sin(dhTable(1,3))*cos(dhTable(1,0)) << cos(dhTable(1,3))*cos(dhTable(1,0)) << -sin(dhTable(1,0)) << -sin(dhTable(1,0))*dhTable(1,2) << endr
		<< sin(dhTable(1,3))*sin(dhTable(1,0)) << cos(dhTable(1,3))*sin(dhTable(1,0)) << cos(dhTable(1,0)) << cos(dhTable(1,0))*dhTable(1,2) << endr
		<< 0 << 0 << 0 << 1 << endr;

	T2_3 << cos(dhTable(2,3)) << -sin(dhTable(2,3)) << 0 << dhTable(2,1) << endr
		<< sin(dhTable(2,3))*cos(dhTable(2,0)) << cos(dhTable(2,3))*cos(dhTable(2,0)) << -sin(dhTable(2,0)) << -sin(dhTable(2,0))*dhTable(2,2) << endr
		<< sin(dhTable(2,3))*sin(dhTable(2,0)) << cos(dhTable(2,3))*sin(dhTable(2,0)) << cos(dhTable(2,0)) << cos(dhTable(2,0))*dhTable(2,2) << endr
		<< 0 << 0 << 0 << 1 << endr;

	T3_4 << cos(dhTable(3,3)) << -sin(dhTable(3,3)) << 0 << dhTable(3,1) << endr
		<< sin(dhTable(3,3))*cos(dhTable(3,0)) << cos(dhTable(3,3))*cos(dhTable(3,0)) << -sin(dhTable(3,0)) << -sin(dhTable(3,0))*dhTable(3,2) << endr
		<< sin(dhTable(3,3))*sin(dhTable(3,0)) << cos(dhTable(3,3))*sin(dhTable(3,0)) << cos(dhTable(3,0)) << cos(dhTable(3,0))*dhTable(3,2) << endr
		<< 0 << 0 << 0 << 1 << endr;

	T4_5 << cos(dhTable(4,3)) << -sin(dhTable(4,3)) << 0 << dhTable(4,1) << endr
		<< sin(dhTable(4,3))*cos(dhTable(4,0)) << cos(dhTable(4,3))*cos(dhTable(4,0)) << -sin(dhTable(4,0)) << -sin(dhTable(4,0))*dhTable(4,2) << endr
		<< sin(dhTable(4,3))*sin(dhTable(4,0)) << cos(dhTable(4,3))*sin(dhTable(4,0)) << cos(dhTable(4,0)) << cos(dhTable(4,0))*dhTable(4,2) << endr
		<< 0 << 0 << 0 << 1 << endr;

	T5_6 << cos(dhTable(5,3)) << -sin(dhTable(5,3)) << 0 << dhTable(5,1) << endr
		<< sin(dhTable(5,3))*cos(dhTable(5,0)) << cos(dhTable(5,3))*cos(dhTable(5,0)) << -sin(dhTable(5,0)) << -sin(dhTable(5,0))*dhTable(5,2) << endr
		<< sin(dhTable(5,3))*sin(dhTable(5,0)) << cos(dhTable(5,3))*sin(dhTable(5,0)) << cos(dhTable(5,0)) << cos(dhTable(5,0))*dhTable(5,2) << endr
		<< 0 << 0 << 0 << 1 << endr;

	T6_7 << cos(dhTable(6,3)) << -sin(dhTable(6,3)) << 0 << dhTable(6,1) << endr
		<< sin(dhTable(6,3))*cos(dhTable(6,0)) << cos(dhTable(6,3))*cos(dhTable(6,0)) << -sin(dhTable(6,0)) << -sin(dhTable(6,0))*dhTable(6,2) << endr
		<< sin(dhTable(6,3))*sin(dhTable(6,0)) << cos(dhTable(6,3))*sin(dhTable(6,0)) << cos(dhTable(6,0)) << cos(dhTable(6,0))*dhTable(6,2) << endr
		<< 0 << 0 << 0 << 1 << endr;

	T7_8 << cos(dhTable(7,3)) << -sin(dhTable(7,3)) << 0 << dhTable(7,1) << endr
		<< sin(dhTable(7,3))*cos(dhTable(7,0)) << cos(dhTable(7,3))*cos(dhTable(7,0)) << -sin(dhTable(7,0)) << -sin(dhTable(7,0))*dhTable(7,2) << endr
		<< sin(dhTable(7,3))*sin(dhTable(7,0)) << cos(dhTable(7,3))*sin(dhTable(7,0)) << cos(dhTable(7,0)) << cos(dhTable(7,0))*dhTable(7,2) << endr
		<< 0 << 0 << 0 << 1 << endr;

	T0_2 = T0_1*T1_2;
	T0_3 = T0_2*T2_3;
	T0_4 = T0_3*T3_4;
	T0_5 = T0_4*T4_5;
	T0_6 = T0_5*T5_6;
	T0_7 = T0_6*T6_7;
	T0_8 = T0_7*T7_8;


	P0_1 = T0_1(span(0, 2), span(3, 3));
	P0_2 = T0_2(span(0, 2), span(3, 3));
	P0_3 = T0_3(span(0, 2), span(3, 3));
	P0_4 = T0_4(span(0, 2), span(3, 3));
	P0_5 = T0_5(span(0, 2), span(3, 3));
	P0_6 = T0_6(span(0, 2), span(3, 3));
	P0_7 = T0_7(span(0, 2), span(3, 3));
	P0_8 = T0_8(span(0, 2), span(3, 3));

	P1_2 = T1_2(span(0, 2), span(3, 3));
	P2_3 = T2_3(span(0, 2), span(3, 3));
	P3_4 = T3_4(span(0, 2), span(3, 3));
	P4_5 = T4_5(span(0, 2), span(3, 3));
	P5_6 = T5_6(span(0, 2), span(3, 3));
	P6_7 = T6_7(span(0, 2), span(3, 3));
	P7_8 = T7_8(span(0, 2), span(3, 3));


}

void Kinematics::getJacobian(double TH1, double TH2, double TH3, double TH4, double TH5, double TH6, double TH7) {

	//Jacobian Matrix

	mat w00(1, 3);
	w00.zeros();
	mat v00(1, 3);
	v00.zeros();
	
	

	//linear and angular velocity using joints instead of frame 
	mat z0 = mat{ {0},{0},{1} };
	mat z1 = T0_1(span(0, 2), span(2, 2));
	mat z2 = T0_2(span(0, 2), span(2, 2));
	mat z3 = T0_3(span(0, 2), span(2, 2));
	mat z4 = T0_4(span(0, 2), span(2, 2));
	mat z5 = T0_5(span(0, 2), span(2, 2));
	mat z6 = T0_6(span(0, 2), span(2, 2));
	mat z7 = T0_7(span(0, 2), span(2, 2));
	mat z8 = T0_8(span(0, 2), span(2, 2));

	//z0.print("z0");
	//z1.print("z1");
	//P0_8.print("p0_8");

	mat v0 = cross(z0, P0_8);
	mat v1 = cross(z1, P0_8 - P0_1);
	mat v2 = cross(z2, P0_8 - P0_2);
	mat v3 = cross(z3, P0_8 - P0_3);
	mat v4 = cross(z4, P0_8 - P0_4);
	mat v5 = cross(z5, P0_8 - P0_5);
	mat v6 = cross(z6, P0_8 - P0_6);
	mat v7 = cross(z7, P0_8 - P0_7);

	mat VJ = v1;
	VJ.insert_cols(1, v2);
	VJ.insert_cols(2, v3);
	VJ.insert_cols(3, v4);
	VJ.insert_cols(4, v5);
	VJ.insert_cols(5, v6);
	VJ.insert_cols(6, v7);
	//VJ.print("VJ = ");

	mat WJ = z1;
	WJ.insert_cols(1, z2);
	WJ.insert_cols(2, z3);
	WJ.insert_cols(3, z4);
	WJ.insert_cols(4, z5);
	WJ.insert_cols(5, z6);
	WJ.insert_cols(6, z7);
	//WJ.insert_cols(8, w88);

	mat J = VJ;
	J.insert_rows(3, WJ);

	//minimum SAngles
	mat MinJointAngles(7,1);
	MinJointAngles << -179 << endr <<
					-140 << endr <<
					-180 << endr <<
					0.01 << endr <<
					-180 << endr <<
					-45 <<  endr <<
					-30 << endr;
	//mat MinJointAngles = mat{ {-179},{ -140}, {-180},{.01},{-180}, {-45},{-30} };
//	mat MaxJointAngles = mat{ {179},{ .01}, {180},{125},{180},{65}, {30} };


	MinJointAngles = MinJointAngles*(M_PI / 180);
	//MinJointAngles.print("min joint");

	//maximum SAngles
	mat MaxJointAngles(7, 1);
	MaxJointAngles << 179 << endr
				   << 0.01 << endr	
				   << 180 << endr 
					<<125 <<endr
					<< 180 <<endr	
					<< 65 <<endr 
					<< 30 << endr;
	MaxJointAngles = MaxJointAngles*(M_PI / 180);
	//MaxJointAngles.print("max joint");

	mat dH = zeros<mat>(7, 1);
	mat CurrentAngles = mat{ {TH1}, {TH2}, {TH3}, {TH4}, {TH5}, {TH6}, {TH7} };
	//CurrentAngles.print("Angles in radians");
	//mat dH(1, 7);
	
	//mat dHo(1, 7);
	//dHo.zeros();


	//# of joints
	for (int i = 0; i < 7; i++) {
		//    dH(i) = (((MaxJointAngles(i)-MinJointAngles(i))^2)*(2*CurrentAngles(i)-MaxJointAngles(i)-MinJointAngles(i)))/(4*((MaxJointAngles(i)-CurrentAngles(i))^2)*((CurrentAngles(i)-MinJointAngles(i))^2));
		dH(i) = pow(MaxJointAngles(i, 0) - MinJointAngles(i, 0), 2) * ( 2*CurrentAngles(i, 0) - MaxJointAngles(i, 0) - MinJointAngles(i, 0));
		dH(i) /= ( 4* pow(MaxJointAngles(i, 0) - CurrentAngles(i, 0), 2) * pow(CurrentAngles(i, 0) - MinJointAngles(i, 0), 2) );

		//forced redefinitions
		if (abs(dH(i, 0)) < abs(dHo(i, 0)) && CurrentAngles(i, 0) < MaxJointAngles(i, 0) && CurrentAngles(i, 0) > MinJointAngles(i, 0)) {
			dH(i, 0) = 0;
		}
		else if (abs(dH(i, 0)) < abs(dHo(i, 0)) && CurrentAngles(i, 0) >= MaxJointAngles(i, 0) || CurrentAngles(i, 0) <= MinJointAngles(i, 0)) {
			//dH(i, 0) = 99999999999999999;
			dH(i, 0) = double(0x3f3f3f);
		}
		else if (abs(dH(i, 0)) > abs(dHo(i, 0)) && CurrentAngles(i, 0) >= MaxJointAngles(i, 0) || CurrentAngles(i, 0) <= MinJointAngles(i, 0)) {
			dH(i, 0) = 0;
		}
	}

	//weighted matrix add in refer to paper by chan & dubey
	dHo = dH;
	//dH.print("dH");
	vec tmp = ones< vec>(7);
	//vectorise(abs(dH)).print("vector form");
	//diagmat(tmp).print("1 going down");
	mat W = diagmat( tmp + vectorise(abs(dH)));
	//W.print("W");

	mat WdiagonalTerm = W.diag();

	//WdiagonalTerm.print("WdiagonalTerm");
	tmp.clear();
	tmp << ((double)1 / WdiagonalTerm(0)) << (double)1 / WdiagonalTerm(1) << (double)1 / WdiagonalTerm(2) << (double)1 / WdiagonalTerm(3) 
		<< (double)1 / WdiagonalTerm(4) << (double)1 / WdiagonalTerm(5) << (double)1 / WdiagonalTerm(6) << endr;
	//tmp.print("tmp");
	mat InW = diagmat(tmp);
	//InW.print("InW");

	mat N(1, 7);
	N << TH1 << TH2 << TH3 << TH4 << TH5 << TH6 << TH7 << endr;

	//J8=Jacobian([v88, w88],N);

	//v0.print("V0");
	//v1.print("V1");


	//J = J(span(0, 5), span(1, 7));

	//T0_8.print("T0_8");
	mat R0_8 = T0_8(span(0, 2), span(0, 2));
	//R0_8.print("R0_8");
	mat P0_8 = T0_8(span(0, 2), span(3, 3));
	//P0_8.print("P0_8");
	mat J0Linear = R0_8*VJ;
	mat J0Angular = R0_8*WJ;

	
	J0 = J;
	
	//J0.print("J0 = ");
	
	//J0 inverse calculation
	DetJ0 = double(sqrt(det(J0*W*trans(J0))));

	cout << "\nDetJ0 = " << DetJ0 << endl;

	//this is a variable number
	double w0 = 100000;
	double k0 = 2000;

	mat calc = J0*InW*trans(J0);
	//up to here good

	//calc.print("calc");
	double k;
	if (DetJ0 < w0)
		k = k0*((1 - pow((DetJ0 / w0), 2)));
	else
		k = 0;

	
	//calc.print("calc");
	//InW.print("InW");
	//W.print("W");
	//cout << "\nK=" << k << endl;
	//J0.t().print("J0 transpose");
	//inv(calc + k*eye(6,6)).print("inv");

	J0Inv = (InW)*trans(J0)*inv(calc + k*eye(6,6));
	//J0Inv.print("J0 inverse");
	
}

vector<double> Kinematics::initialStart(mat Ti, mat Tf) {

	vector<double> empty;


	this->Ti = Ti;
	this->Tf = Tf;
	

	double TH1, TH2, TH3, TH4, TH5, TH6, TH7;
	TH1 = TH2 = TH3 = TH4 = TH5 = TH6 = TH7 = 5*((double)M_PI / 180);
	vector<double> theta = { TH1, TH2, TH3, TH4, TH5, TH6, TH7 };

	for (int i = 0; i < 7; i++) {
		theta.at(i) *= ((double)180 / M_PI);
		cout << theta.at(i) << " ";
	}

	if (!Ti.is_square()) {
		printf("\n ERROR! Your Ti is not 4X4 -recheck-");
		return empty;
	}
	else if (!Tf.is_square()) {
		printf("\n ERROR! Your Tf is not 4X4 -recheck-");
		return empty;
	}
	else if (any(Ti(span(3, 3), span(0, 2)).row(0) > 0)) {
		printf("\n ERROR! Your Matrix values are incorrect at Ti(4,1:3)");
		return empty;
	}
	else if (Ti(3, 3) != 1) {
		printf("\n ERROR! Your Matrix values are incorrect at Ti(4,4)");
		return empty;
	}
	else if (any(Tf(span(3, 3), span(0, 2)).row(0) > 0)) {
		printf("\n ERROR! Your Matrix values are incorrect at Tf(4,1:3)");
		return empty;
	}
	else if (Tf(3, 3) != 1) {
		printf("\n ERROR! Your Matrix values are incorrect at Tf(4,4)");
		return empty;
	}

	//vector formation Ti

	rowvec Rx_Ti = Ti(span(0, 0), span(0, 2)).row(0);
	rowvec Ry_Ti = Ti(span(1, 1), span(0, 2)).row(0);
	rowvec Rz_Ti = Ti(span(2, 2), span(0, 2)).row(0);

	//Orthogal Check Ti
	double OrthoXY = dot(Rx_Ti,Ry_Ti);
	double OrthoXZ = dot(Rx_Ti,Rz_Ti);
	double OrthoYZ = dot(Ry_Ti , Rz_Ti);

	//unity check
	double AbsMagVec_X = norm(Rx_Ti);
	
	double AbsMagVec_Y = norm(Ry_Ti);
	double AbsMagVec_Z = norm(Rz_Ti);
	//cout << AbsMagVec_X;
	if (AbsMagVec_X < .99 || AbsMagVec_X > 1.1) {
		cout << AbsMagVec_X;
		printf("\n !CHECK MATRIX! X Unit Vectors not detected for Ti %f", AbsMagVec_X);
		
		return empty;
	}
	else if (AbsMagVec_Y < .99 || AbsMagVec_Y > 1.1) {
		printf("\n !CHECK MATRIX! Y Unit Vectors not detected for Ti");
		cout << AbsMagVec_Y;
		return empty;
	}
	else if (AbsMagVec_Z < .99 || AbsMagVec_Z > 1.1) {
		printf("\n !CHECK MATRIX! Z Unit Vectors not detected for Ti");
		cout << AbsMagVec_Z;
		return empty;
	}
	//how come abs is used to see if the value is less than 0
	else if (abs(OrthoXY) <= 0.000 || abs(OrthoXZ) <= 0.000 || abs(OrthoYZ) <= 0.000) {
		printf("\n !CHECK MATRIX! Ti matrix currently not orthogonal");
		return empty;
	}
	else {
		printf("\nIntial transofmration check good\n");
	}

	rowvec Rx_Tf = Tf(span(0, 0), span(0, 2)).row(0);
	rowvec Ry_Tf = Tf(span(1, 1), span(0, 2)).row(0);
	rowvec Rz_Tf = Tf(span(2, 2), span(0, 2)).row(0);

	//Orthogal Check Ti
	OrthoXY = dot(Rx_Tf, Ry_Tf);
	OrthoXZ = dot(Rx_Tf, Rz_Tf);
	OrthoYZ = dot(Ry_Tf, Rz_Tf);

	//unity check
	AbsMagVec_X = norm(Rx_Tf);
	AbsMagVec_Y = norm(Ry_Tf);
	AbsMagVec_Z = norm(Rz_Tf);

	if (AbsMagVec_X < .99 || AbsMagVec_X > 1.1) {
		printf("\n !CHECK MATRIX! X Unit Vectors not detected for Tf");
		return empty;
	}
	else if (AbsMagVec_Y < .99 || AbsMagVec_Y > 1.1) {
		printf("\n !CHECK MATRIX! Y Unit Vectors not detected for Tf");
		return empty;
	}
	else if (AbsMagVec_Z < .99 || AbsMagVec_Z > 1.1) {
		printf("\n !CHECK MATRIX! Z Unit Vectors not detected for Tf");
		return empty;
	}
	//how come abs is used to see if the value is less than 0
	else if (abs(OrthoXY) <= 0.000 || abs(OrthoXZ) <= 0.000 || abs(OrthoYZ) <= 0.000) {
		printf("\n !CHECK MATRIX! Tf matrix currently not orthogonal");
		return empty;
	}
	else {
		printf("\n final transformation matrix check good \n");
	}


	

	mat Pi = Ti(span(0, 2), span(3, 3));
	mat Pf = Tf(span(0, 2), span(3, 3));

	mat Roti = Ti(span(0, 2), span(0, 2));
	mat Rotf = Tf(span(0, 2), span(0, 2));

	//Roti.print("Roti");
	//Rotf.print("Rotf");

	mat R = Roti.t() * Rotf;
	//R.print("R");

	//find trajetory given rotation and position, n = floor(2/(1/160))
	findDHTable(TH1, TH2, TH3, TH4, TH5, TH6, TH7);
	getTrajetory(Roti, Rotf, Pi, Pf, (double)1/60, 2);
	forwardKinematics(TH1, TH2, TH3,TH4,TH5,TH6,TH7);



	return theta;
}

vector<double> Kinematics::updateTheta(int i, double TH1, double TH2, double TH3, double TH4, double TH5, double TH6, double TH7) {
	vector<double> theta;
	cout << "\nthis is for loop: " << i << endl;



	//convert back to radian
	TH1 *= M_PI / 180.0;
	TH2 *= M_PI / 180.0;
	TH3 *= M_PI / 180.0;
	TH4 *= M_PI / 180.0;
	TH5 *= M_PI / 180.0;
	TH6 *= M_PI / 180.0;
	TH7 *= M_PI / 180.0;

	mat THb = mat{ TH1, TH2, TH3, TH4, TH5, TH6, TH7 };
	THb.print("before");

	//cout << TH1 << " sign " << sign(TH1) << " " << sign(-12.3) << endl;

	//re adjust angles
	TH1 = TH1 - ((2 * M_PI) * floor(abs((TH1 / (2 * M_PI)))) * sign(TH1));
	TH2 = TH2 - ((2 * M_PI) * floor(abs((TH2 / (2 * M_PI)))) * sign(TH2));
	TH3 = TH3 - ((2 * M_PI) * floor(abs((TH3 / (2 * M_PI)))) * sign(TH3));
	TH4 = TH4 - ((2 * M_PI) * floor(abs((TH4 / (2 * M_PI)))) * sign(TH4));
	TH5 = TH5 - ((2 * M_PI) * floor(abs((TH5 / (2 * M_PI)))) * sign(TH5));
	TH6 = TH6 - ((2 * M_PI) * floor(abs((TH6 / (2 * M_PI)))) * sign(TH6));
	TH7 = TH7 - ((2 * M_PI) * floor(abs((TH7 / (2 * M_PI)))) * sign(TH7));
	
	mat TH = mat{ TH1, TH2, TH3, TH4, TH5, TH6, TH7 };
	TH.print("after");

	getJacobian(TH1, TH2, TH3, TH4, TH5, TH6, TH7);
	//J0.print("J0 = ");
	//J0Inv.print("J0 inv");

	//Tarm.slice(i).print();
	double dt = (double)1 / 60;
	cout << dt << endl;
	//Tarm(1, 4, i) - T0_8(1, 4);
	double dx = Tarm.slice(i)(0, 3) - T0_8(0, 3);

	double dy = Tarm.slice(i)(1, 3) - T0_8(1, 3);
	double dz = Tarm.slice(i)(2, 3) - T0_8(2, 3);
	//cout << "dx " << dx << endl;
	//cout << "dt " << dt << "dx " << dx << "dy " << dy << "dz " << dz << endl;
	// eo = 0.5*(cross(T0_8(1:3,1),Tarm(1:3,1,i)) + cross(T0_8(1:3,2),Tarm(1:3,2,i)) + cross(T0_8(1:3,3),Tarm(1:3,3,i)));
	mat eo = 0.5 * (cross(T0_8(span(0, 2), 0), Tarm.slice(i)(span(0, 2), 0)) + cross(T0_8(span(0, 2), 1), Tarm.slice(i)(span(0, 2), 1)) + cross(T0_8(span(0, 2), 2), Tarm.slice(i)(span(0, 2), 2)));

	//(cross(T0_8(span(0, 2), 0), Tarm.slice(i)(span(0, 2), 0))).print();
	//eo.print("eo");
	//Convert distances into cartesian velocities.

		//V_cart = [dx / dt; dy / dt; dz / dt; eo(1) / dt; eo(2) / dt; eo(3) / dt];
	mat V_cart(7, 1);
		V_cart << (double)dx / dt << endr
		<< (double)dy / dt << endr
		<< (double)dz / dt << endr
		<< (double)eo(0)/dt << endr
		<< (double)eo(1)/dt << endr
		<< (double)eo(2)/dt << endr;

		//V_cart.print("V_cart");
	//Convert cartesian velocities into joint velocities
	
		//J0Inv.print("J0 = ");

	mat V_joint = J0Inv * V_cart;

	V_joint.print();
	//Convert joint velocities into joint position differences
	
	double dTH1 = V_joint(0) * dt;
	double dTH2 = V_joint(1) * dt;
	double dTH3 = V_joint(2) * dt;
	double dTH4 = V_joint(3) * dt;
	double dTH5 = V_joint(4) * dt;
	double dTH6 = V_joint(5) * dt;
	double dTH7 = V_joint(6) * dt;

    cout << dTH1 << " " << dTH2 << " " << dTH3 << " " << dTH4 << " " << dTH5 << " " << dTH6 << " " << dTH7 << endl;

	//Update new joint angles.
	//goes into the dynamixel
	TH1 = TH1 + (dTH1);
	TH2 = TH2 + (dTH2);
	TH3 = TH3 + (dTH3);
	TH4 = TH4 + (dTH4);
	TH5 = TH5 + (dTH5);
	TH6 = TH6 + (dTH6);
	TH7 = TH7 + (dTH7);

	forwardKinematics(TH1, TH2, TH3, TH4, TH5, TH6, TH7);
	
	mat THtmp = mat{ TH1, TH2, TH3, TH4, TH5, TH6, TH7 };
	THtmp.print("updated");
	
	Tarm.slice(i).print("Tarm");
	T0_8.print("T0_8");
	

	theta.push_back(TH1);
	theta.push_back(TH2);
	theta.push_back(TH3);
	theta.push_back(TH4);
	theta.push_back(TH5);
	theta.push_back(TH6);
	theta.push_back(TH7);
	cout << endl << "degrees being sent" << endl;
	for (int i = 0; i < 7; i++) {
		theta[i] *= (double)180 / M_PI;
		cout << theta[i] << " ";
	}
	cout << endl;



	return theta;
}



void Kinematics::getTrajetory(mat roti, mat rotf, mat Pi, mat Pf, double dt, double t) {

	double n = ceil((double)t / dt);
	cout << n << endl;

	
	double delx = (double)(Pf(0, 0) - Pi(0, 0)) / n;
	double dely = (double)(Pf(1, 0) - Pi(1, 0)) / n;
	double delz = (double)(Pf(2, 0) - Pi(2, 0)) / n;

	//cout <<"pf " << Pf(0) << "pi " << Pi(0) << "dx " << dx << " dy " << dy << " dz " << dz << endl;

	//single angle determination for initial rotation matrix and refer to Paul pg 30-32
	mat R(3, 3);
	R = roti.t()*rotf;
	R.print("R");


	double nx = R(0, 0);
	double ny = R(1, 0);
	double nz = R(2, 0);

	double ox = R(0, 1);
	double oy = R(1, 1);
	double oz = R(2, 1);

	double ax = R(0, 2);
	double ay = R(1, 2);
	double az = R(2, 2);


	//mat tmp = mat{ nx, ny, nz, ox, oy, oz, ax, ay, az };
	//tmp.print("nx ny nz ox oy oz ax ay az");

	//sa = single angle or theta in book eqn 1.82
	//sa = atan2( (sqrt(   (  (oz - ay)^2)  )+(  (ax - nz)^2  )+(  (ny - ox)^2  )   ),   (nx + oy + az - 1)   );
	long double oz_ay = pow(oz - ay, 2);
	long double ax_nz = pow(ax - nz, 2);
	long double ny_ox = pow(ny - ox, 2);

	//cout << "\n" << oz_ay << " " << ax_nz << " " << ny_ox << endl;

	double sa = atan2(sqrt(oz_ay + ax_nz + ny_ox), (nx + oy + az - 1));

	//cout << fixed << setprecision(12) << sa << " " << sqrt(oz_ay + ax_nz + ny_ox) << " " << (nx + oy + az - 1) << endl;

	double kx;
	double ky;
	double kz;

	//normalization if sa is small
	if (sa < 0.001) {
		kx = 1;
		ky = 0;
		kz = 0;
	}

	//when sa is below 90 deg eqn 1.83 - 1.85
	else if (sa < ((M_PI / 2) - 0.001)) {
		kx = (double)(oz - ay) / (2*sin(sa));
		ky = (double)(ax - nz) / (2*sin(sa));
		kz = (double)(ny - ox) / (2*sin(sa));
	}

	//when sa is above 90 deg eqn 1.92 - 1.94
	else {
		kx = sign(oz - ay)*sqrt((nx - cos(sa)) / (1 - cos(sa)));
		ky = sign(ax - nz)*sqrt((oy - cos(sa)) / (1 - cos(sa)));
		kz = sign(ny - ox)*sqrt((az - cos(sa)) / (1 - cos(sa)));

		//eqn 1.98 - 1.103
		if (kx > ky&& kx > kz) {	//kx largest
			ky = (double)(ny + ox) / (2*kx*(1 - cos(sa)));
			kz = (double)(ax + nz) / (2*kx*(1 - cos(sa)));
		}
		else if (ky > kx&& ky > kz) {	//ky largest
			kx = (double)(ny + ox) / (2*ky*(1 - cos(sa)));
			kz = (double)(oz + ny) / (2*ky*(1 - cos(sa)));
		}
		else {	//kz largest
			kx = (double)(ax + nz) / (2*kz*(1 - cos(sa)));
			ky = (double)(oz + ay) / (2*kz*(1 - cos(sa)));
		}
	}

	double dsa = (double)sa / n;

	mat traj(n, 4);
	traj(0, 0) = Pi(0, 0);
	traj(0, 1) = Pi(1, 0);
	traj(0, 2) = Pi(2, 0);
	traj(0, 3) = 0;


	cube Tarm(4, 4, n);

	mat ti = join_rows(roti, Pi);
	ti.insert_rows(3, mat{ {0,0,0,1} });

	

	Tarm.slice(0) = ti;

	

	for (int j = 1; j < n; j++) {
		traj(j, 0) = Pi(0, 0) + j*delx;
		traj(j, 1) = Pi(1, 0) + j*dely;
		traj(j, 2) = Pi(2, 0) + j*delz;
		traj(j, 3) = j*dsa;
		
		
		double da = traj(j, 3);
		/*
		  da = traj(j,4);
  
  % eqn 1.73
  dR = [(((kx*kx)*(1-cos(da)))+   (cos(da))) (((ky*kx)*(1-cos(da)))-(kz*sin(da))) (((kz*kx)*(1-cos(da)))+(ky*sin(da)));
        (((kx*ky)*(1-cos(da)))+(kz*sin(da))) (((ky*ky)*(1-cos(da)))+   (cos(da))) (((kz*ky)*(1-cos(da)))-(kx*sin(da)));
        (((kx*kz)*(1-cos(da)))-(ky*sin(da))) (((ky*kz)*(1-cos(da)))+(kx*sin(da))) (((kz*kz)*(1-cos(da)))+   (cos(da)))];
  
 Tarm(:,:,j) = [Roti*dR [traj((j),1);traj((j),2);traj((j),3)]; 0 0 0 1];
  
		*/
		//eqn 1.73
		mat dR(3, 3);
		dR << (( (kx*kx)*(1 - cos(da))) + (cos(da))) << (((ky*kx)*(1 - cos(da))) - (kz*sin(da))) << (((kz*kx)*(1 - cos(da)) + (ky*sin(da)))) << endr
			<< (((kx*ky)*(1 - cos(da))) + (kz*sin(da))) << (((ky*ky)*(1 - cos(da))) + (cos(da))) << (((kz*ky)*(1 - cos(da))) - (kx*sin(da))) << endr
			<< (((kx*kz)*(1 - cos(da))) - (ky*sin(da))) << (((ky*kz)*(1 - cos(da))) + (kx*sin(da))) << (((kz*kz)*(1 - cos(da))) + (cos(da))) << endr;

		mat transform = join_rows(roti * dR, mat{ {traj(j, 0)}, {traj(j, 1)}, {traj(j, 2)} });
		transform.insert_rows(3, mat{ {0,0,0,1} });
		//transform.print("Testing J0");
		Tarm.slice(j) = transform;
		//cout << "\nat j " << j << endl;
		//transform.print("Tarm at slice ");

	}

	this->Tarm = Tarm;

}

