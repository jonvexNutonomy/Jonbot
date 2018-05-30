#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace Eigen;



int main() {

	// Eigen::Quaterniond q0(1.0, 2.0, 3.0, 4.0);
	// Eigen::Quaterniond q1(5.0, 6.0, 7.0, 8.0);
	// Eigen::Quaterniond q2(q0.toRotationMatrix() * q1.toRotationMatrix());
	// //Eigen::Quaterniond q3;
	// //q3  = q0 * q1;
	// //cout << q2 << endl;
	// //cout << q3 << endl;
	// cout << q2.w() << endl;
	// cout << q2.x() << endl;
	// cout << q2.y() << endl;
	// cout << q2.z() << endl;
	// Eigen::Quaterniond q3 = q0 * q1;
	// cout << q3.w() << endl;
	// cout << q3.x() << endl;
	// cout << q3.y() << endl;
	// cout << q3.z() << endl;

	// double rot[4] = {1.0, 2.0, 3.0, 4.0};
	// double v[3] = {5.0, 6.0, 7.0}; 
	// double r[3];

	// double ab  =  rot[0]*rot[1], ac = rot[0]*rot[2], ad  =  rot[0]*rot[3];
 //    double nbb = -rot[1]*rot[1], bc = rot[1]*rot[2], bd  =  rot[1]*rot[3];
 //    double ncc = -rot[2]*rot[2], cd = rot[2]*rot[3], ndd = -rot[3]*rot[3];

 //    r[0] = 2*((ncc + ndd)*v[0] + (bc - ad)*v[1] + (ac + bd)*v[2]) + v[0];
 //    r[1] = 2*((ad + bc)*v[0] + (nbb + ndd)*v[1] + (cd - ab)*v[2]) + v[1];
	// r[2] = 2*((bd - ac)*v[0] + (ab + cd)*v[1] + (nbb + ncc)*v[2]) + v[2];
	// cout << "using the formula from libbot" << endl;
	// cout << r[0] << endl;
	// cout << r[1] << endl;
	// cout << r[2] << endl;

	// cout << "using eigen" << endl;

	// Eigen::Vector3d myvec(5.0, 6.0, 7.0);
	// Eigen::Vector3d myres = q0.toRotationMatrix() * myvec;
	// cout << myres(0) << endl; 
	// cout << myres(1) << endl; 
	// cout << myres(2) << endl; 


	// Eigen::Vector3d v0(1.0, 2.0, 3.0);
	// Eigen::Vector3d v1(4.0, 5.0, 6.0);
	// v0 += v1;
	// cout << v0(0) << endl;
	// cout << v0(1) << endl;
	// cout << v0(2) << endl;


}
