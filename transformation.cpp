#include "transformation.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h>

Transformation::Transformation()
{

}


Transformation::~Transformation()
{

}

void Transformation::set_identity() {
	m_quat = Eigen::Quaterniond::Identity();
	m_trans = Eigen::Vector3d::Zero();
}


void Transformation::make_copy(Transformation *copy) {
	m_quat = copy->m_quat;
	m_trans = copy->m_trans;
}

void Transformation::set_quat_trans(const Eigen::Quaterniond quat, const Eigen::Vector3d trans) {
	m_quat = quat;
	m_trans = trans;
}

void Transformation::set_from_velocities(const Eigen::Vector3d omega, const Eigen::Vector3d v, double dt) {
	//maybe ask for help on this one
	//However, I dont see anywhere where this is actually used
}

void Transformation::apply_transformation(Transformation *toApply) {
	m_trans = toApply->m_quat.toRotationMatrix() * m_trans;
	m_trans += toApply->m_trans;
	m_quat = toApply->m_quat * m_quat;
}

void Transformation::apply_transformation_to(Transformation *src1, Transformation *src2) {
	m_quat = src2->m_quat;
	m_trans = src2->m_trans;
	apply_transformation(src1);
}

void Transformation::invert_this() {
	m_trans = -m_trans;
	m_trans = m_quat.inverse().toRotationMatrix() * m_trans;
	m_quat.vec() = -m_quat.vec();
}

void Transformation::invert_and_compose(Transformation *curr, Transformation *prev) {
	make_copy(prev);
	invert_this();
	apply_transformation_to(this, curr);
}

static Eigen::Vector3d Transformation::vector_interpolate_3d(const Eigen::Vector3d A, const Eigen::Vector3d B, double bw) {
	double aw = 1-bw;
	return Eigen::Vector3d(A(0)*aw + B(0)*bw, A(1)*aw + B(1)*bw, A(2)*aw + B(2)*bw);
}


static Eigen::Quaterniond Transformation::quat_interpolate(const Eigen::Quaterniond q0, const Eigen::Quaterniond q1, double u) {
	double cos_omega = q0.dot(q1);
	Eigen::Quaterniond toret;

	if (cos_omega < 0) {
		toret = -q0;
		cos_omega = -cos_omega;
	} else {
		toret = q0;
	}

	if (cos_omega > 1) {
		cos_omega = 1;
	}

	double omega = acos(cos_omega);
	double sin_omega = sin(omega);
	
	if (fabs(sin_omega) < 1e-6) {
		double mu = 1-u;
		toret.w() = toret.w()*mu + q1.w()*u;
		toret.x() = toret.x()*mu + q1.x()*u;
		toret.y() = toret.y()*mu + q1.y()*u;
		toret.z() = toret.z()*mu + q1.z()*u;
		toret.normalize();
	} else {
		double a = sin((1-u) * omega)/ sin_omega;
		double b = sin(u * omega) / sin_omega;
		toret.w() = toret.w()*a + q1.w()*b;
		toret.x() = toret.x()*a + q1.x()*b;
		toret.y() = toret.y()*a + q1.y()*b;
		toret.z() = toret.z()*a + q1.z()*b;
		//maybe normalize
		//the original does not but i'll ask
	}
	return toret;

}


static void Transformation::trans_interpolate(Transformation *A, Transformation *B, double wb) {
	m_trans = vector_interpolate_3d(A->m_trans, B->m_trans, wb);
	m_quat = quat_interpolate(A->m_quat, B->m_quat, wb);
}


Eigen::Vector3d Transformation::apply_trans_to_vec(const Eigen::Vector3d vec) {
	Eigen::Vector3d toret;
	toret = m_quat.toRotationMatrix() * vec;
	toret += m_trans;
	return toret;
}