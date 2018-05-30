#include "transformation.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

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

Eigen::Vector3d Transformation::vector_interpolate_3d(const Eigen::Vector3d A, const Eigen::Vector3d B, double bw) {
	double aw = 1-bw;
	return Eigen::Vector3d(A(0)*aw + B(0)*bw, A(1)*aw + B(1)*bw, A(2)*aw + B(2)*bw);
}


Eigen::Quaterniond Transformation::quat_interpolate(const Eigen::Quaterniond q0, const Eigen::Quaterniond q1, double u) {
	//in rotations.c
}


void Transformation::trans_interpolate(Transformation *A, Transformation *B, double wb) {
	m_trans = vector_interpolate_3d(A->m_trans, B->m_trans, wb);
	m_quat = quat_interpolate(A->m_quat, B->m_quat, wb);
}


Eigen::Vector3d Transformation::apply_trans_to_vec(const Eigen::Vector3d vec) {
	Eigen::Vector3d toret;
	toret = m_quat.toRotationMatrix() * vec;
	toret += m_trans;
	return toret;
}