#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

class Transformation
{
public: 
	Transformation();
	virtual ~Transformation();
	Eigen::Quaterniond m_quat;
	Eigen::Vector3d m_trans;

	//bot_trans_set_identity
	void set_identity();

	//bot_trans_copy
	void make_copy(Transformation *copy);

	//bot_trans_set_from_quat_trans
	void set_quat_trans(const Eigen::Quaterniond quat, const Eigen::Vector3d trans);

	//bot_trans_set_from_velocities
	void set_from_velocities(const Eigen::Vector3d omega, const Eigen::Vector3d v, double dt);

	//bot_trans_apply_trans
	void apply_transformation(Transformation *toApply);

	//bot_trans_apply_trans_to
	void apply_transformation_to(Transformation *src1, Transformation *src2);

	//bot_trans_invert
	void invert_this();

	//bot_trans_invert_and_compose
};

#endif