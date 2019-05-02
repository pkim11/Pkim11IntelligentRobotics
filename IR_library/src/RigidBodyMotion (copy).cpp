#include <RigidBodyMotion.hpp>

namespace IRlibrary {


	/** Returns true if a 3X3 matrix satisfies properties of SO(3) **/
	bool isSO3(SO3Mat mat){
		return true;
	}

	/** Returns true if a 3x3 matrix is skew-symmetric **/
	bool isso3(so3Mat mat){
		return true;
	}

	/** Returns true if a 4x4 matrix is SE(3) **/
	bool isSE3(SE3Mat mat){
		return true;
	}

	/** Returns true if a 4x4 matrix is se(3) **/
	bool isse3(se3Mat mat){
		return true;
	}

	/** Checks if the vector is unit **/
	bool isUnit(Vec2 vec){
		return true;
	}
	bool isUnit(Vec3 vec){
		return true;
	}
	bool isUnit(Vec4 vec){
		return true;
	}

	/** Computes the inverse of rotation matrix **/
	SO3Mat RotInv(SO3Mat mat){
		return mat;
	}

	/** Computes the 3X3 skew-symmetric matrix corresponding to 3x1 omega vector **/
	so3Mat VecToso3(Vec3 omega){
		so3Mat mat;
		mat << 1, 0, 0,
				0, 1, 0, 
				0, 0, 1;
		return mat;
	}

	/** Computes the 3-vector corresponding to 3x3 skew-symmetric matrix **/
	Vec3 so3ToVec(so3Mat mat){
		Vec3 omega;
		return omega;
	}

	/** Extracts the rotation axis, omega_hat, and the rotation amount, theta, from the 3-vector omega_hat theta of exponential coordinates for rotation, expc3 **/
	AxisAngle AxisAng3 (Vec3 expc3) {
		AxisAngle axisAngle;
		axisAngle.theta = expc3.norm();
		axisAngle.omega = expc3.normalized();
		return axisAngle;
	}

	/** Computes the rotation matrix R \in SO(3) corresponding to the matrix exponential of so3Mat **/
	SO3Mat MatrixExp3 (so3Mat in_mat){
		SO3Mat mat;
		mat << 1, 0, 0,
					 0, 1, 0,
					 0, 0, 1;
		return mat;
	}

	/** Computes the matrix logarithm so3mat \in so(3) of the rotation matrix R \in SO(3) **/
	so3Mat MatrixLog3(SO3Mat in_mat){
		so3Mat mat;
		mat << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;
		return mat;
	}

	/** Compute the 4x4 transformation matrix **/
	SE3Mat RpToTrans(SO3Mat R, Vec3 p){
		SE3Mat T;
		T << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		return T;
	}

	/** Extracts the rotation matrix and position vector from homogeneous transformation matrix **/
	void TransToRp(SE3Mat mat, SO3Mat &R, Vec3 &p){
		R = mat.block<3, 3>(0, 0);
		p << 0, 0, 0;
	}

	/** Inverse of transformation matrix **/
	SE3Mat TransInv(SE3Mat mat){
		return mat;
	}

	/** Return the se(3) matrix corresponding to a 6-vector twist V **/
	se3Mat VecTose3(Twist V){
		se3Mat mat;
		mat << 0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0;
		return mat;
	}

	/** Returns Twist from se(3) matrix **/
	Twist se3ToVec(se3Mat mat){
		Twist V;
		V << 0, 0, 0, 0, 0, 0;
		return V;
	}

	/** Compute 6x6 adjoint matrix from T **/
	AdjMat Adjoint(SE3Mat mat) {
		AdjMat adj_mat;
		return adj_mat;
	}

	/** Returns a normalized screw axis representation **/
	ScrewAxis ScrewToAxis(Vec3 q, Vec3 s, double h) {
		ScrewAxis S;
		return S;
	}

	/** Extracts the normalized screw axis S and the distance traveled along the screw theta from the 6-vector of exponential coordinates Stheta **/
	void AxisAng(Twist STheta, ScrewAxis &S, double &theta){
		S << 0, 0, 0, 0, 0, 1;
		theta = 0.0;
	}

	/** Computes the homogeneous transformation matrix T \in SE(3) corresponding to matrix exponential of se3mat \in se(3) **/
	SE3Mat MatrixExp6(se3Mat in_mat){
		SE3Mat mat;
		return mat;
	}

	/** Computes the matrix logarithm se3mat \in se(3) of the homogeneous transformation matrix T \in SE(3) **/
	se3Mat MatrixLog6(SE3Mat T){
		se3Mat mat;
		return mat;
	}

} /* IRlibrary */ 


