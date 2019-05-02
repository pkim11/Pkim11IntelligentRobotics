#ifndef RIGIDBODYMOTION_H
#define RIGIDBODYMOTION_H

#include <eigen3/Eigen/Dense>
#include "TypeDefs.hpp"
#include <cmath>
#include <vector>

# define M_PI           3.14159265358979323846  /* pi */

//using namespace Eigen;

namespace IRlibrary {


	inline bool NearZero(const double val) {
		return (std::abs(val) < .000001);
	}

	inline Eigen::MatrixXd Normalize(Eigen::MatrixXd V) {
		V.normalize();
		return V;
	}

	/** Returns true if a 3X3 matrix satisfies properties of SO(3) **/
	inline bool isSO3(SO3Mat mat){
		if (mat.determinant() > 0)
			return std::abs((mat.transpose() * mat - Eigen::Matrix3d::Identity()).norm()) < 1e-3;
		else
			return std::abs(1.0e9) < 1e-3;
//		return (mat.transpose()*mat == Matrix3f::Identity() and (mat.determinant() == 1));
	}

	/** Returns true if a 3x3 matrix is skew-symmetric **/
	inline bool isso3(so3Mat mat){
		
		return ((mat.transpose()*-1).isApprox(mat));
	}

	/** Returns true if a 4x4 matrix is SE(3) **/
	inline bool isSE3(SE3Mat mat){
		return (isSO3(mat.block(0,0,2,2)) and mat(3,0) == 0 and mat(3,1) ==0 and mat(3,2) == 0 and mat(3,3) == 1);
	}

	/** Returns true if a 4x4 matrix is se(3) **/
	inline bool isse3(se3Mat mat){
		return (isSO3(mat.block(0,0,2,2)) and mat(3,0) == 0 and mat(3,1) ==0 and mat(3,2) == 0 and mat(3,3) == 0);
	}

	/** Checks if the vector is unit **/
	

	// Might use determinant instead of norm,
	inline bool isUnit(Vec2 vec){
		return (vec.norm() == 1);
	}
	inline bool isUnit(Vec3 vec){
		return (vec.norm() == 1);
	}
	inline bool isUnit(Vec4 vec){
		return (vec.norm() == 1);
	}

	/** Computes the inverse of rotation matrix **/
	inline Eigen::MatrixXd RotInv(const Eigen::MatrixXd& rotMatrix) {
		return rotMatrix.transpose();
	}

	/** Computes the 3X3 skew-symmetric matrix corresponding to 3x1 omega vector **/
	inline Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omg) {
		Eigen::Matrix3d m_ret;
		m_ret << 0, -omg(2), omg(1),
			omg(2), 0, -omg(0),
			-omg(1), omg(0), 0;
		return m_ret;
	}
	/** Computes the 3-vector corresponding to 3x3 skew-symmetric matrix **/
	inline Eigen::Vector3d so3ToVec(const Eigen::MatrixXd& so3mat) {
		Eigen::Vector3d v_ret;
		v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
		return v_ret;
	}

	inline Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3) {
		Eigen::Vector4d v_ret;
		v_ret << Normalize(expc3), expc3.norm();
		return v_ret;
	}





// #################################################################





	/** Computes the rotation matrix R \in SO(3) corresponding to the matrix exponential of so3Mat **/
	inline Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat) {
		Eigen::Vector3d omgtheta = so3ToVec(so3mat);

		Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
		if (NearZero(so3mat.norm())) {
			return m_ret;
		}
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = so3mat * (1 / theta);
			return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
		}
	}

	/** Computes the matrix logarithm so3mat \in so(3) of the rotation matrix R \in SO(3) **/
	inline Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d& R) {
		double acosinput = (R.trace() - 1) / 2.0;
		Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Eigen::Vector3d omg;
			if (!NearZero(1 + R(2, 2)))
				omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!NearZero(1 + R(1, 1)))
				omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = std::acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}
	}




// ###############################################





	/** Compute the 4x4 transformation matrix **/
	inline Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
		Eigen::MatrixXd m_ret(4, 4);
		m_ret << R, p,
			0, 0, 0, 1;
		return m_ret;
	}

	inline std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T) {
		std::vector<Eigen::MatrixXd> Rp_ret;
		Eigen::Matrix3d R_ret;
		// Get top left 3x3 corner
		R_ret = T.block<3, 3>(0, 0);

		Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

		Rp_ret.push_back(R_ret);
		Rp_ret.push_back(p_ret);

		return Rp_ret;
	}

	inline Eigen::MatrixXd TransInv(const Eigen::MatrixXd& transform) {
		auto rp = IRlibrary::TransToRp(transform);
		auto Rt = rp.at(0).transpose();
		auto t = -(Rt * rp.at(1));
		Eigen::MatrixXd inv(4, 4);
		inv = Eigen::MatrixXd::Zero(4,4);
		inv.block(0, 0, 3, 3) = Rt;
		inv.block(0, 3, 3, 1) = t;
		inv(3, 3) = 1;
		return inv;
	}




	/** Return the se(3) matrix corresponding to a 6-vector twist V **/
	inline Eigen::MatrixXd VecTose3(const Eigen::VectorXd& V) {
		// Separate angular (exponential representation) and linear velocities
		Eigen::Vector3d exp(V(0), V(1), V(2));
		Eigen::Vector3d linear(V(3), V(4), V(5));

		// Fill in values to the appropriate parts of the transformation matrix
		Eigen::MatrixXd m_ret(4, 4);
		m_ret << VecToso3(exp), linear,
			0, 0, 0, 0;

		return m_ret;
	}

	/** Returns Twist from se(3) matrix **/
	inline Eigen::VectorXd se3ToVec(const Eigen::MatrixXd& T) {
		Eigen::VectorXd m_ret(6);
		m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

		return m_ret;
	}

	/** Compute 6x6 adjoint matrix from T **/
	inline Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& T) {
		std::vector<Eigen::MatrixXd> R = TransToRp(T);
		Eigen::MatrixXd ad_ret(6, 6);
		ad_ret = Eigen::MatrixXd::Zero(6, 6);
		Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
		ad_ret << R[0], zeroes,
			VecToso3(R[1]) * R[0], R[0];
		return ad_ret;
	}

	/** Returns a normalized screw axis representation **/
	inline ScrewAxis ScrewToAxis(Vec3 q, Vec3 s, double h) {
		ScrewAxis S;
		
		auto temp = q.cross(s)+(h*s);

		S << s(0),
			s(1),
			s(2),
			temp(0),
			temp(1),
			temp(2);
		return S;
	}

	/** Extracts the normalized screw axis S and the distance traveled along the screw theta from the 6-vector of exponential coordinates Stheta **/
	inline void AxisAng(Twist STheta, ScrewAxis &S, double &theta){
		
		// norm returns magnitude/length of vector
		// normalize() divides a vector by the magnitude of that
		// vector, which gives us a normalized vector...in units,
		
		// the reason why we normalize for the twist to find the screw axis is because 
		// normalizing gives us a vector in units...which is the axis that it moves around

		// the reason we get the norm of STheta(twist) is to find how far it goes since
		// vec.norm() returns the magnitude/length of a vector
		S = STheta.normalized();
		theta = STheta.norm();
	}



	/** Computes the homogeneous transformation matrix T \in SE(3) corresponding to matrix exponential of se3mat \in se(3) **/

	// SO3: is for rotations, 3x3
	// so3: any skew rotation matrix
	// SE3: is for rotations+translation, 4x4
	// se3: any skew symmetric rotation and position vector has to be real
	
	// Norm() returns magnitude/length of vector
	// normalize() divides some vector by it's magnitude


	inline Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat) {
		// Extract the angular velocity vector from the transformation matrix
		Eigen::Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
		Eigen::Vector3d omgtheta = so3ToVec(se3mat_cut);

		Eigen::MatrixXd m_ret(4, 4);

		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if (NearZero(omgtheta.norm())) {
			// Reuse previous variables that have our required size
			se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
			omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
			m_ret << se3mat_cut, omgtheta,
				0, 0, 0, 1;
			return m_ret;
		}
		// If not negligible, IRlibrary page 105
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
			Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
			Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
			m_ret << MatrixExp3(se3mat_cut), GThetaV,
				0, 0, 0, 1;
			return m_ret;
		}

	}




	/** Computes the matrix logarithm se3mat \in se(3) of the homogeneous transformation matrix T \in SE(3) **/



	inline Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd& T) {
		Eigen::MatrixXd m_ret(4, 4);
		auto rp = IRlibrary::TransToRp(T);
		Eigen::Matrix3d omgmat = MatrixLog3(rp.at(0));
		Eigen::Matrix3d zeros3d = Eigen::Matrix3d::Zero(3, 3);
		if (omgmat.isApprox(zeros3d)) {
			m_ret << zeros3d, rp.at(1),
				0, 0, 0, 0;
		}
		else {
			double theta = std::acos((rp.at(0).trace() - 1) / 2.0);
			Eigen::Matrix3d logExpand1 = Eigen::MatrixXd::Identity(3, 3) - omgmat / 2.0;
			Eigen::Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
			m_ret << omgmat, logExpand*rp.at(1),
				0, 0, 0, 0;
		}
		return m_ret;
	}





} /* IRlibrary */ 

#endif /* ifndef RIGIDBODYMOTION_H */
