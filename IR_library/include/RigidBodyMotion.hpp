#ifndef RIGIDBODYMOTION_H
#define RIGIDBODYMOTION_H

#include <eigen3/Eigen/Dense>
#include "TypeDefs.hpp"
#include <cmath>

//using namespace Eigen;

namespace IRlibrary {


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
	inline SO3Mat RotInv(SO3Mat mat){
		return mat.transpose();
	}

	/** Computes the 3X3 skew-symmetric matrix corresponding to 3x1 omega vector **/
	inline so3Mat VecToso3(Vec3 omega){
		so3Mat mat;
		mat << 0, omega(2), -omega(1),
				-omega(2), 0, omega(0), 
				omega(1), -omega(0), 0;
		return mat;
	}

	/** Computes the 3-vector corresponding to 3x3 skew-symmetric matrix **/
	inline Vec3 so3ToVec(so3Mat mat){
		Vec3 omega;
		omega << mat(1,2),
			mat(2,0),
			mat(0,1);
		return omega;
	}

	/** Extracts the rotation axis, omega_hat, and the rotation amount, theta, from the 3-vector omega_hat theta of exponential coordinates for rotation, expc3 **/
	inline AxisAngle AxisAng3 (Vec3 expc3) {
		AxisAngle axisAngle;
		axisAngle.theta = expc3.norm();
		axisAngle.omega = expc3.normalized();
		return axisAngle;
	}






// #################################################################





	/** Computes the rotation matrix R \in SO(3) corresponding to the matrix exponential of so3Mat **/
	inline SO3Mat MatrixExp3 (so3Mat in_mat){
/*		SO3Mat mat;
		mat << 1, 0, 0,
					 0, 1, 0,
					 0, 0, 1;
		// we will input hte skew symmetric matrix inside a previous function, so3ToVec, to 
		// get a vector. Then we will input that vector into function, AxisAng3, to get the 
		// axis and angle, theta and omega
		// rodrigues formula: rot(wHat, angleTheta) = e^(skewWHat*angleTheta) = 
		// I + sin(theta)*skewWHat + (1-cos(theta)*skewWHat^2
		Vec3 matVec = so3ToVec(in_mat);
		AxisAngle AA = AxisAng3(matVec);
		
		// creating identity matrix, and then rodrigues
		SO3Mat identity;
			identity << 1,0,0,
				0,1,0,
				0,0,1;	
		mat = identity + sin(AA.theta)*AA.omega + ((1 - cos(AA.theta))*(AA.omega*AA.omega));
		return mat;
*/

		//Vec3 omgtheta = so3ToVec(in_mat);
		SO3Mat identity;
			identity << 1,0,0,
				0,1,0,
				0,0,1;

		Vec3 matVec = so3ToVec(in_mat);
		AxisAngle AA = AxisAng3(matVec);
		double theta = AA.theta;
		SO3Mat omgmat = in_mat * (1 / theta);
		return identity + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
	}

	/** Computes the matrix logarithm so3mat \in so(3) of the rotation matrix R \in SO(3) **/
	inline so3Mat MatrixLog3(SO3Mat in_mat){

		//double in_matCos = (in_mat.trace() - 1)/2.0;
		so3Mat mat;
		mat << 0, 0, 0,
				0, 0, 0,
				0, 0, 0;
/*
		SO3Mat identity;
			identity << 1,0,0,
				0,1,0,
				0,0,1;	
		// using the algorithm, if r=i then angle=0,
		// if trace r=-1 then angle=pi, etc???
		Vec3 matVec;
		if (in_mat.isApprox(identity))
			// if rotation==identity, then theta=0 and omega=undefined
			return mat;
		else if (in_mat.trace()==-1){
			AxisAngle conditionTwo = AxisAng3(in_mat);
			
			matVec = (1.0/std::sqrt(2 * (1 + in_mat(2,2)))) * Eigen::Vector3d(in_mat(0,2),in_mat(1,2), 1+in_mat(2,2));
			mat = VecToso3(M_PI*matVec);
			return mat;
		}
		else{
			double theta = std::acos(in_matCos);
			mat = theta / 2.0 / sin(theta)*(in_mat - in_mat.transpose());
			return mat;
		}
*/		
		return mat;
	}




// ###############################################





	/** Compute the 4x4 transformation matrix **/
	inline SE3Mat RpToTrans(SO3Mat R, Vec3 p){
		SE3Mat T;
		
		T.block(0,0,2,2) << R;
		T.row(3) << 0,0,0,1;
		T.col(3).head(3) << p;
		return T;
	}

	/** Extracts the rotation matrix and position vector from homogeneous transformation matrix **/
	inline void TransToRp(SE3Mat mat, SO3Mat &R, Vec3 &p){
		R = mat.block(0,0,2,2);
		p << mat(0,3), mat(1,3), mat(2,3);
	}

	/** Inverse of transformation matrix **/
	inline SE3Mat TransInv(SE3Mat mat){
		SE3Mat inv;
		SE3Mat R;
		SE3Mat P;

		R = (mat.block(0,0,3,3)).transpose();
		P << 1,0,0, - mat(0,3),
			0,1,0,-mat(1,3),
			0,0,1,-mat(2,3),
			0,0,0,1;
		inv = R*P;
		return inv;
	}



	/** Return the se(3) matrix corresponding to a 6-vector twist V **/
	inline se3Mat VecTose3(Twist V){
		
		
		//se3Mat mat;
		//mat << 0,0,0,0,
		//	0,0,0,0,
		//	0,0,0,0,
		//	0,0,0,0;
		// V is a 6 vector. first 3 are angular velocity and 
		// last 3 are linear velocity. Grab first 3, turn into
		// VecToso3, plug into matrix and then the last 3 and plug into
		// matrix to end up with a 4x4 matrix with bottom row zeroes
		
		//Vec3 omega = V.block(0,0,3,1);
		//so3Mat omegaso3 = VecToso3(omega);
		
		// now we plug everything in
		/*mat << (omegaso3(0,0), omegaso3(0,1) omegaso3(0,2), V(3),
			omegaso3(1,0), omegaso3(1,1), omegaso3(1,2), V(4),
			omegaso3(2,0), omegaso3(2,1), omegaso3(2,2), V(5),
			0,0,0,0);*/
		//mat.block(0,0,3,3) = omegaso3.block(0,0,3,3);
		//mat.block(0,3,3,1) = V.block(2,0,3,1);
		//mat.block(3,0,1,4) << 0,0,0,1;
		//return mat;
	
		// First we separate the angular and linear velocities
		Vec3 exp = V.head(3);
		Vec3 linear = V.tail(3);
		so3Mat realExp = VecToso3(exp);
		
		// Next we fill in the transformation matrix
		se3Mat mat;
		mat.block(0,0,3,3) = realExp;
		mat.block(0,3,3,1) = linear;
		mat.row(3) << 0,0,0,0;

		//mat << realExp, linear,
		//	0, 0, 0, 0;



		return mat;
	}

	/** Returns Twist from se(3) matrix **/
	inline Twist se3ToVec(se3Mat mat){
		// grab the rotation matrix and turn to vector
		// then input the vector into top of twist as angular velocity
		// next you take the last column of matrix and plug into
		// twist as linear velocity		
		Twist V;
		auto omegaso3 = VecToso3(mat.block(0,0,2,2));
		
		V << omegaso3(0),
			omegaso3(1), 
			omegaso3(2),
			mat(0,3),
			mat(1,3),
			mat(2,3);
		return V;
	}

	/** Compute 6x6 adjoint matrix from T **/
	inline AdjMat Adjoint(SE3Mat mat) {
		AdjMat adj_mat;
		
		// input matrix is a 4x4, rotation block we can just input
		// the 0 block we can just input
		// the [p]R we have to get the last column of input, then
		// we turn into skew symmetric
		// then we multiply by the R
		// then we input it into the output matrix

		// first we will get the [p]R and then input everything into output matrix
		Vec3 p;
		p << mat(3,0),
			mat(3,1),
			mat(3,2);
		so3Mat pSkew = VecToso3(p);
		auto R = mat.block(0,0,2,2);	
		auto skewPR = pSkew*R;

		adj_mat << mat(0,0),mat(0,1),mat(0,2),0,0,0,
				mat(1,0),mat(1,1),mat(1,2),0,0,0,
				mat(2,0),mat(2,1),mat(2,2),0,0,0,
				skewPR(0,0),skewPR(0,1),skewPR(0,2),mat(0,0),mat(0,1),mat(0,2),
				skewPR(1,0),skewPR(1,1),skewPR(1,2),mat(1,0),mat(1,1),mat(1,2),
				skewPR(2,0),skewPR(2,1),skewPR(2,2),mat(2,0),mat(2,1),mat(2,2);
		return adj_mat;
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


	inline SE3Mat MatrixExp6(se3Mat in_mat){
		so3Mat in_mat_divide = in_mat.block(0,0,3,3);
		Vec3 omgtheta = so3ToVec(in_mat_divide);
		

		SE3Mat mat(4,4);
		
		SO3Mat identity;
			identity << 1,0,0,
				0,1,0,
				0,0,1;
		double theta = omgtheta.norm();
		so3Mat matTheta = in_mat.block(0,0,3,3);
		//so3Mat matTheta = in_mat.block<3,3>(0,0);

		auto omgmat = matTheta/theta;

		auto v = in_mat.block(3,3,3,1)/theta;
		//auto v = in_mat.block<3,1>(3,0);		
		SO3Mat rotMat = identity + sin(theta) * omgmat + (1-cos(theta)) * (omgmat*omgmat);
		auto rightPos = (identity * theta + (1-cos(theta)) * omgmat + (theta + sin(theta)) * (omgmat*omgmat))*v;

		//for some reason this didnt work so i commented out
		//auto expExpand = identity * theta + (1-std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
		
		mat.block(0,0,3,3) = rotMat;
		mat.block(0,3,3,1) = rightPos;
		mat.block(3,0,1,4) << 0,0,0,1;
		
		//input the matrix information using above method, so below is not needed anymore
		//auto linear(in_mat(0,3),in_mat(1,3),in_mat(2,3));
		//auto GThetaV = (expExpand*linear)/ theta;
		//mat << MatrixExp3(in_mat_divide), GThetaV, 0,0,0,1;
		
		return mat;
	}



	/** Computes the matrix logarithm se3mat \in se(3) of the homogeneous transformation matrix T \in SE(3) **/



	inline se3Mat MatrixLog6(SE3Mat T){
		se3Mat mat(4,4);
		
		//We will use the TransToRp function
		/** Extracts the rotation matrix and position vector from homogeneous transformation matrix **/
		SO3Mat r;
		Vec3 p;		
		
		TransToRp(T, r, p);
		auto omgmat = MatrixLog3(r);

		SO3Mat zeros3d;
		zeros3d << 0,0,0,
			0,0,0,
			0,0,0;

		SO3Mat identity;
			identity << 1,0,0,
				0,1,0,
				0,0,1;	

		if (omgmat.isApprox(zeros3d)) {
			mat << zeros3d, r(1),
				0, 0, 0, 0;
		}
		else {
			double theta = std::acos((p.trace() - 1) / 2.0);
			auto logExpand1 = identity - omgmat / 2.0;
			auto logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			auto logExpand = logExpand1 + logExpand2;
			mat << omgmat, logExpand*p,
				0, 0, 0, 0;
		}

		return mat;
	}


} /* IRlibrary */ 

#endif /* ifndef RIGIDBODYMOTION_H */
