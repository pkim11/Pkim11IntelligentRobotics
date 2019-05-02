#ifndef FORWARDKINEMATICS_HPP
#define FORWARDKINEMATICS_HPP

#include <vector>
#include "RigidBodyMotion.hpp"
#include "TypeDefs.hpp"

namespace IRlibrary
{
	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Blist expressed in the end-effector frame, and the list of joint values thetalist **/

	inline Eigen::MatrixXd FKinBody(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetaList) {
		Eigen::MatrixXd T = M;
		for (int i = 0; i < thetaList.size(); i++) {
			T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
		}
		return T;
	}


	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Slist expressed in the space frame, and the list of joint values thetalist **/
	inline Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList) {
		Eigen::MatrixXd T = M;
		for (int i = (thetaList.size() - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}
		return T;
	}

} /* IRlibrary */ 

#endif /* ifndef FORWARDKINEMATICS_HPP */
