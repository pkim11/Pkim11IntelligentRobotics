#ifndef FORWARDKINEMATICS_HPP
#define FORWARDKINEMATICS_HPP

#include <vector>
#include "RigidBodyMotion.hpp"
#include "TypeDefs.hpp"

namespace IRlibrary
{
	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Blist expressed in the end-effector frame, and the list of joint values thetalist **/

	inline SE3Mat FKinBody (SE3Mat M, std::vector <ScrewAxis> Blist, std::vector <double> thetaList){
		SE3Mat T_endEffector = M;

		for (int i = 0; i < int(thetaList.size()); i++) {
			T_endEffector = T_endEffector * MatrixExp6(VecTose3(Blist[i]*thetaList[i]));
		}

		return T_endEffector;
	}

	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Slist expressed in the space frame, and the list of joint values thetalist **/
	inline SE3Mat FKinSpace (SE3Mat M, std::vector <ScrewAxis> Slist, std::vector <double> thetaList){
		SE3Mat T_endEffector = M;

		for (int i = int((thetaList.size()) - 1); i > -1; i--) {
			T_endEffector = MatrixExp6(VecTose3(Slist[i]*thetaList[i])) * T_endEffector;
		}

		return T_endEffector;
	}

} /* IRlibrary */ 

#endif /* ifndef FORWARDKINEMATICS_HPP */
