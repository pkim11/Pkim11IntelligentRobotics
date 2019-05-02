#include <ForwardKinematics.hpp>

namespace IRlibrary
{
	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Blist expressed in the end-effector frame, and the list of joint values thetalist **/
	SE3Mat FKinBody (SE3Mat M, std::vector <ScrewAxis> Blist, std::vector <double> thetaList){
		SE3Mat T_endEffector;
		return T_endEffector;
	}

	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Slist expressed in the space frame, and the list of joint values thetalist **/
	SE3Mat FKinSpace (SE3Mat M, std::vector <ScrewAxis> Slist, std::vector <double> thetaList){
		SE3Mat T_endEffector;
		return T_endEffector;
	}

} /* IRlibrary */ 
