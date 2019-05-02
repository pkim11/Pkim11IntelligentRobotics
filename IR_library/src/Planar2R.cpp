#include "Planar2R.hpp"

namespace IRlibrary
{
	void Planar2R::zeroForwardKinematics() {

		// creating matrix for method
/*
		SE3Mat mat;
			mat<<1,0,0,l[0]+l[1],
				0,1,0,0,
				0,0,1,0,
				0,0,0,1;

		//creating list/vector of ScrewAxis for method
		ScrewAxis S0;
		ScrewAxis S1;		
		S0 << 0,0,1,0,0,0;
		S1 << 0,0,1,-l[0],0,0;		

		std::vector <ScrewAxis> screwList;
		screwList.push_back(S0);
		screwList.push_back(S1);

		//creating list/vector of double for the list of joint values
		std::vector<double> angleList;
		angleList.push_back(q[0]);
		angleList.push_back(q[1]);

		//trying out method
		//xy = FKinSpace(mat, screwList, q);
		auto temp = FKinSpace(mat,screwList,angleList);
		xy= temp.col(3).head(2);

		//xy = base + l;
*/
		//below is the hardcoded option, I have it in case above does not work...
		//created below first before making above, and then realized that I should
		//use the methods we created earlier

		//above did not work. After lots of debugging I believe the problem is in my 
		//matrixExp6 method but I cannot find the problem
		// I will use the hardcoded option for now
		xy[0] = base[0] + l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1]);
		xy[1] = base[1] + l[0] * sin(q[0]) + l[1] * sin(q[0] + q[1]);
	/** Computes the end-effector frame given the zero position of the end-effector M, list of joint screws Slist expressed in the space frame, and the list of joint values thetalist **/
	//SE3Mat FKinSpace (SE3Mat M, std::vector <ScrewAxis> Slist, std::vector <double> thetaList)
	}

	//////////////
	bool Planar2R::inverseKinematics(Vec2 const & xy_in){
		auto xyp = xy_in - base;
		double l1 = l[0];
		double l2 = l[1];
		double pSqr = xyp.squaredNorm();

		xy = xy_in;
		double beta = acos ((l1 * l1 + l2 * l2 - pSqr) / (2 * l1 * l2 ));
		double alpha = acos ((pSqr + l1 * l1 - l2 * l2)/(2 * l1 * std::sqrt(pSqr)));
		double gamma = atan2(xyp[1], xyp[0]);

		q[0] = (gamma + alpha);
		q[1] = (beta - M_PI);
		
		/*
		if (pSqr > (l1 + l1) * (l1 + l2) || pSqr < (l1 - l2) * (l1 - l2)) {
			std::cout << "Planar2R::inverseKinematics [Error] Point outside workspace\n";
		return 1;
		}
		if (pSqr > .99 * (l1 + l1) * (l1 + l2) || pSqr < 1.01 * (l1 - l2) * (l1 - l2)) {
			std::cout << "Planar2R::inverseKinematics [Error] Point close to singularity\n";
		return 1;
		}
		*/
		//xy = xy_in;
		//double beta = acos ((l1 * l1 + l2 * l2 - pSqr) / (2 * l1 * l2 ));
		//double alpha = acos ((pSqr + l1 * l1 - l2 * l2)/(2 * l1 * std::sqrt(pSqr)));
		//double gamma = atan2(xyp[1], xyp[0]);
		

		//q[0] = (gamma - alpha);
		//q[1] = (M_PI - beta);
		//q[0] = (gamma + alpha);
		//q[1] = (beta - M_PI);
		/*
		if(righty){
			q[0] = (gamma - alpha);
			q[1] = (M_PI - beta);
			//q[0] = q[0] + .1*(gamma - alpha);
			//q[1] = q[1] + .1*(M_PI - beta);

		}
		else {
			q[0] = gamma + alpha;
			q[1] = -M_PI + beta;
		}
		*/

		return 0;
	}
	
} /* IRlibrary */ 


