#include "Spatial3R.hpp"
#include <stdio.h>
#include <math.h>
#include <ForwardKinematics.hpp>
#include <RigidBodyMotion.hpp>


namespace IRlibrary
{
	void Spatial3R::zeroForwardKinematics() {
		
		x = base;
		axisAngle.omega = {0, 0, 1};
		axisAngle.theta = 0.0;
		
		/*
		SE3Mat M, T;
		M << 1, 0, 0, this->l(1)+this->l(2),
			 0, 0, -1, 0,
			 0, 1, 0, this->l(0),
			 0, 0, 0, 1;

		ScrewAxis s0, s1, s2;
		s0 << 0, 0, 1, 0, 0, 0;
		s1 << 0, -1, 0, this->l(0), 0, 0;
		s2 << 0, -1, 0, this->l(0), 0, -this->l(1);

		std::vector <ScrewAxis> Slist;
		Slist.push_back(s0);
		Slist.push_back(s1);
		Slist.push_back(s2);

		std::vector <double> thetaList{ this->q(0), this->q(1), this->q(2) };

		T = FKinSpace (M, Slist, thetaList);

		this->x[0] = this->base[0]+T(0, 3);
		this->x[1] = this->base[1]+T(1, 3);
		this->x[2] = this->base[2]+T(2, 3);

		this->axisAngle = AxisAng3(so3ToVec(MatrixLog3(T.block<3, 3>(0, 0))));
		*/
	}

	bool Spatial3R::inverseKinematics(Vec3 const & xyz_in){
		auto xyzp = xyz_in - base;
		double l1 = l[0];
		double l2 = l[1];
		double l3 = l[2];

		//double radius = std::sqrt((xyzp[0]*xyzp[0]) + (xyzp[1]*xyzp[1]));
		double temp0 = xyzp[0];
		double temp1 = xyzp[1];
		double radius = std::sqrt((temp0*temp0)+(temp1*temp1));
		// double radius = xyzp[:1].squaredNorm();
		double s = xyzp[2] - l1;

		x = xyz_in;

		double theta1 = atan2(xyzp[1],xyzp[0]);// * 180/M_PI;
		double theta2 = atan2(s,radius);
		double theta3 = -l2 + std::sqrt(xyzp[0]*xyzp[0] + xyzp[1]*xyzp[1] + (xyzp[2]-l1)*(xyzp[2]-l1));
		q[0] = (theta1);
		q[1] = (theta2);
		q[2] = (theta3);
		
		return 0;
	}
	
} /* IRlibrary */ 


