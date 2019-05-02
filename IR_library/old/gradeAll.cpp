#include <cmath>
#include <gtest/gtest.h>

#include <TypeDefs.hpp>
#include <MathUtils.hpp>
#include <RigidBodyMotion.hpp>
#include <ForwardKinematics.hpp>
#include <Jacobian.hpp>

using namespace IRlibrary;

inline bool equalQ(double val1, double val2, double eps=1e-12){ return fabs(val1-val2) <= eps; }




TEST (JacobianSpace, all){
	double q[] = {M_PI/4., M_PI/6., M_PI/3., 3.0};
	double l1 = 3.0;
	double l2 = 4.0;
	std:vector <ScrewAxis> Slist;
	std:vector <double> thetaList;
	ScrewAxis s;
	S << 0, 0, 1, 0, 0, 0;
	Slist.push_back(S);
	thetaList.push_back(q[0]);

	S << 0, 0, 1, 0, -l1, 0;
	Slist.push_back(S);
	thetaList.push_back(q[1]);

	S << 0, 0, 1, 0, -l1 - l2, 0;
	Slist.push_back(S);
	thetaList.push_back(q[2]);

	S << 0, 0, 0, 0, 0, 1;
	Slist.push_back(S);
	thetaList.push_back(q[3]);

	auto Js = JacobianSpace(Slist, thetaList);

	JacobianMat Jsp (6,4);
	Jsp << 0, 0, 0, 0,
	   	   0, 0, 0, 0,
		   1, 1, 1, 0,
		   0,  l1 * sin(q[0]),  l1 * sin(q[0]) + l2 * sin(q[0] + q[1]), 0,
		   0, -l1 * sin(q[0]), -l1 * sin(q[0]) - l2 * sin(q[0] + q[1]), 0,
		   0,0,0,1;

	EXPECT_TRUE(Jsp.isApprox(Js));

}


int main(int argc, char *argv[])
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
