#include "Planar3R.hpp"

namespace IRlibrary
{
	void Planar3R::zeroForwardKinematics() {
		x[0] = base[0] + l[0];
		x[1] = base[1] + l[0];
		x[2] = q[0] + q[1] + q[2];

	//	xy[0] = base[0] + l[0] * cos(q[0]) + l[1] * cos(q[0] + q[1]);
	//	xy[1] = base[1] + l[0] * sin(q[0]) + l[1] * sin(q[0] + q[1]);
	//	xy[2] = base[2] + l[0] * 
	}
	
} /* IRlibrary */ 


