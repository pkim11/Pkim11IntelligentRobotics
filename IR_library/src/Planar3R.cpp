#include "Planar3R.hpp"

namespace IRlibrary
{
	void Planar3R::zeroForwardKinematics() {
		x[0] = base[0] + l[0];
		x[1] = base[1] + l[0];
		x[2] = q[0] + q[1] + q[2];
	}
	
} /* IRlibrary */ 


