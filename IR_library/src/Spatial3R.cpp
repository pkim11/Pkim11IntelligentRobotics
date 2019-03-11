#include "Spatial3R.hpp"

namespace IRlibrary
{
	void Spatial3R::zeroForwardKinematics() {
		x = base;
		axisAngle.omega = {0, 0, 1};
		axisAngle.theta = 0.0;
	}
	
} /* IRlibrary */ 


