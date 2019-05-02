#include <MathUtils.hpp>
#include <math.h>
#include <cmath>

namespace IRlibrary {

	/** Returns true if value close to zero **/
	bool nearZero (double val, double eps) {
		return (std::abs(val) < eps);
	}

	/** Wraps angle (in radians) between 0 to 2 pi **/
	double wrapTo2PI (double val) {
	    double x = fmod(val,M_PI*2);
	    if (x < 0)
	        x += M_PI*2;
	    return x;
	}

	/** Wraps angle (in radians) between -pi to pi **/
	double wrapToPI (double val) {
		return remainder(val, 2.0*M_PI);
	}

	/** Converts angle from degree to radians **/
	double deg2rad (double val) {
		return val;
	}

	/** Converts angle from radians to degree **/
	double rad2deg (double val) {
		return val;
	}


} /* IRlibrary */
