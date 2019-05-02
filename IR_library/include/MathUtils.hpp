#ifndef MATHUTILS
#define MATHUTILS
#include <cmath>

namespace IRlibrary {

	/** Returns true if value close to zero **/
	/**inline bool nearZero (double val, double eps = 1e-10) {
		if (std::abs(val) < eps){
			return true;
		else
			return false;
	}
	**/
	/** Wraps angle between 0 to 2 pi **/
	inline double wrapTo2PI (double val) {
		
		return (val%(2*M_PI));
	}

	/** Wraps angle between -pi to pi **/
	/**
	inline double wrapToPi (double val) {
		val = wrapTo2PI(val)
		if (val>=0 and val<M_PI){
			return val;
		else
			return (val-(2*M_PI));
	}
	
	}
	**/

} /* IRlibrary */
#endif /* ifndef MATHUTILS */
