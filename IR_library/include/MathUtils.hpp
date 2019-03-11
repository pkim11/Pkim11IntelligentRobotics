#ifndef MATHUTILS
#define MATHUTILS

namespace IRlibrary {

	/** Returns true if value close to zero **/
	bool nearZero (double val, double eps = 1e-10) {
		return true;
	}

	/** Wraps angle between 0 to 2 pi **/
	double wrapTo2PI (double val) {
		return val;
	}

	/** Wraps angle between -pi to pi **/
	double wrapToPi (double val) {
		return val;
	}

} /* IRlibrary */
#endif /* ifndef MATHUTILS */
