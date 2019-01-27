#ifndef IRLIBRARY_PLANAR3R_H
#define IRLIBRARY_PLANAR3R_H


#include <array>
#include <cmath>
#include "Planar2R.hpp"

namespace IRlibrary {
	typedef std::array <double, 3> Vec3;
	class Planar3R {
		protected:
			Vec3 l;
			Vec3 q; // Configuration space (angle1, angle2, angle3)
			Vec3 x; // Task space Cartesian cordinates and end effector angle (x, y, theta)
			Vec2 base; // Cordinates of the base
			Planar2R planar2r;
		public:
			// Constructors
			Planar3R() : l({0, 0, 0}), q({0, 0, 0}), base({0, 0}) { zeroForwardKinematics(); };
			Planar3R(double l1_i, double l2_i, double l3_i) : l({l1_i, l2_i, l3_i}), q({0, 0, 0}), base({0, 0}) { zeroForwardKinematics(); };

			void setConfig(const Vec3 &q_i) { q = q_i; zeroForwardKinematics(); }
			void setX(const Vec3 &x_i) { x = x_i; }
			void setOrigin(const Vec2 &o_i) { base = o_i; }
			void setLinks(double l1_i, double l2_i, double l3_i) {l[0] = l1_i; l[1] = l2_i; l[2] = l3_i; zeroForwardKinematics();}

			Vec3 getConfig() { return q; }
			Vec3 getX() { return x; }
			double getAngle () { return x[2]; }

			virtual void zeroForwardKinematics();

	};
} /* IRlibrary */ 

#endif /* ifndef IRLIBRARY_PLANAR3R_H */
