#ifndef IRLIBRARY_PLANAR2Rmod_H
#define IRLIBRARY_PLANAR2Rmod_H

#include "Planar2R.hpp"

namespace IRlibrary {
	typedef std::array <double, 2> Vec2;
	class Planar2Rmod:public Planar2R {
		public:
			// Constructors
			Planar2Rmod() : Planar2R() { zeroForwardKinematics(); };
			Planar2Rmod(double l1_i, double l2_i) : Planar2R(l1_i, l2_i){ zeroForwardKinematics(); };

			void setConfig(const Vec2 &q_i) { q = q_i; zeroForwardKinematics(); }
			void setXY(const Vec2 &xy_i) { xy = xy_i; }
			void setOrigin(const Vec2 &o_i) { base = o_i; }
			void setLinks(double l1_i, double l2_i) {l[0] = l1_i; l[1] = l2_i; zeroForwardKinematics();}

			Vec2 getConfig() { return q; }
			Vec2 getXY() { return xy; }
			double getAngle () { double ang = q[0] + q[1]; if(ang > 2 * M_PI) ang -= 2 * M_PI; if(ang < 2 * M_PI) ang += 2 * M_PI; return ang; }

			void zeroForwardKinematics();

	};
} /* IRlibrary */ 

#endif /* ifndef IRLIBRARY_PLANAR2R_H */
