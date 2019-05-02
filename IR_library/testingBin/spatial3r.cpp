#include "Planar2R.hpp"
#include <iostream>
#include "Spatial3R.hpp"
int main(int argc, char ** argv) {
	IRlibrary::Spatial3R obj(2, 3, 1);
	IRlibrary::Vec3 q = {3.14, 0.3, .2};
	q = {0,0,0};
	q = {M_PI/3., M_PI/6., M_PI/9.};
	obj.setConfig(q);

	////////
	
	auto xy1 = obj.getX();
	obj.setX(xy1);
	auto q1 = obj.getConfig();
	std::cout <<  q[0] << " " <<  q[1] << " " <<  q[2] << std::endl;
	std::cout << q1[0] << " " << q1[1] << " " << q1[2] << std::endl;
	return 0;
}

