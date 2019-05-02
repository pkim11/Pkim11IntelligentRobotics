#include <iostream>
#include <Spatial3R.hpp>
int main(int argc, char ** argv) {
	//	IRlibrary::Vec3 q = {3.14, 0.3, .2};
	//q = {M_PI/3., M_PI/6., M_PI/9.};
	IRlibrary::Spatial3R obj3r(3.,3.,2.);
	//IRlibrary::Vec3 q = {M_PI/3., M_PI/4., M_PI/5.};
	IRlibrary::Vec3 q = {0.,0.,0.};
	obj3r.setConfig(q);

	////////
	
	auto xyzPosition = obj3r.getX();
	obj3r.setX(xyzPosition);
	auto q1 = obj3r.getConfig();
	//std::cout << "xyz position: " << std::endl; std::cout << xyzPosition << std::endl;
	//std::cout << "ik  position: " << std::endl; std::cout <<  q1 << std::endl;
	std::cout <<  q[0] << " " <<  q[1] << " " <<  q[2] << std::endl;
	std::cout << q1[0] << " " << q1[1] << " " << q1[2] << std::endl;
	return 0;
}



