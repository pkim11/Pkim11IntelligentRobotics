#include <Planar2R.hpp>
#include <iostream>
int main(int argc, char ** argv) {
	IRlibrary::Planar2R obj(2, 3);
	IRlibrary::Vec2 q = {3.14, 0.3};
	q = {M_PI/3., M_PI/6.};
	obj.setConfig(q);
	auto xy1 = obj.getXY();
	
	////////
	
	obj.setXY(xy1);
	auto q1 = obj.getConfig();
	std::cout << "xyz position: " << std::endl; std::cout << xy1 << std::endl;
	std::cout << "ik  position: " << std::endl; std::cout <<  q1 << std::endl;
	std::cout << q[0] << " " << q[1] << std::endl;
	std::cout << q1[0] << " " << q1[1] << std::endl;
	return 0;
}

