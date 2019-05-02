#include <Jacobian.hpp>
//#include <JACOBIAN_HPP>
//#include <TypeDefs.hpp>

namespace IRlibrary{
	/** Computes the body Jacobian Jb(θ) ∈ R6×n given a list of joint 
	screws Bi expressed in the body frame and a list of joint angles.**/
	Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd& Blist, const Eigen::MatrixXd& thetaList) {
		Eigen::MatrixXd Jb = Blist;
		Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
		Eigen::VectorXd bListTemp(Blist.col(0).size());
		for (int i = thetaList.size() - 2; i >= 0; i--) {
			bListTemp << Blist.col(i + 1) * thetaList(i + 1);
			T = T * MatrixExp6(VecTose3(-1 * bListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Jb.col(i) = Adjoint(T) * Blist.col(i);
		}
		return Jb;
	}



	/**Computes the space Jacobian Js(θ) ∈ R6×n given a list of joint 
	screws Si expressed in the fixed space frame and a list of joint angles.**/
	Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& thetaList) {
		Eigen::MatrixXd Js = Slist;
		Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
		Eigen::VectorXd sListTemp(Slist.col(0).size());
		for (int i = 1; i < thetaList.size(); i++) {
			sListTemp << Slist.col(i - 1) * thetaList(i - 1);
			T = T * MatrixExp6(VecTose3(sListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Js.col(i) = Adjoint(T) * Slist.col(i);
		}

		return Js;
	}
}/* IRlibrary */
