#include <Jacobian.hpp>
//#include <JACOBIAN_HPP>
//#include <TypeDefs.hpp>

namespace IRlibrary{
	/** Computes the body Jacobian Jb(θ) ∈ R6×n given a list of joint 
	screws Bi expressed in the body frame and a list of joint angles.**/
	JacobianMat JacobianBody (std::vector <ScrewAxis> Blist, std::vector <double> thetaList){

		// from my understanding of the body jacobian, it seems like we would work backwards
		// instead of starting from the base of the mechanism, or Blist[0], we would start from
		// the end of it, or Blist[end-1]

		// We would set the last column of hte body jacobina, Jb, to the multiplication of
		// the last body position and theta...in this case, the last body position would be
		// identity and theta would just be theta. 

		// Then we would work in a similar fashion as the space jacobian, but backwards.
		// Something to keep in mind is that the skew symmetric is negative when
		// calculating the exponential

		// Here are the steps I will take
		// We have the ScrewAxis and Angles
		// We need to get a body jacobian
		// this means we need to get the...
		// skew symmetric(Adjoint(e^(-bn,thetan)*any previous)) * bi
		// so we have the screw axis, from here we can get the axis and angle to for the exponential
		// but before exponential, we have to get the negative of the axis
		// next we take the exponential and save it (since we are continiously multiplying)
		// after that, we can take the adjoint of the outcome
		// multiply the adjoint by the current screw axis, ScrewAxis[i]
		// input that into the column
		// repeat until no more

		size_t n = Blist.size();
		JacobianMat Jb(6, n);

		Jb.col(n-1) = Blist[n-1];
		SE3Mat T = SE3Mat::Identity();
		AdjMat adjVar = AdjMat::Identity();
		ScrewAxis inputCol;

		// starting from the end of the list, we want to loop until we reach every
		// point, or until our iterator is below 0
		for (int i=n-2; i<0; i--){
			// Multiply screw axis (i+1) with joint angle (i+1)
			ScrewAxis previous = Blist[i+1]*thetaList[i+1];

			// se3 matrix
			// VecTose3 gives us an se3Mat from a twist/screwAxis,
			// We can use this to find the SE3Mat by computing the matrix exponential
			se3Mat beforeExp = VecTose3(previous);

			// now we want to set the rotation portion of hte se3Mat to negative,
			// RotInv sets an SO3Matrix to it's inverse and returns it
			// we can set an arbitrary matrix to the rotation portion of the above se3Mat
			// then we can find the inverse of the new matrix
			// and finally we can set the old matrix rotation area to the new inverted 3x3 matrix
			SO3Mat settingInverse = beforeExp.block(0,0,3,3);
			settingInverse = RotInv(settingInverse);
			beforeExp.block(0,0,3,3) = settingInverse;
			
			// Exponent
			// MatrixExp6 gives us an SE3Mat from an se3Mat by computing the exponential, e^[w]*theta
			// This is used to find the adjoint, which we need to multiply by the angle to get our Vs
			// we multiply this by the next one so that we will always have the previous exponential inside
			// it is to simulate how the previous joint and angle impacts the next joint and angle
			T = T * MatrixExp6(beforeExp);

			// Adjoint 
			// AdjMat Adjoint(SE3Mat mat) gives us an adjoint matrix from an SE3Mat
			// this final number will be multiplied by the corresponding angle, thetaList[i] in order
			// to give us the space jacobian for that place in the matrix
			adjVar = Adjoint(T);

			// Multiply with screw axis (i)
			inputCol = adjVar*Blist[i];
			// set the corresponding column to the resulting multiplication
			Jb.col(i) = inputCol;
		}
		return Jb;
	}



	/**Computes the space Jacobian Js(θ) ∈ R6×n given a list of joint 
	screws Si expressed in the fixed space frame and a list of joint angles.**/
	JacobianMat JacobianSpace (std::vector <ScrewAxis> Slist, std::vector <double> thetaList){

		size_t n = Slist.size();
		JacobianMat Js (6, n);
		
		Js.col(0) = Slist[0];
		SE3Mat T = SE3Mat::Identity();
		AdjMat adjVar = AdjMat::Identity();
		ScrewAxis inputCol;

		// what we want to do is multiply the axisAngle and corresponding angle with 
		// the next axisAngle and it's corresponding angle, but we cannot simply do this
		// we will need to:
		// get the skew symmetric of the AxisOfRotation and angle from the jointScrew[i]
		// above can be combined to give us an se3Mat, using the VecTose3(Twist V)
		// Now we have a se3Matrix, but we need to find the exponent in order to get the SE3Mat(Rot(w,theta))
		// To go from se3Mat to SE3Mat, we can use the MatrixExp6, which
		// takes the exponent of the input se3Mat and gives us Rot(w,theta)
		// Now we can find the adjoint of the Rot(w,theta) from previous step
		// Set the above adjoint equal to some variable (we will call this AdjVar)
		// multiply by the corresponding screw axis, so AdjVar*Slist[i]
		// loop until no more

		// For loop i = 1 to < n
		for (int i=1; i<n; i++){
			// Multiply screw axis (i-1) with joint angle (i-1)
			ScrewAxis previous = Slist[i-1]*thetaList[i-1];

			// se3 matrix
			// VecTose3 gives us an se3Mat from a twist/screwAxis,
			// We can use this to find the SE3Mat by computing the matrix exponential
			se3Mat beforeExp = VecTose3(previous);
			
			// Exponent
			// MatrixExp6 gives us an SE3Mat from an se3Mat by computing the exponential, e^[w]*theta
			// This is used to find the adjoint, which we need to multiply by the angle to get our Vs
			// we multiply this by the next one so that we will always have the previous exponential inside
			// it is to simulate how the previous joint and angle impacts the next joint and angle
			T = T * MatrixExp6(beforeExp);

			// Adjoint 
			// AdjMat Adjoint(SE3Mat mat) gives us an adjoint matrix from an SE3Mat
			// this final number will be multiplied by the corresponding angle, thetaList[i] in order
			// to give us the space jacobian for that place in the matrix
			adjVar = Adjoint(T);

			// Multiply with screw axis (i)
			inputCol = adjVar*Slist[i];
			// set the corresponding column to the resulting multiplication
			Js.col(i) = inputCol;
		}


		return Js;
	}
}/* IRlibrary */