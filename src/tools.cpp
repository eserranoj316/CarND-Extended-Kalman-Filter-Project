#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
		const vector<VectorXd> &ground_truth) {
	/**
  TODO:
	 * Calculate the RMSE here.
	 */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		//cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/**
  TODO:
	 * Calculate a Jacobian here.
	 */

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE
	float p = px * px + py * py;
	//check division by zero
	if(fabs(p) < 0.001){
		return Hj;
	}

	//compute the Jacobian matrix
	float sqrt_p = sqrt(p);
	float p_sqrt_p = (p * sqrt_p);
	Hj << (px/sqrt_p), (py/sqrt_p), 0, 0,
			-(py/p),(px/p), 0, 0,
			py*(vx*py - vy*px)/p_sqrt_p, px*(vy*px - vx*py)/p_sqrt_p, px/sqrt_p, py/sqrt_p;
	return Hj;

}
