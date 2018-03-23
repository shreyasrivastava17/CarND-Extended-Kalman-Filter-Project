#include <iostream>
#include "tools.h"
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
 
  VectorXd rmse(4);
  rmse << 0,0,0,0;

	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
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
 
  MatrixXd Hj(3,4);
  float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001){
		c1 = 0.0001;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;

}

VectorXd  Tools::ConvertPolarToCartesian(float rho, float phi, float rhodot){
  VectorXd radarstatevector_;
  radarstatevector_ = VectorXd(4)
  const float px = rho*cos(phi);
  const float py = rho*sin(phi);
  const float vx = rhodot*cos(phi);
  const float vy = rhodot*sin(phi);
  radarstatevector_ << px,py,vx,vy;
  return radarstatevector_;
}

VectorXd Tools::ConvertCartesianToPolar(VectorXd& x_){
  VectorXd polarvector_(3);
  const double pi = 3.14159; 
  float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);
  float sum = pow(px,2)+pow(py,2);
  float a = px+vx+py*vy;
  float phi= arctan(px/py);
  if(phi > pi){
    phi -= 2*pi;
  }
  if(phi < -pi){
    phi += 2*pi;
  }
  polarvector_ << (sqrt(sum), phi, a/sqrt(sum));
  return polarvector_;
}

