#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateShared(const VectorXd &z, const VectorXd &y) {
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) { // z: 2 by 1
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

	VectorXd z_pred = H_ * x_; // dimension: 2 by 1
	VectorXd y = z - z_pred; // dimension: 2 by 1
	UpdateShared(z, y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) { // z: 3 by 1
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	Tools tools;

	VectorXd h_ = VectorXd(3);
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];

	float c1 = px*px + py*py;
	float c2 = sqrt(c1);

	//check division by zero
	if (fabs(c1) < 0.0001) {
		return;
	}

	h_[0] = c2;
	h_[1] = atan2(py, px);
	h_[2] = (px*vx + py*vy) / c2;

	VectorXd y = z - h_;
	UpdateShared(z, y);
}
