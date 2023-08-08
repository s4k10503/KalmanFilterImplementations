#include "KalmanFilter.h"
#include <Eigen/Dense>
#include <boost/math/distributions/chi_squared.hpp>

KalmanFilter::KalmanFilter(Eigen::MatrixXd F, Eigen::MatrixXd B, Eigen::MatrixXd H, 
                           Eigen::MatrixXd x, Eigen::MatrixXd P, Eigen::MatrixXd Q, 
                           Eigen::MatrixXd R)
    : F(F), B(B), H(H), x(x), P(P), Q(Q), R(R) {}

void KalmanFilter::update_state_transition_matrix(double dt) {
    int n = F.rows();
    for (int i = 0; i < n / 2; i++) {
        F(i, i + n / 2) = dt;
    }
}

// Predict step: update the state and error covariance
void KalmanFilter::predict(Eigen::MatrixXd u) {
    x = F * x + B * u;
    P = F * P * F.transpose() + Q;
}

// Update step: update the state and error covariance based on the observation
void KalmanFilter::update(Eigen::MatrixXd z, double outlier_threshold) {
    Eigen::MatrixXd y = z - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;

    // Calculate the Mahalanobis distance
    double D = sqrt((y.transpose() * S.inverse() * y)(0, 0));

    // Calculate the threshold based on the chi-squared distribution
    boost::math::chi_squared chi_squared(y.size());
    double threshold = boost::math::quantile(chi_squared, 1 - outlier_threshold);

    // If the Mahalanobis distance is greater than the threshold,
    // consider the observation as an outlier and skip the update step
    if (D > threshold) {
        return;
    }

    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K * H) * P;
}

// Get the current state estimate
Eigen::MatrixXd KalmanFilter::get_current_state() {
    return x;
}
