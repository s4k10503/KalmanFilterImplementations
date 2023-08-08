#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    Eigen::MatrixXd F, B, H, x, P, Q, R;

    KalmanFilter(Eigen::MatrixXd F, Eigen::MatrixXd B, Eigen::MatrixXd H, 
                 Eigen::MatrixXd x, Eigen::MatrixXd P, Eigen::MatrixXd Q, 
                 Eigen::MatrixXd R);

    void update_state_transition_matrix(double dt);
    void predict(Eigen::MatrixXd u);
    void update(Eigen::MatrixXd z, double outlier_threshold = 0.01);
    Eigen::MatrixXd get_current_state();
};

#endif
