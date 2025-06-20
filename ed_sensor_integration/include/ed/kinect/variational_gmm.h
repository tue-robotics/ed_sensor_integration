#ifndef VARIATIONAL_GMM_H_
#define VARIATIONAL_GMM_H_

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <ros/console.h>
#include <geolib/datatypes.h>

class VBGMM {
public:
    VBGMM(int n_components = 2,
          const std::vector<geo::Vec3>& points = std::vector<geo::Vec3>());

    void fit(const std::vector<geo::Vec3>& points, const geo::Pose3D& sensor_pose);
    std::vector<int> get_labels() const;
    int get_inlier_component() const;
    Eigen::MatrixXd get_responsibilities() const;

    // VB-specific methods
    double get_lower_bound() const;
    std::vector<Eigen::Vector3d> get_posterior_means() const;
    std::vector<Eigen::Matrix3d> get_posterior_covariances() const;

private:
    int K_;  // Number of components
    int D_;  // Dimensionality (3 for 3D points)

    // Variational parameters (posterior hyperparameters)
    Eigen::VectorXd alpha_tilde_;           // Dirichlet posterior
    std::vector<Eigen::Vector3d> m_tilde_;  // Mean posterior means
    Eigen::VectorXd beta_tilde_;            // Mean posterior precisions
    std::vector<Eigen::Matrix3d> W_tilde_;  // Wishart posterior scale matrices
    Eigen::VectorXd nu_tilde_;              // Wishart posterior degrees of freedom

    // Prior hyperparameters
    double alpha0_;
    Eigen::Vector3d m0_;
    double beta0_;
    Eigen::Matrix3d W0_;
    double nu0_;

    // Variational responsibilities
    Eigen::MatrixXd resp_;

    // Cached values for efficiency
    std::vector<double> log_det_W_tilde_;
    std::vector<double> log_lambda_tilde_;
    Eigen::VectorXd log_pi_tilde_;

    // Results
    std::vector<int> labels_;
    int inlier_component_;
    double lower_bound_;

    void setupPriors(const std::vector<geo::Vec3>& points);
    void initializeVariationalParameters(const Eigen::MatrixXd& data);
    double vbEStep(const Eigen::MatrixXd& data);
    void vbMStep(const Eigen::MatrixXd& data);
    void updateCachedValues();
    double computeLowerBound(const Eigen::MatrixXd& data);
    void determineInlierComponent();

    // Helper functions
    double digamma(double x) const;
    double logGammaMultivariate(double a, int p) const;
};

#endif  // VARIATIONAL_GMM_H_