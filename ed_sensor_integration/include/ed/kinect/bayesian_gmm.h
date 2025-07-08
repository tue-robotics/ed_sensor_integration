#ifndef BAYESIAN_GMM_H_
#define BAYESIAN_GMM_H_

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <ros/console.h>
#include <geolib/datatypes.h>

struct GMMParams {
    float alpha;
    float kappa0;
    float psi0;
    float nu0;
};

class MAPGMM {
public:
    MAPGMM(int n_components = 2,
           const std::vector<geo::Vec3>& points = std::vector<geo::Vec3>(), const GMMParams& params = GMMParams());
    void fit(const std::vector<geo::Vec3>& points, const geo::Pose3D& sensor_pose);
    std::vector<int> get_labels() const;
    int get_inlier_component() const;

private:
    int K_;  // Number of components
    std::vector<Eigen::Vector3d> means_;
    std::vector<Eigen::Matrix3d> covs_;
    Eigen::VectorXd weights_;
    std::vector<int> labels_;
    int inlier_component_;

    //Prior parameters
    GMMParams params_;

    // Prior hyperparameters for every component
    double alpha_;  // Dirichlet prior for weights
    std::vector<Eigen::Vector3d> mu0_;  // Prior means
    std::vector<double> kappa0_;  // Prior mean strengths
    std::vector<Eigen::Matrix3d> Psi0_;  // Prior covariance matrices
    std::vector<double> nu0_;  // Prior degrees of freedom

    Eigen::MatrixXd resp_;  // Store responsibilities matrix
    double volume_;  // Volume of bounding box for uniform outlier component

    void setupPriors(const std::vector<geo::Vec3>& points);

    double eStep(const Eigen::MatrixXd& data, Eigen::MatrixXd& resp);
    void mStep(const Eigen::MatrixXd& data, const Eigen::MatrixXd& resp);
    void determineInlierComponent();
    void computeBoundingVolume(const Eigen::MatrixXd& data);
};

#endif  // BAYESIAN_GMM_H_