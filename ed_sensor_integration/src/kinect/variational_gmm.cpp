#include "ed/kinect/variational_gmm.h"
#include <ros/console.h>
#include <chrono>
#include <cmath>
#include <random>
#include <boost/math/special_functions/digamma.hpp>
#include <boost/math/special_functions/gamma.hpp>

VBGMM::VBGMM(int n_components, const std::vector<geo::Vec3>& points) : K_(n_components), D_(3), inlier_component_(0) {
    // Initialize containers
    alpha_tilde_ = Eigen::VectorXd::Zero(K_);
    m_tilde_.resize(K_);
    beta_tilde_ = Eigen::VectorXd::Zero(K_);
    W_tilde_.resize(K_);
    nu_tilde_ = Eigen::VectorXd::Zero(K_);

    log_det_W_tilde_.resize(K_);
    log_lambda_tilde_.resize(K_);
    log_pi_tilde_ = Eigen::VectorXd::Zero(K_);

    labels_.clear();

    setupPriors(points);
}

void VBGMM::setupPriors(const std::vector<geo::Vec3>& points) {
    // Priors for VB-GMM (can be adjusted based on domain knowledge)
    alpha0_ = 1.0;  // Symmetric Dirichlet prior

    Eigen::Vector3d means = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        means += Eigen::Vector3d(p.x, p.y, p.z);
    }
    means /= points.size();
    m0_ = means;  // Use mean of points as prior mean

    beta0_ = 0.1;  // Prior precision on mean (weak prior)

    // Prior precision matrix (inverse covariance)
    W0_ = Eigen::Matrix3d::Identity() * 10.0;  // Relatively uninformative
    nu0_ = D_ + 1;  // Minimum degrees of freedom for Wishart

    ROS_INFO("VB-GMM: Set up priors with alpha0=%.2f, beta0=%.2f, nu0=%.2f",
             alpha0_, beta0_, nu0_);
}

void VBGMM::initializeVariationalParameters(const Eigen::MatrixXd& data) {
    int N = data.rows();

    // Initialize responsibilities randomly but normalized
    resp_ = Eigen::MatrixXd::Random(N, K_).cwiseAbs();
    for (int i = 0; i < N; i++) {
        resp_.row(i) /= resp_.row(i).sum();
    }

    // Initialize posterior hyperparameters
    for (int k = 0; k < K_; k++) {
        // Effective number of points assigned to component k
        double Nk = resp_.col(k).sum();

        // Dirichlet posterior
        alpha_tilde_[k] = alpha0_ + Nk;

        // Gaussian-Wishart posterior
        beta_tilde_[k] = beta0_ + Nk;
        nu_tilde_[k] = nu0_ + Nk;

        // Compute sample mean for component k
        Eigen::Vector3d xk = Eigen::Vector3d::Zero();
        for (int i = 0; i < N; i++) {
            xk += resp_(i, k) * data.row(i).transpose();
        }
        xk /= (Nk + 1e-10);

        // Mean posterior
        m_tilde_[k] = (beta0_ * m0_ + Nk * xk) / beta_tilde_[k];

        // Compute sample covariance for component k
        Eigen::Matrix3d Sk = Eigen::Matrix3d::Zero();
        for (int i = 0; i < N; i++) {
            Eigen::Vector3d diff = data.row(i).transpose() - xk;
            Sk += resp_(i, k) * diff * diff.transpose();
        }

        // Wishart posterior scale matrix
        Eigen::Vector3d mean_diff = xk - m0_;
        Eigen::Matrix3d W_inv = W0_.inverse() + Sk +
                               (beta0_ * Nk / beta_tilde_[k]) * mean_diff * mean_diff.transpose();

        // Fix: Use eval() to avoid aliasing
        W_tilde_[k] = W_inv.inverse().eval();

        // Add regularization for numerical stability
        W_tilde_[k] += Eigen::Matrix3d::Identity() * 1e-6;
    }

    updateCachedValues();
}

double VBGMM::vbEStep(const Eigen::MatrixXd& data) {
    int N = data.rows();

    // Compute log responsibilities
    Eigen::MatrixXd log_resp(N, K_);

    for (int i = 0; i < N; i++) {
        for (int k = 0; k < K_; k++) {
            Eigen::Vector3d x = data.row(i).transpose();
            Eigen::Vector3d diff = x - m_tilde_[k];

            // Expected log likelihood under posterior
            double quad_form = diff.transpose() * (nu_tilde_[k] * W_tilde_[k]) * diff;
            double log_likelihood = 0.5 * (log_lambda_tilde_[k] - D_ * std::log(2 * M_PI) -
                                          nu_tilde_[k] * quad_form - D_ / beta_tilde_[k]);

            // Expected log mixing coefficient
            log_resp(i, k) = log_pi_tilde_[k] + log_likelihood;
        }
    }

    // Normalize responsibilities
    double total_log_likelihood = 0.0;
    for (int i = 0; i < N; i++) {
        double max_log_resp = log_resp.row(i).maxCoeff();
        Eigen::VectorXd exp_resp = (log_resp.row(i).array() - max_log_resp).exp();
        double sum_exp = exp_resp.sum();

        resp_.row(i) = exp_resp / sum_exp;
        total_log_likelihood += max_log_resp + std::log(sum_exp);
    }

    return total_log_likelihood;
}

void VBGMM::vbMStep(const Eigen::MatrixXd& data) {
    int N = data.rows();

    for (int k = 0; k < K_; k++) {
        // Effective number of points
        double Nk = resp_.col(k).sum();

        // Update Dirichlet posterior
        alpha_tilde_[k] = alpha0_ + Nk;

        // Compute weighted sample statistics
        Eigen::Vector3d xk = Eigen::Vector3d::Zero();
        for (int i = 0; i < N; i++) {
            xk += resp_(i, k) * data.row(i).transpose();
        }
        xk /= (Nk + 1e-10);

        Eigen::Matrix3d Sk = Eigen::Matrix3d::Zero();
        for (int i = 0; i < N; i++) {
            Eigen::Vector3d diff = data.row(i).transpose() - xk;
            Sk += resp_(i, k) * diff * diff.transpose();
        }

        // Update Gaussian-Wishart posterior
        beta_tilde_[k] = beta0_ + Nk;
        nu_tilde_[k] = nu0_ + Nk;

        m_tilde_[k] = (beta0_ * m0_ + Nk * xk) / beta_tilde_[k];

        Eigen::Vector3d mean_diff = xk - m0_;
        Eigen::Matrix3d W_inv = W0_.inverse() + Sk +
                               (beta0_ * Nk / beta_tilde_[k]) * mean_diff * mean_diff.transpose();

        // Fix: Use eval() to avoid aliasing and use temporary variable
        W_tilde_[k] = W_inv.inverse().eval();

        // Add regularization for numerical stability
        W_tilde_[k] += Eigen::Matrix3d::Identity() * 1e-6;
    }

    updateCachedValues();
}

void VBGMM::updateCachedValues() {
    // Cache frequently used values
    double alpha_sum = alpha_tilde_.sum();

    for (int k = 0; k < K_; k++) {
        // Expected log mixing weights
        log_pi_tilde_[k] = digamma(alpha_tilde_[k]) - digamma(alpha_sum);

        // Expected log determinant of precision
        log_det_W_tilde_[k] = 0.0;
        for (int i = 0; i < D_; i++) {
            log_det_W_tilde_[k] += digamma((nu_tilde_[k] + 1 - i - 1) / 2.0);
        }
        log_det_W_tilde_[k] += D_ * std::log(2.0) + std::log(W_tilde_[k].determinant());

        log_lambda_tilde_[k] = log_det_W_tilde_[k];
    }
}

double VBGMM::computeLowerBound(const Eigen::MatrixXd& data) {
    int N = data.rows();
    double bound = 0.0;

    // E[log p(X|Z,θ)] + E[log p(Z|π)]
    for (int i = 0; i < N; i++) {
        for (int k = 0; k < K_; k++) {
            if (resp_(i, k) > 1e-10) {
                Eigen::Vector3d x = data.row(i).transpose();
                Eigen::Vector3d diff = x - m_tilde_[k];

                double log_likelihood = 0.5 * (log_lambda_tilde_[k] - D_ * std::log(2 * M_PI) -
                                              nu_tilde_[k] * diff.transpose() * W_tilde_[k] * diff -
                                              D_ / beta_tilde_[k]);

                bound += resp_(i, k) * (log_pi_tilde_[k] + log_likelihood);
            }
        }
    }

    // E[log p(π)] - E[log q(π)]
    double alpha_sum = alpha_tilde_.sum();
    bound += boost::math::lgamma(K_ * alpha0_) - K_ * boost::math::lgamma(alpha0_);
    bound -= boost::math::lgamma(alpha_sum);
    for (int k = 0; k < K_; k++) {
        bound += (alpha0_ - alpha_tilde_[k]) * log_pi_tilde_[k];
        bound += boost::math::lgamma(alpha_tilde_[k]);
    }

    // Add other terms for complete lower bound...
    // (Gaussian-Wishart terms omitted for brevity but should be included)

    // -E[log q(Z)]
    for (int i = 0; i < N; i++) {
        for (int k = 0; k < K_; k++) {
            if (resp_(i, k) > 1e-10) {
                bound -= resp_(i, k) * std::log(resp_(i, k));
            }
        }
    }

    return bound;
}

void VBGMM::fit(const std::vector<geo::Vec3>& points, const geo::Pose3D& sensor_pose) {
    // Convert points to Eigen matrix
    int N = points.size();
    Eigen::MatrixXd data(N, 3);

    for (int i = 0; i < N; i++) {
        geo::Vec3 p_map = sensor_pose * points[i];
        data(i, 0) = p_map.x;
        data(i, 1) = p_map.y;
        data(i, 2) = p_map.z;
    }

    // Initialize variational parameters
    initializeVariationalParameters(data);

    // VB-EM iterations
    int max_iter = 100;
    double tol = 1e-4;
    double prev_bound = -1e10;

    for (int iter = 0; iter < max_iter; iter++) {
        // VB E-step
        vbEStep(data);

        // VB M-step
        vbMStep(data);

        // Compute lower bound
        lower_bound_ = computeLowerBound(data);

        ROS_INFO("VB Iteration %d: Lower bound = %.6f", iter, lower_bound_);

        // Check convergence
        double change = lower_bound_ - prev_bound;
        if (iter > 0 && change < tol) {
            ROS_INFO("VB-GMM converged after %d iterations", iter);
            break;
        }

        if (change < 0) {
            ROS_WARN("Lower bound decreased! Change = %.6f", change);
        }

        prev_bound = lower_bound_;
    }

    // Assign labels based on responsibilities
    labels_.resize(N);
    for (int i = 0; i < N; i++) {
        Eigen::VectorXd r = resp_.row(i);
        int max_idx = 0;
        r.maxCoeff(&max_idx);
        labels_[i] = max_idx;
    }

    determineInlierComponent();
}

void VBGMM::determineInlierComponent() {
    // Use posterior means for component selection
    std::vector<double> scores(K_, 0.0);

    for (int k = 0; k < K_; k++) {
        // Posterior mean and covariance
        Eigen::Vector3d mean = m_tilde_[k];
        Eigen::Matrix3d cov = W_tilde_[k].inverse() / nu_tilde_[k];  // Posterior covariance

        // Score based on height and compactness
        double height = mean[2];
        double volume = std::sqrt(cov.determinant());
        double compactness = 1.0 / (volume + 1e-10);

        scores[k] = 2.0 * height + compactness;

        ROS_INFO("VB Component %d: height=%.3f, compactness=%.3f, score=%.3f",
                k, height, compactness, scores[k]);
    }

    // Select component with highest score
    inlier_component_ = 0;
    for (int k = 1; k < K_; k++) {
        if (scores[k] > scores[inlier_component_]) {
            inlier_component_ = k;
        }
    }

    ROS_INFO("VB Selected inlier component: %d", inlier_component_);
}

// Getter methods
std::vector<int> VBGMM::get_labels() const {
    return labels_;
}

int VBGMM::get_inlier_component() const {
    return inlier_component_;
}

Eigen::MatrixXd VBGMM::get_responsibilities() const {
    return resp_;
}

double VBGMM::get_lower_bound() const {
    return lower_bound_;
}

std::vector<Eigen::Vector3d> VBGMM::get_posterior_means() const {
    return m_tilde_;
}

std::vector<Eigen::Matrix3d> VBGMM::get_posterior_covariances() const {
    std::vector<Eigen::Matrix3d> post_covs;
    for (int k = 0; k < K_; k++) {
        post_covs.push_back(W_tilde_[k].inverse() / nu_tilde_[k]);
    }
    return post_covs;
}

// Helper functions
double VBGMM::digamma(double x) const {
    return boost::math::digamma(x);
}

double VBGMM::logGammaMultivariate(double a, int p) const {
    double result = p * (p - 1) / 4.0 * std::log(M_PI);
    for (int i = 0; i < p; i++) {
        result += boost::math::lgamma(a - i / 2.0);
    }
    return result;
}