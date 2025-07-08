#include "ed/kinect/bayesian_gmm.h"


MAPGMM::MAPGMM(int n_components, const std::vector<geo::Vec3>& points, const GMMParams & params) : K_(n_components), inlier_component_(0), params_(params) {
    // Initialize all containers to proper sizes
    means_.resize(K_);
    covs_.resize(K_);
    weights_ = Eigen::VectorXd::Zero(K_);
    labels_.clear();  // Will be resized during fit()

    // Initialize prior parameter containers
    mu0_.resize(K_);
    kappa0_.resize(K_);
    Psi0_.resize(K_);
    nu0_.resize(K_);

    // Initialize with identity matrices to avoid uninitialized memory
    for (int k = 0; k < K_; k++) {
        means_[k] = Eigen::Vector3d::Zero();
        covs_[k] = Eigen::Matrix3d::Identity();
        mu0_[k] = Eigen::Vector3d::Zero();
        Psi0_[k] = Eigen::Matrix3d::Identity();
    }

    // Set up actual prior values
    setupPriors(points);
}

void MAPGMM::fit(const std::vector<geo::Vec3>& points, const geo::Pose3D& sensor_pose) {
    // Convert points to Eigen matrix
    int N = points.size();
    Eigen::MatrixXd data(N, 3);

    for (int i = 0; i < N; i++) {
        geo::Vec3 p_map = sensor_pose * points[i];
        data(i, 0) = p_map.x;
        data(i, 1) = p_map.y;
        data(i, 2) = p_map.z;
    }
    // Compute bounding volume for the data
    computeBoundingVolume(data);

    // Initialize parameters
    weights_ = Eigen::VectorXd::Constant(K_, 1.0/K_);

    means_.resize(K_);
    covs_.resize(K_);

    // Simple initialization: random points as centers
    for (int k = 0; k < K_; k++) {
        int idx = rand() % N;
        means_[k] = data.row(idx).transpose();
        covs_[k] = Eigen::Matrix3d::Identity() * 0.01;  // Small initial cov NEED TO PUT THIS ON THE PARAMETERS AS WELL
    }


    // EM iterations
    int max_iter = 100;
    double tol = 1e-4;
    double prev_log_likelihood = -1e10;


    resp_ = Eigen::MatrixXd::Zero(N, K_);

    for (int iter = 0; iter < max_iter; iter++) {
        // E-step: Calculate responsibilities
        double log_likelihood = eStep(data, resp_);

        // Check convergence
        double change = std::abs((log_likelihood - prev_log_likelihood) / (std::abs(prev_log_likelihood) + 1e-10));
        if (iter > 0 && change < tol) {
            ROS_INFO("MAP-GMM converged after %d iterations", iter);
            break;
        }
        prev_log_likelihood = log_likelihood;

        // M-step: Update parameters with MAP
        mStep(data, resp_);
    }

    // Assign labels based on highest responsibility
    labels_.resize(N);
    for (int i = 0; i < N; i++) {
        Eigen::VectorXd r = resp_.row(i);
        int max_idx = 0;
        r.maxCoeff(&max_idx);
        labels_[i] = max_idx;
    }

    // Determine which component is the inlier
    determineInlierComponent();
}

std::vector<int> MAPGMM::get_labels() const {
    return labels_;
}
int MAPGMM::get_inlier_component() const {
    return inlier_component_;
}



void MAPGMM::setupPriors(const std::vector<geo::Vec3>& points) {
    if (mu0_.size() != K_ || kappa0_.size() != K_ ||
        Psi0_.size() != K_ || nu0_.size() != K_) {
        ROS_ERROR("Prior vectors not properly initialized in MAPGMM constructor");
        return;
    }
    // Dirichlet prior for weights (alpha>1 favors more uniform weights)
    alpha_ = params_.alpha;
    Eigen::Vector3d means = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        means += Eigen::Vector3d(p.x, p.y, p.z);
    }
    means /= points.size();
    for (int k = 0; k < K_; k++) {
        // Same prior for all components initially
        mu0_[k] = means;

        kappa0_[k] = params_.kappa0;  // Weak prior

        // Same covariance prior for all components
        Eigen::Matrix3d psi = Eigen::Matrix3d::Identity() * params_.psi0;
        Psi0_[k] = psi;
        nu0_[k] = params_.nu0;  // Minimum value for 3D (d+1)
    }
}

void MAPGMM::computeBoundingVolume(const Eigen::MatrixXd& data) {
    Eigen::Vector3d min_vals = data.colwise().minCoeff();
    Eigen::Vector3d max_vals = data.colwise().maxCoeff();
    volume_ = (max_vals - min_vals).prod();  // volume = (xmax - xmin) * (ymax - ymin) * (zmax - zmin)
}

double MAPGMM::eStep(const Eigen::MatrixXd& data, Eigen::MatrixXd& resp_) {
    int N = data.rows();
    double log_likelihood = 0.0;

    // Calculate log probabilities for each point and component
    Eigen::MatrixXd log_probs(N, K_);

    for (int k = 0; k < K_; k++) {
        // Compute multivariate normal density for all points
        for (int i = 0; i < N; i++) {
            Eigen::Vector3d x = data.row(i).transpose();
            if (k == 0) {
                // For the outlier component, use a uniform distribution over the bounding volume
                double uniform_prob = 1.0 / volume_;
                log_probs(i, k) = std::log(weights_[k]) + std::log(uniform_prob);
                continue;
            }
            else{
            Eigen::Vector3d diff = x - means_[k];

            double log_prob = -0.5 * diff.transpose() * covs_[k].inverse() * diff
                            - 0.5 * std::log(covs_[k].determinant())
                            - 1.5 * std::log(2 * M_PI);

            log_probs(i, k) = std::log(weights_[k]) + log_prob;
            }
        }
    }

    // Calculate responsibilities (and normalize)
    for (int i = 0; i < N; i++) {
        // Numerical stability: subtract max value
        double max_log_prob = log_probs.row(i).maxCoeff();
        Eigen::VectorXd exp_probs = (log_probs.row(i).array() - max_log_prob).exp();
        double sum_exp = exp_probs.sum();

        // Set responsibilities (p_z=k|x)
        resp_.row(i) = exp_probs / sum_exp;

        // Add to log likelihood
        log_likelihood += max_log_prob + std::log(sum_exp);
    }

    return log_likelihood;
}

void MAPGMM::mStep(const Eigen::MatrixXd& data, const Eigen::MatrixXd& resp_) {
    int N = data.rows();

    // For each component
    for (int k = 0; k < K_; k++) {
        // Component responsibility sum
        double Nk = resp_.col(k).sum();

        // MAP update for weight (Dirichlet prior)
        weights_[k] = (Nk + alpha_ - 1.0) / (N + K_ * alpha_ - K_);

        if (k == 0) continue;  // Skip outlier component for weight update since uniform distribution does not depend on data

        // Calculate weighted mean of data
        Eigen::Vector3d mean_data = Eigen::Vector3d::Zero();
        for (int i = 0; i < N; i++) {
            mean_data += resp_(i, k) * data.row(i).transpose();
        }
        mean_data /= Nk;

        // MAP update for mean (Normal-Wishart prior)
        //kappa0_[k] += Nk;  // Update kappa prior
        means_[k] = (Nk * mean_data + kappa0_[k] * mu0_[k]) / (Nk + kappa0_[k]);

        // Calculate weighted covariance of data
        Eigen::Matrix3d cov_data = Eigen::Matrix3d::Zero();
        for (int i = 0; i < N; i++) {
            Eigen::Vector3d diff = data.row(i).transpose() - mean_data;
            cov_data += resp_(i, k) * diff * diff.transpose();
        }
        cov_data /= Nk;

        // Additional term from the mean update
        Eigen::Vector3d mean_diff = mean_data - mu0_[k];
        Eigen::Matrix3d mean_cov = (kappa0_[k] * Nk / (kappa0_[k] + Nk)) *
                                    mean_diff * mean_diff.transpose();

        // MAP update for covariance (Inverse-Wishart prior)
        covs_[k] = (Psi0_[k] + Nk * cov_data + mean_cov) / (Nk + nu0_[k] + 3 + 1);

        // Add small regularization to ensure positive definiteness
        covs_[k] += Eigen::Matrix3d::Identity() * 1e-6;
    }

    // Normalize weights
    weights_ /= weights_.sum();
}

void MAPGMM::determineInlierComponent() {
     // Compute N_k: total responsibility mass for each component
    std::vector<double> Nk(K_, 0.0);
    int N = labels_.size();  // or you can use resp.rows()

    for (int i = 0; i < N; i++) {
        for (int k = 0; k < K_; k++) {
            Nk[k] += resp_(i, k);  // You'll need to save `resp_` as a class member
        }
    }

    // Find the component with max Nk
    // Skip the outlier uniform distribution component (k=0)
    inlier_component_ = 1;
    for (int k = 1; k < K_; k++) {
        if (Nk[k] > Nk[inlier_component_]) {
            inlier_component_ = k;
        }
    }

    ROS_INFO("Bayesian selection: inlier component = %d (Nk = %.2f)", inlier_component_, Nk[inlier_component_]);

}