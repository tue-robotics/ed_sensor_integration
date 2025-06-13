#include "ed/kinect/bayesian_gmm.h"


MAPGMM::MAPGMM(int n_components) : K_(n_components), inlier_component_(0) {
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
        kappa0_[k] = 1.0;
        nu0_[k] = 4.0;  // Minimum for 3D
    }

    // Set up actual prior values
    setupPriors();
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

    // Initialize parameters
    weights_ = Eigen::VectorXd::Constant(K_, 1.0/K_);

    means_.resize(K_);
    covs_.resize(K_);

    // Simple initialization: random points as centers
    for (int k = 0; k < K_; k++) {
        int idx = rand() % N;
        means_[k] = data.row(idx).transpose();
        covs_[k] = Eigen::Matrix3d::Identity() * 0.01;  // Small initial cov
    }

    // Set up priors
    //setupPriors();

    // EM iterations
    int max_iter = 100;
    double tol = 1e-4;
    double prev_log_likelihood = -1e10;

    Eigen::MatrixXd resp(N, K_);

    for (int iter = 0; iter < max_iter; iter++) {
        // E-step: Calculate responsibilities
        double log_likelihood = eStep(data, resp);

        // Check convergence
        double change = std::abs((log_likelihood - prev_log_likelihood) / (std::abs(prev_log_likelihood) + 1e-10));
        if (iter > 0 && change < tol) {
            ROS_INFO("MAP-GMM converged after %d iterations", iter);
            break;
        }
        prev_log_likelihood = log_likelihood;

        // M-step: Update parameters with MAP
        mStep(data, resp);
    }

    // Assign labels based on highest responsibility
    labels_.resize(N);
    for (int i = 0; i < N; i++) {
        Eigen::VectorXd r = resp.row(i);
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



void MAPGMM::setupPriors() {
    if (mu0_.size() != K_ || kappa0_.size() != K_ ||
        Psi0_.size() != K_ || nu0_.size() != K_) {
        ROS_ERROR("Prior vectors not properly initialized in MAPGMM constructor");
        return;
    }
    // Dirichlet prior for weights (alpha>1 favors more uniform weights)
    alpha_ = 1.5;

    for (int k = 0; k < K_; k++) {
        // Same prior for all components initially
        mu0_[k] = Eigen::Vector3d(0.0, 0.0, 0.03);  // Neutral height
        kappa0_[k] = 0.1;  // Weak prior

        // Same covariance prior for all components
        Eigen::Matrix3d psi = Eigen::Matrix3d::Identity() * 0.005;
        Psi0_[k] = psi;
        nu0_[k] = 4;  // Minimum value for 3D (d+1)
    }
}

double MAPGMM::eStep(const Eigen::MatrixXd& data, Eigen::MatrixXd& resp) {
    int N = data.rows();
    double log_likelihood = 0.0;

    // Calculate log probabilities for each point and component
    Eigen::MatrixXd log_probs(N, K_);

    for (int k = 0; k < K_; k++) {
        // Compute multivariate normal density for all points
        for (int i = 0; i < N; i++) {
            Eigen::Vector3d x = data.row(i).transpose();
            Eigen::Vector3d diff = x - means_[k];

            double log_prob = -0.5 * diff.transpose() * covs_[k].inverse() * diff
                            - 0.5 * std::log(covs_[k].determinant())
                            - 1.5 * std::log(2 * M_PI);

            log_probs(i, k) = std::log(weights_[k]) + log_prob;
        }
    }

    // Calculate responsibilities (and normalize)
    for (int i = 0; i < N; i++) {
        // Numerical stability: subtract max value
        double max_log_prob = log_probs.row(i).maxCoeff();
        Eigen::VectorXd exp_probs = (log_probs.row(i).array() - max_log_prob).exp();
        double sum_exp = exp_probs.sum();

        // Set responsibilities
        resp.row(i) = exp_probs / sum_exp;

        // Add to log likelihood
        log_likelihood += max_log_prob + std::log(sum_exp);
    }

    return log_likelihood;
}

void MAPGMM::mStep(const Eigen::MatrixXd& data, const Eigen::MatrixXd& resp) {
    int N = data.rows();

    // For each component
    for (int k = 0; k < K_; k++) {
        // Component responsibility sum
        double Nk = resp.col(k).sum();

        // MAP update for weight (Dirichlet prior)
        weights_[k] = (Nk + alpha_ - 1.0) / (N + K_ * alpha_ - K_);

        // Calculate weighted mean of data
        Eigen::Vector3d mean_data = Eigen::Vector3d::Zero();
        for (int i = 0; i < N; i++) {
            mean_data += resp(i, k) * data.row(i).transpose();
        }
        mean_data /= Nk;

        // MAP update for mean (Normal-Wishart prior)
        means_[k] = (Nk * mean_data + kappa0_[k] * mu0_[k]) / (Nk + kappa0_[k]);

        // Calculate weighted covariance of data
        Eigen::Matrix3d cov_data = Eigen::Matrix3d::Zero();
        for (int i = 0; i < N; i++) {
            Eigen::Vector3d diff = data.row(i).transpose() - mean_data;
            cov_data += resp(i, k) * diff * diff.transpose();
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
    // Use covariance properties to identify inlier component
    std::vector<double> scores(K_);

    for (int k = 0; k < K_; k++) {
        // Eigendecomposition of covariance
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covs_[k]);
        Eigen::Vector3d eigenvalues = eig.eigenvalues();

        // Sort eigenvalues
        std::vector<double> evals = {eigenvalues(0), eigenvalues(1), eigenvalues(2)};
        std::sort(evals.begin(), evals.end());

        // Elongation (real objects have lower elongation)
        double elongation = evals[2] / (evals[1] + 1e-6);

        // XY:Z ratio (artifacts spread more in XY than Z)
        double xy_to_z_ratio = std::sqrt(evals[1] * evals[2]) / (evals[0] + 1e-6);

        // Height (real objects are higher)
        double height = means_[k][2];

        // Combined score
        scores[k] = -elongation + 2.0 * height - std::abs(std::log(xy_to_z_ratio));
    }

    // Select component with highest score
    inlier_component_ = 0;
    if (K_ > 1 && scores[1] > scores[0]) {
        inlier_component_ = 1;
    }
}