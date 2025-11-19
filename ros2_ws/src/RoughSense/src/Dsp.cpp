/**
 * @file Dsp.cpp
 * @author XX.XX@uni.XX
 * @version 0.1
 * @date 2025-01-26
 * 
 * @copyright XX XX | 2024
 * @brief Implementation file for Class BandStopFilter.
 * @details This is a C++ band stop filter used to filter the resonnance frequencies of the rover.
 */

#include "Dsp.hpp"



// =====================================================
// FILTER
// =====================================================

void Dsp::create_filter(float f_center_p, float bandwidth_p, float fs_p)
{
    // --- Define filter design parameters ---
    // For the prototype design, you choose:
    //   ftype: the analog prototype (e.g., Butterworth)
    //   btype: the filter band type (e.g., band-stop for a notch filter)
    //   format: coefficient format (typically second-order sections (SOS))
    //   order: filter order (try 4 or 6 to start)
    //   fc: cutoff frequency (normalized: 0 < fc < 0.5; 0.5 corresponds to Nyquist frequency)
    //   f0: center frequency of the notch (normalized)
    //   Ap: passband ripple in dB (typically a small value, e.g., 1 dB)

    // =====================================================
    // User-specified parameters (in Hz)
    // =====================================================
    // float f_center  = 27.5f;   // Desired center frequency (notch center)
    // float bandwidth = 5.0f;    // Desired bandwidth of the notch
    // float fs        = 400.0f;  // Sampling frequency (2 times the signal's frequency)

    float f_center = f_center_p;    // Desired center frequency (notch center)
    float bandwidth = bandwidth_p;  // Desired bandwidth of the notch
    float fs = fs_p*2;            // Sampling frequency (*2 because of the Nyquist frequency)

    // =====================================================
    // Compute the lower and upper edges of the notch
    // =====================================================
    float f_low  = f_center - (bandwidth / 2.0f);
    float f_high = f_center + (bandwidth / 2.0f);


    // =====================================================
    // Compute the Nyquist frequency
    // =====================================================
    // Nyquist Criterion:
    // For a signal with a maximum frequency of 200 Hz, you should sample at least at 400 Hz to capture all the frequency information without ambiguity. 
    // Sampling at only 100 Hz is far below this requirement.
    float nyquist = fs / 2.0f;


    // =====================================================
    // Validate frequency ranges
    // =====================================================
    if (f_low <= 0.0f || f_high >= nyquist) {
        cerr << "Error: Frequency parameters out of range.\n"
                  << "f_low = " << f_low << " Hz, f_high = " << f_high 
                  << " Hz, Nyquist = " << nyquist << " Hz\n";
        return;
    }

    // =====================================================
    // Compute normalized frequencies (0 to 1; 1 corresponds to Nyquist)
    // =====================================================
    float fc_norm = f_low / nyquist;        // Normalized cutoff (lower edge) frequency
    float f0_norm = f_center / nyquist;     // Normalized center frequency

    cout << "Normalized lower edge (fc_norm): " << fc_norm << "\n";
    cout << "Normalized center frequency (f0_norm): " << f0_norm << "\n";


    // Create the band-stop (notch) filter.
    // The prototype design interface will compute the necessary second-order sections.
    _filter = iirfilt_crcf_create_prototype(
      LIQUID_IIRDES_BUTTER,                 // Analog prototype type (Butterworth)
      LIQUID_IIRDES_BANDSTOP,               // Filter type: band-stop (notch)
      LIQUID_IIRDES_SOS,                    // Design format: use second-order sections
      order,                                // Filter's order
      fc_norm,                              // Cutoff frequency
      f0_norm,                              // Notch (center) frequency
      Ap,                                   // Passband ripple in dB
      As                                    // Stopband attenuation in dB
    );

    if (!_filter) {
      cerr <<"Failed to create IIR filter!"<< endl;
      return;
    }
};


void Dsp::process_sample(complex<float> input, complex<float> *output)
{
    // Execute one filtering iteration.
    iirfilt_crcf_execute(_filter, input, output);
};

// =====================================================
// TOOLS
// =====================================================

// Generate a fake sinusoid signal
void Dsp::generate_simulated_signal()
{
    // Parameters for the sinusoid.
    double amplitude = 1.0;       // Amplitude of the sinusoid.
    double frequency = 10.0;      // Frequency in Hz.
    double fs = 1000.0;           // Sampling frequency in Hz.
    double duration = 1.0;        // Duration in seconds.

    // Frequencies (in Hz) and their amplitudes for the sinusoids.
    double f1 = 10.0, f2 = 26.0, f3 = 35.0, f4 = 30.0, f5 = 20.0;
    double a1 = 1.0, a2 = 1.0, a3 = 1.0, a4 = 1.0, a5 = 1.0;

    // Calculate the total number of samples.
    int N = static_cast<int>(fs * duration);

    // Create a vector to store the samples.
    sinusoid.reserve(N);

    // Generate the sinusoid.
    for (int n = 0; n < N; ++n) {
        double t = n / fs; // Current time in seconds.
        double sample = a1 * sin(2 * M_PI * f1 * t) +
                        a2 * sin(2 * M_PI * f2 * t) +
                        a3 * sin(2 * M_PI * f3 * t) +
                        a4 * sin(2 * M_PI * f4 * t) +
                        a5 * sin(2 * M_PI * f5 * t)
                        ;
        sinusoid.push_back(sample);
    }
};


// Recursive Least Square algorithm
void Dsp::initialize_rls()
{
    // Initialize theta (two parameters: bias and slope)
    theta = Eigen::VectorXd(2);
    theta << 0.0, 1.0; // Start with identity mapping

    // Initialize covariance matrix with high values
    P = Eigen::MatrixXd::Identity(2, 2) * 1e3;
};








// Update RLS with a new observation (point cloud roughness -> IMU roughness)
bool Dsp::rls_update(double predicted_roughness, double roughness_observed) {
    // 0) Input validity
    if (!std::isfinite(predicted_roughness) || !std::isfinite(roughness_observed)) {
        // still age covariance so the filter "moves on"
        P /= kLambda;
        return false;
    }

    // 1) Build regressor
    Eigen::Vector2d phi;
    phi << predicted_roughness, 1.0;

    // 2) Residual (based on current theta)
    double yhat = phi.dot(theta);
    double e    = roughness_observed - yhat;
    if (!std::isfinite(e) || std::abs(e) > kResidualGateAbs) {
        // Outlier -> skip the update but keep forgetting
        P /= kLambda;
        return false;
    }

    // 3) Denominator with floor
    double denom = kLambda + (phi.transpose() * P * phi)(0,0);
    if (!std::isfinite(denom) || denom < kDenomFloor) {
        // Rescue: slightly inflate P and skip this update
        P += kInflationOnBad * Eigen::Matrix2d::Identity();
        P /= kLambda;
        return false;
    }

    // 4) Gain
    Eigen::Vector2d K = (P * phi) / denom;

    // 5) Parameter update with optional clipping
    theta.noalias() += K * e;
    for (int i = 0; i < theta.size(); ++i)
        theta[i] = std::clamp(theta[i], -kThetaClip, kThetaClip);

    // 6) Covariance update (RLS “Joseph-like”, re-symmetrize)
    //    P = (P - K*phiᵀ*P)/lambda
    P.noalias() -= K * (phi.transpose() * P);
    P = 0.5 * (P + P.transpose());  // enforce symmetry
    P /= kLambda;

    // 7) Final sanity
    if (!P.allFinite() || !theta.allFinite()) {
        // Soft reset (keep theta if finite, otherwise reset)
        if (!theta.allFinite()) theta.setZero();
        P = 1e3 * Eigen::Matrix2d::Identity();
        return false;
    }

    // std::cout << "θ: " << theta.transpose() << "\n";
    return true;
}

double Dsp::rls_correction(double predicted_roughness) 
{
    if (!std::isfinite(predicted_roughness) || !theta.allFinite())
        return std::numeric_limits<double>::quiet_NaN();
    return theta(0) * predicted_roughness + theta(1); // alpha*Rpc + beta
}




// // Update RLS with a new observation (point cloud roughness -> IMU roughness)
// void Dsp::rls_update(double predicted_roughness, double roughness_observed) {
//     Eigen::VectorXd phi(2);
//     phi << predicted_roughness, 1; // [point cloud roughness, Bias term]

//     // Compute Kalman gain
//     Eigen::MatrixXd K = P * phi / (lambda + phi.transpose() * P * phi);

//     // Update parameters
//     theta = theta + K * (roughness_observed - phi.transpose() * theta);

//     // Update covariance matrix
//     P = (P - K * phi.transpose() * P) / lambda;

//     std::cout << "Updated Model: theta_0 = " << theta(0) << ", theta_1 = " << theta(1) << std::endl;
// };


// double Dsp::rls_correction(double predicted_roughness)
// {
//     return theta(0) * predicted_roughness + theta(1) ; // alpha * Rpc + beta
// };



// IDW Interpolation function
double Dsp::idw_interpolation(const vector<Cell>& observed_cells, double x, double y, double predicted_roughness) 
{
    double numerator = 0.0;
    double denominator = 0.0;
    double eps = 1e-9; // Small value to avoid division by zero

    for (const auto& cell : observed_cells) {
        double dist = distance(cell, {x, y, 0.0});
        
        // If the query point coincides with a known point, return its value
        if (dist < eps) {
            return cell.roughness_corrected;
        }

        // Compute the correction for this cell
        double delta = cell.roughness_corrected - cell.pred_norm;

        double weight = 1.0 / pow(dist, idw_power);
        numerator += weight * delta;
        denominator += weight;
    }

    // If denominator is zero (should not occur if there are no observations), return the predicted value.
    if (denominator == 0.0) {
        return predicted_roughness;
    }

    // Interpolated correction for cell (x, y)
    double interpolatedCorrection = numerator / denominator;

    // Updated roughness is the predicted roughness plus the interpolated correction.
    return predicted_roughness + interpolatedCorrection;
};


// Function to compute Euclidean distance
double Dsp::distance(const Cell& a, const Cell& b) {
    return sqrt((a.local_x - b.local_x) * (a.local_x - b.local_x) + (a.local_y - b.local_y) * (a.local_y - b.local_y));
};
