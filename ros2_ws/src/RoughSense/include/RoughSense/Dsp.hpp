/**
 * @file Dsp.hpp
 * @author XX.XX@uni.XX
 * @version 0.1
 * @date 2025-01-26
 * 
 * @copyright XX XX | 2024
 * @brief Implementation file for Class BandStopFilter.
 * @details This is a C++ band stop filter used to filter the resonnance frequencies of the rover.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip> // For better output formatting
#include <algorithm> // For min/max element
#include <deque>
#include <numeric>
#include <complex>
#include <Eigen/Dense> // Eigen for matrix operations


// Liquid-DSP
#include <liquid/liquid.h>

// Used for IDW
struct Cell {
    double local_x, local_y, global_x , global_y, pred_raw, pred_norm, observed_raw, observed_norm, theta_a, theta_b, roughness_corrected, delta_idw, roughness_final ;
};


using namespace std;


class Dsp {
    public:
        Dsp(){};
        ~Dsp()
        {
            if (_filter)
                iirfilt_crcf_destroy(_filter);
        };

        // ===========================
        // Attributes
        // ===========================
        // The sinusoid attribute is used for simulating IMU data (debug)
        vector<double> sinusoid;
        
        // RLS
        // Tunables
        static constexpr double kLambda          = 0.995;   // forgetting factor (0.95..1.0)
        static constexpr double kDenomFloor      = 1e-12;   // avoids tiny denominators
        static constexpr double kInflationOnBad  = 1e-6;    // PD rescue
        static constexpr double kResidualGateAbs = 1e6;     // simple |residual| gate (tune)
        static constexpr double kThetaClip       = 1e6;     // parameter clipping
        double lambda;          // Forgetting factor        
        Eigen::VectorXd theta;  // Model parameters [theta_0, theta_1]
        Eigen::MatrixXd P;      // Covariance matrix

        //IDW
        double idw_power = 2.0;


        // Liquid-DSP filter object (Celler type defined by Liquid-DSP)
        iirfilt_crcf _filter {nullptr};

        // ===========================
        // Methods
        // ===========================
        void create_filter(float f_center_p, float bandwidth_p, float fs_p);
        void process_sample(complex<float> input, complex<float> *output);
        void generate_simulated_signal();


        // RLS
        void initialize_rls();
        bool rls_update(double predicted_roughness, double roughness_observed);
        double rls_correction(double predicted_roughness);

        // IDW
        double idw_interpolation(const vector<Cell>& observed_cells, double x, double y, double predicted_roughness); 
        double distance(const Cell& a, const Cell& b);



    protected:
        // ===========================
        // Attributes
        // ===========================

        // Filter design parameters
        unsigned int order = 4;   // Filter order
        float Ap = 1.0f;          // Passband ripple in dB
        float As = 40.0f;         // Stopband attenuation in dB


        
};
