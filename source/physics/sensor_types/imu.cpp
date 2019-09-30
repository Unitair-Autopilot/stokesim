/** @file
Implementation of imu.h
*/
#include "stokesim/physics/sensor_types/imu.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {
namespace sensor_types {

//////////////////////////////////////////////////

IMU::IMU(YAML::Node const& config, Craft const* container) :
    Sensor(config, container),
    output_downsample_ratio(yget("output_downsample_ratio", config).as<int>()),
    integrate_sensors(yget("integrate_sensors", config).as<bool>()),
    use_integrated_units(yget("use_integrated_units", config).as<bool>()),
    gyro(yget("gyro", config), container),
    accel(yget("accel", config), container),
    dt_output(1.0/this->freq * output_downsample_ratio),
    dt_sample(1.0/this->freq) {
    // Check the consistency of frequency specifications
    if (gyro.freq != accel.freq || gyro.freq != this->freq) {
        throw std::runtime_error("IMU freq parameter must match that of its contained Accelerometer and Gyroscope.");
    }
    // Gyro measurement names
    for(auto const& measurement_name : gyro.measurement_names) {
        measurement_names.push_back("gyro_"+measurement_name);
    }
    // Accel measurement names
    for(auto const& measurement_name : accel.measurement_names) {
        measurement_names.push_back("accel_"+measurement_name);
    }
    // Initialize measurements dict
    for(auto const& name : measurement_names) {measurements[name];}
    // Gyro state names
    for(auto const& state_name : gyro.state_names) {
        state_names.push_back("gyro_"+state_name);
    }
    // Accel state names
    for(auto const& state_name : accel.state_names) {
        state_names.push_back("accel_"+state_name);
    }
    // Device logic state names
    state_names.push_back("output_dt");
    state_names.push_back("is_integrated");
    state_names.push_back("in_integrated_units");
    // Initialize state dict
    for(auto const& name : state_names) {states[name];}
    // Initialize the countdown counter for downsampled outputs
    output_counter = output_downsample_ratio;
}

//////////////////////////////////////////////////

void IMU::update(Cmd const& cmd) {
    gyro.update(cmd);
    accel.update(cmd);
    // Update kill status
    if(cmd.kill.given()) {killed = cmd.kill;}
    if(killed) {return;}
    // When output counter has counted down, update the IMU outputs and reset the counter
    if (update_sensor_buffers()) {output_counter--;}
    if (output_counter == 0) {
        update_measurements(cmd);
        update_states(cmd, 0.0);
        if(cmd.update_sequence) {
            timestamp = craft->world->clock.t;
            ++sequence_number;
        }
        output_counter = output_downsample_ratio;
    }
}

//////////////////////////////////////////////////

void IMU::update_states(Cmd const& cmd, Escal dt) {
    // Update gyro states
    for(auto const& state_name : gyro.state_names) {
        states.at("gyro_"+state_name) = gyro.get_state(state_name);
    }
    // Update accel states
    for(auto const& state_name : accel.state_names) {
        states.at("accel_"+state_name) = accel.get_state(state_name);
    }
    // Update device logic states
    states.at("output_dt")           = dt_output;
    states.at("is_integrated")       = integrate_sensors;
    states.at("in_integrated_units") = use_integrated_units;
}

//////////////////////////////////////////////////

void IMU::update_measurements(Cmd const& cmd) {
    measurements.at("accel_x") = processed.accel(0);
    measurements.at("accel_y") = processed.accel(1);
    measurements.at("accel_z") = processed.accel(2);
    if (!integrate_sensors) {
        measurements.at("gyro_x") = processed.gyro(0);
        measurements.at("gyro_y") = processed.gyro(1);
        measurements.at("gyro_z") = processed.gyro(2);
        if (use_integrated_units){
            for (auto const meas_name : measurement_names) {
                measurements.at(meas_name) *= dt_output;
            }
        }
    } else {
        processed.gyro = integration_states.alpha + integration_states.beta;
        measurements.at("gyro_x") = processed.gyro(0);
        measurements.at("gyro_y") = processed.gyro(1);
        measurements.at("gyro_z") = processed.gyro(2);
        if (!use_integrated_units){
            // Convert integrated units data back to equivalent angular velocity and acceleration by dividing by dt
            for (auto const meas_name : measurement_names) {
                measurements.at(meas_name) /= dt_output;
            }
        }
        reset_integration_states();
    }
}

//////////////////////////////////////////////////

bool IMU::update_sensor_buffers() {
    bool new_gyro  = false;
    bool new_accel = false;
    if (sensor_seqs.gyro != gyro.sequence_number) {
        new_gyro = true;
        raw_meas.gyro(0) = gyro.get_measurement("x");
        raw_meas.gyro(1) = gyro.get_measurement("y");
        raw_meas.gyro(2) = gyro.get_measurement("z");
        sensor_seqs.gyro = gyro.sequence_number;
    }
    if (sensor_seqs.accel != accel.sequence_number) {
        new_accel = true;
        raw_meas.accel(0) = accel.get_measurement("x");
        raw_meas.accel(1) = accel.get_measurement("y");
        raw_meas.accel(2) = accel.get_measurement("z");
        sensor_seqs.accel = accel.sequence_number;
    }
    if (new_gyro && new_accel) {
        if (integrate_sensors ) {
            // Implement integration of angular velocity per the algorithm outlined in equations (46) and (47) of
            // "Strapdown Inertial Navigation Integration Algorithm Design Part 1: Attitude Algorithms",
            // Paul G. Savage, Journal of Guidance, Control. and Dynamics, Vol. 21, No. 1, Jan-Feb 1998
            //
            // Compute delta angle with coning compensation
            integration_states.d_alpha = raw_meas.gyro * dt_sample; // Euler integrate into the alpha channel
            // Integrate the coning terms into beta
            integration_states.beta += (0.5 * (integration_states.alpha + (1.0 / 6.0) * integration_states.d_alpha_last)).cross(integration_states.d_alpha);
            integration_states.alpha += integration_states.d_alpha;
            integration_states.d_alpha_last = integration_states.d_alpha;

            // Integrate accelerometers with simple Euler integration
            processed.accel += raw_meas.accel * dt_sample;
        } else {
            // Just copy the measured gyro / accel vectors into the processed vectors
            processed.gyro  = raw_meas.gyro;
            processed.accel = raw_meas.accel;
        }
    }
    return new_gyro;
}

//////////////////////////////////////////////////

void IMU::reset_integration_states() {
    integration_states.alpha.setZero();
    integration_states.d_alpha.setZero();
    integration_states.d_alpha_last.setZero();
    integration_states.beta.setZero();
    processed.gyro.setZero();
    processed.accel.setZero();
}

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
