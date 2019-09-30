/** @file
Declaration of the IMU class.
*/
#pragma once
#include "stokesim/physics/sensor.h"
#include "stokesim/physics/sensor_types/gyroscope.h"
#include "stokesim/physics/sensor_types/accelerometer.h"

namespace stokesim {
namespace physics {
class Craft;
namespace sensor_types {

//////////////////////////////////////////////////

/// IMU containing three-axis gyroscope and three-axis accelerometer
/// subject to white and Ornstein noise, and thermal variation.
class IMU : public Sensor {
public:
    /// @name Provided
    //@{
    int  const output_downsample_ratio; ///< Number of update steps on the underlying sensors per IMU output
    bool const integrate_sensors; ///< Integrate the gyros and accels between outputs
    bool const use_integrated_units; ///< Output in integrated units (delta-angles / delta-velocities) instead of (angular velocity / acceleration)
    //@}

    /// @name Derived
    //@{
    Gyroscope gyro; ///< Gyroscope model
    Accelerometer accel; ///< Accelerometer model
    Escal dt_output; ///< The nominal IMU output sample period (a positive integer ratio of the sample times of the gyro and accel models)
    Escal dt_sample; ///< Fast sampling period of gyro / accel
    //@}

private:
    /// @name Core
    //@{
    friend class stokesim::physics::Craft;
    IMU(YAML::Node const& config, Craft const* container);
    void update(Cmd const& cmd) override; ///< Need override of this method to handle the containerization of the gyro and accel models
    void update_states(Cmd const& cmd, Escal dt) override;
    void update_measurements(Cmd const& cmd) override;
    bool update_sensor_buffers(); ///< Buffer data from the gyro and accel sensors and integrate if specified
    void reset_integration_states(); ///< Reset integration states

    int output_counter;

    struct {
        uint32_t gyro;
        uint32_t accel;
    } sensor_seqs;

    struct {
        Evec3 gyro;  ///< Raw gyro measurement (rad/s)
        Evec3 accel; ///< Raw accel measurement (m/s/s)
    } raw_meas;

    struct {
        Evec3 gyro;  ///< Processed gyro measurement (holds integrated quantity delta-angles if integration enabled)
        Evec3 accel; ///< Processed accel measurement (holds integrated quantity delta-velocities if integration enabled)
    } processed;

    struct {
        Evec3 alpha;
        Evec3 d_alpha;
        Evec3 d_alpha_last;
        Evec3 beta;
    } integration_states;

    //@}

    NO_COPY(IMU)
};

//////////////////////////////////////////////////

} // namespace sensor_types
} // namespace physics
} // namespace stokesim
