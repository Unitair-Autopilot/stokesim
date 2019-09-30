/** @file
This example plugin establishes a ROS node that publishes a specified
craft's state, actuator wrenches, and sensor measurements, and subscribes
to actuator effort commands in a specified order.
*/
#include <signal.h>
#include <mutex>
#include <ros/ros.h>
#include <ros/master.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "stokesim/io/plugin.h"

namespace stokesros {

//////////////////////////////////////////////////

/// Establishes a ROS node for communicating with the simulator.
class StokesROS : public stokesim::io::Plugin {
private:
    /// @name Provided
    //@{
    std::string craft_name; ///< Name of the craft of interest
    std::vector<std::string> command_order; ///< Sequence in which actuator command information is communicated
    std::string radio_name; ///< (Optional) Name of its radio sensor
    //@}

    /// @name Derived
    //@{
    bool connected_to_ros = false; ///< Whether or not this plugin has connected to a ROS master
    tf::TransformBroadcaster* tfbr = nullptr; ///< For publishing TF coordinate-system information
    ros::Publisher state_pub; ///< Publisher for body state information
    ros::Publisher euler_pub; ///< Publisher for the (redundant) message with the body attitude information expressed as euler angles
    ros::Publisher lla_pub; ///< Publisher for the true geodetic coordinates of the body
    ros::Publisher wind_pub; ///< Publisher for the ENU wind velocity at the body's location
    stokesim::Dict<ros::Publisher> actuator_pubs; ///< All the publishers for each actuator wrench
    stokesim::Dict<ros::Publisher> sensor_pubs; ///< All the publishers for each sensor measurement
    stokesim::Dict<ros::Publisher> sensor_state_pubs; ///< All the publishers for each sensor state
    stokesim::Dict<uint32_t> sensor_seqs; ///< Last sequence number seen for each sensor
    ros::Subscriber command_sub; ///< Subscriber for actuator effort command information
    ros::Subscriber kill_sub; ///< Subscriber for kill-toggling strings, e.g. "some_actuator", "some_sensor", or "craft"
    ros::Subscriber rc_channels_sub; ///< Subscriber for RC simulator
    stokesim::physics::Craft const* pcraft = nullptr; ///< Pointer to the current globally relevant immutable stokesim Craft object
    Output output; ///< Current output to give to the simulator based on latest command message received
    std::mutex output_mutex; ///< For asynchronous access to output
    //@}

public:
    /// Implements Plugin::~Plugin
    ~StokesROS() override {
        // Release any manual memory allocations
        delete tfbr;
    }

    /// Implements Plugin::configure
    bool configure(YAML::Node const& config, stokesim::physics::World const& world) override {
        // Read plugin-specific data from the config YAML with proper error handling
        craft_name = stokesim::yget("craft_name", config).as<std::string>();
        command_order = stokesim::yget("command_order", config).as<std::vector<std::string>>();
        if(config["radio_name"]) {
            radio_name = stokesim::yget("radio_name", config).as<std::string>();
        } else {
            radio_name = "rc_radio";
        }
        // Verify that the world actually has the specified craft and then get it
        if(!world.has_craft(craft_name)) {
            std::cerr << "World is missing craft with name '" + craft_name + "'." << std::endl;
            return false;
        }
        pcraft = &(world.get_craft(craft_name));
        // Iterate over each specified actuator name
        for(auto const& name : command_order) {
            // Verify that the craft actually has said actuator
            if(!pcraft->has_actuator(name)) {
                std::cerr << "Craft '" + craft_name + "' is missing actuator '" + name + "'." << std::endl;
                return false;
            }
        }
        // Initialize this ROS node (only NodeHandles need a ROS master)
        int dummy_argc = 0;
        char** dummy_argv = nullptr;
        ros::init(dummy_argc, dummy_argv, "stokesim");
        if(ros::master::check()) {connect_to_ros();}
        else {stokesim::print("(warning: no ROS master found, will retry)");}
        return true;
    }

    /// Implements Plugin::work
    Output work(stokesim::physics::World const& world) override {
        // Point at craft of interest
        pcraft = &(world.get_craft(craft_name));
        // Handle ROS connection
        if(!connected_to_ros) {
            if(ros::master::check()) {connect_to_ros();}
            if(connected_to_ros) {stokesim::print("(stokesros info: '"+craft_name+"' successfully connected to ROS master)");}
            std::lock_guard<std::mutex> guard(output_mutex);
            return output;
        }
        ros::Time ros_time = ros::Time::now();
        // Handle ROS messaging
        ros::spinOnce(); // subscriber callbacks
        publish_state_info(ros_time);
        publish_actuator_info(ros_time);
        publish_sensor_info(ros_time);
        publish_wind_info(ros_time, world.env.geoid.ecef_enu * world.env.atmos.wind_at_lla(pcraft->body.lla));

        // Terminate sim if ROS died, else just provide current output
        std::lock_guard<std::mutex> guard(output_mutex);
        if(!ros::ok()) {
            if(connected_to_ros) {
                stokesim::print("(stokesros warn: '"+craft_name+"' lost connection with the ROS master)");
                connected_to_ros = false;
            }
            output.terminate = true;
        }
        return output;
    }

private:
    /// Establishes a connection to ROS master and initializes publishers and subscribers
    void connect_to_ros() {
        // Connect to ROS master
        ros::NodeHandle nh;
        tfbr = new tf::TransformBroadcaster;
        // Establish publishers
        state_pub = nh.advertise<nav_msgs::Odometry>("stokesim/"+craft_name+"/state", 1);
        euler_pub = nh.advertise<geometry_msgs::Vector3>("stokesim/"+craft_name+"/euler", 1);
        lla_pub = nh.advertise<sensor_msgs::NavSatFix>("stokesim/"+craft_name+"/lla", 1);
        wind_pub = nh.advertise<geometry_msgs::Vector3>("stokesim/"+craft_name+"/wind", 1);
        for(auto const& name : pcraft->actuator_names) {
            // Each actuator publishes a ROS Wrench message
            actuator_pubs.insert({name, nh.advertise<geometry_msgs::WrenchStamped>("stokesim/"+craft_name+"/actuators/"+name, 1)});
        }
        for(auto const& name : pcraft->sensor_names) {
            if(pcraft->get_sensor(name).type == "imu") {
                // IMU sensors use ROS IMU message
                sensor_pubs.insert({name, nh.advertise<sensor_msgs::Imu>("stokesim/"+craft_name+"/sensors/"+name, 1)});
            } else {
                // Other sensors put all measurements into a float-array
                sensor_pubs.insert({name, nh.advertise<std_msgs::Float64MultiArray>("stokesim/"+craft_name+"/sensors/"+name, 1)});
            }
            if(pcraft->get_sensor(name).has_states()) {
                // Sensors with internal states (like biases) publish those as well
                sensor_state_pubs.insert({name, nh.advertise<std_msgs::Float64MultiArray>("stokesim/"+craft_name+"/sensor_states/"+name, 1)});
            }
            // Sequence numbers keep track of when each sensor's measurement should be new
            sensor_seqs.insert({name, 0});
        }
        // Establish subscribers
        command_sub = nh.subscribe("stokesim/"+craft_name+"/commands", 1, &StokesROS::command_callback, this);
        kill_sub = nh.subscribe("stokesim/"+craft_name+"/kill", 1, &StokesROS::kill_callback, this);
        if(pcraft->has_sensor(radio_name)) {
            rc_channels_sub = nh.subscribe("stokesim/"+craft_name+"/"+radio_name+"_in", 1, &StokesROS::rc_channel_callback, this);
        }
        // Set name of craft as a ROS parameter
        nh.setParam("vehicle_list/"+craft_name+"/name", craft_name);
        // Setup handler for clean exiting
        signal(SIGINT, sigint_handler);
        // Announce successful connection
        connected_to_ros = true;
    }

    void publish_state_info(ros::Time const& ros_time) {
        auto const& body = pcraft->body;
        // Publish state info
        nav_msgs::Odometry state_msg;
        state_msg.header.stamp = ros_time;
        state_msg.header.frame_id = "stokesim/ENU";
        state_msg.child_frame_id = "stokesim/"+craft_name+"/FLU";
        state_msg.pose.pose.position.x = body.pos(0);
        state_msg.pose.pose.position.y = body.pos(1);
        state_msg.pose.pose.position.z = body.pos(2);
        state_msg.pose.pose.orientation.w = body.ori.w();
        state_msg.pose.pose.orientation.x = body.ori.x();
        state_msg.pose.pose.orientation.y = body.ori.y();
        state_msg.pose.pose.orientation.z = body.ori.z();
        state_msg.twist.twist.linear.x = body.vel(0);
        state_msg.twist.twist.linear.y = body.vel(1);
        state_msg.twist.twist.linear.z = body.vel(2);
        state_msg.twist.twist.angular.x = body.avel(0);
        state_msg.twist.twist.angular.y = body.avel(1);
        state_msg.twist.twist.angular.z = body.avel(2);
        state_pub.publish(state_msg);
        // Publish attitude as euler angles for easier analysis
        geometry_msgs::Vector3 euler_msg;
        stokesim::Euler euler = body.compute_euler();
        euler_msg.x = euler.roll;
        euler_msg.y = euler.pitch;
        euler_msg.z = euler.yaw;
        euler_pub.publish(euler_msg);
        // Publish true geodetic coordinates
        sensor_msgs::NavSatFix lla_msg;
        lla_msg.header.stamp = ros_time;
        lla_msg.header.frame_id = "stokesim/GEO";
        lla_msg.latitude = body.lla.lat;
        lla_msg.longitude = body.lla.lon;
        lla_msg.altitude = body.lla.alt;
        lla_pub.publish(lla_msg);
        // Broadcast the TF corresponding to the state
        tf::Transform state_frame;
        state_frame.setOrigin(tf::Vector3(body.pos(0), body.pos(1), body.pos(2)));
        state_frame.setRotation(tf::Quaternion(body.ori.x(), body.ori.y(), body.ori.z(), body.ori.w()));
        tfbr->sendTransform(tf::StampedTransform(state_frame, ros_time, state_msg.header.frame_id, state_msg.child_frame_id));
        // Broadcast the TF corresponding to the offset between the craft's initialized home-coordinates and stokesim ENU coordinates
        tf::Transform home_frame;
        home_frame.setOrigin(tf::Vector3(body.pos0(0), body.pos0(1), body.pos0(2)));
        home_frame.setRotation(tf::Quaternion(0, 0, 0, 1));
        tfbr->sendTransform(tf::StampedTransform(home_frame, ros_time, state_msg.header.frame_id, "stokesim/"+craft_name+"/ENU"));
    }

    void publish_actuator_info(ros::Time const& ros_time) {
        // Publish actuator info
        const std::string child_frame_id = "stokesim/"+craft_name+"/FLU";
        for(auto const& name : pcraft->actuator_names) {
            auto const& actuator = pcraft->get_actuator(name);
            geometry_msgs::WrenchStamped wrench_msg;
            wrench_msg.header.stamp = ros_time;
            wrench_msg.header.frame_id = "stokesim/"+craft_name+"/actuators/"+name;
            wrench_msg.wrench.force.x = actuator.force(0);
            wrench_msg.wrench.force.y = actuator.force(1);
            wrench_msg.wrench.force.z = actuator.force(2);
            wrench_msg.wrench.torque.x = actuator.torque(0);
            wrench_msg.wrench.torque.y = actuator.torque(1);
            wrench_msg.wrench.torque.z = actuator.torque(2);
            actuator_pubs.at(name).publish(wrench_msg);
            tf::Transform actuator_frame;
            if(actuator.type == "thruster") {
                // Cast to obtain thruster-specific information
                auto const& thruster = static_cast<stokesim::physics::actuator_types::Thruster const&>(actuator);
                actuator_frame.setOrigin(tf::Vector3(thruster.pos(0), thruster.pos(1), thruster.pos(2)));
                actuator_frame.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
                tfbr->sendTransform(tf::StampedTransform(actuator_frame, ros_time, child_frame_id, wrench_msg.header.frame_id));
            } else if(actuator.type == "surface") {
                // Cast to obtain surface-specific information
                auto const& surface = static_cast<stokesim::physics::actuator_types::Surface const&>(actuator);
                actuator_frame.setOrigin(tf::Vector3(surface.origin(0), surface.origin(1), surface.origin(2)));
                actuator_frame.setRotation(tf::Quaternion(surface.orient.x(), surface.orient.y(), surface.orient.z(), surface.orient.w()));
                tfbr->sendTransform(tf::StampedTransform(actuator_frame, ros_time, child_frame_id, wrench_msg.header.frame_id));
            }
        }
    }

    void publish_sensor_info(ros::Time const& ros_time) {
        // Publish sensor info
        for(auto const& name : pcraft->sensor_names) {
            auto const& sensor = pcraft->get_sensor(name);
            // If the sensor measurement is new
            if(sensor_seqs.at(name) != sensor.sequence_number) {
                sensor_seqs.at(name) = sensor.sequence_number;
                if(sensor.type == "imu") {
                    // Cast to obtain IMU-specific information
                    auto const& imu_sensor = static_cast<stokesim::physics::sensor_types::IMU const&>(sensor);
                    auto imu_pub = sensor_pubs.at(name);
                    publish_imu_info(imu_sensor, name, imu_pub, ros_time);
                } else {
                    // Other sensors generically publish float-arrays ordered like measurement_names
                    std_msgs::Float64MultiArray sensor_msg;
                    for(auto const& measurement_name : sensor.measurement_names) {
                        sensor_msg.data.push_back(sensor.get_measurement(measurement_name));
                    }
                    sensor_pubs.at(name).publish(sensor_msg);
                }
                if(sensor.has_states()) {
                    // Publish any internal states
                    std_msgs::Float64MultiArray sensor_state_msg;
                    for(auto const& state_name : sensor.state_names) {
                        sensor_state_msg.data.push_back(sensor.get_state(state_name));
                    }
                    sensor_state_pubs.at(name).publish(sensor_state_msg);
                }
            }
        }
    }

    void publish_imu_info(stokesim::physics::sensor_types::IMU const& imu_sensor, std::string const& sensor_name, ros::Publisher & imu_pub, ros::Time const& ros_time) {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros_time;
        // Set frame info string to carry device info, output units set and dt, e.g.
        //  "dev_type=MS3025,units=integrated,dt=0.02" for integrated units or "dev_type=MS3025,units=rates_accel,dt=0.0"
        auto & device_info = imu_msg.header.frame_id;
        device_info = "dev_type="+sensor_name;
        device_info += ",units=";
        device_info += (imu_sensor.use_integrated_units? "integrated" : "rates_accel");
        device_info += ",dt=";
        char buf[10];
        snprintf( buf, sizeof(buf)-1, "%.3f", imu_sensor.dt_output);
        device_info += std::string(buf);
        // Set covariance
        imu_msg.orientation_covariance[0] = -1; // implies no orientation is given (http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html)
        // Set measurements
        imu_msg.angular_velocity.x = imu_sensor.get_measurement("gyro_x");
        imu_msg.angular_velocity.y = imu_sensor.get_measurement("gyro_y");
        imu_msg.angular_velocity.z = imu_sensor.get_measurement("gyro_z");
        imu_msg.linear_acceleration.x = imu_sensor.get_measurement("accel_x");
        imu_msg.linear_acceleration.y = imu_sensor.get_measurement("accel_y");
        imu_msg.linear_acceleration.z = imu_sensor.get_measurement("accel_z");
        // Publish
        imu_pub.publish(imu_msg);
    }

    void publish_wind_info(ros::Time const& ros_time, stokesim::Evec3 const& wind) {
        geometry_msgs::Vector3 wind_msg;
        wind_msg.x = wind(0);
        wind_msg.y = wind(1);
        wind_msg.z = wind(2);
        wind_pub.publish(wind_msg);
    }

    void command_callback(std_msgs::Float64MultiArray::ConstPtr const& msg) {
        // Write the corresponding commands to each actuator in the actuator order list
        std::lock_guard<std::mutex> guard(output_mutex);
        for(uint i=0; i<std::min(command_order.size(), msg->data.size()); ++i) {
            output.world_cmd.craft_cmds[craft_name].actuator_cmds[command_order[i]].values["effort"] = msg->data[i];
        }
    }

    void kill_callback(std_msgs::String::ConstPtr const& msg) {
        std::lock_guard<std::mutex> guard(output_mutex);
        // Toggle kill for the whole craft?
        if((msg->data == craft_name) || (msg->data == "craft") || (msg->data == "")) {
            output.world_cmd.craft_cmds[craft_name].kill = !(pcraft->killed);
            stokesim::print("(stokesros info: toggled kill for craft '"+craft_name+"')");
        // or just an actuator?
        } else if(pcraft->has_actuator(msg->data)) {
            output.world_cmd.craft_cmds[craft_name].actuator_cmds[msg->data].kill = !(pcraft->get_actuator(msg->data).killed);
            stokesim::print("(stokesros info: toggled kill for actuator '"+msg->data+"' of craft '"+craft_name+"')");
        // or just a sensor?
        } else if(pcraft->has_sensor(msg->data)) {
            output.world_cmd.craft_cmds[craft_name].sensor_cmds[msg->data].kill = !(pcraft->get_sensor(msg->data).killed);
            stokesim::print("(stokesros info: toggled kill for sensor '"+msg->data+"' of craft '"+craft_name+"')");
        } else {
            stokesim::print("(stokesros warn: no actuator or sensor with name '"+msg->data+"' found for craft '"+craft_name+"')");
        }
    }

    void rc_channel_callback(std_msgs::Float64MultiArray::ConstPtr const& msg) {
        assert(msg->data.size() == 20);
        // Write all possible rc channel values and RSSI to output
        std::lock_guard<std::mutex> guard(output_mutex);
        auto& rc_channels = output.world_cmd.craft_cmds[craft_name].sensor_cmds[radio_name];
        // Channels
        rc_channels.values["chan_1"]  = msg->data[0];
        rc_channels.values["chan_2"]  = msg->data[1];
        rc_channels.values["chan_3"]  = msg->data[2];
        rc_channels.values["chan_4"]  = msg->data[3];
        rc_channels.values["chan_5"]  = msg->data[4];
        rc_channels.values["chan_6"]  = msg->data[5];
        rc_channels.values["chan_7"]  = msg->data[6];
        rc_channels.values["chan_8"]  = msg->data[7];
        rc_channels.values["chan_9"]  = msg->data[8];
        rc_channels.values["chan_10"] = msg->data[9];
        rc_channels.values["chan_11"] = msg->data[10];
        rc_channels.values["chan_12"] = msg->data[11];
        rc_channels.values["chan_13"] = msg->data[12];
        rc_channels.values["chan_14"] = msg->data[13];
        rc_channels.values["chan_15"] = msg->data[14];
        rc_channels.values["chan_16"] = msg->data[15];
        rc_channels.values["chan_17"] = msg->data[16];
        rc_channels.values["chan_18"] = msg->data[17];
        // Meta information
        rc_channels.values["rssi"] = msg->data[18];
        rc_channels.values["chancount"] = msg->data[19];
        rc_channels.interrupt = true;
        rc_channels.update_sequence = false;
    }

    /// Allows for clean ctrl+c exits
    static void sigint_handler(int sig) {
        std::cout << std::endl;
        ros::shutdown();
    }
};

//////////////////////////////////////////////////

}  // namespace stokesros

EXPORT_AS_PLUGIN(stokesros::StokesROS)
