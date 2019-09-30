/** @file
This example plugin establishes a ROS node that forwards the simulator clock to ROS.
*/
#include <signal.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <sensor_msgs/TimeReference.h>
#include <stokesim/io/plugin.h>

namespace stokesros {

//////////////////////////////////////////////////

/// Establishes a ROS node for communicating with the simulator.
class RosClock : public stokesim::io::Plugin {
private:
    /// @name Derived
    //@{
    bool _connected_to_ros = false;  ///< Whether or not this plugin has connected to a ROS master
    ros::Publisher _time_pub;  ///< Single publisher for publishing a time reference for the craft
    int _curr_iter{0};
    int _num_iters_to_wait{500};
    //@}

public:
    /// Implements Plugin::~Plugin
    ~RosClock() override {}

    /// Implements Plugin::configure
    bool configure(YAML::Node const& config, stokesim::physics::World const& world) override {
        // Initialize this ROS node (only NodeHandles need a ROS master)
        int dummy_argc = 0;
        char** dummy_argv = nullptr;
        ros::init(dummy_argc, dummy_argv, "stokesim_clock");
        if(ros::master::check()) {_connect_to_ros();}
        else {stokesim::print("(warning: no ROS master found, will retry)");}
        return true;
    }

    /// Implements Plugin::work
    Output work(stokesim::physics::World const& world) override {
        Output output;

        bool master_alive = _connected_to_ros;
        if (_curr_iter >= _num_iters_to_wait) {
          master_alive = ros::master::check();
          _curr_iter = 0;
        } else {
          _curr_iter++;
        }

        if(!_connected_to_ros && master_alive) {
            _connect_to_ros();
            return output;
        } else if (!master_alive && _connected_to_ros) {
            _connected_to_ros = false;
            return output;
        } else if (_connected_to_ros) {
            ros::Time current_ros_time = ros::Time::now();
            ros::Time current_stokesim_time(world.clock.t);
            sensor_msgs::TimeReference to_send;
            to_send.header.stamp = current_ros_time;
            to_send.time_ref = current_stokesim_time;
            to_send.source = "stokesim";
            _time_pub.publish(to_send);
            ros::spinOnce();
        }

        return output;
    }

private:
    /// Establishes a connection to ROS master and initializes publishers and subscribers
    void _connect_to_ros() {
        // Connect to ROS master
        ros::NodeHandle nh;
        _time_pub = nh.advertise<sensor_msgs::TimeReference>("stokesim/clock", 10);
        signal(SIGINT, sigint_handler);
        _connected_to_ros = true;
    }

    /// Allows for clean ctrl+c exits
    static void sigint_handler(int sig) {
        std::cout << std::endl;
        ros::shutdown();
    }
};

}  // namespace stokesros

//////////////////////////////////////////////////

EXPORT_AS_PLUGIN(stokesros::RosClock)
