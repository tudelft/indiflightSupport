#include <boost/filesystem.hpp>

#include "unified_mocap_client.hpp"

#ifdef USE_CLIENT_CONSOLE
    #include "console_client.hpp"
#endif

#ifdef USE_CLIENT_IVY
    #include "ivy_client.hpp"
#endif

#ifdef USE_CLIENT_UDP
    #include "udp_client.hpp"
#endif

#ifdef USE_CLIENT_LOG
    #include "log_client.hpp"
#endif

#if defined(USE_CLIENT_ROS2) || defined(USE_CLIENT_ROS2PX4)
    #include "ros2_client.hpp"
    #include "rclcpp/rclcpp.hpp"
#endif

namespace{
    std::function<void(int)> shutdown_handler;
    void signal_handler(int signal) { shutdown_handler(signal); }
}

int main(int argc, char const *argv[])
{
    boost::filesystem::path p(argv[0]);

    shutdown_handler = [p](int signum) 
    { 
        std::cout << "Shutting down... Done. " << std::endl;
#if defined(USE_CLIENT_ROS2) || defined(USE_CLIENT_ROS2PX4)  
        if (p.filename() == "mocap2ros2"
            || p.filename() == "mocap2ros2px4")
        {
            rclcpp::shutdown();
        }
#endif
         exit(0);
    };

	signal(SIGINT, signal_handler);

#ifdef USE_CLIENT_CONSOLE
    if (p.filename() == "mocap2console") {
        Mocap2Console client = Mocap2Console();
        client.start(argc, argv);
    } else
#endif

#ifdef USE_CLIENT_IVY
    if (p.filename() == "mocap2ivy") {
        Mocap2Ivy client = Mocap2Ivy(); client.start(argc, argv);
    } else 
#endif

#ifdef USE_CLIENT_UDP
    if (p.filename() == "mocap2udp") {
        Mocap2Udp client = Mocap2Udp(); client.start(argc, argv);
    } else 
#endif

#ifdef USE_CLIENT_LOG
    if (p.filename() == "mocap2log") {
        Mocap2Log client = Mocap2Log(); client.start(argc, argv);
    } else 
#endif

#if defined(USE_CLIENT_ROS2) || defined(USE_CLIENT_ROS2PX4)

    if (p.filename() == "mocap2ros2" || p.filename() == "mocap2ros2px4") {
        // Init ROS2
        rclcpp::init(argc, argv);

        // Disable Default logger
        rclcpp::get_logger("rclcpp").set_level(rclcpp::Logger::Level::Error);

        // Init Client
        Mocap2Ros2 client = Mocap2Ros2();

        // Start the thread
        client.start(argc, argv);
    }
#endif
    {
        std::cout << "Support for client " << p.filename() << "was not compiled into the program." << std::endl;
        return 1;
    }

    return 0;
}

