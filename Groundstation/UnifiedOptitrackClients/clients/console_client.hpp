#include "unified_mocap_client.hpp"

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

class Mocap2Console : public UnifiedMocapClient
{
public:
    Mocap2Console()
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout<< R"(
##   ___                  _      ################################################
##  / __|___ _ _  ___ ___| |___  ##
## | (__/ _ \ ' \(_-</ _ \ / -_) ##
##  \___\___/_||_/__/\___/_\___| ##
###################################
)" << std::endl;
        // do as little setup here as possible. Use pre_start instead
    }

    ~Mocap2Console()
    {
        // put cleanup here if needed
    }

private:
    void add_extra_po(boost::program_options::options_description &desc) override
    {
        // add extra commandlien options if you need to.
        // avoid those already used in UnifiedMocapClient::read_po()
        //desc.add_options()
        //    ("dontmindme,d", boost::program_options::value<std::string>(), "Optimal extra argument for demonstration purposes")
        //    ("listofint,i", boost::program_options::value<std::vector<unsigned int>>()->multitopken(), "Optional list of values for demonstration purposes")
        //;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        /*
        if (vm.count("dontmindme")) {
            // value can be accessed with vm["dontmindme"].as<std::string>();
            std::cout << "Give user some feedback about the chosen" << vm["dontmindme"].as<std::string>() << std::endl;
        } else {
            // fail if not found, or use some default value
            std::cout << "dontmindme not specified. aborting..." << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("listofint")) {
            // values can be accessed via vm["listofint"].as<std::vector<unsigned int>>();
        } else {
            // do something
        }
        */
    }

    void pre_start() override
    {
        // gets called after:
        // 1. UnifiedMocapClient is constructed
        // 2. this class is constructed
        // 3. arguments are parsed
        // 4. MocapClient and its callback handlers are constructed

        // do your setup work here, like initializing ports, opening lopfiles, etc
    }

    /*
    void post_start() override
    {
        // this gets called after publish_loop thread is spun
        // must be blocking!!
    }
    */

    void publish_data() override
    {
        // this gets called every this->publish_dt seconds. Publish data here.
        // for instance, this is how to just print the z-position of all tracked bodies

        // don't run anything expensive in here, just publishing

        /*
        for(uint8_t i = 0; i < this->getNTrackedRB(); i++)
        {
            if (this->isUnpublishedRB(i)) {
                unsigned int streaming_id = this->getStreamingIds()[i];

                pose_t pose = this->getPoseRB(i);
                //pose_der_t pose_der = this->getPoseDerRB(i); // derivative

                std::cout << "Rigid body with streaming id " << streaming_id 
                << " has z position " << pose.z << std::endl;
            }
        }
        */
    }

private:
    // can be used for extra class members
};
