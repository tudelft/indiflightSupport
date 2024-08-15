#include "unified_mocap_client.hpp"
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <Ivy/ivyloop.h>
#include <unistd.h>
#include <csignal>

class Mocap2Ivy : public UnifiedMocapClient
{
public:
    Mocap2Ivy()
    {
        std::cout<< R"(
##  ___           ###############################################################
## |_ _|_ ___  _  ##
##  | |\ V / || | ##
## |___|\_/ \_, | ##
##          |__/  ##
####################
)" << std::endl;
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("ac_id,ac", boost::program_options::value<std::vector<unsigned int>>()->multitoken(), "Aircraft Id to forward IVY messages to.")
            ("broadcast_address,b", boost::program_options::value<std::string>(), "Ivy broadcast ip address.")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        // TODO: make this a list to correspond to rigid_body ids
        if (vm.count("ac_id"))
        {
            this->_ac_id = vm["ac_id"].as<std::vector<unsigned int>>();
            if (this->_ac_id.size() != this->getStreamingIds().size()) {
                std::cout << "Number of ac_ids and streaming_ids passed must be equal"
                    << std::endl;
                std::raise(SIGINT);
            }
            std::cout << "AC IDs set to";
            for(unsigned int id : this->_ac_id) std::cout << " " << id << " ";
            std::cout << std::endl;
        } else {
            std::cout << "No aircraft id passed, but is required." << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("broadcast_address"))
        {
            std::string val = vm["broadcast_address"].as<std::string>();
            std::cout << "Ivy broadcast ip set to " << val << std::endl;
            this->bip = val;
        } else {
            std::cout << "No ivy broadcast ip passed, assume 127.255.255.255" << std::endl;
            this->bip = "127.255.255.255";
        }
    }

    void pre_start() override
    {
        IvyInit ("Mocap2Ivy", "Mocap2Ivy READY", NULL, NULL, NULL, NULL);
        IvyStart(this->bip.c_str());
    }

    void post_start() override
    {
        // must be blocking!
        IvyMainLoop();
    }

    void publish_data() override
    {
        for(uint8_t i = 0; i < this->getNTrackedRB(); i++)
        {
            if (this->isUnpublishedRB(i)) {
                pose_t pose = this->getPoseRB(i);
                pose_der_t pose_der = this->getPoseDerRB(i);
                IvySendMsg("datalink EXTERNAL_POSE %d %lu  %f %f %f  %f %f %f  %f %f %f %f",
                    _ac_id[i], pose.timeUs/1000,  //todo: probably not the right timestamp
                    pose.x, pose.y, pose.z,
                    pose_der.x, pose_der.y, pose_der.z,
                    pose.qw, pose.qx, pose.qy, pose.qz);
            }
        }
    }

private:
    std::vector<unsigned int> _ac_id;
    std::string bip;
};
