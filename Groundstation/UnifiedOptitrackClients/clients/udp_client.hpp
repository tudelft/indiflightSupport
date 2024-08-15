#include "unified_mocap_client.hpp"
#include <unistd.h>
#include <csignal>
#include "boost/asio.hpp"

using namespace boost::asio;

class Mocap2Udp : public UnifiedMocapClient
{
public:
    Mocap2Udp() : _socket{_io_service}
    {
        std::cout<< R"(
##  _   _ ___  ___  #############################################################
## | | | |   \| _ \ ##
## | |_| | |) |  _/ ##
##  \___/|___/|_|   ##
######################
)" << std::endl;
    }
    ~Mocap2Udp()
    {
        this->_socket.close();
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("client_ip,i", boost::program_options::value<std::string>(), "IP to stream the UDP data to.")
            ("port,p", boost::program_options::value<unsigned short int>(), "UDP Port.")
            ("ac_id,ac", boost::program_options::value<std::vector<unsigned int>>()->multitoken(), "Optional Aircraft ID corresponding to the rigid body id(s). These ids will be included in the UDP messages")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("client_ip")) {
            std::string val = vm["client_ip"].as<std::string>();
            std::cout << "UDP client ip " << val << std::endl;
            this->_client_ip = val;
        } else {
            std::cout << "No UDP client ip passed" << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("port")) {
            unsigned short int val = vm["port"].as<unsigned short int>();
            std::cout << "Streaming UDP on port " << val << std::endl;
            this->_port = val;
        } else {
            std::cout << "No UDP port given. Assume 5005" << std::endl;
            this->_port = 5005;
        }

        if (this->getStreamingIds().size() > 1) {
            std::cout << "Number of streaming_ids and ac_ids must be equal to 1 for the udp client. Multiple not (yet) supported"
                << std::endl;
            std::raise(SIGINT);
        }

        if(vm.count("ac_id")) {
            this->_ac_id = vm["ac_id"].as<std::vector<unsigned int>>();
            if ( this->_ac_id.size() != this->getStreamingIds().size() ) {
                std::cout << "Number of ac_ids must be equal to streaming_ids" << std::endl;
                std::raise(SIGINT);
            }
            std::cout << "AC IDs set to";
            for(unsigned int id : this->_ac_id) std::cout << " " << id << " ";
            std::cout << std::endl;
        } else {
            this->_ac_id.push_back(0);
            std::cout << "No ac_id passed. Assuming 0." << std::endl;
        }
    }

    void pre_start() override
    {
        this->_remote_endpoint = ip::udp::endpoint(ip::address::from_string(this->_client_ip), this->_port);
        this->_socket.open(ip::udp::v4());
    }

    void publish_data() override
    {
        static constexpr size_t Ni = sizeof(this->_ac_id[0]);
        static constexpr size_t Np = sizeof(pose_t);
        static constexpr size_t Nd = sizeof(pose_der_t);

        unsigned int i = 0;

        //for(uint8_t i = 0; i < this->getNTrackedRB(); i++)
        //{
            if (this->isUnpublishedRB(i)) {
                pose_t pose = this->getPoseRB(i);
                pose_der_t pose_der = this->getPoseDerRB(i);
                uint8_t buf[Ni+Np+Nd];
                memcpy(buf, &(this->_ac_id[i]), Ni);
                memcpy(buf+Ni, &pose, Np);
                memcpy(buf+Ni+Np, &pose_der, Nd);

                boost::system::error_code err;
                auto sent = this->_socket.send_to(buffer(buf, Ni+Np+Nd), this->_remote_endpoint, 0, err);
                if (err.failed())
                    std::cout << "Failed with error " << err << std::endl;
            }
        //}
    }

private:
    std::vector<unsigned int> _ac_id;
    unsigned short int _port;
    std::string _client_ip;
    io_service _io_service;
    ip::udp::socket _socket;
    ip::udp::endpoint _remote_endpoint;
};
