#include "unified_mocap_client.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <boost/algorithm/string.hpp>
#include <functional>

#ifdef _WIN32
#   include <conio.h>
#else
#   include <unistd.h>
#   include <termios.h>
#endif
#include <inttypes.h>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetRequests.h>

// stream operators for enums
std::ostream& operator<<(std::ostream& lhs, ErrorCode e) {
    switch(e) {
    case ErrorCode_OK: lhs <<               "OK"; break;
    case ErrorCode_Internal: lhs <<         "Internal"; break;
    case ErrorCode_External: lhs <<         "External"; break;
    case ErrorCode_Network: lhs <<          "Network"; break;
    case ErrorCode_Other: lhs <<            "Other"; break;
    case ErrorCode_InvalidArgument: lhs <<  "InvalidArgument"; break;
    case ErrorCode_InvalidOperation: lhs << "InvalidOperation"; break;
    case ErrorCode_InvalidSize: lhs <<      "InvalidSize"; break;
    }
    return lhs;
} 

std::ostream& operator<<(std::ostream& lhs, CoordinateSystem e) {
    switch(e) {
    case UNCHANGED: lhs << "UNCHANGED"; break;
    case NED: lhs << "NED"; break;
    case ENU: lhs << "ENU"; break;
    }
    return lhs;
} 

std::ostream& operator<<(std::ostream& lhs, ArenaDirection e) {
    switch(e) {
    case RIGHT: lhs << "RIGHT"; break;
    case FAR_SIDE: lhs << "FAR_SIDE"; break;
    case LEFT: lhs << "LEFT"; break;
    case NEAR_SIDE: lhs << "NEAR_SIDE"; break;
    }
    return lhs;
} 

std::ostream& operator<<(std::ostream& lhs, UpAxis e) {
    switch(e) {
    case NOTDETECTED: lhs << "NOTDETECTED"; break;
    case X: lhs << "X"; break;
    case Y: lhs << "Y"; break;
    case Z: lhs << "Z"; break;
    }
    return lhs;
}

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData) {
    UnifiedMocapClient* that = (UnifiedMocapClient*) pUserData;
    that->natnet_data_handler(data);
};

UnifiedMocapClient::UnifiedMocapClient(const UnifiedMocapClient &other)
{
    (void) other;
    std::cerr << "Copy constructor for UnifiedMocapClient not supported. Exiting." << std::endl;
    std::raise(SIGINT);
}

UnifiedMocapClient::UnifiedMocapClient()
    : publish_dt{1.0 / 100.0}, streaming_ids{1}, co{CoordinateSystem::ENU}, co_north{ArenaDirection::FAR_SIDE}, true_north_deg{0.}, long_edge{ArenaDirection::RIGHT}, craft_nose{ArenaDirection::FAR_SIDE}, pClient{NULL}, up_axis{UpAxis::NOTDETECTED}, printMessages{false},
    nTrackedRB{0}, testMode{false}
{
    // TODO: use builtin forward prediction with the latency estimates plus a 
    // user-defined interval (on the order of 10ms)?

    this->print_startup();

    // Initialize non-trivial arrays
    for(unsigned int i = 0; i < MAX_TRACKED_RB; i++)
    {
        this->setPublishedAllRB();
        this->poseRB[i] = pose_t();
        this->poseDerRB[i] = pose_der_t(); 
    }

}

UnifiedMocapClient::~UnifiedMocapClient()
{
}

// Non-action implementation of the virtual function to make the implementation optional
void UnifiedMocapClient::publish_data()
{
}

// Non-action implementation of the virtual function to make the implementation optional
void UnifiedMocapClient::pre_start()
{
}

// Non-action implementation of the virtual function to make the implementation optional
void UnifiedMocapClient::post_start()
{
    // must be blocking!
    this->pubThread.join();
    this->keyThread.join();
}

// Non-action implementation of the virtual function to make the implementation optional
void UnifiedMocapClient::add_extra_po(boost::program_options::options_description &desc)
{
    (void)desc;
}

// Non-action implementation of the virtual function to make the implementation optional
void UnifiedMocapClient::parse_extra_po(const boost::program_options::variables_map &vm)
{
    (void)vm;
}

double UnifiedMocapClient::seconds_since_mocap_ts(uint64_t us)
{
   if(this->testMode)
        return 0.0;
   else
        return this->pClient->SecondsSinceHostTimestamp(us);
}

void UnifiedMocapClient::publish_loop()
{
    using namespace std::chrono_literals;
    bool run = true;
    auto ts = std::chrono::steady_clock::now();
    float sleep_time = this->publish_dt;

    while(run)
    {
        if (this->testMode)
        {
            sFrameOfMocapData fakeData;
            const auto microsecondsSinceEpoch = std::chrono::time_point_cast<std::chrono::microseconds>(ts).time_since_epoch().count();
            fakeData.CameraMidExposureTimestamp = microsecondsSinceEpoch;
            fakeData.nRigidBodies = 1;
            fakeData.RigidBodies[0].ID = this->get_streaming_ids()[0];
            fakeData.RigidBodies[0].MeanError = 0.001;
            fakeData.RigidBodies[0].qw = 1.;
            fakeData.RigidBodies[0].qx = 0.;
            fakeData.RigidBodies[0].qy = 0.;
            fakeData.RigidBodies[0].qz = 0.;
            fakeData.RigidBodies[0].x = 1.;
            fakeData.RigidBodies[0].y = 2.;
            fakeData.RigidBodies[0].z = 3.;
            fakeData.RigidBodies[0].params = 0x01; // valid
            this->natnet_data_handler(&fakeData);
        }

        // TODO: do not publish if no new data!

        this->publish_data();
        this->setPublishedAllRB();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(sleep_time * 1s);

        /* We measure the duration of publish_data and adjust 
         * the publish_dt based on that in a closed loop fashion
         * in order to achieve the desired frequency */
        auto ts_new = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration = ts_new - ts;

        sleep_time += 0.25 * (this->publish_dt - duration.count()); 

        ts = ts_new;
    }
}

void UnifiedMocapClient::keystroke_loop()
{
    // wait for keystrokes
    if (this->testMode)
        std::cout << std::endl << "Faking messages in test mode! Press q to quit, Press t to toggle message printing" << std::endl;
    else
        std::cout << std::endl << "Listening to messages! Press q to quit, Press t to toggle message printing" << std::endl;

	while ( const int c = getch() )
    {
        switch(c)
        {
            case 'q':
                std::raise(SIGINT);
                break;
            case 't':
                this->togglePrintMessages();
                break;
        }
    }
}

void UnifiedMocapClient::start(int argc, const char *argv[])
{
    this->read_po(argc, argv);

    // instantiate client and make connection
    if (!(this->testMode))
    {
        this->pClient = new NatNetClient();
        ErrorCode ret = this->connectAndDetectServerSettings();
        if (ret != ErrorCode_OK) {
            // returning from main is best for cleanup?
            std::raise(SIGINT);
            return;
        }

        // register callback
        ret = this->pClient->SetFrameReceivedCallback( DataHandler, this );
        if (ret != ErrorCode_OK) {
            std::cout << "Registering frame received callback failed with Error Code " << ret << std::endl;
            return;
        }
    }

    this->print_coordinate_system();

    // initialize filters for derivatives
    for (unsigned int i=0; i < MAX_TRACKED_RB; i++)
        derFilter[i] = FilteredDifferentiator(10., 5., this->fSample);

    this->pre_start();
    this->pubThread = std::thread(&UnifiedMocapClient::publish_loop, this);
    this->keyThread = std::thread(&UnifiedMocapClient::keystroke_loop, this);
    this->post_start();
}

void UnifiedMocapClient::read_po(int argc, char const *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("publish_frequency,f", po::value<float>(), "publishing frequency of the MoCap odometry")
        ("streaming_ids,s", po::value<std::vector<unsigned int> >()->multitoken(), "streaming ids to track")
        ("coordinate_system,c", po::value<std::string>(), "coordinate system convention to use [unchanged, ned, enu]")
        ("coordinate_north,r", po::value<std::string>(), "where north should be relative to the observer. Either non-zero number describing right-hand rotation of north axis from the far side, or one of [right, far_side, left, near_side].")
        ("long_edge,l", po::value<std::string>(), "direction of long edge during Motive ground-plane calibration [right, far_side, left, near_side]")
        ("craft_nose,n", po::value<std::string>(), "direction of aircraft nose when creating the rigid body in Motive [right, far_side, left, near_side]")
        ("test", "Send test data instead")
    ;

    // Adding any extra program options from the sub-class
    this->add_extra_po(desc);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        exit(0);
    }

    if (vm.count("publish_frequency")) {
        this->publish_dt = 1.0 / vm["publish_frequency"].as<float>();
        std::cout << "Publish frequency was set to " 
                  << 1.0 / this->publish_dt << " Hz" << std::endl;
    } else {
        std::cout << "Publish frequency not set, defaulting to " 
                  << 1.0 / this->publish_dt
                  << " Hz" << std::endl;   
    }

    if(vm.count("streaming_ids"))
    {
        this->streaming_ids = vm["streaming_ids"].as<std::vector<unsigned int>>();
        std::cout << "Streaming IDs set to";
        for(unsigned int id : this->streaming_ids) std::cout << " " << id << " ";
        std::cout << std::endl;
    }
    else
    {
        std::cout << "Streaming IDs not set" <<std::endl;
        exit(1);
    }

    if(vm.count("coordinate_system"))
    {
        std::string co_name = vm["coordinate_system"].as<std::string>();
        boost::algorithm::to_lower(co_name);

        if(co_name.compare("unchanged") == 0) { this->co = CoordinateSystem::UNCHANGED; }
        else if(co_name.compare("ned") == 0) { this->co = CoordinateSystem::NED; }
        else if (co_name.compare("enu") == 0) { this->co = CoordinateSystem::ENU; }
        else {
            std::cout << "Coordinate system " << co_name << " not definied. Exiting" << std::endl;
            std::raise(SIGINT);
        }
        std::cout << "Coordinate system set to " << this->co << std::endl;
    }
    else
    {
        std::cout << "Coordinate System not set, defaulting to "
                  << this->co << std::endl;
    }

    if(vm.count("coordinate_north"))
    {
        if (this->co == CoordinateSystem::UNCHANGED) {
            std::cout << "Can only specify --coordinate_north/-r when coordinate system is not UNCHANGED. Exiting" << std::endl;
            std::raise(SIGINT);
        }
        std::string co_north = vm["coordinate_north"].as<std::string>();
        boost::algorithm::to_lower(co_north);

        if(co_north.compare("right") == 0)
        {
            this->co_north = ArenaDirection::RIGHT;
        }
        else if(co_north.compare("far_side") == 0)
        {
            this->co_north = ArenaDirection::FAR_SIDE;
        }
        else if (co_north.compare("left") == 0)
        {
            this->co_north = ArenaDirection::LEFT;
        }
        else if (co_north.compare("near_side") == 0)
        {
            this->co_north = ArenaDirection::NEAR_SIDE;
        }
        else
        {
            this->co_north = ArenaDirection::TRUE_NORTH;
            this->true_north_deg = std::atof(co_north.c_str());
            if (this->true_north_deg == 0.0) {
                std::cout << "Coordinate system argument " << co_north << " is neither [near_side, far_side, right, left], nor float (for 0.0 use far_side). Exiting" << std::endl;
                std::raise(SIGINT);
            }
        }

        std::cout << "Coordinate system north set to " << this->co_north << std::endl;
    }
    else
    {
        std::cout << "Coordinate System north not set, defaulting to "
                  << this->co_north << std::endl;
    }

    if(vm.count("long_edge"))
    {
        if (this->co == CoordinateSystem::UNCHANGED) {
            std::cout << "Can only specify --long_edge/-l when coordinate system is not UNCHANGED. Exiting" << std::endl;
            std::raise(SIGINT);
        }
        std::string le = vm["long_edge"].as<std::string>();
        boost::algorithm::to_lower(le);

        if(le.compare("right") == 0)
        {
            this->long_edge = ArenaDirection::RIGHT;
        }
        else if(le.compare("far_side") == 0)
        {
            this->long_edge = ArenaDirection::FAR_SIDE;
        }
        else if (le.compare("left") == 0)
        {
            this->long_edge = ArenaDirection::LEFT;
        }
        else if (le.compare("near_side") == 0)
        {
            this->long_edge = ArenaDirection::NEAR_SIDE;
        }
        else
        {
            std::cout << "Long Edge Direction " << le << " not definied. Exiting" << std::endl;
            std::raise(SIGINT);
        }
        std::cout << "Long Edge direction set to " << this->long_edge << std::endl;
    }
    else
    {
        std::cout << "Long Edge direction not set, defaulting to "
                  << this->long_edge << std::endl;
    }

    if(vm.count("craft_nose"))
    {
        if (this->co == CoordinateSystem::UNCHANGED) {
            std::cout << "Can only specify --craft_nose/-n when coordinate system is not UNCHANGED. Exiting" << std::endl;
            std::raise(SIGINT);
        }
        std::string le = vm["craft_nose"].as<std::string>();
        boost::algorithm::to_lower(le);

        if(le.compare("right") == 0)
        {
            this->craft_nose = ArenaDirection::RIGHT;
        }
        else if(le.compare("far_side") == 0)
        {
            this->craft_nose = ArenaDirection::FAR_SIDE;
        }
        else if (le.compare("left") == 0)
        {
            this->craft_nose = ArenaDirection::LEFT;
        }
        else if (le.compare("near_side") == 0)
        {
            this->craft_nose = ArenaDirection::NEAR_SIDE;
        }
        else
        {
            std::cout << "A/C Nose Direction " << le << " not definied. Exiting" << std::endl;
            std::raise(SIGINT);
        }

        std::cout << "A/C nose direction set to " << this->craft_nose << std::endl;

    }
    else
    {
        std::cout << "A/C nose direction not set, defaulting to "
                  << this->craft_nose << std::endl;
    }

    if(vm.count("test"))
    {
        std::cout << "Sending test data instead of attempting Motive Connect" << std::endl;
        this->testMode = true;
        this->up_axis = UpAxis::Y;
        this->fSample = 100;
        this->serverConfig.HighResClockFrequency = 1e6;
    }

    // process streaming ids
    for (unsigned int i : this->streaming_ids) {
        if (this->trackRB(i) == -1) {
            std::cout << "Cannot track Rigid Body with streaming id " << i << ". Too many rigid bodies." << std::endl;
        }
    }

    // Parsing the extra program options from the sub-class
    this->parse_extra_po(vm);
}

void UnifiedMocapClient::print_startup()
{
    // generator and font: https://patorjk.com/software/taag/#p=display&f=Small&t=Type%20Something%20
    std::cout<< R"(
#################################################################################
##  _   _      _  __ _        _ __  __                    ___ _ _         _    ##
## | | | |_ _ (_)/ _(_)___ __| |  \/  |___  __ __ _ _ __ / __| (_)___ _ _| |_  ##
## | |_| | ' \| |  _| / -_) _` | |\/| / _ \/ _/ _` | '_ \ (__| | / -_) ' \  _| ##
##  \___/|_||_|_|_| |_\___\__,_|_|  |_\___/\__\__,_| .__/\___|_|_\___|_||_\__| ##
##                                                 |_|                         ##)";
}

void UnifiedMocapClient::print_coordinate_system() const
{
    // There's probably a better way to do this but I can't think
    // of it. So a bunch of nested switch-case statements it is.
    std::cout<< R"( 
               far
     +──────────────────────────+
     │                          │)";
    switch(this->craft_nose)
    {
        case ArenaDirection::RIGHT:
            std::cout << R"(
     │                          │
     │       [craft] ▶          │
     │                          │)";
            break;
        case ArenaDirection::FAR_SIDE:
            std::cout << R"(
     │            ▲             │
     │         [craft]          │
     │                          │)";
            break;
        case ArenaDirection::LEFT:
            std::cout << R"(
     │                          │
     │       ◀ [craft]          │
     │                          │)";
            break;
        case ArenaDirection::NEAR_SIDE:
            std::cout << R"(
     │                          │
     │         [craft]          │
     │            ▼             │)";
            break;
    }

    std::cout << R"(
left │                          │ right )";

    // If the up_axis is not detected the CO is not well defined
    // and we can't draw the coordinate system
    if(this->up_axis == UpAxis::NOTDETECTED)
    {
        std::cout << R"(
     │ UpAxis could not be      │
     │ detected. CO not well    │ 
     │ defined. Use a newer     │ 
     │ version of motive.       │)" << std::endl;
        return;
    }
    switch(this->co)
    {
        case CoordinateSystem::UNCHANGED:
            switch(this->up_axis)
            {
                case UpAxis::X:
                    std::cout << R"( 
     │      ↑                   │
     │    x ⊙ →                 │
     │                          │
     │    y-z Unchanged         │)";
                    break;
                case UpAxis::Y:
                    std::cout << R"( 
     │      ↑                   │
     │    y ⊙ →                 │
     │                          │
     │    x-z Unchanged         │)";
                    break;
                case UpAxis::Z:
                    std::cout << R"( 
     │      ↑                   │
     │    z ⊙ →                 │
     │                          │
     │    x-y Unchanged         │)";
                    break;
            }
            break;
        case CoordinateSystem::NED:
            switch (this->co_north)
            {
                case ArenaDirection::RIGHT:
                    std::cout << R"( 
     │                          │
     │   z  ⓧ → x               │
     │    y ↓                   │)";
                    break;
                case ArenaDirection::FAR_SIDE:
                    std::cout << R"( 
     │    x ↑                   │
     │   z  ⓧ → y               │
     │                          │)";
                    break;
                case ArenaDirection::LEFT:
                    std::cout << R"( 
     │    y ↑                   │
     │  x ← ⓧ z                 │
     │                          │)";
                    break;
                case ArenaDirection::NEAR_SIDE:
                    std::cout << R"( 
     │                          │
     │  y ← ⓧ z                 │
     │    x ↓                   │)";
                    break;
                case ArenaDirection::TRUE_NORTH: {
                    int rounded = static_cast<int>(std::round(this->true_north_deg));
                    char sign = (this->true_north_deg < 0) ? '-' : '+';
                    std::cout << R"( 
     │north )" << sign << std::setw(3) << std::setfill(' ') << std::abs(rounded) << R"(° from far side │)" << R"(
     │      ⓧ                   │
     │     z                    │)";
                    break;
                }
            }
            break;
        case CoordinateSystem::ENU:
            switch (this->co_north)
            {
                case ArenaDirection::FAR_SIDE:
                    std::cout << R"( 
     │    y ↑                   │
     │   z  ⊙ → x               │
     │                          │)";
                    break;
                case ArenaDirection::LEFT:
                    std::cout << R"( 
     │    x ↑                   │
     │  y ← ⊙ z                 │
     │                          │)";
                    break;
                case ArenaDirection::NEAR_SIDE:
                    std::cout << R"( 
     │                          │
     │  x ← ⊙ z                 │
     │    y ↓                   │)";
                    break;
                case ArenaDirection::RIGHT:
                    std::cout << R"( 
     │                          │
     │   z  ⊙ → y               │
     │    x ↓                   │)";
                    break;
                case ArenaDirection::TRUE_NORTH: {
                    int rounded = static_cast<int>(std::round(this->true_north_deg));
                    char sign = (this->true_north_deg < 0) ? '-' : '+';
                    std::cout << R"( 
     │north )" << sign << std::setw(3) << std::setfill(' ') << std::abs(rounded) << R"(° from far side │)" << R"(
     │      ⊙                   │
     │     z                    │)";
                    break;
                }
            }
            break;

    }
    
    std::cout<<R"(
     │                          │
     +──────────────────────────+
     │    Observers  (near)     │
     └──────────────────────────┘
     Assuming calibration triangle 
     long-edge has pointed to )" << this->long_edge << std::endl;
}

ErrorCode UnifiedMocapClient::connectAndDetectServerSettings()
{
    ErrorCode ret;
    static constexpr unsigned int DISCOVERY_TIMEOUT = 1000;
    static constexpr unsigned int MAX_DISCOVERY = 10;
    std::cout<<std::endl<<"Discovering NatNet servers (timeout " << DISCOVERY_TIMEOUT << "ms)... ";

    int n = 1;
    sNatNetDiscoveredServer availableServers[MAX_DISCOVERY]; // just support one for now
    ret = NatNet_BroadcastServerDiscovery(availableServers, &n, DISCOVERY_TIMEOUT);
    if ((ret != ErrorCode_OK) || (n == 0)) {
        if (ret != ErrorCode_OK)
            std::cout << "Failed with Error code " << ret << std::endl;
        else {
            std::cout << "Failed: No servers found" << std::endl;
            ret = ErrorCode_Network;
        }

        std::cout<<std::endl<<"Troubleshooting: " << std::endl;
        std::cout<<"1. Verify connected to Motive network" << std::endl;
        std::cout<<"2. Verify that 'interface' is NOT set to 'loopback' in Motive 'Data Streaming Pane'" << std::endl;
        return ret;

    } else if (n > 1) {
        std::cout << "Failed: more than 1 server found:" << std::endl;
        for (int i=0; i<MAX_DISCOVERY; i++) {
            if (i >= n) { break; }
            std::cout << availableServers[i].serverAddress << std::endl;
        }
        return ErrorCode_Network;

    }
    /*
    else if (!(availableServers[0].serverDescription.bConnectionInfoValid)) {
        std::cout << "Failed: server ConnectionInfo invalid" << std::endl;
    }
    */

    std::cout << "Successful!" << std::endl;

    this->connectParams.connectionType\
        = availableServers[0].serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
    this->connectParams.serverCommandPort = availableServers[0].serverCommandPort;
    this->connectParams.serverDataPort = availableServers[0].serverDescription.ConnectionDataPort;
    this->connectParams.serverAddress = availableServers[0].serverAddress;
    this->connectParams.localAddress = availableServers[0].localAddress;

    char mcastAddress[kNatNetIpv4AddrStrLenMax];
#ifdef _WIN32
    _snprintf_s(
#else
    snprintf(
#endif
        mcastAddress, sizeof mcastAddress,
        "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
        availableServers[0].serverDescription.ConnectionMulticastAddress[0],
        availableServers[0].serverDescription.ConnectionMulticastAddress[1],
        availableServers[0].serverDescription.ConnectionMulticastAddress[2],
        availableServers[0].serverDescription.ConnectionMulticastAddress[3]
    );
    this->connectParams.multicastAddress = mcastAddress;

/*
    this->connectParams.connectionType = ConnectionType_Multicast;
    this->connectParams.serverCommandPort = NATNET_DEFAULT_PORT_COMMAND;
    this->connectParams.serverDataPort = NATNET_DEFAULT_PORT_DATA;
    this->connectParams.serverAddress = "192.168.209.81";
    //this->connectParams.localAddress = "192.168.0.255"; // better to leave these blank, then it autodetects, i think
    this->connectParams.multicastAddress = NATNET_DEFAULT_MULTICAST_ADDRESS;
    this->connectParams.subscribedDataOnly = false; // no idea what this does
    memset(this->connectParams.BitstreamVersion, 0, sizeof(this->connectParams.BitstreamVersion)); // no idea what this is
*/

    std::cout << std::endl << "Attempting connection to " << this->connectParams.serverAddress << "... ";
    ret = this->pClient->Connect(this->connectParams);
    if (ret == ErrorCode_OK) {
        std::cout<<"Successful!"<<std::endl;
    } else {
        std::cout<<"Failed with unknown error code "<< ret <<std::endl;
/*
        std::cout<<"Failed with error code "<< ret <<std::endl;
        std::cout<<std::endl<<"Troubleshooting: " << std::endl;
        std::cout<<"1. Verify connected to Motive network" << std::endl;
        std::cout<<"2. Verify that 'interface' is NOT set to 'local' in Motive 'Data Streaming Pane'" << std::endl;
*/
        return ret;
    }

    void* response;
    int nBytes;

    // detect host clock settings (for accurate time calcs and version detection)
    std::cout<<"Detecting Server Configuration.. ";
    memset( &(this->serverConfig), 0, sizeof( this->serverConfig ) );
    ret = this->pClient->GetServerDescription(&(this->serverConfig));
    if (ret == ErrorCode_OK) {
        std::cout << "Done" << std::endl;
    } else {
        std::cout << "Error code " << ret << std::endl;
        return ret;
    }

    // abort if unsupported NatNetVersion
    if (this->serverConfig.NatNetVersion[0] < 3) {
        std::cout << "ERROR: NatNet Version < 3 detected. Use Motive 2.x or newer"
            << std::endl;
        return ErrorCode::ErrorCode_External;
    }

    // detect frame rate
    std::cout<<"Detecting frame rate... ";
    ret = this->pClient->SendMessageAndWait("FrameRate", &response, &nBytes);
    if (ret == ErrorCode_OK) {
        this->fSample = (double) *((float*)response);
        std::cout << this->fSample << "Hz";
        if (this->fSample < (1.0 / this->publish_dt))
            std::cout << " WARNING: Publish frequency was set higher which has no effect: incomming messages will only be published once.";
        
        std::cout << std::endl;
    } else {
        std::cout << "Error code " << ret << std::endl;
        return ret;
    }

    // detect up axis
    std::cout<<"Detecting up axis... ";
    ret = this->pClient->SendMessageAndWait("GetProperty,,Up Axis", &response, &nBytes);
    if (ret == ErrorCode_OK) {
        this->up_axis = static_cast<UpAxis>( ((char*)response)[0] - '0' );
        std::cout << this->up_axis << std::endl;
    } else {
        std::cout << "Error code " << ret << std::endl;
        return ret;
    }

    // inform user
    std::cout << std::endl << "INFO: if you see this message but you still don't receive messages, check:" << std::endl;
    std::cout << "1. Rigid body streaming id(s) are correct" << std::endl;
    std::cout << "2. Rigid body(s) are selected in Motive" << std::endl;
    std::cout << "3. 'Multicast' is selected in 'Data Streaming Pane' in Motive" << std::endl;

    return ErrorCode_OK;
}

void UnifiedMocapClient::natnet_data_handler(sFrameOfMocapData* data)
{
    // get timestamp
    uint64_t timeAtExpoUs = data->CameraMidExposureTimestamp / (this->serverConfig.HighResClockFrequency * 1e-6);

    // loop over bodies in frame and process the ones we listen to
    bool printedHeader = false;
	for(int i=0; i < data->nRigidBodies; i++) {

        int idx = this->getIndexRB(data->RigidBodies[i].ID);
        if (idx == -1)
            continue; // untracked by us

        bool bTrackingValid = data->RigidBodies[i].params & 0x01;
        if (!bTrackingValid)
            continue;

        if ( (this->printMessages) && (!printedHeader) ) {
            std::cout << "Received NatNet data for " << data->nRigidBodies << " rigid bodies for host time: " << timeAtExpoUs << "us. Tracked bodies:" << std::endl;
            printedHeader = true;
        }

        pose_t newPose {
            timeAtExpoUs,
            data->RigidBodies[i].x,
            data->RigidBodies[i].y,
            data->RigidBodies[i].z,
            data->RigidBodies[i].qx,
            data->RigidBodies[i].qy,
            data->RigidBodies[i].qz,
            data->RigidBodies[i].qw,
        };

        /* Transform the pose to the desired coordinate system */
        newPose = transform_pose(this->co,
                                this->co_north,
                                this->true_north_deg,
                                this->up_axis,
                                this->long_edge,
                                this->craft_nose,
                                newPose);

        /* Thread safely setting the new values */
        pose_der_t newPoseDer = derFilter[idx].apply(newPose);
        this->setPoseDerRB(idx, newPoseDer);
        this->setPoseRB(idx, newPose);

        if (this->printMessages) {
		    printf("Incoming Rigid Body Data Frame [ID=%d Error=%3.4f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
		    printf("\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		    printf("Incoming: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n",
		    	data->RigidBodies[i].x,
		    	data->RigidBodies[i].y,
		    	data->RigidBodies[i].z,
		    	data->RigidBodies[i].qx,
		    	data->RigidBodies[i].qy,
		    	data->RigidBodies[i].qz,
		    	data->RigidBodies[i].qw);
		    printf("Published: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n",
		    	newPose.x,
		    	newPose.y,
		    	newPose.z,
		    	newPose.qx,
		    	newPose.qy,
		    	newPose.qz,
		    	newPose.qw);
            printf("\t\tvx\tvy\tvz\twx\twy\twz\n");
		    printf("Published: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n\n",
                newPoseDer.x,
                newPoseDer.y,
                newPoseDer.z,
                newPoseDer.wx,
                newPoseDer.wy,
                newPoseDer.wz);
        }
    }

    if ( (this->printMessages) && (!printedHeader) )
        std::cout << "Received NatNet data for " << data->nRigidBodies << " rigid bodies for host time: " << timeAtExpoUs << "us, but none are tracked." << std::endl;

    return;
}


// helper function to get character presses
#ifndef _WIN32
char getch()
{
    char buf = 0;
    termios old = { 0, 0, 0, 0, 0, "\0", 0, 0 };

    fflush( stdout );

    if ( tcgetattr( 0, &old ) < 0 )
        perror( "tcsetattr()" );

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if ( tcsetattr( 0, TCSANOW, &old ) < 0 )
        perror( "tcsetattr ICANON" );

    if ( read( 0, &buf, 1 ) < 0 )
        perror( "read()" );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;

    if ( tcsetattr( 0, TCSADRAIN, &old ) < 0 )
        perror( "tcsetattr ~ICANON" );

    //printf( "%c\n", buf );

    return buf;
}
#endif
