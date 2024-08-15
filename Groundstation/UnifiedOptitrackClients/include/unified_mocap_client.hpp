#ifndef H_UNIFIED_OPTITRACK_CLIENT
#define H_UNIFIED_OPTITRACK_CLIENT

#include <vector>
#include <mutex>
#include <boost/program_options.hpp>
#include "NatNetClient.h"
#include "NatNetTypes.h"
#include "pose_calculations.hpp"

#include <iostream>
#include <csignal>
#include <thread>

#ifndef _WIN32
char getch();
#endif

constexpr unsigned int MAX_TRACKED_RB = 10;

class UnifiedMocapClient 
{
private:
    float publish_dt;
    CoordinateSystem co;
    ArenaDirection co_north;
    float true_north_deg;
    ArenaDirection long_edge;
    ArenaDirection craft_nose;
    NatNetClient* pClient;
    sNatNetClientConnectParams connectParams;
    UpAxis up_axis;
    bool testMode;

    // container for tracking the rigid bodies
    bool printMessages;
    uint8_t nTrackedRB;
    int trackedRB[MAX_TRACKED_RB];
    bool unpublishedDataRB[MAX_TRACKED_RB];
    pose_t poseRB[MAX_TRACKED_RB];
    std::mutex poseMutexes[MAX_TRACKED_RB];
    pose_der_t poseDerRB[MAX_TRACKED_RB];
    std::mutex poseDerMutexes[MAX_TRACKED_RB];
    double fSample;
    sServerDescription serverConfig;

    FilteredDifferentiator derFilter[MAX_TRACKED_RB];

    void read_po(int argc, char const *argv[]);
    void print_startup();
    void print_coordinate_system() const;

    ErrorCode connectAndDetectServerSettings();

    bool _initialized;
    std::thread pubThread;
    std::thread keyThread;

    inline void togglePrintMessages() { printMessages ^= true; };
    std::vector<unsigned int> streaming_ids;

protected:
    unsigned int getStreamingId(unsigned int i) {return this->streaming_ids[i]; };
    std::vector<unsigned int> getStreamingIds() {return this->streaming_ids; };
    int trackRB(unsigned int id) {
        int i = this->getIndexRB(id);
        if (i > -1) { return i; } // already tracked, that's fine
        if (this->nTrackedRB >= MAX_TRACKED_RB) { return -1; } // cannot add, too many RBs
        this->trackedRB[this->nTrackedRB] = id;
        return this->nTrackedRB++;
    };

    int8_t getNTrackedRB(){
        return this->nTrackedRB;
    }

    std::vector<unsigned int> get_streaming_ids()
    {
        return this->streaming_ids;
    }
    int getIdRB(unsigned int idx)
    {
        return this->trackedRB[idx];
    }
    bool isUnpublishedRB(unsigned int idx)
    {
        return this->unpublishedDataRB[idx];
    }
    void setPublishedAllRB(void)
    {
        for (unsigned int idx = 0; idx < getNTrackedRB(); idx++)
            this->unpublishedDataRB[idx] = false;
    }
    void setUnpublishedRB(unsigned int idx)
    {
        for (unsigned int idx = 0; idx < getNTrackedRB(); idx++)
            this->unpublishedDataRB[idx] = true;
    }
    /* Thread safe mutators and accessors for the current pose 
     * and pose derivative values. */
    bool setPoseRB(unsigned int idx, pose_t pose) {
        // Lock respective mutex
        this->poseMutexes[idx].lock();

        memcpy(&(this->poseRB[idx]), &pose, sizeof(pose_t));
        this->setUnpublishedRB(idx);

        this->poseMutexes[idx].unlock();
        return true;
    };
    pose_t getPoseRB(unsigned int idx){
        // Lock respective mutex
        this->poseMutexes[idx].lock();
        pose_t pose(this->poseRB[idx]);
        this->poseMutexes[idx].unlock();
        return pose;
    };
    bool setPoseDerRB(int idx, pose_der_t poseDer) {
        // Lock respective mutex
        this->poseDerMutexes[idx].lock();

        memcpy(&(this->poseDerRB[idx]), &poseDer, sizeof(pose_der_t));
        this->setUnpublishedRB(idx);

        this->poseDerMutexes[idx].unlock();
        return true;
    };
    pose_der_t getPoseDerRB(unsigned int idx){
        // Lock respective mutex
        this->poseDerMutexes[idx].lock();
        pose_der_t pose_der(this->poseDerRB[idx]);
        this->poseDerMutexes[idx].unlock();
        return pose_der;
    };

    CoordinateSystem getCO() const
    {
        return this->co;
    }

    /* Returns the seconds since the a timestamp in
     * MoCap reference time */
    double seconds_since_mocap_ts(uint64_t us);
    /* Function that is used to spin up the publish thread */
    void publish_loop();
    void keystroke_loop();
    /* Virtual Function to be implemented by base classes */
    // called just before/after start of the publishing thread
    virtual void pre_start(); 
    virtual void post_start(); // must be blocking
    // Extra Program Options
    virtual void add_extra_po(boost::program_options::options_description &desc);
    virtual void parse_extra_po(const boost::program_options::variables_map &vm);
    // Publishing functions
    virtual void publish_data();
    

public:
    UnifiedMocapClient();
    UnifiedMocapClient(const UnifiedMocapClient &other);
    ~UnifiedMocapClient();
    void natnet_data_handler(sFrameOfMocapData* data);

    /* Function that needs to be called after creating
     * the client to start everything */
    void start(int argc, char const *argv[]);

    int getIndexRB(int id) { 
        for (unsigned int i=0; i < this->nTrackedRB; i++) {
            if (this->trackedRB[i] == id)
                return i;
        }
        return -1;
    };
    
};


#endif //H_UNIFIED_OPTITRACK_CLIENT