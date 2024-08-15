#ifndef H_POSE_CALCULATIONS
#define H_POSE_CALCULATIONS

#include <iostream>
#include <csignal>
#include <cmath>

typedef struct pose_s {
    uint64_t timeUs;
    float x;
    float y;
    float z;
    float qx;
    float qy;
    float qz;
    float qw;
} pose_t;

typedef struct pose_der_s {
    uint64_t timeUs;
    float x;
    float y;
    float z;
    float wx;
    float wy;
    float wz;
} pose_der_t;

enum CoordinateSystem { UNCHANGED=0, NED, ENU};
enum UpAxis { NOTDETECTED=-1, X=0, Y, Z };
enum ArenaDirection{RIGHT=0, FAR_SIDE, LEFT, NEAR_SIDE, TRUE_NORTH};

pose_t transform_pose(const CoordinateSystem co,
                      const ArenaDirection co_north,
                      const float true_north_deg,
                      const UpAxis up_axis,
                      const ArenaDirection long_edge,
                      const ArenaDirection craft_nose,
                      const pose_t newPose);


class PureDifferentiator
{
    protected:
        pose_t _pose;
        pose_der_t _unfiltered;
        bool _valid;
    public:
        PureDifferentiator() : _valid{false} { _pose.qw = 1.; };
        //~PureDifferentiator();
        pose_der_t getUnfiltered() { return _unfiltered; };
        virtual pose_der_t getFiltered() { return _unfiltered; };
        virtual pose_der_t apply(pose_t newPose) {
            // default: no filtering at all
            newSample(newPose);
            return _unfiltered;
        };

    protected:
        void newSample(pose_t newPose);
};

class FilteredDifferentiator : public PureDifferentiator
{
    private:
        double _fBreakVel;
        double _fBreakRate;
        double _fSample;
        double _kVel;
        double _kRate;
        pose_der_t _filtered;
        bool _initialized;

    public:
        FilteredDifferentiator(double fBreakVel, double fBreakRate, double fSample);

        FilteredDifferentiator() : FilteredDifferentiator(1., 1., 1.) {
            // C++11
            // probably you can instead use a vector instead of a array to store FilteredDifferentiator instances?
        }

        pose_der_t apply(pose_t newPose);
};

#endif // H_POSE_CALCULATINOS