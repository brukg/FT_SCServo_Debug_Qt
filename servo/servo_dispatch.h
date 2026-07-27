#ifndef SERVO_DISPATCH_H
#define SERVO_DISPATCH_H

// Per-series dispatch for multi-servo control. These free functions take the
// three control-class instances and route by ServoProfile::series, so the same
// tested path is shared by the Debug tab and the Joint Control tab.
//
// The interesting one is sync_write_group: a SYNC WRITE is one broadcast to many
// IDs at the same register with an identical payload, but the payload differs by
// series (HLS carries torque at bytes 44/45 where STS carries zeros, and SCS uses
// a different address and range). So the armed joints are bucketed by series and
// one sync packet is emitted per series present.

#include <vector>
#include "servo/scserial.h"

namespace feetech_servo
{

struct GroupTarget
{
    uint8_t      id;
    ServoProfile profile;
    int          pos;
};

// Enable/disable torque on one servo, dispatched by series.
inline void enable_torque_for(SCSCL *scs, SMS_STS *sms, HLSCL *hls,
                              uint8_t id, const ServoProfile &p, bool on)
{
    if(!p.known)
        return;
    switch(p.series)
    {
        case SCS:
        case SCS2: scs->enable_torque(id, on); break;
        case HLS:  hls->enable_torque(id, on); break;
        default:   sms->enable_torque(id, on); break;
    }
}

// Write one servo's goal position, dispatched by series. torque is used by HLS only.
inline int write_goal_for(SCSCL *scs, SMS_STS *sms, HLSCL *hls,
                          uint8_t id, const ServoProfile &p,
                          int pos, int speed, int acc, int torque)
{
    if(!p.known)
        return -1;
    switch(p.series)
    {
        case SCS:
        case SCS2:
            return scs->write_pos(id, pos, 0, speed);
        case HLS:
            hls->servo_mode(id);
            return hls->write_pos_ex(id, pos, speed, acc, torque);
        default:
            sms->rotation_mode(id);
            return sms->write_pos_ex(id, pos, speed, acc);
    }
}

// Sync-write a group, one packet per series present. Cross-series simultaneity is
// best-effort (packets sent back to back); within a series it is a single packet.
inline void sync_write_group(SCSCL *scs, SMS_STS *sms, HLSCL *hls,
                             const std::vector<GroupTarget> &targets,
                             int speed, int acc, int torque)
{
    // SCS / SCS2 bucket
    {
        std::vector<uint8_t>  ids;
        std::vector<uint16_t> pos, times, speeds;
        for(const auto &t : targets)
            if(t.profile.known && (t.profile.series == SCS || t.profile.series == SCS2))
            {
                ids.push_back(t.id);
                pos.push_back((uint16_t)t.pos);
                times.push_back(0);
                speeds.push_back((uint16_t)speed);
            }
        if(!ids.empty())
            scs->sync_write_pos(ids.data(), ids.size(), pos.data(), times.data(), speeds.data());
    }

    // HLS bucket
    {
        std::vector<uint8_t>  ids, accs;
        std::vector<int16_t>  pos;
        std::vector<uint16_t> speeds, torques;
        for(const auto &t : targets)
            if(t.profile.known && t.profile.series == HLS)
            {
                ids.push_back(t.id);
                pos.push_back((int16_t)t.pos);
                speeds.push_back((uint16_t)speed);
                accs.push_back((uint8_t)acc);
                torques.push_back((uint16_t)torque);
            }
        if(!ids.empty())
            hls->sync_write_pos_ex(ids.data(), ids.size(), pos.data(), speeds.data(), accs.data(), torques.data());
    }

    // SMS / STS / SMBL bucket (everything else known)
    {
        std::vector<uint8_t>  ids, accs;
        std::vector<int16_t>  pos;
        std::vector<uint16_t> speeds;
        for(const auto &t : targets)
            if(t.profile.known && t.profile.series != SCS && t.profile.series != SCS2 && t.profile.series != HLS)
            {
                ids.push_back(t.id);
                pos.push_back((int16_t)t.pos);
                speeds.push_back((uint16_t)speed);
                accs.push_back((uint8_t)acc);
            }
        if(!ids.empty())
            sms->sync_write_pos_ex(ids.data(), ids.size(), pos.data(), speeds.data(), accs.data());
    }
}

// Position slider range by series resolution.
inline int position_max_for(ModelSeries series)
{
    return (series == SCS || series == SCS2) ? 1023 : 4095;
}

}

#endif // SERVO_DISPATCH_H
