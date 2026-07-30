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

// ---- compliance controls ------------------------------------------------

// Set a servo's work mode: 0 = position/servo, 1 = wheel/speed, 2 = current-force.
// Current-force (compliance) exists on HLS only; STS/SMS fall back to position for
// mode 2, and SCS stays position (this UI drives SCS in position only).
inline void set_work_mode_for(SCSCL *scs, SMS_STS *sms, HLSCL *hls,
                              uint8_t id, const ServoProfile &p, int mode)
{
    (void)scs;
    if(!p.known) return;
    switch(p.series)
    {
        case HLS:
            if(mode == 2)      hls->ele_mode(id);      // constant current / force
            else if(mode == 1) hls->wheel_mode(id);
            else               hls->servo_mode(id);
            break;
        case SCS:
        case SCS2:
            break;                                     // position-only here
        default:                                       // SMS / STS / SMBL
            if(mode == 1) sms->wheel_mode(id);
            else          sms->rotation_mode(id);
            break;
    }
}

// Stiffness = Torque Limit (reg 48), the max holding force in position mode.
// High = stiff, low = compliant. Reg 48 is Torque Limit on HLS/STS/SMS but is the
// Lock flag on SCS, so SCS is skipped.
inline void set_stiffness_for(SCSerial *raw, uint8_t id, const ServoProfile &p, int limit)
{
    if(!p.known || p.series == SCS || p.series == SCS2) return;
    raw->set_end(p.end);
    raw->write_word(id, 48, (uint16_t)limit);
}

// In current-force (compliance) mode, command a target current/force directly.
inline void write_force_for(SCSCL *scs, SMS_STS *sms, HLSCL *hls,
                            uint8_t id, const ServoProfile &p, int force)
{
    (void)scs; (void)sms;
    if(!p.known || p.series != HLS) return;
    hls->write_ele(id, (int16_t)force);
}

}

#endif // SERVO_DISPATCH_H
