/* Copyright (c) nuTonomy Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Author: eric@nutonomy.com
 *         pratik@nutonomy.com
 */

#ifndef NUCORE_NUTILS_NUTRANS_H_
#define NUCORE_NUTILS_NUTRANS_H_

#include <bot_core/timestamp.h>
#include <bot_param/param_client.h>
#include <lcm/lcm.h>

#include <nutils/lang/TimerTask.hpp>

#include <nutils/nuparam.h>
#include "nuframe.h"

#include <glib.h>
#include <iostream>

namespace nutils
{

/**
 * nutrans_t is a small wrapper around bot2-frames
 *
 * TODO "we can add more functionality later"
 *
 */
struct nutrans_t {
    TimerTask taskErrNoPose;

public:
    nutrans_t(lcm_t*, const nuparam_t* const);

    ~nutrans_t();

    int get_local_pos(double pos[3], double* heading = NULL);
    int get_local_pose(bot_core_pose_t* pose);
    int vehicle_in_local(double pos[3]);
    bool getTf(std::string from_frame, std::string to_frame, int64_t utime, double *bot_tf);
    void transformPoint(double v[3], double bot_tf[12]);
    std::unique_ptr<NuFrame> m_frames;
   


private:
    static int curr_printfs_;
    const static int max_printfs_ = 25;

    static void on_pose(const lcm_recv_buf_t* rbuf, const char* channel,
                        const bot_core_pose_t* msg, void* user);
    static void on_frames_update(BotFrames* _frames, const char* frame,
                                 const char* relative_to, int64_t utime,
                                 void* user);

    lcm_t* lcm_;
    const nuparam_t* const nuparam_;

    // save last pose here
    bot_core_pose_t_subscription_t* pose_subscription_;
    bot_core_pose_t last_pose_;
    int have_last_pose_;

    std::string channel_pose_;
};

} // namespace nutils

#endif
