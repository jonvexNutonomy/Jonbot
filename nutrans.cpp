/* Copyright (c) nuTonomy Inc. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Author: eric@nutonomy.com
 *         pratik@nutonomy.com
 */

#include <nutils/common.h>
#include <nutils/assert.h>
#include "nutrans.h"

#define NUM_SECONDS (int32_t)10
#define NUM_NSECONDS (int32_t)0

namespace nutils
{

int nutrans_t::curr_printfs_;
nutrans_t::nutrans_t(lcm_t* lcm, const nuparam_t* const nuparam) :
    lcm_(lcm),
    nuparam_(nuparam)
{
    nuparam_->getParam("channel.pose", channel_pose_, std::string("POSE"));

    taskErrNoPose.setRepeated(1);
    have_last_pose_ = 0;
    pose_subscription_ =
        bot_core_pose_t_subscribe(lcm_, channel_pose_.c_str(), nutrans_t::on_pose, this);

    // we won't need nuparam and frames in every instantiation of nutrans
    if (nuparam_) {
        m_frames = std::make_unique<NuFrame>(lcm_, nuparam_->getBotParam(), NUM_SECONDS, NUM_NSECONDS);
        //frames = bot_frames_get_global(lcm_, nuparam_->getBotParam());
    } else {
        frames = NULL;
    }
}

nutrans_t::~nutrans_t() {
    // frames is a pointer to a global variable.
    // Do not delete
    //if (frames)
    //    bot_frames_destroy(frames);

    if (pose_subscription_)
        bot_core_pose_t_unsubscribe(lcm_, pose_subscription_);
}

void nutrans_t::on_pose(const lcm_recv_buf_t* rbuf, const char* channel,
                        const bot_core_pose_t* msg, void* user)
{
    if(nullptr == rbuf || nullptr == channel || nullptr == msg || nullptr == user)
        return;

    nutrans_t* self = (nutrans_t*)user;
    self->last_pose_ = *msg;
    self->have_last_pose_ = 1;
}

/**
 * get vehicle positions, i.e., [0,0,0] in body frame
 * in local frame
 * Returns 1 if success, 0 if failure
 */
int nutrans_t::vehicle_in_local(double pos[3]) { 
    return get_local_pos(pos);
}

bool nutrans_t::getTf(std::string from_frame, std::string to_frame, int64_t utime, double *bot_tf)
{
    //calls:
    //bot_frames_get_trans_with_utime
    //  bot_ctrans_get_trans
    //      _get_path
    //          bot_ctrans_get_new_path: this is the function with djikstra
    //      bot_ctrans_path_to_trans: only calls functions that are math I think
    //          bot_trans_set_identity: just math
    //          _link_get_trans_interp: i think jsut math but a lot
    //          bot_trans_invert: math
    //          bot_trans_apply_trans
    //bot_trans_get_mat_3x4:just math and a math function call
    // if(!bot_frames_get_trans_mat_3x4_with_utime(frames, from_frame.c_str(),
    //                                             to_frame.c_str(), utime, bot_tf))
    // {
    //     std::cout<<"Warning: transform point failed to find frame from "<<from_frame<<"-->"<<to_frame<<std::endl;
    //     return false;
    // }
    // return true;


    //Jon try
    ros::Time t((uint32_t)(msg->utime/1000000), (uint32_t)((msg->utime % 1000000) * 1000));
    
    geometry_msgs::TransformStamped ts;
    try {
        ts = m_frames->m_BC->lookupTransform(to_frame, from_frame, t);
    } catch (...) {
        std::cout<<"Warning: transform point failed to find frame from "<<from_frame<<"-->"<<to_frame<<std::endl;
        return false;
    }

    double quat[4];
    quat[0] = ts.transform.rotation.w;
    quat[1] = ts.transform.rotation.x;
    quat[2] = ts.transform.rotation.y;
    quat[3] = ts.transform.rotation.z;

    

    double norm = quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3];
    //if (fabs(norm) < 1e-10)
    //    return -1;

    norm = 1/norm;
    double x = quat[1]*norm;
    double y = quat[2]*norm;
    double z = quat[3]*norm;
    double w = quat[0]*norm;

    double x2 = x*x;
    double y2 = y*y;
    double z2 = z*z;
    double w2 = w*w;
    double xy = 2*x*y;
    double xz = 2*x*z;
    double yz = 2*y*z;
    double wx = 2*w*x;
    double wy = 2*w*y;
    double wz = 2*w*z;
    // double rot[9];
    // rot[0] = w2+x2-y2-z2;  
    // rot[1] = xy-wz;  
    // rot[2] = xz+wy;
    // rot[3] = xy+wz;  
    // rot[4] = w2-x2+y2-z2;  
    // rot[5] = yz-wx;
    // rot[6] = xz-wy; 
    // rot[7] = yz+wx; 
    // rot[8] = w2-x2-y2+z2;
    bot_tf[0] = w2+x2-y2-z2; 
    bot_tf[1] = xy-wz;
    bot_tf[2] = xz+wy;
    bot_tf[3] = ts.transform.translation.x;
    bot_tf[4] = xy+wz;
    bot_tf[5] = w2-x2+y2-z2;
    bot_tf[6] = yz-wx;
    bot_tf[7] = ts.transform.translation.y;
    bot_tf[8] = xz-wy;
    bot_tf[9] = yz+wx;
    bot_tf[10] = w2-x2-y2+z2;
    bot_tf[11] = ts.transform.translation.z;
    return true;
}

void nutrans_t::transformPoint(double v[3], double bot_tf[12])
{
    double result[3];
    result[0] = bot_tf[0]*v[0] + bot_tf[1]*v[1] + bot_tf[2]*v[2] + bot_tf[3];
    result[1] = bot_tf[4]*v[0] + bot_tf[5]*v[1] + bot_tf[6]*v[2] + bot_tf[7];
    result[2] = bot_tf[8]*v[0] + bot_tf[9]*v[1] + bot_tf[10]*v[2] + bot_tf[11];
    

    //bot_vector_affine_transform_3x4_3d(bot_tf, v, result);
    memcpy(v, result, sizeof(result));
}


/**
 * get the local (x,y,z) position along with heading
 */
int nutrans_t::get_local_pos(double pos[3], double* heading) {
    // nu_assert(frames != NULL);

    // BotTrans bt;
    // //calls:
    // //bot_ctrans_get_trans_latest
    // //  _get_path: so djikstra
    // //  bot_ctrans_path_to_trans
    // //      A bunch of stuff: see nutrans_t::getTf

    // if (!bot_frames_get_trans(frames, "body", "local", &bt)) {
    //     ERR("get_local_pos() did not get body to local transform\n");
    //     return 0;
    // }
    // // bot_trans_print_trans(&bt);
    // bot_trans_get_trans_vec(&bt, pos);

    // if (heading) {
    //     double rpy[3];
    //     bot_quat_to_roll_pitch_yaw(bt.rot_quat, rpy);
    //     *heading = rpy[2];
    // }
    // return 1;




    //jon try
    nu_assert(m_frames != NULL);

    ///
    //deal with exception
    //geometry_msgs::TransformStamped ts = m_frames->m_BC->lookupTransform("local", "body", 0);
    ///


    geometry_msgs::TransformStamped ts;
    try {
        ts = m_frames->m_BC->lookupTransform("local", "body", 0);
    } catch (...) {
        std::cout<<"Warning: transform point failed to find frame from "<<from_frame<<"-->"<<to_frame<<std::endl;
        return false;
    }

    
    pos[0] = ts.transform.translation.x;
    pos[1] = ts.transform.translation.y;
    pos[2] = ts.transform.translation.z;
    if (heading) {
        double q[4];
        q[0] = ts.transform.rotation.w;
        q[1] = ts.transform.rotation.x;
        q[2] = ts.transform.rotation.y;
        q[3] = ts.transform.rotation.z;

        /// probably don't need rpy 0 and 1
        double rpy[3];
        double roll_a = 2 * (q[0]*q[1] + q[2]*q[3]);
        double roll_b = 1 - 2 * (q[1]*q[1] + q[2]*q[2]);
        rpy[0] = atan2(roll_a, roll_b);

        double pitch_sin = 2 * (q[0]*q[2] - q[3]*q[1]);
        rpy[1] = asin(pitch_sin);

        ///

        double yaw_a = 2 * (q[0]*q[3] + q[1]*q[2]);
        double yaw_b = 1 - 2 * (q[2]*q[2] + q[3]*q[3]);
        rpy[2] = atan2(yaw_a, yaw_b);

        *heading = rpy[2];
        
    }
    return 1;



}

int nutrans_t::get_local_pose(bot_core_pose_t* pose) {
    if (have_last_pose_) {
        memcpy(pose, &last_pose_, sizeof(bot_core_pose_t));
        // printf("pose: pos-(%.3f,%.3f,%.3f)\n", pose->pos[0], pose->pos[1],
        // pose->pos[2]);
        // printf("pose: last_pose-(%.3f,%.3f,%.3f)\n", last_pose.pos[0],
        // last_pose.pos[1], last_pose.pos[2]);
        return 1;
    } else {
        if (taskErrNoPose.isReady())
            ERR("get_local_pose(): does not have pose, is POSE message being "
                "published?\n");
        return 0;
    }
}

} // namespace nutils

