#ifndef NUFRAME_H
#define NUFRAME_H

#include <bot_core/timestamp.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <lcm/lcm.h>
#include <unordered_map>
#include <buffer_core.h>
#include <duration.h>
#include <utility>

#include <nutils/lang/TimerTask.hpp>

#include <nutils/nuparam.h>

#include <glib.h>
#include <iostream>

typedef struct {
	std::string frame_name;
	std::string relative_to;
	bot_core_rigid_transform_t_subscription_t * transform_subscription;
  	bot_core_pose_t_subscription_t * pose_subscription;
  	//bot_frames_update_t_subscription_t * update_subscription;
} channel_info_t;

class NuFrame 
{
public:
	NuFrame(lcm_t *lcm, BotParam* param, int32_t sec, int32_t nsec);
	virtual ~NuFrame();

	static void on_pose_update(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_pose_t *msg, void *user_data);
	static void on_frames_update(const lcm_recv_buf_t *rbuf, const char *channel, const bot_frames_update_t *msg, void *user_data);
	static void on_transform_update(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_rigid_transform_t *msg, void *user_data);

	std::unordered_map<std::string, channel_info_t> m_channels;
	std::unique_ptr<tf2::BufferCore> m_BC;
private:
	lcm_t *m_lcm;
	bot_frames_update_t_subscription_t *m_default_update;
	void unsubscribe();


};

#endif