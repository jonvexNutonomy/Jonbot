#include "nuframe.h"

NuFrame::NuFrame(lcm_t *lcm, BotParam* param, int32_t sec, int32_t nsec) :
	m_BC(std::make_unique<tf2::BufferCore>(ros::Duration(sec, nsec))),
	m_lcm(lcm)
{
	// //
	// //Don't really understand this
	// //maybe don't need
	// ros::init();
	// //

	// ros::NodeHandle node;



	int num_frames = bot_param_get_num_subkeys(param, "coordinate_frames");
	if (num_frames <= 0) {
		throw "No coordinate_frames";
	}


	//Need to somehow create the root frame
	BotParamElement *Element = param->root;
	char *ElementName;
	if (bot_param_get_str(param, "coordinate_frames.root_frame", &ElementName) < 0) {
		//root_frame not defined
		throw "root_frame not defined";
	}

	char **frame_names = bot_param_get_num_subkeys(param, "coordinate_frames");

	int i;
	for (i = 0; i < num_frames; i++) {
		char *frame_name = strdup(frame_names[i]);
		char param_key[2048];
		sprintf(param_key, "coordinate_frames.%s", frame_name);
		int num_sub_keys = bot_param_get_num_subkeys(param, param_key);
		if (num_sub_keys == 0) {
			continue;
		}

		sprintf(param_key, "coordinate_frames.%s.relative_to", frame_name);
    	char *relative_to;
    	if (bot_param_get_str(param, param_key, &relative_to) < 0) {
    		unsubscribe();
      		throw "Frame " + std::string(frame_name) + " does not have \'relative_to\' field block";
		}

		//maybe do something with history here
		sprintf(param_key, "coordinate_frames.%s.initial_transform", frame_name);
    	if (bot_param_get_num_subkeys(self->bot_param, param_key) != 2) {
    		unsubscribe();
      		throw "Frame " + std::string(frame_name) + " does not have correct nuber of field in \'initial_transform\'";
		}

		BotTrans init_trans;
    	if (bot_param_get_trans(param, param_key, &init_trans) < 0) {
    		unsubscribe();
      		throw "Could not get \'initial_transform\' for frame " + std::string(frame_name);
		}

		geometry_msgs::TransfromStamped ts;
		ts.header.frame_id = std::string(relative_to);
		ts.child_frame_id = std::string(frame_name);
		ts.transform.translation.x = init_trans.trans_vec[0];
		ts.transform.translation.y = init_trans.trans_vec[1];
		ts.transform.translation.z = init_trans.trans_vec[2];
		ts.transform.rotation.w = init_trans.rot_quat[0];
		ts.transform.rotation.x = init_trans.rot_quat[1];
		ts.transform.rotation.y = init_trans.rot_quat[2];
		ts.transform.rotation.z = init_trans.rot_quat[3];

		//don't understand what authority is
		//Actually, it looks like authority is unused in setTransform
		if (!m_BC->setTransform(ts, std::string("Constructor"), false)) {
			unsubscribe();
			throw "Cannot setTransform in constructor for frame" + std::string(frame_name);
		}

		//get update channel
		char *update_channel = NULL;
		sprintf(param_key, "coordinate_frames.%s.update_channel", frame_name);
		int pose_update_channel = 0;
		if (bot_param_get_str(param, param_key, &update_channel) < 0) {
			sprintf(param_key, "coordinate_frames.%s.pose_update_channel", frame_name);
			if (bot_param_get_str(param, param_key, &update_channel) == 0) {
				pose_update_channel = 1;
			} else {
				//maybe should add, they didn't in libbot
			}
		}

		channel_info_t CI;
		CI.frame_name = std::string(frame_name);
		CI.relative_to = std::string(relative_to);
		if (!pose_update_channel) {
			CI.transform_subscription = bot_core_rigid_transform_t_subscribe(m_lcm, update_channel, NuFrame::on_transform_update, (void *)this);
		} else {
			CI.pose_subscription = bot_core_pose_t_subscribe(m_lcm, update_channel, NuFrame::on_pose_update, (void *)this);
		}

		m_channels.insert(std::make_pair<std::string, channel_info_t>(std::string(update_channel), CI));

	}

	//this function name may have to be changed 
	m_default_update = bot_frames_update_t_subscribe(m_lcm, "BOT_FRAMES_UPDATE", NuFrame::on_frames_update, (void *)this);



}

NuFrame::~NuFrame()
{
	
	unsubscribe();
	if (m_default_update != NULL) {
		//this function name may have to be changed 
		bot_frames_update_t_unsubscribe(m_lcm, m_default_update);
	}

}


void NuFrame::unsubscribe() {
	if (m_channels != NULL) {
		std::unordered_map<std::string,channel_info_t>::iterator it = m_channels.begin();
		while (it != m_channels.end()) {
			if (it->second.transform_subscription != NULL) {
				bot_core_rigid_transform_t_unsubscribe(m_lcm, it->second.transform_subscription);
			}

			if (it->second.pose_subscription != NULL) {
				bot_core_pose_t_unsubscribe(m_lcm, it->second.pose_subscription);
			}
			it++;
		}
	}
}

static void NuFrame::on_pose_update(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_pose_t *msg, void *user_data) {
	NuFrame *nf = (NuFrame *)user_data;

	geometry_msgs::TransfromStamped ts;
	ts.transform.translation.x = msg->pos[0];
	ts.transform.translation.y = msg->pos[1];
	ts.transform.translation.z = msg->pos[2];

	ts.transform.rotation.w = msg->orientation[0];
	ts.transform.rotation.x = msg->orientation[1];
	ts.transform.rotation.y = msg->orientation[2];
	ts.transform.rotation.z = msg->orientation[3];
	std::unordered_map<std::string,channel_info_t>::const_iterator got = nf->m_channels.find(std::string(channel));
	assert(got != nf->m_channels.end());
	channel_info_t CI = got->second;
	ts.header.frame_id = CI.relative_to;
	ts.child_frame_id = CI.frame_name;
	ts.header.stamp.sec = (int)(msg->utime/1000000);
	ts.header.stamp.nsec = (int)((msg->utime % 1000000) * 1000);
	nf->m_BC->setTransform(ts, std::string("PoseUpdate"), false);


}

static void NuFrame::on_frames_update(const lcm_recv_buf_t *rbuf, const char *channel, const bot_frames_update_t *msg, void *user_data) {
	NuFrame *nf = (NuFrame *)user_data;
	geometry_msgs::TransfromStamped ts;
	ts.transform.translation.x = msg->trans[0];
	ts.transform.translation.y = msg->trans[1];
	ts.transform.translation.z = msg->trans[2];

	ts.transform.rotation.w = msg->quat[0];
	ts.transform.rotation.x = msg->quat[1];
	ts.transform.rotation.y = msg->quat[2];
	ts.transform.rotation.z = msg->quat[3];
	ts.header.frame_id = std::string(msg->relative_to);
	ts.child_frame_id = std::string(msg->frame);
	ts.header.stamp.sec = (int)(msg->utime/1000000);
	ts.header.stamp.nsec = (int)((msg->utime % 1000000) * 1000);
	nf->m_BC->setTransform(ts, std::string("FramesUpdate"), false);

}

static void NuFrame::on_transform_update(const lcm_recv_buf_t *rbuf, const char *channel, const bot_core_rigid_transform_t *msg, void *user_data) {
	NuFrame *nf = (NuFrame *)user_data;

	geometry_msgs::TransfromStamped ts;
	ts.transform.translation.x = msg->trans[0];
	ts.transform.translation.y = msg->trans[1];
	ts.transform.translation.z = msg->trans[2];

	ts.transform.rotation.w = msg->quat[0];
	ts.transform.rotation.x = msg->quat[1];
	ts.transform.rotation.y = msg->quat[2];
	ts.transform.rotation.z = msg->quat[3];
	std::unordered_map<std::string,channel_info_t>::const_iterator got = nf->m_channels.find(std::string(channel));
	assert(got != nf->m_channels.end());
	channel_info_t CI = got->second;
	ts.header.frame_id = CI.relative_to;
	ts.child_frame_id = CI.frame_name;
	ts.header.stamp.sec = (int)(msg->utime/1000000);
	ts.header.stamp.nsec = (int)((msg->utime % 1000000) * 1000);
	nf->m_BC->setTransform(ts, std::string("TransformUpdate"), false);

}