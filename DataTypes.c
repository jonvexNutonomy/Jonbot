Data Types:


//-----------------------------------------
// BotParam
//-----------------------------------------
struct _BotParam {
    BotParamElement * root;
    GMutex * lock;
    int64_t server_id;
    int64_t sequence_number;

    GList * update_callbacks;

};




struct _BotParamElement {
    BotParamType type;
    BotParamDataType data_type;
    BotParamElement * parent;
    char * name;
    BotParamElement * next;
    BotParamElement * children;
    int num_values;
    char ** values;
};





typedef enum {
    BotParamContainer, BotParamArray
} BotParamType;





typedef enum {
    BotParamDataString, BotParamDataInt, BotParamDataBool, BotParamDataDouble
} BotParamDataType;


//-----------------------------------------
// BotFrames
//-----------------------------------------


struct _BotFrames {
  BotCTrans * ctrans;
  lcm_t *lcm;
  BotParam *bot_param;

  GMutex * mutex;
  int num_frames;
  char * root_name;
  GHashTable* frame_handles_by_name;
  GHashTable* frame_handles_by_channel;

  bot_frames_update_t_subscription_t * update_subscription;
  GList * update_callbacks;

};


typedef struct {
  int frame_num;
  char * frame_name;
  char * relative_to;
  char * update_channel;
  bot_core_rigid_transform_t_subscription_t * transform_subscription;
  bot_core_pose_t_subscription_t * pose_subscription;

  BotCTransLink * ctrans_link;
  int was_updated;
} frame_handle_t;



typedef struct {
    int64_t utime;
    BotTrans trans;
} TimestampedTrans;

//-----------------------------------------
// BotCTrans
//-----------------------------------------

struct _BotCTrans
{
    GHashTable * frames;

    GHashTable * links;

    GHashTable * path_cache;
};

struct _BotCTransLink
{
    BotCTransFrame *frame_from;
    BotCTransFrame *frame_to;
    char * id;
    int history_maxlen;

    BotTrans static_trans;
    BotCircular * trans_history;
};


typedef struct _BotCTransFrame BotCTransFrame;
struct _BotCTransFrame
{
    char *id;
    GPtrArray * links;
};


struct _BotCTransPath {
    int nlinks;
    BotCTransLink ** links;
    int *invert;
};


//---------------------------------------------------




typedef struct {
  bot_frames_link_update_handler_t * callback_func;
  void * user;
} update_handler_t;




typedef struct _BotTrans BotTrans;
struct _BotTrans
{
    double rot_quat[4];
    double trans_vec[3];
};

//----------------------------------------------
// Update stuff

struct update_t
{
    int64_t utime;              // utime that the actual measurement took place
    
    string frame;               // name of the frame to update
    string relative_to;         // frame that the update is relative to
    
    double trans[3];            // translation vector component (x,y,z)
    double quat[4];             // rotation quaternion component (w,x,y,z)

                               
}

struct rigid_transform_t
{
    int64_t utime;              // utime that the actual measurement took place

    double trans[3];            // translation vector component (x,y,z)

    double quat[4];      		// rotation quaternion component (w,x,y,z)

                               
}


struct pose_t
{
    int64_t utime;              // utime that the actual measurement took place

    double pos[3];              // position x,y,z in meters in local frame
    double vel[3];              // velocity in m/s, expressed in local frame,
                                // _NOT_ body frame

    double orientation[4];      // rotate vector in body coordinate frame 
                                // (X-forward, Z-up) by this to get that vector
                                // in local frame

    double rotation_rate[3];    // angular velocity vector of the vehicle
                                // in rad/s.  This is expressed in the local
                                // frame.

    double accel[3];            // Acceleration vector in the local frame in
                                // m/s/s.  Does not include the gravity vector.
}

