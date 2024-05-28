#include "station_control_node.h"
#include "agiprobot_msgs/Database.h"
#include "ros/service_server.h"

//-------------------------------------------------------------------------------------
// global variables
//-------------------------------------------------------------------------------------
std::string g_node_name;

/* data structures */
//! shared database
Database g_db;
//! shared FSM
FSM g_fsm;

/* user input management */
JoyInput g_joy_input;
KeyInput g_key_input;

/* rtde robot control */
//! holding link of TCP pose
std::vector<double> g_tcp_pose                = g_db.fixed_tcp_poses.at(STATE::STANDBY);
std::vector<double> g_optimal_inspection_pose = g_db.fixed_tcp_poses.at(STATE::INSPECT);
//! counter for tcp update requests
int g_tcp_update_counter = 0;

/* pheripheral hardware management */
//! the tool going to be mounted
TOOL g_target_tool = TOOL::NO_TOOL;
//! the actual mounted tool
TOOL g_current_tool = TOOL::NO_TOOL;
//! actual status of gripper
GRIPPER g_gripper_state = GRIPPER::SLEEP;

/* rgb-d image processing */
//! filtered depth map
cv::Mat g_filtered_depth_map = cv::Mat::zeros(480, 848, CV_16UC1);
//! rgb image
cv::Mat g_rgb_image = cv::Mat::zeros(480, 848, CV_8UC3);
//! tcp coordinate in image
cv::Point g_tcp_coordinate(414, 90); // calibrate here
//! tcp sample point coordinate in image
cv::Point g_tcp_sample_point(414, 100);//(418, 128); (425,92)
//! tcp ROI radius in [pixel]
int g_tcp_roi_radius = 4;
//! actual tcp distance to object in meter
double g_tcp_depth = 0;

/* screw depth */
//! average depth of all detected screws
double g_shrink_factor       = 1.0;
double g_average_screw_depth = g_db.tcp_to_camera_dist;
double g_max_screw_depth     = 0.0;
double g_min_screw_depth     = 0.0;
int g_average_screw_x        = 0;
int g_average_screw_y        = 0;

/* final engament factors */
double g_tcp_to_screw_dist = 0;
double g_scale_factor_x    = 0.0;
double g_scale_factor_y    = 0.0;

/* screw scheduling */
//! max screw number detected
int g_max_screw_num = 0;
Screw_list g_current_screw_list;
//! acutal yolo detected screws and their meta data
std::vector<Screw> g_screw_list_now;
//! last step yolo detected screws and their meta data
std::vector<Screw> g_screw_list_last;
//! actual target screw i.e. the 1st screw in screw list
Screw g_target_screw;
SCREW_TYPE g_target_screw_type = SCREW_TYPE::HEXAGONAL_BOLT;


//-------------------------------------------------------------------------------------
// control flags
//-------------------------------------------------------------------------------------
bool g_autonomous_mode_on        = false;
bool g_enable_moveit             = false;
bool g_joy_updated               = false;
bool g_key_updated               = false;
bool g_communication_established = false;
bool g_rgb_updated               = false;
bool g_screws_focused            = false;
bool g_screw_list_updated        = false;
bool g_start_adjust_height       = false;
bool g_start_focus_screws        = false;
bool g_start_focus_target_screw  = false;
bool g_inspection_done           = false;
bool g_new_motor_placed          = false;

//-------------------------------------------------------------------------------------
// service servers
//-------------------------------------------------------------------------------------
ros::ServiceServer g_motor_server;
ros::ServiceServer g_fsm_server;
ros::ServiceServer g_tool_server;

//-------------------------------------------------------------------------------------
// service clients
//-------------------------------------------------------------------------------------
ros::ServiceClient g_pick_tool_client;
ros::ServiceClient g_drop_tool_client;
ros::ServiceClient g_move_to_client;
ros::ServiceClient g_unscrew_client;
ros::ServiceClient g_fetch_tcp_pose_client;
ros::ServiceClient g_push_tcp_pose_client;
ros::ServiceClient g_teachMode_client;
ros::ServiceClient g_gripper_initial;
ros::ServiceClient g_gripper_execute;
ros::ServiceClient g_moveit_client;
ros::ServiceClient g_yolo_client;
ros::ServiceClient g_screw_alignment_client;

//-------------------------------------------------------------------------------------
// ros publishers
//-------------------------------------------------------------------------------------
ros::Publisher g_fsm_state_pub;

//-------------------------------------------------------------------------------------
// main process
//-------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  //--------------------------------------------------------------------
  // node Initialization and Startup.
  //--------------------------------------------------------------------
  ros::init(argc, argv, "ur10e_disassembly_station_control_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  //--------------------------------------------------------------------
  // read ros parameters defined in launch file
  //--------------------------------------------------------------------
  g_node_name = ros::this_node::getName();
  //! ros loop frequency
  int node_frequency;
  priv_nh.getParam("frequency", node_frequency);

  //! distance threshhold from tcp to screw
  priv_nh.getParam("tcp_dist", g_tcp_to_screw_dist);

  //! whether to show the image debug stream
  bool show_debug_stream;
  priv_nh.getParam("show_debug_stream", show_debug_stream);

  //! whether to start PID for height/f
  priv_nh.getParam("start_with_PID_z", g_start_adjust_height);
  priv_nh.getParam("start_with_PID_xy", g_start_focus_screws);

  //! starting state at launch
  std::string start_state;
  priv_nh.getParam("start_state", start_state);
  STATE starting_state = g_db.convert_string_to_state(start_state);

  //! whether to use moveit
  priv_nh.getParam("enable_moveit", g_enable_moveit);

  //! whether to use full automatic mode
  priv_nh.getParam("autonomous_mode", g_autonomous_mode_on);

  priv_nh.getParam("scale_factor_x", g_scale_factor_x);
  priv_nh.getParam("scale_factor_y", g_scale_factor_y);

  priv_nh.getParam("shrink_factor", g_shrink_factor);

  ROS_INFO("\033[32m (%s): "
           "\n=============================================================================\n  "
           "STARTED CENTRAL CONTROL NODE "
           "FOR THE UR10E ROBOT DISASSEMBLE STATION "
           "\n=============================================================================\n "
           "FREQUENCY: "
           "%d \033[0m",
           g_node_name.c_str(),
           node_frequency);

  //-------------------------------------------------------------------------------------
  // ROS serivce server
  //-------------------------------------------------------------------------------------
  // clang-format off
  g_motor_server          = nh.advertiseService("motor_placed", motorPlacedCb);
  g_fsm_server            = nh.advertiseService("trigger_fsm_to", fsmCb);
  g_tool_server           = nh.advertiseService("set_target_tool", toolCb);
  //-------------------------------------------------------------------------------------
  // ROS service client
  //-------------------------------------------------------------------------------------
  g_pick_tool_client      = nh.serviceClient<agiprobot_msgs::tool>("pickTool");
  g_drop_tool_client      = nh.serviceClient<agiprobot_msgs::tool>("dropTool");
  g_unscrew_client        = nh.serviceClient<agiprobot_msgs::unscrew>("unscrew");
  g_fetch_tcp_pose_client = nh.serviceClient<agiprobot_msgs::tcp>("fetchTCPPose");
  g_push_tcp_pose_client  = nh.serviceClient<agiprobot_msgs::tcp>("pushTCPPose");
  g_teachMode_client      = nh.serviceClient<agiprobot_msgs::teachMode>("teachMode");
  g_gripper_initial       = nh.serviceClient<screw_gripper::Initiate>("/gripper_node/initiate");
  g_gripper_execute       = nh.serviceClient<screw_gripper::Execute>("/gripper_node/execute");
  g_moveit_client         = nh.serviceClient<agiprobot_msgs::goal>("moveItTo");
  g_yolo_client           = nh.serviceClient<agiprobot_msgs::detection>("detect_screws");
  g_screw_alignment_client = nh.serviceClient<agiprobot_msgs::align>("screw_alignment");
  //--------------------------------------------------------------------
  // ROS topic subscribers
  //--------------------------------------------------------------------
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, joyCommandCb);
  ros::Subscriber key_sub = nh.subscribe<agiprobot_msgs::Keyboard>("keyboard_state", 1, keyCommandCb);
  ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 1, depthImageCb);
  ros::Subscriber rgb_sub = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, rgbImageCb);

  //-------------------------------------------------------------------------------------
  // ROS topic publishers
  //-------------------------------------------------------------------------------------
  g_fsm_state_pub = nh.advertise<std_msgs::Int8>("FSM_state", 1);

  // clang-format on
  //-------------------------------------------------------------------------------------
  // wait 3s for OPCUA to boot up, then initialize gripper
  //-------------------------------------------------------------------------------------
  ros::Duration(3.0).sleep();
  screw_gripper::Initiate init;
  g_gripper_initial.call(init);
  g_gripper_state = GRIPPER::UP;

  //-------------------------------------------------------------------------------------
  // initialize ROS loop parameters
  //-------------------------------------------------------------------------------------
  ros::Rate loop_rate(node_frequency);
  int loop_counter  = 0;
  int error_counter = 0;
  ros::Time().useSystemTime();
  auto current_time = ros::Time().now();
  auto last_time    = ros::Time().now();

  //-------------------------------------------------------------------------------------
  // initialize CV visulization windows
  //-------------------------------------------------------------------------------------
  if (show_debug_stream)
  {
    cv::namedWindow("RGB with Screws", cv::WINDOW_NORMAL);
    cv::namedWindow("Aligned & filtered Depth", cv::WINDOW_NORMAL);
  }
  //-------------------------------------------------------------------------------------
  // initialize PID controllers
  //-------------------------------------------------------------------------------------
  //! PID controller in X-Y plane (pixel coordinate) targeting to tcp
  PIDController PID_tcp_x(0.0002, 0.0000005, 0.0000005, g_tcp_sample_point.x, 2);
  PIDController PID_tcp_y(0.0002, 0.0000005, 0.0000005, g_tcp_sample_point.y + 20 , 2);
  //! PID controller in Z distance (meter) targeting to tcp
  PIDController PID_tcp_z(
    0.3, 0.0001, 0.0001, g_db.tcp_to_camera_dist + g_tcp_to_screw_dist, 0.002);

  //! PID controller in X-Y plane targeting to view center, used when focus screws
  PIDController PID_center_x(0.0001, 0.0000005, 0.0000005, 424, 1); // X
  PIDController PID_center_y(0.0001, 0.0000005, 0.0000005, 230, 1); // Y

  //-------------------------------------------------------------------------------------
  // start main ROS loop, start communicating with server
  //-------------------------------------------------------------------------------------
  while (ros::ok())
  {
    //-------------------------------------------------------------------------------------
    // measure actual node frequency
    //-------------------------------------------------------------------------------------
    current_time            = ros::Time().now();
    auto time_diff          = current_time - last_time;
    double actual_frequency = 1000000000.0 / time_diff.nsec;
    ROS_INFO_STREAM(g_node_name << "\033[32m : EXPECTED: " << node_frequency
                                << " HZ, ACTUALLY RUNS AT: " << actual_frequency << " HZ. \033[0m");

    ROS_ERROR_STREAM("TARGET TOOL: " << static_cast<int>(g_target_tool)<< ", CURRENT TOOL: " << static_cast<int>(g_current_tool));
    last_time = current_time;
    ROS_WARN_STREAM("TCP: " << g_tcp_pose[0] << ", " << g_tcp_pose[1] << ", " <<g_tcp_pose[2] <<", " << g_tcp_pose[3] <<", " << g_tcp_pose[4] <<", " <<g_tcp_pose[5]);

    //-------------------------------------------------------------------------------------
    // show realtime debug stream
    //-------------------------------------------------------------------------------------
    if (show_debug_stream)
      output_debug_image_stream();

    //engage_screw_head();
    
    
    
    //-------------------------------------------------------------------------------------
    // initailize the communication with UR robot via a handshake
    //-------------------------------------------------------------------------------------
    if (!g_communication_established) // only do it once at starting
    {
      //-------------------------------------------------------------------------------------
      // let's check if we can fetch the actual robot state
      //-------------------------------------------------------------------------------------
      
      if (fetch_tcp_pose())
      {
        //-------------------------------------------------------------------------------------
        // trigger FSM to starting state
        //-------------------------------------------------------------------------------------
        trigger_fsm_to(starting_state);

        //-------------------------------------------------------------------------------------
        // welcome message
        //-------------------------------------------------------------------------------------
        ROS_INFO("\033[32m SUCCEESFULLY INTIALIZED UR10E DISASSEMBLY STATIOIN, NOW YOU CAN PLAY"
                 "WIHT IT IN THE MOVEIT GUI:) \033[0m");
        ROS_INFO("\033[32m TO START AUTOMATIC DISASSEMBLY, MOUNT THE OBJECT WELL AND PRESS "
                 "space+enter \033[0m");

        //-------------------------------------------------------------------------------------
        // refresh error counter, set flag
        //-------------------------------------------------------------------------------------
       
        error_counter               = 0;
        g_communication_established = true;
        bool unscrewed = engage_screw_head();
        if (unscrewed) {
          trigger_fsm_to(STATE::GRASP);
        }
        
        kill_all_nodes();
      }
      else
      {
        //-------------------------------------------------------------------------------------
        // means we can neither hear or control the robot, need retry
        //-------------------------------------------------------------------------------------
        ROS_ERROR_STREAM(g_node_name
                         << ": WAITING FOR SERVER RESPOND TO START FURTHER FUNCTION...");

        //-------------------------------------------------------------------------------------
        // let's try a maximal number of times, if still not working, try reboot the whole pipeline
        //-------------------------------------------------------------------------------------
        error_counter++;
        if (error_counter == 10000)
        {
          ROS_ERROR_STREAM(g_node_name << " : TIMEOUT, STARTING OVER THE SYSTEM...");
          kill_all_nodes();
        }

        //-------------------------------------------------------------------------------------
        // do the ROS synchro, then directly retry this handshake
        //-------------------------------------------------------------------------------------
        ros::spinOnce();
        loop_rate.sleep();
        continue;
      }
    }

    //-------------------------------------------------------------------------------------
    // handle gamepad and keyboard input
    //-------------------------------------------------------------------------------------
    if (g_joy_updated)
    {
      handle_joy_input();
    }

    if (g_key_updated)
    {
      handle_key_input();
    }

    //-------------------------------------------------------------------------------------
    // do visual servoing based on different control flags
    //-------------------------------------------------------------------------------------
    // for focusing screws
    if (g_start_focus_screws && g_screw_list_updated)
    {
      double output_x = PID_center_x.update(g_average_screw_x, time_diff.nsec / 1000000000.0, 0.1);
      double output_y = PID_center_y.update(g_average_screw_y, time_diff.nsec / 1000000000.0, 0.1);

      // reflect PID ouput to tcp pose XY
      if (output_x == 0.0 && output_y == 0.0)
      {
        ROS_INFO_STREAM(g_node_name << "\033[32m : focused screws. \033[m");
       

        //-------------------------------------------------------------------------------------
        // transit to height adjustment
        //-------------------------------------------------------------------------------------
        g_start_focus_screws  = false;
        g_start_adjust_height = true;
      }
      else if (output_x == 0.0)
      {
        ROS_INFO_STREAM(g_node_name << "\033[32m : reached target X point. \033[m");
        ROS_WARN_STREAM(g_node_name << " : running PID for Y.");
        g_tcp_pose[0] -= output_y;
      }
      else if (output_y == 0.0)
      {
        ROS_INFO_STREAM(g_node_name << "\033[32m : reached target Y point. \033[m");
        ROS_WARN_STREAM(g_node_name << " : running PID for X. ");
        g_tcp_pose[1] -= output_x;
      }
      else
      {
        ROS_WARN_STREAM(g_node_name << " : running PID for X and Y. ");
        g_tcp_pose[1] -= output_x;
        g_tcp_pose[0] -= output_y;
      }
    }

    // for z direction servoing
    if (g_start_adjust_height)
    {
      ros::spinOnce();
      ROS_WARN_STREAM(g_target_screw.depth);
      double output_z = PID_tcp_z.update(
        g_max_screw_depth, time_diff.nsec / 1000000000.0, 0.1); // g_target_screw.depth
      if (output_z == 0.0)
      {
        ROS_INFO_STREAM(g_node_name << "\033[32m : height OK. \033[m");
        g_start_adjust_height = false;
        // save the starting point locally
        g_optimal_inspection_pose = g_tcp_pose;
        // from now on, we're ready to start unscrew process
        g_inspection_done = true;

        // proceed to visual servoing
        if (g_autonomous_mode_on)
          trigger_fsm_to(STATE::SERVO);
      }
      else
      {
        ROS_WARN_STREAM(g_node_name << " : running PID for Z. ");
        g_tcp_pose[2] += output_z;
      }
    }

    // for focusing target screw
    if (g_start_focus_target_screw && g_screw_list_updated)
    {
      //-------------------------------------------------------------------------------------
      // keep the adjusted height unvariant
      //-------------------------------------------------------------------------------------
      g_tcp_pose[2] = g_optimal_inspection_pose[2];

      //-------------------------------------------------------------------------------------
      // calculate PID XY
      //-------------------------------------------------------------------------------------
      double output_x = PID_tcp_x.update(g_target_screw.x, time_diff.nsec / 1000000000.0, 0.1);
      double output_y = PID_tcp_y.update(g_target_screw.y, time_diff.nsec / 1000000000.0, 0.1);

      // reflect PID ouput to tcp pose XY
      if (output_x == 0.0 && output_y == 0.0)
      {
        ROS_INFO_STREAM(g_node_name << "\033[32m : focused target screw. \033[m");
       

        //-------------------------------------------------------------------------------------
        // exit visual servoing
        //-------------------------------------------------------------------------------------
        g_start_focus_target_screw = false;
        g_start_adjust_height      = false;
        g_start_focus_screws       = false;

        //-------------------------------------------------------------------------------------
        // final alignment based on x-y difference and height difference
        //-------------------------------------------------------------------------------------
        engage_screw_head();

        //-------------------------------------------------------------------------------------
        // proceed to unscrew
        //-------------------------------------------------------------------------------------
        if (g_autonomous_mode_on)
          trigger_fsm_to(STATE::UNSCREW);
  
      }
      else if (output_x == 0.0)
      {
        ROS_INFO_STREAM(g_node_name << "\033[32m : reached target X point. \033[m");
        ROS_WARN_STREAM(g_node_name << " : running PID for Y.");
        g_tcp_pose[0] -= output_y;
      }
      else if (output_y == 0.0)
      {
        ROS_INFO_STREAM(g_node_name << "\033[32m : reached target Y point. \033[m");
        ROS_WARN_STREAM(g_node_name << " : running PID for X. ");
        g_tcp_pose[1] -= output_x;
      }
      else
      {
        ROS_WARN_STREAM(g_node_name << " : running PID for X and Y. ");
        g_tcp_pose[1] -= output_x;
        g_tcp_pose[0] -= output_y;
      }
    }

    //-------------------------------------------------------------------------------------
    // advertise the current FSM state, so that other sub-systems can sync
    //-------------------------------------------------------------------------------------
    publish_fsm_state();

    //-------------------------------------------------------------------------------------
    // holding link synchronization of TCP pose
    //-------------------------------------------------------------------------------------
    auto_set_heading();
    push_tcp_pose();

    //-------------------------------------------------------------------------------------
    // regular ROS synchronization
    //-------------------------------------------------------------------------------------
    loop_counter++;
    ros::spinOnce();
    loop_rate.sleep();
    
  }
  
  //-------------------------------------------------------------------------------------
  // if the node is closed by user, we stop other running part too, since it is the central unit
  //-------------------------------------------------------------------------------------
  ROS_INFO_STREAM("\033[32m SYSTEM SHUTED DOWN BY USER AFTER RUNNED"
                  << loop_counter << "LOOPS , GOODBYE:) \033[0m");
  cv::destroyAllWindows();
  kill_all_nodes();
  return 0;
}

//-------------------------------------------------------------------------------------
// ROS topic callbacks
//-------------------------------------------------------------------------------------
void joyCommandCb(const sensor_msgs::Joy::ConstPtr& msg)
{
  //--------------------------------------------------------------------
  // read in all controller inputs
  //--------------------------------------------------------------------
  g_joy_input.left_joy_axes_x  = msg->axes[0]; // 1->left, -1->right
  g_joy_input.left_joy_axes_y  = msg->axes[1]; // 1->up, -1->down
  g_joy_input.left_trigger     = msg->axes[2]; //-1->max, 1->min
  g_joy_input.right_joy_axes_x = msg->axes[3];
  g_joy_input.right_joy_axes_y = msg->axes[4];
  g_joy_input.right_trigger    = msg->axes[5]; //-1->max, 1->min
  g_joy_input.button_axes_x    = msg->axes[6];
  g_joy_input.button_axes_y    = msg->axes[7];
  g_joy_input.a_button         = msg->buttons[0]; // 1->pressing, 0->release
  g_joy_input.b_button         = msg->buttons[1];
  g_joy_input.x_button         = msg->buttons[2];
  g_joy_input.y_button         = msg->buttons[3];
  g_joy_input.left_button      = msg->buttons[4];
  g_joy_input.right_button     = msg->buttons[5];
  g_joy_input.window           = msg->buttons[6];
  g_joy_input.menu             = msg->buttons[7];
  g_joy_input.xbox             = msg->buttons[8];
  g_joy_input.left_joy_button  = msg->buttons[9];
  g_joy_input.right_joy_button = msg->buttons[10];

  //--------------------------------------------------------------------
  // raise request to be handled
  //--------------------------------------------------------------------
  g_joy_updated = true;
}

void keyCommandCb(const agiprobot_msgs::Keyboard::ConstPtr& msg)
{
  //--------------------------------------------------------------------
  // read in all keyboard inputs
  //--------------------------------------------------------------------
  g_key_input.w           = msg->w;
  g_key_input.a           = msg->a;
  g_key_input.s           = msg->s;
  g_key_input.d           = msg->d;
  g_key_input.arrow_up    = msg->arrow_up;
  g_key_input.arrow_down  = msg->arrow_down;
  g_key_input.arrow_left  = msg->arrow_left;
  g_key_input.arrow_right = msg->arrow_right;
  g_key_input.space       = msg->space;
  g_key_input.enter       = msg->enter;
  g_key_input.left_ctrl   = msg->left_ctrl;
  g_key_input.home        = msg->home;
  g_key_input.escape      = msg->escape;

  //--------------------------------------------------------------------
  // raise request to be handled
  //--------------------------------------------------------------------
  g_key_updated = true;
}

void depthImageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  //-------------------------------------------------------------------------------------
  // first, read meta data
  //-------------------------------------------------------------------------------------
  int index = msg->header.seq;
  ROS_INFO_STREAM(g_node_name << ": got depth frame with id: " << index);

  //-------------------------------------------------------------------------------------
  // then, convert the depth image message to a cv::Mat
  //-------------------------------------------------------------------------------------
  cv_bridge::CvImageConstPtr depth_frame_ptr;
  try
  {
    depth_frame_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //! unfiltered depth map
  cv::Mat unfiltered_depth_map = cv::Mat::zeros(480, 848, CV_16UC1);

  // convert mm to m
  depth_frame_ptr->image.convertTo(unfiltered_depth_map, CV_32FC1, 0.001f);
  //-------------------------------------------------------------------------------------
  // now we have the depth map which is aligned to RGB frame, let's do some filtering to remove salt
  // & pepper noise
  //-------------------------------------------------------------------------------------
  // clean up the current depth map
  g_filtered_depth_map.release();

  // remove noise
  cv::Mat filtered_depth;
  cv::medianBlur(unfiltered_depth_map, filtered_depth, 5);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(filtered_depth, filtered_depth, cv::MORPH_OPEN, kernel);

  // value smaller than 0.1[m] are neglect and set to 0
  cv::threshold(filtered_depth, g_filtered_depth_map, 0.1, 255, cv::THRESH_TOZERO);

  //-------------------------------------------------------------------------------------
  // read the tcp depth in actual depth matrix and store it globally
  //-------------------------------------------------------------------------------------
  int y_min = g_tcp_sample_point.y - g_tcp_roi_radius;
  int y_max = g_tcp_sample_point.y + g_tcp_roi_radius;
  int x_min = g_tcp_sample_point.x - g_tcp_roi_radius;
  int x_max = g_tcp_sample_point.x + g_tcp_roi_radius;

  // sample point below tcp
  if (y_min < 0)
    y_min = 0;
  if (y_min > g_filtered_depth_map.rows - 1)
    y_min = g_filtered_depth_map.rows - 1;
  if (x_min < 0)
    x_min = 0;
  if (x_min > g_filtered_depth_map.cols)
    x_min = g_filtered_depth_map.cols - 1;

  cv::Mat tcp_roi      = g_filtered_depth_map(cv::Range(y_min, y_max), cv::Range(x_min, x_max));
  cv::Scalar tcp_depth = cv::mean(tcp_roi);
  g_tcp_depth          = tcp_depth[0];
  tcp_roi.release(); // clean up
}

void rgbImageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  //-------------------------------------------------------------------------------------
  // first, receive image and convert to cv::Mat
  //-------------------------------------------------------------------------------------
  int index = msg->header.seq;
  ROS_INFO_STREAM(g_node_name << ": got RGB frame with id: " << index);
  g_rgb_image.release();
  try
  {
    g_rgb_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    ROS_INFO_STREAM(g_node_name << ": actual image input size: " << g_rgb_image.cols << " , "
                                << g_rgb_image.rows);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //-------------------------------------------------------------------------------------
  // then, call up yolo detector to detect the screws in this image frame
  //-------------------------------------------------------------------------------------
  agiprobot_msgs::detection detection;
  detection.request.rgb_frame = *msg;
  g_yolo_client.call(detection);
  g_screw_list_updated = false;

  if (detection.response.result.bounding_boxes.empty()) // if we detected no screw
  {
    g_screw_list_last = g_screw_list_now;
    g_screw_list_now.clear(); // no screws
  }
  else // if we detected screws, handle and store them in screw list
  {
    // step forward
    g_screw_list_last = g_screw_list_now;
    g_screw_list_now.clear();

    // get number of screws
    int number_of_screws = detection.response.result.bounding_boxes.size();

    if (number_of_screws > g_screw_list_last.size())
    {
      ROS_INFO_STREAM(
        g_node_name << "\033[32m : detected " << number_of_screws - g_screw_list_last.size()
                    << " new screws, now in total " << number_of_screws << " screws. \033[0m");
    }

    if (number_of_screws > g_max_screw_num)
    {
      g_max_screw_num = number_of_screws;
      ROS_INFO_STREAM(g_node_name << "\033[32m :  max screw number updated. \033[0m");
    }
    else
    {
      ROS_INFO_STREAM(g_node_name << "\033[32m : left " << number_of_screws << " screws. \033[0m");
    }

    //-------------------------------------------------------------------------------------
    // now handle the screws one by one, determine their meta datas from the database and push
    // them into screw list
    //-------------------------------------------------------------------------------------
    for (auto bounding_box : detection.response.result.bounding_boxes)
    {
      // get screw informations
      Screw current_screw;
      // center point of screw head
      current_screw.x      = (bounding_box.xmax + bounding_box.xmin) / 2;
      current_screw.y      = (bounding_box.ymax + bounding_box.ymin) / 2;
      current_screw.radius = std::sqrt(std::pow(bounding_box.xmax - bounding_box.xmin, 2) +
                                       std::pow(bounding_box.ymax - bounding_box.ymin, 2)) /
                             2 * g_shrink_factor;
      // get screw type
      current_screw.type = g_db.convert_string_to_screw_type(bounding_box.Class);
      // get from database the tool needed for this screw
      current_screw.tool = g_db.check_tool_type(current_screw);
      // get from database the unscrew loops needed for weber controller
      current_screw.unscrew_loops = g_db.check_unscrew_loops(current_screw);
      // get from database the unscrew torque needed for weber controller (to be impl.)
      // current_screw.torque = g_db.check_torque(current_screw);

      // read in depth map the screw distance
      int y_min = current_screw.y - current_screw.radius;
      int y_max = current_screw.y + current_screw.radius;
      int x_min = current_screw.x - current_screw.radius;
      int x_max = current_screw.x + current_screw.radius;

      if (y_min < 0)
        y_min = 0;
      if (y_min > g_rgb_image.rows - 1)
        y_min = g_rgb_image.rows - 1;
      if (x_min < 0)
        x_min = 0;
      if (x_min > g_rgb_image.cols)
        x_min = g_rgb_image.cols - 1;

      cv::Mat screw_roi = g_filtered_depth_map(cv::Range(y_min, y_max), cv::Range(x_min, x_max));
      cv::Scalar screw_depth = cv::mean(screw_roi);

      current_screw.depth = screw_depth[0];
      screw_roi.release(); // free up

      // push to screw list
      g_screw_list_now.push_back(current_screw);
      g_current_screw_list.screws.push_back(current_screw);
    }
  }

  //-------------------------------------------------------------------------------------
  // detect the printed markers points, add to screw list (test & debug purpose)
  //-------------------------------------------------------------------------------------
  // detect_markers();

  //-------------------------------------------------------------------------------------
  // handle screw list
  //-------------------------------------------------------------------------------------
  if (!g_screw_list_now.empty())
  {
    //-------------------------------------------------------------------------------------
    // sort the screw list based on its X, Y position
    //-------------------------------------------------------------------------------------
    sortScrews(g_screw_list_now);

    // create sub screw list for current interested screw type
    std::vector<Screw> sub_screw_list;
    for (auto screw : g_screw_list_now)
    {
      if (screw.type == g_target_screw_type)
      {
        sub_screw_list.push_back(screw);
      }
    }

    //-------------------------------------------------------------------------------------
    // as we sorted the screw list, we can pop the first one as the target screw everytime
    //-------------------------------------------------------------------------------------
    if (!sub_screw_list.empty())
      g_target_screw = sub_screw_list.front();

    //-------------------------------------------------------------------------------------
    // calculate average depth and coordinate of detected screws
    //-------------------------------------------------------------------------------------
    g_average_screw_depth  = 0.0;
    double min_screw_depth = 50;
    double max_screw_depth = 0;
    g_average_screw_x      = 0.0;
    g_average_screw_y      = 0.0;

    for (auto screw : g_screw_list_now)
    {
      g_average_screw_depth += screw.depth;
      g_average_screw_x += screw.x;
      g_average_screw_y += screw.y;

      if (screw.depth > max_screw_depth)
        max_screw_depth = screw.depth;
      if (screw.depth < min_screw_depth)
        min_screw_depth = screw.depth;
    }
    g_average_screw_depth = g_average_screw_depth / g_screw_list_now.size();
    g_average_screw_x     = g_average_screw_x / g_screw_list_now.size();
    g_average_screw_y     = g_average_screw_y / g_screw_list_now.size();

    g_max_screw_depth = max_screw_depth;
    g_min_screw_depth = min_screw_depth;

    //-------------------------------------------------------------------------------------
    // tell the system that screw list is updated
    //-------------------------------------------------------------------------------------
    g_screw_list_updated = true;
  }
}


//-------------------------------------------------------------------------------------
// function implementations
//-------------------------------------------------------------------------------------
bool execute_gripper()
{
  if (g_gripper_state == GRIPPER::ERROR || g_gripper_state == GRIPPER::SLEEP)
  {
    ROS_ERROR_STREAM(g_node_name << ": Gripper not available, check it up.");
  }
  else
  {
    screw_gripper::Execute execute;
    g_gripper_execute.call(execute);
    if (execute.response.success)
    {
      if (g_gripper_state == GRIPPER::UP)
      {
        g_gripper_state = GRIPPER::DOWN;
        ROS_WARN_STREAM(g_node_name << ": Finished execute gripper, currently it is down.");
        return true;
      }
      else if (g_gripper_state == GRIPPER::DOWN)
      {
        g_gripper_state = GRIPPER::UP;
        ROS_WARN_STREAM(g_node_name << ": Finished execute gripper, currently it is up.");
        return true;
      }
      else
      {
        g_gripper_state = GRIPPER::ERROR;
        return false;
      }
    }
  }
  return false;
}

bool fetch_tcp_pose()
{
  agiprobot_msgs::tcp fetch_tcp;
  g_fetch_tcp_pose_client.call(fetch_tcp);
  if (fetch_tcp.response.success)
  {
    g_tcp_pose = fetch_tcp.response.pose;
    ROS_WARN_STREAM(g_tcp_pose[0] << " " << g_tcp_pose[1] << " " << g_tcp_pose[2] << "\n"
                                  << g_tcp_pose[3] << " " << g_tcp_pose[4] << " " << g_tcp_pose[5]);
  }
  else
  {
    ROS_ERROR_STREAM(g_node_name << ": FAILED FETCHING TCP POSE, SERVER NOT RESPONDING...");
  }
  return fetch_tcp.response.success;
}

bool push_tcp_pose()
{
  //-------------------------------------------------------------------------------------
  // send g_tcp_pose to robot
  //-------------------------------------------------------------------------------------
  g_tcp_update_counter++;
  agiprobot_msgs::tcp tcp;
  tcp.request.pose = g_tcp_pose;
  g_push_tcp_pose_client.call(tcp);
  if (tcp.response.success)
  {
    //-------------------------------------------------------------------------------------
    // update g_tcp_pose to current value read from robot
    //-------------------------------------------------------------------------------------
    g_tcp_pose = tcp.response.pose;
    ROS_INFO_STREAM(g_node_name << ": POSE UPDATE NR. " << g_tcp_update_counter
                                << " SUCCESS, SYSTEM RUNING WELL...");
  }
  else
  {
    ROS_ERROR_STREAM(g_node_name << ": POSE UPDATE NR. " << g_tcp_update_counter << " FAILED.");
  }
  return tcp.response.success;
}

bool lift_tcp(double dist)
{
  g_tcp_pose[2] += dist;
  return push_tcp_pose();
}

void circular_inspect(double center_x, double center_y, double radius, int step)
{
  //-------------------------------------------------------------------------------------
  // circle path with 10 step
  //-------------------------------------------------------------------------------------
  ROS_WARN_STREAM("starting circular inspection");
  for (int i = 0; i < step; i++)
  {
    double angle  = i * M_PI * 2 / step;
    g_tcp_pose[0] = center_x + radius * cos(angle);
    g_tcp_pose[1] = center_y + radius * sin(angle);
    //-------------------------------------------------------------------------------------
    // execute the step
    //-------------------------------------------------------------------------------------
    push_tcp_pose();
    //-------------------------------------------------------------------------------------
    // check up and responde to new incoming ros messages
    //-------------------------------------------------------------------------------------
    ros::spinOnce();
    if (g_screw_list_updated)
    {
      ROS_INFO_STREAM("\033[32m found screw, exiting circular inspect... \033[0m");
      return;
    }
  }
  //-------------------------------------------------------------------------------------
  // back to circle center if there's no finding
  //-------------------------------------------------------------------------------------
  g_tcp_pose[0] = center_x;
  g_tcp_pose[1] = center_y;
  push_tcp_pose();
}


void handle_tool_request()
{
  agiprobot_msgs::tool tool;
  tool.request.fromSlot = static_cast<int8_t>(g_target_tool);
  switch (g_fsm.get_current_state())
  {
    case STATE::PICK_TOOL:
      g_current_tool = g_target_tool;
      g_pick_tool_client.call(tool);

      break;
    case STATE::DROP_TOOL:
      g_current_tool = TOOL::NO_TOOL;
      g_drop_tool_client.call(tool);

      break;
    default:
      break;
  }
  if (!tool.response.success)
  {
    ROS_ERROR_STREAM(g_node_name << " : Toolchange failed.");
  }
}

void moveit_to(STATE target_scene)
{
  //-------------------------------------------------------------------------------------
  // call MoveIt, plan trajectory to target scenario
  //-------------------------------------------------------------------------------------
  agiprobot_msgs::goal goal;
  goal.request.goal = static_cast<int8_t>(target_scene);
  g_moveit_client.call(goal); // this service won't return, so we don't wait for response

  //-------------------------------------------------------------------------------------
  // wait 5s for execution
  //-------------------------------------------------------------------------------------
  ros::Duration(5.0).sleep();
  return;
}

void publish_fsm_state()
{
  std_msgs::Int8 state;
  state.data = static_cast<int8_t>(g_fsm.get_current_state());
  g_fsm_state_pub.publish(state);
  g_fsm.print_current_state();
  return;
}

void trigger_fsm_to(const STATE target_state)
{
  //-------------------------------------------------------------------------------------
  // trasite FSM state and advertise the actual FSM state
  //-------------------------------------------------------------------------------------
  g_fsm.set_state(target_state);
  ROS_INFO_STREAM("\033[32m FSM triggered at: \033[0m" << g_node_name);
  g_fsm.print_current_state();

  //-------------------------------------------------------------------------------------
  // based on different target state trigger control events
  //-------------------------------------------------------------------------------------
  if (target_state == STATE::STANDBY)
  {
    ros::spinOnce();
    //-------------------------------------------------------------------------------------
    // transit to standby point
    //-------------------------------------------------------------------------------------
    if (g_enable_moveit)
    {
      moveit_to(target_state);
      g_tcp_pose = g_db.fixed_tcp_poses.at(target_state);
      push_tcp_pose();
    }

    //-------------------------------------------------------------------------------------
    // reset system, making system ready for the next run
    //-------------------------------------------------------------------------------------
    g_new_motor_placed    = false;
    g_inspection_done     = false;
    g_start_adjust_height = false;
    g_start_focus_screws  = false;
    g_max_screw_num       = 0;
    g_rgb_updated         = false;
    g_tcp_depth           = 0;
    g_screw_list_now.clear();
    g_screw_list_last.clear();
    g_optimal_inspection_pose.clear();

    return;
  }
  else if (target_state == STATE::INSPECT)
  {
    ros::spinOnce();
    //-------------------------------------------------------------------------------------
    // transit to inspect point
    //-------------------------------------------------------------------------------------
    if (g_enable_moveit)
    {
      moveit_to(target_state);

      // if we did inspection before, start from where we lefted, otherwise start from default point
      if (g_inspection_done)
        g_tcp_pose = g_optimal_inspection_pose;
      else
        g_tcp_pose = g_db.fixed_tcp_poses.at(target_state);

      push_tcp_pose();
      
    }

    //-------------------------------------------------------------------------------------
    // use viausl servoing to focus screw in view center, deceide target screw
    //-------------------------------------------------------------------------------------
    g_start_focus_screws = true;

    //-------------------------------------------------------------------------------------
    // make sure following operation flags are reseted
    //-------------------------------------------------------------------------------------
    g_start_adjust_height      = false; // triggered to true after screw focused
    g_start_focus_target_screw = false; // triggered to true at SERVO state

    return;
  }
  if (target_state == STATE::SERVO)
  {
    ros::spinOnce();
    //-------------------------------------------------------------------------------------
    // set target tool based on screw schduling
    //-------------------------------------------------------------------------------------
    g_target_screw = g_screw_list_now.front();
    g_target_tool  = g_target_screw.tool;

    //-------------------------------------------------------------------------------------
    // check if tool changing is needed
    //-------------------------------------------------------------------------------------
    if (g_current_tool != g_target_tool)
    {
      ROS_WARN_STREAM(g_node_name << "current mounted tool doesn't match target screw!");
      if (g_autonomous_mode_on)
        trigger_fsm_to(STATE::DROP_TOOL);
    }

    //-------------------------------------------------------------------------------------
    // X-Y visual servoing to focus target screw
    //-------------------------------------------------------------------------------------
    g_start_focus_target_screw = true;
    return;
  }
  else if (target_state == STATE::UNSCREW)
  {
    ros::spinOnce();
    //-------------------------------------------------------------------------------------
    // re-check gripper state, if it's down, move it up
    // because if it's down it will block the tool from the screw
    //-------------------------------------------------------------------------------------
    if (g_gripper_state == GRIPPER::DOWN)
    {
      execute_gripper();
    }

    ROS_INFO_STREAM(g_node_name << ": \033[32m Starting to unscrew \033[0m");
    agiprobot_msgs::unscrew unscrew;
    unscrew.request.loops = g_target_screw.unscrew_loops;
    g_unscrew_client.call(unscrew);

    // fsm transition
    if (g_autonomous_mode_on)
      trigger_fsm_to(STATE::GRASP);

    return;
  }
  else if (target_state == STATE::GRASP)
  {
    ros::spinOnce();
    // grasp screw
    if (g_gripper_state == GRIPPER::ERROR)
    {
      ROS_ERROR_STREAM(g_node_name << ": girpper in error state, exiting, check gripper.");
    }
    else if (g_gripper_state == GRIPPER::SLEEP)
    {
      screw_gripper::Initiate init;
      g_gripper_initial.call(init);
      g_gripper_state = GRIPPER::UP;
    }
    else if (g_gripper_state == GRIPPER::DOWN)
    {
      execute_gripper();
      execute_gripper();
    }
    else if (g_gripper_state == GRIPPER::UP)
    {
      execute_gripper();
    }

    fetch_tcp_pose();
    ros::Duration(0.5).sleep();
    // pick up screw
    lift_tcp(0.2);
    //fetch_tcp_pose();
    g_tcp_pose = g_db.fixed_tcp_poses.at(STATE::DROP_SCREW);
    push_tcp_pose();
    fetch_tcp_pose();
    ros::Duration(0.5).sleep();
    lift_tcp(-0.35);
    execute_gripper();
    g_tcp_pose = g_db.fixed_tcp_poses.at(STATE::DROP_SCREW);
    push_tcp_pose();
    g_tcp_pose = g_db.fixed_tcp_poses.at(STATE::SAFE);
    push_tcp_pose();

    // fsm transition
    if (g_autonomous_mode_on)
      trigger_fsm_to(STATE::RECYCLE);

    return;
  }
  else if (target_state == STATE::RECYCLE)
  {
    ros::spinOnce();

    if (g_inspection_done)
    {
      //-------------------------------------------------------------------------------------
      // if we depart from clamp device, go straight to recycle without moveit, as it's near
      //-------------------------------------------------------------------------------------
      g_tcp_pose = g_db.fixed_tcp_poses.at(target_state);
    }
    else
    {
      if (g_enable_moveit)
      {
        moveit_to(target_state);
        g_tcp_pose = g_db.fixed_tcp_poses.at(target_state);
      }
    }
    push_tcp_pose();

    // move down robot head
    lift_tcp(-0.3);

    // release gripper
    execute_gripper();

    ros::Duration(0.1).sleep();

    // move back up
    lift_tcp(0.3);

    //-------------------------------------------------------------------------------------
    // if we finshed all screws, go to standby, elsewise continue process
    //-------------------------------------------------------------------------------------
    if (g_autonomous_mode_on)
    {
      if (g_inspection_done)
        trigger_fsm_to(STATE::INSPECT);
      else
        trigger_fsm_to(STATE::STANDBY);
    }

    return;
  }
  else if (target_state == STATE::PICK_TOOL)
  {
    ros::spinOnce();

    //-------------------------------------------------------------------------------------
    // check tool in hand
    //-------------------------------------------------------------------------------------
    if (g_current_tool != TOOL::NO_TOOL)
    {
      ROS_ERROR("there is tool already in hand, we can not pick a tool, drop first.");
      return;
    }

    //-------------------------------------------------------------------------------------
    // move to tool station
    //-------------------------------------------------------------------------------------
    if (g_enable_moveit)
    {
      moveit_to(target_state);
      g_tcp_pose = g_db.fixed_tcp_poses.at(target_state);
      push_tcp_pose();
    }
    return;
  }
  else if (target_state == STATE::DROP_TOOL)
  {
    ros::spinOnce();

    //-------------------------------------------------------------------------------------
    // check tool in hand
    //-------------------------------------------------------------------------------------
    if (g_current_tool == TOOL::NO_TOOL)
    {
      ROS_ERROR("no tool in hand, no need drop tool.");
      return;
    }

    //-------------------------------------------------------------------------------------
    // move to tool station
    //-------------------------------------------------------------------------------------
    if (g_enable_moveit)
    {
      moveit_to(target_state);
      g_tcp_pose = g_db.fixed_tcp_poses.at(target_state);
      push_tcp_pose();
    }

    handle_tool_request();

    //-------------------------------------------------------------------------------------
    // transit to pick tool
    //-------------------------------------------------------------------------------------
    if (g_autonomous_mode_on)
      trigger_fsm_to(STATE::PICK_TOOL);
    return;
  }
  else
  {
    ros::spinOnce();
    return;
  }
}

void handle_joy_input()
{
  fetch_tcp_pose();
  // programs
  // unscrew: LB+RB
  if (g_joy_input.left_button == 1.0 && g_joy_input.right_button == 1.0)
  {
    trigger_fsm_to(STATE::SERVO);
  }
  // move to check points: LT + A,B,X,Y
  else if (g_joy_input.y_button == 1.0 && g_joy_input.left_trigger == -1.0)
  {
    trigger_fsm_to(STATE::STANDBY);
  }
  else if (g_joy_input.a_button == 1.0 && g_joy_input.left_trigger == -1.0)
  {
    trigger_fsm_to(STATE::INSPECT);
  }
  else if (g_joy_input.b_button == 1.0 && g_joy_input.left_trigger == -1.0)
  {
    trigger_fsm_to(STATE::RECYCLE);
  }
  else if (g_joy_input.x_button == 1.0 && g_joy_input.left_trigger == -1.0)
  {
    // trigger_fsm_to(STATE::PICK_TOOL);
    moveit_to(STATE::PICK_TOOL);
  }
  else if (g_joy_input.button_axes_y == 1.0 && g_joy_input.left_trigger == -1.0) // free drive
  {
    agiprobot_msgs::teachMode teach_mode;
    teach_mode.request.switchOn = true;
    g_teachMode_client.call(teach_mode);
  }
  else if (g_joy_input.left_trigger == -1.0 && g_joy_input.button_axes_x == -1.0)
  {
    trigger_fsm_to(STATE::PICK_TOOL);
    g_target_tool = TOOL::TOOL1;
    handle_tool_request();
  }
  else if (g_joy_input.left_trigger == -1.0 && g_joy_input.button_axes_x == 1.0)
  {
    trigger_fsm_to(STATE::PICK_TOOL);
    g_target_tool = TOOL::TOOL3;
    handle_tool_request();
  }
  else if (g_joy_input.right_trigger == -1.0 && g_joy_input.button_axes_x == -1.0)
  {
    trigger_fsm_to(STATE::DROP_TOOL);
    g_target_tool = TOOL::TOOL1;
    handle_tool_request();
  }
  else if (g_joy_input.right_trigger == -1.0 && g_joy_input.button_axes_x == 1.0)
  {
    trigger_fsm_to(STATE::DROP_TOOL);
    g_target_tool = TOOL::TOOL3;
    handle_tool_request();
  }
  else if (g_joy_input.left_trigger == -1.0 && g_joy_input.right_trigger == -1.0) // lift
  {
    lift_tcp(0.2);
  }
  else if (g_joy_input.a_button == 1.0 && g_joy_input.right_trigger == -1.0) // gripper
  {
    execute_gripper();
  }
  else if (g_joy_input.left_button == 1.0 && g_joy_input.button_axes_y == -1.0)
  {
    // g_start_adjust_height = !g_start_adjust_height;
    agiprobot_msgs::unscrew unscrew;
    unscrew.request.loops = 1; // g_target_screw.unscrew_loops;
    g_unscrew_client.call(unscrew);
  }
  else if (g_joy_input.left_button == 1.0 && g_joy_input.button_axes_x == -1.0)
  {
  }
  else if (g_joy_input.left_button == 1.0 && g_joy_input.button_axes_x == 1.0)
  {
    g_tcp_pose = g_optimal_inspection_pose;
  }
  else
  {
    //-------------------------------------------------------------------------------------
    // positiond
    //-------------------------------------------------------------------------------------
    // (x,y)
    g_tcp_pose[1] -= g_joy_input.left_joy_axes_x * 0.01;
    g_tcp_pose[1] -= g_joy_input.button_axes_x * 0.001; // 1mm
    g_tcp_pose[0] -= g_joy_input.left_joy_axes_y * 0.01;
    g_tcp_pose[0] -= g_joy_input.button_axes_y * 0.001;
    // z: up
    g_tcp_pose[2] += g_joy_input.right_button * 0.01; // RB
    // z: down
    g_tcp_pose[2] -= g_joy_input.left_button * 0.01; // LB

    //-------------------------------------------------------------------------------------
    // orientation (to be impl.)
    //-------------------------------------------------------------------------------------
    // AxisAngle axis_angle;
    // axis_angle.push_back(g_tcp_pose[3]);
    // axis_angle.push_back(g_tcp_pose[4]);
    // axis_angle.push_back(g_tcp_pose[5]);
    // Quat quat = axisAngleToQuaternions(axis_angle);
  }

  /* ---------------------------------------------------------------
   * reset flag
   * --------------------------------------------------------------- */
  g_joy_updated = false;
}

void handle_key_input()
{
  fetch_tcp_pose();
  // full automatic run
  if (g_key_input.enter)
  {
    // demo_unscrew();
  }
  // semi-automatic control
  else if (g_key_input.h)
  {
    trigger_fsm_to(STATE::STANDBY);
    ROS_WARN_STREAM(g_node_name << ": start moving to: STANDBY");
  }
  else if (g_key_input.escape)
  {
    ROS_INFO_STREAM("\033[32m SYSTEM SHUTED DOWN BY USER, GOODBYE:) \033[0m");
    kill_all_nodes();
  }
  else if (g_key_input.c)
  {
    trigger_fsm_to(STATE::CALIBRATE);
    ROS_WARN_STREAM(g_node_name << ": start moving to: CALIBRATE");
  }
  else if (g_key_input.i)
  {
    trigger_fsm_to(STATE::INSPECT);
    ROS_WARN_STREAM(g_node_name << ": start moving to: INSPECT");
  }
  else if (g_key_input.r)
  {
    trigger_fsm_to(STATE::RECYCLE);
    ROS_WARN_STREAM(g_node_name << ": start moving to: RECYCLE");
  }
  else if (g_key_input.t)
  {
    trigger_fsm_to(STATE::PICK_TOOL);
    ROS_WARN_STREAM(g_node_name << ": start moving to: TOOL_CHANGE");
  }
  else if (g_key_input.f) // free drive
  {
    agiprobot_msgs::teachMode teach_mode;
    teach_mode.request.switchOn = true;
    g_teachMode_client.call(teach_mode);
    ROS_WARN_STREAM(g_node_name << ": Starting teach mode");
  }
  else if (g_key_input.p && g_key_input.num_1)
  {
    trigger_fsm_to(STATE::PICK_TOOL);
    g_target_tool = TOOL::TOOL1;
    handle_tool_request();
  }
  else if (g_key_input.p && g_key_input.num_3)
  {
    trigger_fsm_to(STATE::PICK_TOOL);
    g_target_tool = TOOL::TOOL3;
    handle_tool_request();
  }
  else if (g_key_input.d && g_key_input.num_1)
  {
    trigger_fsm_to(STATE::DROP_TOOL);
    g_target_tool = TOOL::TOOL1;
    handle_tool_request();
  }
  else if (g_key_input.d && g_key_input.num_3)
  {
    trigger_fsm_to(STATE::DROP_TOOL);
    g_target_tool = TOOL::TOOL3;
    handle_tool_request();
  }
  else if (g_key_input.arrow_up && g_key_input.num_1) // lift
  {
    lift_tcp(0.1);
  }
  else if (g_key_input.arrow_up && g_key_input.num_2) // lift
  {
    lift_tcp(0.2);
  }
  else if (g_key_input.arrow_up && g_key_input.num_3) // lift
  {
    lift_tcp(0.3);
  }
  else if (g_key_input.arrow_up && g_key_input.num_4) // lift
  {
    lift_tcp(0.4);
  }
  else if (g_key_input.arrow_down && g_key_input.num_1) // lift
  {
    lift_tcp(-0.1);
  }
  else if (g_key_input.arrow_down && g_key_input.num_2) // lift
  {
    lift_tcp(-0.2);
  }
  else if (g_key_input.arrow_down && g_key_input.num_3) // lift
  {
    lift_tcp(-0.3);
  }
  else if (g_key_input.arrow_down && g_key_input.num_4) // lift
  {
    lift_tcp(-0.4);
  }
  else if (g_key_input.left_ctrl && g_key_input.g) // gripper
  {
    execute_gripper();
  }
  else if (g_key_input.num_0)
  {
    g_start_adjust_height = !g_start_adjust_height;
  }
  else if (g_key_input.num_1)
  {
  }
  else
  {
    //-------------------------------------------------------------------------------------
    // position
    //-------------------------------------------------------------------------------------
    // (x,y)
    g_tcp_pose[1] -= g_key_input.a * 0.001;
    g_tcp_pose[1] += g_key_input.d * 0.001;
    g_tcp_pose[0] -= g_key_input.w * 0.001;
    g_tcp_pose[0] += g_key_input.s * 0.001;
    // z
    g_tcp_pose[2] += g_key_input.arrow_up * 0.01;
    g_tcp_pose[2] -= g_key_input.arrow_down * 0.01;

    //-------------------------------------------------------------------------------------
    // orientation (to be impl.)
    //-------------------------------------------------------------------------------------
    // AxisAngle axis_angle;
    // axis_angle.push_back(g_tcp_pose[3]);
    // axis_angle.push_back(g_tcp_pose[4]);
    // axis_angle.push_back(g_tcp_pose[5]);
    // Quat quat = axisAngleToQuaternions(axis_angle);
  }

  /* ---------------------------------------------------------------
   * reset flag
   * --------------------------------------------------------------- */
  g_key_updated = false;
}

void kill_all_nodes()
{
  //-------------------------------------------------------------------------------------
  // get all running node name
  //-------------------------------------------------------------------------------------
  std::string command = "rosnode list";
  FILE* fp            = popen(command.c_str(), "r");

  std::vector<std::string> nodes;
  char buffer[1024];
  while (fgets(buffer, 1024, fp) != nullptr)
  {
    std::string nodeName(buffer);
    nodeName.erase(nodeName.find_last_not_of(" \n\r\t") + 1);
    nodes.push_back(nodeName);
  }
  pclose(fp);

  //-------------------------------------------------------------------------------------
  // kill them one by one
  //-------------------------------------------------------------------------------------
  for (const auto& nodeName : nodes)
  {
    command = "rosnode kill " + nodeName;
    system(command.c_str());
  }
}

void output_debug_image_stream()
{
  //-------------------------------------------------------------------------------------
  // define text size and position
  //-------------------------------------------------------------------------------------
  int font         = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.5;
  double thickness = 1.5;
  int baseline     = 0;
  cv::Size textSize =
    cv::getTextSize(std::to_string(g_tcp_depth), font, fontScale, thickness, &baseline);
  //-------------------------------------------------------------------------------------
  // real-time depth map after processing
  //-------------------------------------------------------------------------------------
  cv::Mat depth_visual;

  // convert to 3 channel, since we want tcp information show in color
  cv::cvtColor(g_filtered_depth_map, depth_visual, cv::COLOR_GRAY2BGR);

  // draw tcp sample point
  cv::circle(depth_visual, g_tcp_sample_point, g_tcp_roi_radius, cv::Scalar(0, 0, 255), 1);

  // print depth value at tcp sample point
  int position_x, position_y; // text position
  position_x = g_tcp_sample_point.x - textSize.width / 2;
  position_y = g_tcp_sample_point.y - 10 - textSize.height / 2;
  if (position_x < 0)
    position_x = 0;
  else if (position_x > g_rgb_image.cols - 1)
    position_x = g_rgb_image.cols - 1;
  if (position_y < 0)
    position_y = 0;
  else if (position_x > g_rgb_image.rows - 1)
    position_y = g_rgb_image.rows - 1;

  cv::Point textPosition1(position_x, position_y);
  cv::putText(depth_visual,
              std::to_string(g_tcp_depth),
              textPosition1,
              font,
              fontScale,
              cv::Scalar(0, 255, 0),
              thickness);

  //-------------------------------------------------------------------------------------
  // RGB view
  //-------------------------------------------------------------------------------------
  cv::Mat rgb_visual = g_rgb_image.clone();
  // draw tcp
  cv::circle(rgb_visual, g_tcp_sample_point, g_tcp_roi_radius, cv::Scalar(0, 0, 255), 1);
  cv::putText(rgb_visual,
              std::to_string(g_tcp_depth) + "m",
              textPosition1,
              font,
              fontScale,
              cv::Scalar(0, 255, 0),
              thickness);

  // draw actual detected screws in green circle
  for (auto screw : g_screw_list_now)
  {
    // circle position
    cv::circle(rgb_visual, cv::Point(screw.x, screw.y), screw.radius, cv::Scalar(0, 90, 255), 1);

    // print classification and depth of each screw
    int position_x = screw.x - textSize.width / 2;
    int position_y = screw.y - 10 - textSize.height / 2;
    if (position_x < 0)
      position_x = 0;
    else if (position_x > g_rgb_image.cols - 1)
      position_x = g_rgb_image.cols - 1;
    if (position_y < 0)
      position_y = 0;
    else if (position_x > g_rgb_image.rows - 1)
      position_y = g_rgb_image.rows - 1;

    cv::Point textPosition2(position_x, position_y);
    cv::putText(rgb_visual,
                g_db.output_screw_type(screw) + std::to_string(screw.depth) + "[m]",
                textPosition2,
                font,
                fontScale,
                cv::Scalar(0, 90, 255),
                thickness);
  }

  // draw last step detected screws in blue circle
  for (auto screw : g_screw_list_last)
  {
    cv::circle(rgb_visual, cv::Point(screw.x, screw.y), screw.radius, cv::Scalar(255, 0, 0), 1);
  }

  //-------------------------------------------------------------------------------------
  // show both depth and RGB window
  //-------------------------------------------------------------------------------------
  cv::imshow("RGB with Screws", rgb_visual);
  cv::imshow("Aligned & filtered Depth", depth_visual);

  //-------------------------------------------------------------------------------------
  // update with cv and then release memory
  //-------------------------------------------------------------------------------------
  cv::waitKey(1);
  depth_visual.release();
  rgb_visual.release();
}

void detect_markers()
{
  if (g_rgb_image.empty())
    return;
  // Convert the image to HSV color space
  cv::Mat hsv;
  cv::cvtColor(g_rgb_image, hsv, cv::COLOR_BGR2HSV);

  // Define color thresholds for red, blue, and green markers
  cv::Scalar lowerRed(0, 50, 50);      // lower HSV threshold for red
  cv::Scalar upperRed(10, 255, 255);   // upper HSV threshold for red
  cv::Scalar lowerBlue(100, 50, 50);   // lower HSV threshold for blue
  cv::Scalar upperBlue(130, 255, 255); // upper HSV  threshold for blue
  cv::Scalar lowerGreen(40, 50, 50);   // lower HSV threshold for green
  cv::Scalar upperGreen(70, 255, 255); // upper HSV threshold for green

  // Threshold the image for each color
  cv::Mat redMask, blueMask, greenMask;
  cv::inRange(hsv, lowerRed, upperRed, redMask);
  cv::inRange(hsv, lowerBlue, upperBlue, blueMask);
  cv::inRange(hsv, lowerGreen, upperGreen, greenMask);

  // Detect contours in each color mask
  std::vector<std::vector<cv::Point> > redContours, blueContours, greenContours;
  cv::findContours(redMask, redContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  cv::findContours(blueMask, blueContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  cv::findContours(greenMask, greenContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // red markers
  for (const auto& contour : redContours)
  {
    // Find the circle center and radius
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);

    Screw marker;
    // position
    marker.x      = center.x;
    marker.y      = center.y;
    marker.radius = double(radius);
    // distance
    int y_min = center.y - radius;
    int y_max = center.y + radius;
    int x_min = center.x - radius;
    int x_max = center.x + radius;
    if (y_min < 0)
      y_min = 0;
    if (y_min > g_filtered_depth_map.rows - 1)
      y_min = g_filtered_depth_map.rows - 1;
    if (x_min < 0)
      x_min = 0;
    if (x_min > g_filtered_depth_map.cols)
      x_min = g_filtered_depth_map.cols - 1;

    cv::Mat marker_roi = g_filtered_depth_map(cv::Range(y_min, y_max), cv::Range(x_min, x_max));
    cv::Scalar marker_depth = cv::mean(marker_roi);
    marker.depth            = marker_depth[0];
    // type
    marker.type = SCREW_TYPE::RED_MARKER;
    // Add to screw list
    g_screw_list_now.push_back(marker);
  }

  // green markers
  for (const auto& contour : greenContours)
  {
    // Find the circle center and radius
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);

    Screw marker;
    // position
    marker.x      = center.x;
    marker.y      = center.y;
    marker.radius = double(radius);
    // distance
    int y_min = center.y - radius;
    int y_max = center.y + radius;
    int x_min = center.x - radius;
    int x_max = center.x + radius;
    if (y_min < 0)
      y_min = 0;
    if (y_min > g_filtered_depth_map.rows - 1)
      y_min = g_filtered_depth_map.rows - 1;
    if (x_min < 0)
      x_min = 0;
    if (x_min > g_filtered_depth_map.cols)
      x_min = g_filtered_depth_map.cols - 1;
    cv::Mat marker_roi = g_filtered_depth_map(cv::Range(y_min, y_max), cv::Range(x_min, x_max));
    cv::Scalar marker_depth = cv::mean(marker_roi);
    marker.depth            = marker_depth[0];
    // type
    marker.type = SCREW_TYPE::GREEN_MARKER;
    // Add to screw list
    g_screw_list_now.push_back(marker);
  }

  // blue markerssmall_motor_demo
  for (const auto& contour : blueContours)
  {
    // find the circle center and radius
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);

    Screw marker;
    // position
    marker.x      = center.x;
    marker.y      = center.y;
    marker.radius = double(radius);
    // distance
    int y_min = center.y - radius;
    int y_max = center.y + radius;
    int x_min = center.x - radius;
    int x_max = center.x + radius;
    if (y_min < 0)
      y_min = 0;
    if (y_min > g_filtered_depth_map.rows - 1)
      y_min = g_filtered_depth_map.rows - 1;
    if (x_min < 0)
      x_min = 0;
    if (x_min > g_filtered_depth_map.cols)
      x_min = g_filtered_depth_map.cols - 1;
    cv::Mat marker_roi = g_filtered_depth_map(cv::Range(y_min, y_max), cv::Range(x_min, x_max));
    cv::Scalar marker_depth = cv::mean(marker_roi);
    marker.depth            = marker_depth[0];
    // type
    marker.type = SCREW_TYPE::BLUE_MARKER;

    // Add to screw list
    g_screw_list_now.push_back(marker);
  }
  return;
}

void auto_set_heading()
{
  STATE state = g_fsm.get_current_state();
  if (state == STATE::STANDBY || state == STATE::PICK_TOOL || state == STATE::DROP_TOOL)
  {
    std::copy_n(g_db.fixed_tcp_headings.at(HEADING::BACK).begin(), 3, g_tcp_pose.begin() + 3);
    ROS_INFO_STREAM(g_node_name << " : heading BACK");
  }
  else
  {
    std::copy_n(g_db.fixed_tcp_headings.at(HEADING::RIGHT).begin(), 3, g_tcp_pose.begin() + 3);
    ROS_INFO_STREAM(g_node_name << " : heading RIGHT");
  }
}

bool engage_screw_head()
{
  agiprobot_msgs::align engage;
  g_screw_alignment_client.call(engage);

  if (engage.response.success) {
    ROS_INFO_STREAM(g_node_name << " ------------- SCREW ENGAGED AND UNSCREWED ------------- " << g_tcp_update_counter
                                << " SUCCESS, SYSTEM RUNING WELL...");
    //execute_gripper();
    return true;
  }
  else
  {
    ROS_ERROR_STREAM(g_node_name << " ------------- SCREW NOT ENGAGED ------------- ");
    return false;
  }
  /*
  return engage.response.success;
  // get newest detection
  ros::spinOnce();

  // align x-y plane
  double delta_x = g_target_screw.x - g_tcp_coordinate.x;
  double delta_y = g_target_screw.y - g_tcp_coordinate.y;

  g_tcp_pose[1] += delta_x * g_scale_factor_x;
  g_tcp_pose[0] += delta_y * g_scale_factor_y;
  push_tcp_pose();

  ros::Duration(0.1).sleep();

  // align z
  lift_tcp(-g_tcp_to_screw_dist);
  ros::Duration(0.1).sleep();
  lift_tcp(-0.003); */
}

bool motorPlacedCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (g_fsm.get_current_state() == STATE::STANDBY)
  {
    ROS_INFO_STREAM(g_node_name << "\033[32m got new motor, ready for another round. \033[0m");
    if (g_autonomous_mode_on)
      trigger_fsm_to(STATE::INSPECT);
    g_new_motor_placed = true;
    res.success        = true;
  }
  else
  {
    res.message = "disassembly station is in running, not able to accept new motor, please try it "
                  "again when it's standby.";
    res.success = false;
  }
  return res.success;
}

bool fsmCb(agiprobot_msgs::fsm::Request& req, agiprobot_msgs::fsm::Response& res)
{
  //-------------------------------------------------------------------------------------
  // first, check if the target state is valid
  //-------------------------------------------------------------------------------------
  if (req.target_state >= 0 && req.target_state <= 8)
  {
    //-------------------------------------------------------------------------------------
    // handle the automatic transition flag
    //-------------------------------------------------------------------------------------
    if (req.enable_automatic_transition)
      g_autonomous_mode_on = true;
    else
      g_autonomous_mode_on = false;

    //-------------------------------------------------------------------------------------
    // trigger FSM with target state
    //-------------------------------------------------------------------------------------
    trigger_fsm_to(static_cast<STATE>(req.target_state));

    res.success = true;
  }
  else
    res.success = false;

  return res.success;
}

bool toolCb(agiprobot_msgs::tool::Request& req, agiprobot_msgs::tool::Response& res)
{
  if(req.fromSlot >= 0 && req.fromSlot <= 3)
  {
    g_target_tool = static_cast<TOOL>(req.fromSlot);
    res.success = true;
    handle_tool_request();
  }
  else res.success = false;

  return res.success;
}