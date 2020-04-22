#include "../include/t265_realsense_node.h"

using namespace realsense2_camera;

T265RealsenseNode::T265RealsenseNode(ros::NodeHandle& nodeHandle,
                                     ros::NodeHandle& privateNodeHandle,
                                     rs2::device dev,
                                     const std::string& serial_no) : 
                                     BaseRealSenseNode(nodeHandle, privateNodeHandle, dev, serial_no),
                                     _pose_snr(dev.first<rs2::pose_sensor>()),
                                     _wo_snr(dev.first<rs2::wheel_odometer>()),
                                     _use_odom_in(false) 
                                     {
                                         _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_MOTION_MODULE_TEMPERATURE};
                                         initializeOdometryInput();
                                     }

void T265RealsenseNode::importLocalization()
{
    std::string localization_file;
    _pnh.param("localization_file", localization_file, std::string(""));
    if (localization_file.empty())
    {
        ROS_INFO("No localization_file. No localization data loaded.");
        return;
    }
    
    importLocalization(localization_file);
}

// Copied from https://github.com/IntelRealSense/librealsense/blob/master/examples/ar-advanced/rs-ar-advanced.cpp
std::vector<uint8_t> bytes_from_raw_file(const std::string& filename)
{
  std::ifstream file(filename.c_str(), std::ios::binary);
  if (!file.good())
    throw std::runtime_error("Invalid binary file specified. Verify the source path and location permissions");
  
  // Determine the file length
  file.seekg(0, std::ios_base::end);
  std::size_t size = file.tellg();
  if (!size)
    throw std::runtime_error("Invalid binary file -zero-size");
  file.seekg(0, std::ios_base::beg);
  
  // Create a vector to store the data
  std::vector<uint8_t> v(size);
  
  // Load the data
  file.read((char*)&v[0], size);
  
  return v;
}

void raw_file_from_bytes(const std::string& filename, const std::vector<uint8_t> bytes)
{
  std::ofstream file(filename, std::ios::binary | std::ios::trunc);
  if (!file.good())
    throw std::runtime_error("Invalid binary file specified. Verify the target path and location permissions");
  file.write((char*)bytes.data(), bytes.size());
}

void T265RealsenseNode::importLocalization(const std::string& localization_file)
{
    try {
      _pose_snr.import_localization_map(bytes_from_raw_file(localization_file));
      ROS_INFO_STREAM("Map loaded from " << localization_file);
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error loading map from " << localization_file << ": " << e.what());; 
    }
    
    /**
     * Load relocalization map onto device. Only one relocalization map can be imported at a time;
     * any previously existing map will be overwritten.
     * The imported map exists simultaneously with the map created during the most recent tracking session after start(),
     * and they are merged after the imported map is relocalized.
     * This operation must be done before start().
     * \param[in] lmap_buf map data as a binary blob
     * \return true if success
     */
    //bool import_localization_map(const std::vector<uint8_t>& lmap_buf) const
    
    /**
     * Get relocalization map that is currently on device, created and updated during most recent tracking session.
     * Can be called before or after stop().
     * \return map data as a binary blob
     */
    //std::vector<uint8_t> export_localization_map() const
    
    
    /**
     * Creates a named virtual landmark in the current map, known as static node.
     * The static node's pose is provided relative to the origin of current coordinate system of device poses.
     * This function fails if the current tracker confidence is below 3 (high confidence).
     * \param[in] guid unique name of the static node (limited to 127 chars).
     * If a static node with the same name already exists in the current map or the imported map, the static node is overwritten.
     * \param[in] pos position of the static node in the 3D space.
     * \param[in] orient_quat orientation of the static node in the 3D space, represented by a unit quaternion.
     * \return true if success.
     */
    //set_static_node(const std::string& guid, const rs2_vector& pos, const rs2_quaternion& orient) const
    
    /**
     * Gets the current pose of a static node that was created in the current map or in an imported map.
     * Static nodes of imported maps are available after relocalizing the imported map.
     * The static node's pose is returned relative to the current origin of coordinates of device poses.
     * Thus, poses of static nodes of an imported map are consistent with current device poses after relocalization.
     * This function fails if the current tracker confidence is below 3 (high confidence).
     * \param[in] guid unique name of the static node (limited to 127 chars).
     * \param[out] pos position of the static node in the 3D space.
     * \param[out] orient_quat orientation of the static node in the 3D space, represented by a unit quaternion.
     * \return true if success.
     */
    //bool get_static_node(const std::string& guid, rs2_vector& pos, rs2_quaternion& orient) const
    //bool remove_static_node(const std::string& guid) const
    
}
                                     
void T265RealsenseNode::initializeOdometryInput()
{
    std::string calib_odom_file;
    _pnh.param("calib_odom_file", calib_odom_file, std::string(""));
    if (calib_odom_file.empty())
    {
        ROS_INFO("No calib_odom_file. No input odometry accepted.");
        return;
    }
    std::ifstream calibrationFile(calib_odom_file);
    if (not calibrationFile)
    {
        ROS_FATAL_STREAM("calibration_odometry file not found. calib_odom_file = " << calib_odom_file);
        throw std::runtime_error("calibration_odometry file not found" );
    }
    const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
        std::istreambuf_iterator<char>());
    const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

    if (!_wo_snr.load_wheel_odometery_config(wo_calib))
    {
        ROS_FATAL_STREAM("Format error in calibration_odometry file: " << calib_odom_file);
        throw std::runtime_error("Format error in calibration_odometry file" );
    }
    _use_odom_in = true;
}

void T265RealsenseNode::publishTopics()
{
    BaseRealSenseNode::publishTopics();
    setupSubscribers();
}

void T265RealsenseNode::setupSubscribers()
{
    if (not _use_odom_in) return;

    std::string topic_odom_in;
    _pnh.param("topic_odom_in", topic_odom_in, DEFAULT_TOPIC_ODOM_IN);
    ROS_INFO_STREAM("Subscribing to in_odom topic: " << topic_odom_in);

    _odom_subscriber = _node_handle.subscribe(topic_odom_in, 1, &T265RealsenseNode::odom_in_callback, this);
}

void T265RealsenseNode::odom_in_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_DEBUG("Got in_odom message");
    rs2_vector velocity {-(float)(msg->twist.twist.linear.y),
                          (float)(msg->twist.twist.linear.z),
                         -(float)(msg->twist.twist.linear.x)};

    ROS_DEBUG_STREAM("Add odom: " << velocity.x << ", " << velocity.y << ", " << velocity.z);
    _wo_snr.send_wheel_odometry(0, 0, velocity);
}

void T265RealsenseNode::calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile)
{
    // Transform base to stream
    tf::Quaternion quaternion_optical;
    quaternion_optical.setRPY(M_PI / 2, 0.0, -M_PI / 2);    //Pose To ROS
    float3 zero_trans{0, 0, 0};

    ros::Time transform_ts_ = ros::Time::now();

    rs2_extrinsics ex;
    try
    {
        ex = getAProfile(stream).get_extrinsics_to(base_profile);
    }
    catch (std::exception& e)
    {
        if (!strcmp(e.what(), "Requested extrinsics are not available!"))
        {
            ROS_WARN_STREAM(e.what() << " : using unity as default.");
            ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
        }
        else
        {
            throw e;
        }
    }

    auto Q = rotationMatrixToQuaternion(ex.rotation);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
    if (stream == POSE)
    {
        Q = Q.inverse();
        publish_static_tf(transform_ts_, trans, Q, _frame_id[stream], _base_frame_id);
    }
    else
    {
        publish_static_tf(transform_ts_, trans, Q, _base_frame_id, _frame_id[stream]);
        publish_static_tf(transform_ts_, zero_trans, quaternion_optical, _frame_id[stream], _optical_frame_id[stream]);

        // Add align_depth_to if exist:
        if (_align_depth && _depth_aligned_frame_id.find(stream) != _depth_aligned_frame_id.end())
        {
            publish_static_tf(transform_ts_, trans, Q, _base_frame_id, _depth_aligned_frame_id[stream]);
            publish_static_tf(transform_ts_, zero_trans, quaternion_optical, _depth_aligned_frame_id[stream], _optical_frame_id[stream]);
        }
    }
}
