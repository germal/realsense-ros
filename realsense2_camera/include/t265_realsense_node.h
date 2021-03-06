#pragma once

#include <base_realsense_node.h>

namespace realsense2_camera
{
    class T265RealsenseNode : public BaseRealSenseNode
    {
        public:
            T265RealsenseNode(ros::NodeHandle& nodeHandle,
                          ros::NodeHandle& privateNodeHandle,
                          rs2::device dev,
                          const std::string& serial_no);
            ~T265RealsenseNode();
            void publishTopics();

        protected:
            void calcAndPublishStaticTransform(const stream_index_pair& stream, const rs2::stream_profile& base_profile) override;

        private:
            void initializeOdometryInput();
            void importLocalization();
            void importLocalization(const std::string& localization_file);
            void exportLocalization();
            void exportLocalization(const std::string& export_file);
            void initMapFrame(bool relocalizing);
            
            void setupSubscribers();
            void odom_in_callback(const nav_msgs::Odometry::ConstPtr& msg);

            rs2::pose_sensor _pose_snr;
            ros::Subscriber _odom_subscriber;
            rs2::wheel_odometer _wo_snr;
            bool _use_odom_in;
            ros::Timer _timer;
            //std::string relocalization_node_guid;
            bool relocalization_pose_initialized;
            tf2_ros::TransformBroadcaster _dynamic_tf_broadcaster;
            //std::string _map_frame_id;
    };
}
