#ifndef FLIGHTGOGGLESCLIENT2_H
#define FLIGHTGOGGLESCLIENT2_H
/**
 * @file   FlightGogglesClient2.hpp
 * @author Winter Guerra
 * @date   Feb 22, 2018
 * @brief  Library class that abstracts interactions with FlightGoggles.
 */

#include <fstream>
#include <chrono>

// Include ZMQ bindings for comms with Unity.
#include <iostream>
#include <string>
#include <zmqpp/zmqpp.hpp>

// Include JSON message type definitions.
#include "jsonMessageSpec.hpp"
#include "json.hpp"
using json = nlohmann::json;

// For image operations
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include <ros/time.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include "mav_msgs/Actuators.h"
#include "mav_msgs/RateThrust.h"
#include "geometry_msgs/Pose.h"
#include "tf2_msgs/TFMessage.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// For transforms
#include <Eigen/Geometry>
#include <Eigen/Dense>

#define PRECISION 10
#define DONTALIGNCOLS 1

// define some eigen types and formats
typedef Eigen::Vector3d Vector3;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Quaternion<double> Quaternionx;
typedef Eigen::Affine3d Transform3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::Matrix3d Matrix3;



class FlightGogglesClient2
{
  public:

    ros::NodeHandle node_;

    /// @name Public variables
    //@{
    // Base status object (which holds camera settings, env settings, etc)
    unity_outgoing::StateMessage_t state;

    tf2_ros::Buffer tfBuffer_;
	tf2_ros::TransformListener tfListener_;
    ros::Subscriber tfSubscriber_;

    sensor_msgs::CameraInfo cameraInfoLeft_;
    image_transport::ImageTransport it_;
	image_transport::CameraPublisher imagePubLeft_;
    ros::Publisher transformPub_;
    ros::Publisher motorPub_;
    ros::Publisher rateThrustPub_;
    ros::Publisher collisionPub_;

    sensor_msgs::CameraInfo cameraInfoLeft;
    float baseline_ = 0.32;
	int imageWidth_ = 1024;
	int imageHeight_ = 768;
    bool render_stereo = false;
    int numSimulationStepsSinceLastRender_ = 0;
    const int numSimulationStepsBeforeRenderRequest_ = 15;

    geometry_msgs::TransformStamped imu_T_Camera_;

    // ZMQ connection parametersmav_msgs
    std::string client_address = "tcp://*";
    std::string download_port = "10254";
    // Socket variables
    zmqpp::context context;
    zmqpp::socket download_socket {
        context, 
        zmqpp::socket_type::subscribe};

    // Temporary buffer for reshaping and casting of received images
    std::vector<uint8_t> _castedInputBuffer;

    // Keep track of time of last sent/received messages
    int64_t last_uploaded_utime = 0;
    int64_t last_downloaded_utime = 0;
    int64_t last_upload_debug_utime = 0;
    int64_t last_download_debug_utime = 0;
    int64_t u_packet_latency = 0;
    int64_t num_frames = 0;
    //@}

    /// Constructors.
    FlightGogglesClient2(ros::NodeHandle ns);

    /// Connects to FlightGoggles.
    void initializeConnections();

    /// Set camera pose using ROS coordinates.
    void setCameraPoseUsingROSCoordinates(Eigen::Affine3d ros_pose, int cam_index);

    /// Send render request to Unity
    bool requestRender();

    ///////////////////////////////////////////
    // FLIGHTGOGGLES INCOMING MESSAGE HANDLERS
    ///////////////////////////////////////////

    /// @name Flightgoggles incoming message handlers
    // Ensure that input buffer can handle the incoming message.
    inline void ensureBufferIsAllocated(unity_incoming::RenderMetadata_t renderMetadata){
        // Check that buffer size is correct
        uint64_t requested_buffer_size = renderMetadata.camWidth * renderMetadata.camHeight * 3;
        // Resize if necessary
        if (_castedInputBuffer.size() != requested_buffer_size)
        {
            _castedInputBuffer.resize(requested_buffer_size);
        }
    };

    // Blocking call. Returns rendered images and render metadata whenever 
    // it becomes available.
    unity_incoming::RenderOutput_t handleImageResponse();

    /// @name Helper functions
    static inline int64_t getTimestamp(){
        int64_t time = std::chrono::high_resolution_clock::now().time_since_epoch() /
                    std::chrono::microseconds(1);
        return time;
    };

    void populateRenderSettings();

    void publishState(ros::Time timestamp,
                      geometry_msgs::TransformStamped transform,
                      mav_msgs::Actuators motorspeedsMessage,
                      mav_msgs::RateThrust::Ptr rateThrust);


    void tfCallback(tf2_msgs::TFMessage::Ptr msg);

};

FlightGogglesClient2 init_flightgoggles_client(ros::NodeHandle &node) {
    FlightGogglesClient2 fgc(node);
    return fgc;
}

#endif