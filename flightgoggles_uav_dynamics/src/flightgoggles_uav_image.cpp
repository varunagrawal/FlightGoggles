#include "flightgoggles_uav_image.hpp"

/**
 * @brief Converts right hand rule North East Down (NED) global poses to 
 * left handed coordinates (as used by the Unity3D backend of FlightGoggles).
 * 
 * @param NEDworld_T_object 
 * @return Transform3 
 */
Transform3 convertNEDGlobalPoseToGlobalUnityCoordinates2(
    Transform3 NEDworld_T_object)
{
    // Switch axis to unity axis.
    Matrix4 unity_pose_T_NED_pose;
    // x->z, y->x, z->-y
    // clang-format off
    unity_pose_T_NED_pose <<    0, 1, 0,  0,
                                0, 0, -1, 0,
                                1, 0, 0,  0,
                                0, 0, 0,  1;
    // clang-format on

    Transform3 unity_pose;

    unity_pose.matrix() = unity_pose_T_NED_pose * 
                          NEDworld_T_object.matrix() *
                          unity_pose_T_NED_pose.transpose();

    return unity_pose;
}

/**
 * @brief Converts right hand rule North East Down (NED) poses to left handed 
 * rule coordinates (as used by Unity3D). Takes in an additional parameter 
 * for the translational and rotational offset between the FlightGoggles world
 * and the global NED poses used by the robot.
 * 
 * @deprecated - will be removed soon.
 * 
 * @param NEDworld_T_object 
 * @param unityWorld_T_NEDworld 
 * @return Transform3 
 */
Transform3 convertNEDGlobalPoseToGlobalUnityCoordinates2(
    Transform3 NEDworld_T_object, Transform3 unityWorld_T_NEDworld)
{
    // Switch axis to unity axis.
    Matrix4 unity_pose_T_NED_pose;
    // x->z, y->x, z->-y
    // clang-format off
    unity_pose_T_NED_pose <<    0, 1, 0,  0,
                                0, 0, -1, 0,
                                1, 0, 0,  0,
                                0, 0, 0,  1;
    // clang-format on

    Transform3 unity_pose;

    unity_pose.matrix() = unity_pose_T_NED_pose * unityWorld_T_NEDworld.matrix() *
                          NEDworld_T_object.matrix() *
                          unity_pose_T_NED_pose.transpose();

    return unity_pose;
}

/**
 * @brief Converts ROS global poses to NED poses. These poses are then converted to 
 * Unity left handed coordinates. Assumes that ROS global coordinates for translation are 
 * East north up (ENU) and robot local coordinates are North west up (NWU). E.g. the robot 
 * local X axis is going out of the camera.  
 * 
 * @param ENUworld_T_object 
 * @return Transform3 
 */
Transform3 convertROSToNEDCoordinates2(Transform3 ENUworld_T_object)
{
    // Switch ENU axis to NED axis.
    Matrix4 NED_T_ENU;
    // x->y, y->x, z->-z, w->w
    // clang-format off
    NED_T_ENU <<  1, 0, 0,  0,
                    0, -1, 0,  0,
                     0, 0, -1, 0,
                     0, 0, 0,  1;
    // clang-format on

    Transform3 NED_pose;
    NED_pose.matrix() = NED_T_ENU * ENUworld_T_object.matrix() * NED_T_ENU.transpose();

    // Rotate robot pose by -90deg about robot z axis since ROS
    // expects that "X" is forward in robot frame.
    // Quaternionx Y_front_to_X_front_quat(sqrt(0.5f),	0.0f, 0.0f, -sqrt(0.5f));

    // NED_pose = Y_front_to_X_front_quat * NED_pose;


    return NED_pose;
}

/**
 * @brief Converts ROS global poses to NED poses. These poses are then converted to
 * Unity left handed coordinates. Assumes that ROS global coordinates for translation are
 * East north up (ENU) and robot local coordinates are North west up (NWU). E.g. the robot
 * local X axis is going out of the camera.
 *
 * @param ENUworld_T_object
 * @return Transform3
 */
Transform3 convertEDNToNEDCoordinates2(Transform3 EDNworld_T_object)
{
    // Switch ENU axis to NED axis.
    Matrix4 NED_T_EDN;
    // x->y, y->x, z->-z, w->w
    // clang-format off
    NED_T_EDN <<  0, 1, 0,  0,
                  1, 0, 0,  0,
                  0, 1, 0, 0,
                  0, 0, 0,  1;
    // clang-format on

    Transform3 NED_pose;
    NED_pose.matrix() = NED_T_EDN * EDNworld_T_object.matrix(); //EDN_pose_T_NED_pose * EDNworld_T_object.matrix() * EDN_pose_T_NED_pose.transpose();

    // Rotate robot pose by -90deg about robot z axis since ROS
    // expects that "X" is forward in robot frame.
    // Quaternionx Y_front_to_X_front_quat(sqrt(0.5f),	0.0f, 0.0f, -sqrt(0.5f));

    // NED_pose = Y_front_to_X_front_quat * NED_pose;


    return NED_pose;
}

Transform3 convertEDNGlobalPoseToGlobalUnityCoordinates2(
        Transform3 EDNworld_T_object)
{
    // Switch axis to unity axis.
    Matrix4 ENU_T_NED;
    // x->z, y->x, z->-y
    // clang-format off
    ENU_T_NED <<    1, 0,  0,  0,
                    0, -1, 0, 0,
                    0, 0,  1,  0,
                    0, 0,  0,  1;
    // clang-format on

    Transform3 unity_pose;

    unity_pose.matrix() = ENU_T_NED *
                          EDNworld_T_object.matrix() *
                          ENU_T_NED.transpose();

    return unity_pose;
}

// Converts a given LCM drone pose and relative camera pose into a global Unity
// pose for the camera.
/* Move world_T_cam from drone (right handed) coordinate system to Unity (left
handed) coordinate system:
Input vector in unity coordinates goes through the following
transformations:
Unity coordinates -> Drone coordinates -> perform rotation -> unity
coordinates
*/
Transform3 convertCameraAndDronePoseToUnityCoordinates(
    Transform3 world_T_body, Transform3 body_T_cam,
    Transform3 unityWorld_T_NEDworld)
{

    Transform3 world_T_cam = world_T_body * body_T_cam;
    Matrix4 cam_T_unitycam;
    // clang-format off
    cam_T_unitycam <<   0, 1, 0, 0,
                        0, 0, 1, 0,
                        1, 0, 0, 0,
                        0, 0, 0, 1;
    // clang-format on

    Transform3 world_T_unitycam;
    world_T_unitycam.matrix() =
        world_T_cam *
        cam_T_unitycam; // Move from z aligned camera to x aligned camera

    // x->z, y->x, z->-y
    Matrix4
        unity_from_drone; /**< Coordinate change from drone coordinate system to
                            unity coordinate system */
    // clang-format off
    unity_from_drone << 0, 1,  0, 0,
                        0, 0, -1, 0,
                        1, 0,  0, 0,
                        0, 0,  0, 1;
    // clang-format on

    Transform3 unityWorld_T_unitycam_unity; /**< Transformation from camera to
                                                world in unity coordinates */
    unityWorld_T_unitycam_unity.matrix() =
        unity_from_drone * unityWorld_T_NEDworld.matrix() *
        world_T_unitycam.matrix() * unity_from_drone.transpose();

    return unityWorld_T_unitycam_unity;
}


FlightGogglesClient2::FlightGogglesClient2(ros::NodeHandle ns):
    node_(ns),
    // TF listener
    tfListener_(tfBuffer_),
    //Image transport
    it_(ns)
{
    // Wait for static transforms between imu/cameras.
    try{
        imu_T_Camera_ = tfBuffer_.lookupTransform("uav/camera/left", "uav/imu", ros::Time(0),
                                                    ros::Duration(10.0));

    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT find required uav to camera transform: %s", ex.what());
        exit(1);
    }

    if (!ros::param::get("/uav/flightgoggles_ros_bridge/render_stereo", render_stereo)) {
        std::cout << "Did not get argument for render_stereo. Defaulting to false" << std::endl;
    }
    
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/image_width", imageWidth_)) {
        std::cout << "Did not get argument for image width. Defaulting to 1024 px" << std::endl;
    }
    
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/image_height", imageHeight_)) {
        std::cout << "Did not get argument for image height. Defaulting to 768 px" << std::endl;
    }
    
    if (!ros::param::get("/uav/flightgoggles_ros_bridge/baseline", baseline_)) {
        std::cout << "Did not get argument for baseline. Defaulting to 0.32 m" << std::endl;
    }


    // Load params
    populateRenderSettings();

    // init image publisher
    imagePubLeft_ = it_.advertiseCamera("/uav/camera/left/image_rect_color", 1);
    

    // Collision publisher
    collisionPub_ = node_.advertise<std_msgs::Empty>("/uav/collision", 1);


    // Subscribe to TF messages
    tfSubscriber_ = node_.subscribe("/tf", 1, &FlightGogglesClient2::tfCallback, this);


    initializeConnections();

    // init image publisher
    imagePubLeft_ = it_.advertiseCamera("/state/camera/left/image_rect_color", 1);
    transformPub_ = node_.advertise<geometry_msgs::TransformStamped>("/state/transform", 1);
    motorPub_ = node_.advertise<mav_msgs::Actuators>("/state/motorspeed", 1);
    rateThrustPub_ = node_.advertise<mav_msgs::RateThrust>("state/rateThrust", 1);
}

void FlightGogglesClient2::populateRenderSettings() {
    // Scene/Render settings
    /*
    Available scenes:
    sceneFilename = "Butterfly_World";
    sceneFilename = "FPS_Warehouse_Day";
    sceneFilename = "FPS_Warehouse_Night";
    sceneFilename = "Hazelwood_Loft_Full_Day";
    sceneFilename = "Hazelwood_Loft_Full_Night";

    // NEW for FlightGoggles v2.x.x
    flightGoggles.state.sceneFilename = "Abandoned_Factory_Morning";
    flightGoggles.state.sceneFilename = "Abandoned_Factory_Sunset";
    */

    state.sceneFilename = "Abandoned_Factory_Morning";
    state.camWidth = imageWidth_;
    state.camHeight = imageHeight_;

    // Prepopulate metadata of cameras
    unity_outgoing::Camera_t cam_RGB_left;
    cam_RGB_left.ID = "Camera_RGB_left";
    cam_RGB_left.channels = 3;
    cam_RGB_left.isDepth = false;
    cam_RGB_left.outputIndex = 0;
    cam_RGB_left.hasCollisionCheck = true;
    cam_RGB_left.doesLandmarkVisCheck = true;
    // Add cameras to persistent state
    state.cameras.push_back(cam_RGB_left);
    // set up the CameraInfo struct
    cameraInfoLeft = {};
    cameraInfoLeft.width = state.camWidth;
    cameraInfoLeft.height = state.camHeight;
    cameraInfoLeft.distortion_model = "plum_bob";
    float f = (cameraInfoLeft.height / 2.0) / tan((M_PI * (state.camFOV / 180.0)) / 2.0);
    float cx = cameraInfoLeft.width / 2.0;
    float cy = cameraInfoLeft.height / 2.0;
    float tx = 0.0;
    float ty = 0.0;
    cameraInfoLeft.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cameraInfoLeft.K = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
    cameraInfoLeft.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cameraInfoLeft.P = {f, 0.0, cx, tx, 0.0, f, cy, ty, 0.0, 0.0, 1.0, 0.0};
}

/// Connects to FlightGoggles.
void FlightGogglesClient2::initializeConnections()
{
    std::cout << "Initializing ZMQ connections..." << std::endl;
    // create and bind a download_socket
    download_socket.set(zmqpp::socket_option::receive_high_water_mark, 6);
    // download_socket.set(zmqpp::socket_option::receive_buffer_size, 1024*768*3*6);
    download_socket.bind(client_address + ":" + download_port);
    download_socket.subscribe("");
    std::cout << "Done!" << std::endl;
}

// Subscribe to all TF messages to avoid lag.
void FlightGogglesClient2::tfCallback(tf2_msgs::TFMessage::Ptr msg){
    //geometry_msgs::TransformStamped world_to_uav;
    bool found_transform = false;

    // Check if TF message is for world->uav/imu
    // This is output by dynamics node.
    for (auto transform : msg->transforms){
        if (transform.child_frame_id == "uav/imu"){
            //world_to_uav = transform;
            found_transform = true;
        }
    }

    // Skip if do not have transform
    if (!found_transform) return;

    // Skip every other transform to get an update rate of 60hz
    if (numSimulationStepsSinceLastRender_ >= numSimulationStepsBeforeRenderRequest_){

        // Get transform for left camera
        geometry_msgs::TransformStamped camLeftTransform;

        try{
            camLeftTransform = tfBuffer_.lookupTransform("world", "uav/camera/left/internal_nwu", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could NOT find transform for /uav/camera/left/nwu: %s", ex.what());
        }

        Transform3 camLeftPose = tf2::transformToEigen(camLeftTransform);
        setCameraPoseUsingROSCoordinates(camLeftPose, 0);

        // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
        state.ntime = camLeftTransform.header.stamp.toNSec();
        // request render
        // requestRender();

        numSimulationStepsSinceLastRender_ = 0;

    } else {
        numSimulationStepsSinceLastRender_++;
    }

}

/// Set camera pose using ROS coordinates.
void FlightGogglesClient2::setCameraPoseUsingROSCoordinates(Eigen::Affine3d ros_pose, int cam_index) {
    // To transforms
    Transform3 NED_pose = convertROSToNEDCoordinates2(ros_pose);
    Transform3 unity_pose = convertNEDGlobalPoseToGlobalUnityCoordinates2(NED_pose);
    //Transform3 unity_pose = convertEDNGlobalPoseToGlobalUnityCoordinates(ros_pose);

    // Extract position and rotation
    std::vector<double> position = {
        unity_pose.translation()[0],
        unity_pose.translation()[1],
        unity_pose.translation()[2],
    };

    Eigen::Matrix3d rotationMatrix = unity_pose.rotation();
    Quaternionx quat(rotationMatrix);

    std::vector<double> rotation = {
        quat.x(),
        quat.y(),
        quat.z(),
        quat.w(),
    };

    // Set camera position and rotation
    state.cameras[cam_index].position = position;
    state.cameras[cam_index].rotation = rotation;
}

/// Send render request to Unity
bool FlightGogglesClient2::requestRender() {
    // Create new message object
    zmqpp::message msg;
    // Add topic header
    msg << "Pose";

    // Update timestamp
    last_uploaded_utime = state.ntime;

    // Create JSON object for status update & append to message.
    json json_msg = state;
    msg << json_msg.dump();

    // Output debug messages at a low rate
    if (state.ntime > last_upload_debug_utime + 1e9)
    {
        // reset time of last debug message
        last_upload_debug_utime = state.ntime;
    }
    // // Send message without blocking.
    // upload_socket.send(msg, true);
    return true;
}

void FlightGogglesClient2::publishState(ros::Time timestamp,
                    geometry_msgs::TransformStamped transform,
                    mav_msgs::Actuators motorspeedsMessage,
                    mav_msgs::RateThrust::Ptr rateThrust) {
    // requestRender();

    // Wait for render result (blocking).
    unity_incoming::RenderOutput_t renderOutput = handleImageResponse();

    // Convert OpenCV image to image message
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", renderOutput.images[0]).toImageMsg();
    msg->header.stamp = timestamp;
    msg->header.frame_id = "/uav/camera/left";
    // Add Camera info message for camera
    sensor_msgs::CameraInfoPtr cameraInfoMsgCopy(new sensor_msgs::CameraInfo(cameraInfoLeft_));
    cameraInfoMsgCopy->header.frame_id = "/state/camera/left";
    cameraInfoMsgCopy->header.stamp = timestamp;
    imagePubLeft_.publish(msg, cameraInfoMsgCopy);

    transformPub_.publish(transform);

    // Publish the motorspeeds
    motorPub_.publish(motorspeedsMessage);
    
    // rateThrustPub_.publish(rateThrust);
    
}

/**
 * handleImageResponse handles the image response from Unity
 * Note: This is a blocking call.
 *
 * @return RenderOutput_t returns the render output with metadata
 */
unity_incoming::RenderOutput_t FlightGogglesClient2::handleImageResponse()
{
    // Populate output
    unity_incoming::RenderOutput_t output;

    // Get data from client as fast as possible
    zmqpp::message msg;
    download_socket.receive(msg);

    // Unpack message metadata.
    std::string json_metadata_string = msg.get(0);
    // Parse metadata.
    unity_incoming::RenderMetadata_t renderMetadata = json::parse(json_metadata_string);

    // Log the latency in ms (1,000 microseconds)
    if (!u_packet_latency)
    {
        u_packet_latency = (getTimestamp() - renderMetadata.ntime);
    }
    else
    {
        // avg over last ~10 frames in ms.
        u_packet_latency =
            ((u_packet_latency * (9) + (getTimestamp() - renderMetadata.ntime*1e-3)) / 10.0f);
    }

    ensureBufferIsAllocated(renderMetadata);

    output.images.resize(renderMetadata.cameraIDs.size());

    // For each camera, save the received image.
    for (uint i = 0; i < renderMetadata.cameraIDs.size(); i++)
    {
        // Reshape the received image
        // Calculate how long the casted and reshaped image will be.
        uint32_t imageLen = renderMetadata.camWidth * renderMetadata.camHeight * renderMetadata.channels[i];
        // Get raw image bytes from ZMQ message.
        // WARNING: This is a zero-copy operation that also casts the input to an array of unit8_t.
        // when the message is deleted, this pointer is also dereferenced.
        const uint8_t* imageData;
        msg.get(imageData, i + 1);
        // ALL images comes as 3-channel RGB images from Unity. Calculate the row length
        uint32_t bufferRowLength = renderMetadata.camWidth * renderMetadata.channels[i];

        // Pack image into cv::Mat
        cv::Mat new_image = cv::Mat(renderMetadata.camHeight, renderMetadata.camWidth, CV_MAKETYPE(CV_8U, renderMetadata.channels[i]));
	    memcpy(new_image.data, imageData, imageLen);
	    // Flip image since OpenCV origin is upper left, but Unity's is lower left.
	    cv::flip(new_image, new_image, 0);

        // Tell OpenCv that the input is RGB.
        if (renderMetadata.channels[i]==3){
            cv::cvtColor(new_image, new_image, CV_RGB2BGR);
        }

        // Add image to output vector
        output.images.at(i) = new_image;
    }

    // Add metadata to output
    output.renderMetadata = renderMetadata;

    // Output debug at 1hz
    if (getTimestamp() > last_download_debug_utime + 1e6)
    {
        last_download_debug_utime = getTimestamp();
        num_frames = 0;
    }
    num_frames++;

    return output;
}
