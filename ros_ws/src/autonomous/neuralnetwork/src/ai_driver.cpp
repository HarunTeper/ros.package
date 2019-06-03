#include "ai_driver.h"

#include "ai_util.h"

using namespace ai_driver;

AiDriver::AiDriver()
{
    ros::NodeHandle private_node_handle("~");

    std::string config_folder;
    std::string config_file;

    private_node_handle.getParam(PARAMETER_CONFIG_FOLDER, config_folder);
    private_node_handle.getParam(PARAMETER_CONFIG_FILE, config_file);
    private_node_handle.getParam(PARAMETER_UPDATE_RATE, m_update_rate);

    m_lidar_subscriber =
        m_node_handle.subscribe<sensor_msgs::LaserScan>(TOPIC_LASER_SCAN_SUBSCRIBE, 1, &AiDriver::lidarCallback, this);
    m_net_deploy_subscriber = m_node_handle.subscribe<neuralnetwork::net_param>(TOPIC_NET_DEPLOY_SUBSCRIBE, 1,
                                                                                &AiDriver::netDeployCallback, this);
    m_drive_parameters_publisher = m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAMETERS_PUBLISH, 1);
    m_timer = m_node_handle.createTimer(ros::Duration(1.0 / m_update_rate), &AiDriver::timerCallback, this);

    std::string file_path = config_folder + "/" + config_file;
    if (config_file.compare("") == 0)
    {
        ROS_INFO_STREAM("ai_driver no config file given (waiting for deployment)");
        m_deployed = false;
    }
    else if (m_net.create_from_file(file_path))
    {
        ROS_INFO_STREAM("successfully loaded ai driver from " + file_path);
        m_deployed = true;
    }
    else
    {
        ROS_ERROR_STREAM("could not load " + file_path);
        ROS_ERROR_STREAM("maybe the folder or reading permission is missing");
    }
}

void AiDriver::timerCallback(const ros::TimerEvent&)
{
    update();
}

void AiDriver::publishDriveParameters(fann_type velocity, fann_type angle)
{
    if (velocity < -1 || velocity > 1)
    {
        ROS_WARN_STREAM("ai_driver speed should be -1 < x < 1 but it is " << std::to_string(velocity));
    }
    if (angle < -1 || angle > 1)
    {
        ROS_WARN_STREAM("ai_driver angle should be -1 < x < 1 but it is " << std::to_string(angle));
    }
    drive_msgs::drive_param drive_parameters;
    drive_parameters.velocity = velocity;
    drive_parameters.angle = angle;
    m_drive_parameters_publisher.publish(drive_parameters);
}

void AiDriver::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    for (int i = 0; i < 5; i++)
    {
        float value = lidar->ranges[LIDAR_INDICES[i]];
        value = std::min(value, lidar->range_max);
        value = std::max(value, lidar->range_min);
        m_input[i + 2] = value;
    }
    m_changes_lidar++;
}

void AiDriver::netDeployCallback(const neuralnetwork::net_param::ConstPtr& data)
{
    long size = data->size;
    FANN::connection arr[size];
    m_net.get_connection_array(arr);
    for (int i = 0; i < size; i++)
    {
        arr[i].weight = data->weights[i];
    }
    m_net.create_standard_array(ai_config::DEFAULT_NUM_LAYERS, ai_config::DEFAULT_LAYER_ARRAY);
    m_net.set_weight_array(arr, size);

    m_deployed = true;
}

void AiDriver::update()
{
    if (m_deployed == false)
    {
        return;
    }

    // check data age
    if (m_changes_lidar == 0)
    {
        ROS_WARN_STREAM("ai_driver: no lidar update since last neural net update");
    }
    m_changes_lidar = 0;

    // run network
    m_output = m_net.run(m_input);

    // redirecting speed and angle of the output back as inputs
    m_input[0] = m_output[0];
    m_input[1] = m_output[1];

    // publish outputs
    fann_type speed = m_output[0];
    fann_type angle = m_output[1] * 2 - 1;

    publishDriveParameters(speed, angle);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ai_driver");
    AiDriver ai_driver;
    ros::spin();
    return EXIT_SUCCESS;
}