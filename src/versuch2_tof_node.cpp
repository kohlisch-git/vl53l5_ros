#include "versuch2_tof_node.h"

/**
 * @brief Construct a new versuch2 tof node::versuch2 tof node object
 * @details initialize ros, publishers, parameters and open serial
 */
versuch2_tof_node::versuch2_tof_node()
{
    std::string device_name, output_topic;
    ros::NodeHandle nh("~");
    nh.param("device_name", device_name, std::string("/dev/ttyACM0"));
    nh.param("output_topic", output_topic, std::string("/vl53l5/cloud"));
    nh.param("sensor_frame", sensor_frame, std::string("map"));
    _tof_dev = new SerialPort(device_name.c_str(), B115200);
    _pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
    cloud_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

}

/**
 * @brief Destroy the versuch2 tof node::versuch2 tof node object
 * @details delete singelton
 */
versuch2_tof_node::~versuch2_tof_node() 
{
     if (_instance) 
        delete _instance; 
    _instance = 0; 
};


/**
 * @brief Endless loop
 * @details There is no spin, since serial has no callback, so we poll
 * 
 */
void versuch2_tof_node::run(void)
{
    ros::Rate rate(400);
    while (ros::ok()) {
        if (_tof_dev->receive(rx_buf, PKG_SIZE)) {
            /*********** DEBUGGING ***********/
            // std::cout << "msg:\n";
            // for (int i = 0; i < 6; ++i){
            //     for (int j = 0; j < 8; ++j)
            //         std::cout << (unsigned)((uint8_t)rx_buf[i*8+j]) << "\t";
            //     std::cout << "\n";
            // }
            convert();
        }
        ros::spinOnce();
        rate.sleep();
    }
}

/**
 * @brief Convert data received by serial
 * @details Previously thought to capture multiple packages (IMU + TOF) but the PCBs were incompatible
 * 
 */
void versuch2_tof_node::convert(void)
{
    if (rx_buf[(unsigned)PkgType] == (unsigned)RangePkg) {
        convertRange();
    } else {
        std::cout << "msg unknown: " << (int)rx_buf[(unsigned)PkgType] << "\n"; 
        _tof_dev->flush();
    }
}


/**
 * @brief Convert bytes from serial to uint16 measurements
 * @details fill arry with bytes of 1 or 3 packages
 */
void versuch2_tof_node::convertRange(void)
{
    uint16_t tmp;
    unsigned sensor_id;

    if (rx_buf[(unsigned)PkgNumEst] == 1) {
        memset((uint8_t*)&_distance_mm[0], 0, MAX_N_ZONES * sizeof(uint16_t)); // reset possible mesurements from other resolution
        _msg_resolution_4x4 = true;
    } else
        _msg_resolution_4x4 = false;

    for (unsigned i = PKG_HEAD; i < PKG_SIZE; i += sizeof(tmp)) {
        tmp = ((((uint16_t)rx_buf[i]) & 0x00FF) | (uint16_t)((rx_buf[i+1] << 8) & 0xFF00));
        sensor_id = ((i - PKG_HEAD) + (rx_buf[(unsigned)PkgNum] - 1) * (PKG_SIZE - PKG_HEAD)) / sizeof(tmp);
        if (sensor_id >= MAX_N_ZONES) {
            break;
        }
        _distance_mm[sensor_id] = tmp;
    }
    if (rx_buf[(unsigned)PkgNum] == rx_buf[(unsigned)PkgNumEst]) {
        // std::cout << "Zone results:\n";
        // for (int i = 0; i < 8; ++i){
        //     for (int j = 0; j < 8; ++j)
        //         std::cout << (int)_distance_mm[i*8+j] << "\t";
        //     std::cout << "\n";
        // }
        publishCloud();
    }
}

/**
 * @brief Publish a point cloud from measurements
 * @details Do a conversion from polar to Cartesian
 */
void versuch2_tof_node::publishCloud(void)
{

    float sensor_h_fov = 45.f, sensor_v_fov = 45.f;
    float v_angle_inc, h_angle_inc, v_angle_initial, h_angle_initial, x, y, z, h_angle, v_angle;
    v_angle_initial = sensor_v_fov * 0.5f;
    h_angle_initial = sensor_h_fov * 0.5f;
    uint8_t step = 0, v_pos, h_pos, n_zones;

    if (_msg_resolution_4x4) {
        /* 4x4 zones -> points in center of zone */
        v_angle_inc = sensor_v_fov / 5.f;
        h_angle_inc = sensor_h_fov / 5.f;
        step = 4;
        n_zones = 16;
    }
    else {
        /* 8x8 zones -> points in center of zone */
        v_angle_inc = sensor_v_fov / 9.f;
        h_angle_inc = sensor_h_fov / 9.f;
        step = 8;
        n_zones = MAX_N_ZONES;
    }
    /* append points to cloud */
    for (uint8_t i = 0; i < n_zones; ++i) {
        /* skip invalid measurements */
        if (_distance_mm[i] > 0x1FFF)
            continue;
        /* align points in center of zone */
        v_pos = (i % step) + 1;
        h_pos = (i / step) + 1;
        v_angle = (v_angle_initial - v_pos * v_angle_inc) * M_PI/ 180.f;
        h_angle = (h_angle_initial - h_pos * h_angle_inc) * M_PI/ 180.f;
        /* x-axis is distance, y-axis left to right on sensor, z is bottom to top on sensor */
        x = _distance_mm[i] * 0.001;
        if (_distance_mm[i] == 0)   // having points in the origin is dangerous!
            x = 0.0001;             // -> make them close to origin, but not 0!
        y = x * std::sin(v_angle);
        z = x * std::sin(h_angle) * std::cos(v_angle);
        cloud_ptr->insert(cloud_ptr->end(), pcl::PointXYZ(x,y,z));
    }
    /* convert to ROS-format and publish */
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_ptr, output);
    output.header.frame_id = sensor_frame;
    output.header.stamp = ros::Time::now();
    _pub_cloud.publish(output);
    cloud_ptr->clear();
    
}

/**
 * @brief 
 * 
 * @param cfg 
 */
void versuch2_tof_node::setCfg(const versuch2_tof::SensorConfig &cfg)
{
    char buf[5] = {0};
    if (cfg.ranging_mode)
        buf[0] |= 0x01 << (uint8_t)mode_continuous;
    if (cfg.resolution)
        buf[0] |= 0x01 << (uint8_t)resolution_4x4;
    if (cfg.sensor_to_sleep)
        buf[0] |= 0x01 << (uint8_t)sensor_to_sleep;
    if (cfg.target_order)
        buf[0] |= 0x01 << (uint8_t)order_by_strongest;

    if (cfg.frequency <=  60 && cfg.frequency > 0) {
        if (cfg.resolution)
            buf[1] = cfg.frequency;
        else if (cfg.resolution == 0 && cfg.frequency <= 15)
            buf[1] = cfg.frequency;
        else
            buf[1] = 15;
    } else
        buf[1] = 15;

    if (cfg.sharpener >= 0 && cfg.sharpener < 100)
        buf[2] = cfg.sharpener;
    else 
        buf[2] = 5;

    if (cfg.integration_time > 0 && cfg.integration_time < 1000) {
        buf[3] = (uint8_t)(cfg.integration_time & 0x000000FF);
        buf[4] = (uint8_t)((cfg.integration_time >> 8) & 0x000000FF);
    } else {
        buf[3] = 2;
        buf[4] = 0;
    }

    while (_tof_dev->send(buf, 5) != 5);
}

/**
 * @brief 
 * 
 * @param cfg 
 * @param level 
 */
void reconfigureCallback(versuch2_tof::SensorConfig &cfg, uint32_t level)
{
    ROS_INFO("Params changed via reconfigure");
    (void)level;
    versuch2_tof_node::getInstance()->setCfg(cfg);
}

/**
 * @brief main application entry point
 * 
 * @param argc number of command line arguments
 * @param argv command line arguments
 * @return int 
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name.c_str());
    dynamic_reconfigure::Server<versuch2_tof::SensorConfig> srv;
    dynamic_reconfigure::Server<versuch2_tof::SensorConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2);
    srv.setCallback(f);
    versuch2_tof_node::getInstance()->run();
    return 0;
}