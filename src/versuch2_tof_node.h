#ifndef VERSUCH2_TOF_NODE_H
#define VERSUCH2_TOF_NODE_H

#pragma once

#include <ros/ros.h>
#include "SerialPort.h"
#include <dynamic_reconfigure/server.h>
#include <stdint.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <versuch2_tof/SensorConfig.h>

constexpr unsigned PKG_SIZE = 48;
constexpr unsigned PKG_HEAD = 4;
constexpr unsigned MAX_N_ZONES = 64;

const std::string node_name("versuch2_tof_node");

typedef enum PackageType {
	Empty,
	RangePkg
} PackageType;

typedef enum StartingBytes {
	PkgType,
	PkgNum,
	PkgNumEst,
	PkgId
} StartingBytes;

typedef enum ControlFlags {
	mode_continuous,
	power_on,
	sensor_to_sleep,
	resolution_4x4,
	order_by_strongest
} ControlFlags;

class versuch2_tof_node
{
public:
    versuch2_tof_node();
    ~versuch2_tof_node();
    void run(void);
    void setCfg(const versuch2_tof::SensorConfig &cfg);
    void convert(void);

    static versuch2_tof_node* getInstance(void) 
        { if (!_instance) _instance = new versuch2_tof_node; return _instance; };
    inline static versuch2_tof_node* _instance = 0;
private:
    SerialPort* _tof_dev;
    char rx_buf[PKG_SIZE];
    uint16_t _distance_mm[MAX_N_ZONES] = {0};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = nullptr;
    std::string sensor_frame;

    ros::Publisher _pub_cloud;
    void convertRange(void);
    void publishCloud(void);
    bool _msg_resolution_4x4;

};

#endif