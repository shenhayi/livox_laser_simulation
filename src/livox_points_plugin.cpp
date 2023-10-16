//
// Created by lfc on 2021/2/28.
//

#include "livox_laser_simulation/livox_points_plugin.h"
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>

#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
namespace gazebo {

GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

LivoxPointsPlugin::LivoxPointsPlugin() {}

LivoxPointsPlugin::~LivoxPointsPlugin() {}

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos) {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (auto &data : datas) {
        if (data.size() == 3) {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;  //转化成标准的右手系角度
        } else {
            ROS_INFO_STREAM("data size is not 3!");
        }
    }
}

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {
    std::vector<std::vector<double>> datas;
    std::string file_name = sdf->Get<std::string>("csv_file_name");
    ROS_INFO_STREAM("load csv file name:" << file_name);
    if (!CsvReader::ReadCsvFile(file_name, datas)) {
        ROS_INFO_STREAM("cannot get csv file!" << file_name << "will return !");
        return;
    }
    sdfPtr = sdf;
    int argc = 0;
    char **argv = nullptr;
    auto curr_scan_topic = sdf->Get<std::string>("ros_topic");

    ros::init(argc, argv, curr_scan_topic);
    rosNode.reset(new ros::NodeHandle);
    rosPointPub = rosNode->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);
    rosPointNoReturnPub = rosNode->advertise<sensor_msgs::PointCloud2>(curr_scan_topic + "_no_returns", 5);
    rosMarkersPub = rosNode->advertise<visualization_msgs::MarkerArray>(curr_scan_topic + "_no_returns_marker", 1);
    raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

    node = transport::NodePtr(new transport::Node());
    node->Init(raySensor->WorldName());
    scanPub = node->Advertise<msgs::LaserScanStamped>(_parent->Topic(), 50);
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);
    ROS_INFO_STREAM("scan info size:" << aviaInfos.size());
    maxPointSize = aviaInfos.size();

    RayPlugin::Load(_parent, sdfPtr);
    laserMsg.mutable_scan()->set_frame(_parent->ParentName());

    parentEntity = this->world->EntityByName(_parent->ParentName());
    auto physics = world->Physics();
    laserCollision = physics->CreateCollision("multiray", _parent->ParentName());

    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(_parent->Pose());
    laserCollision->SetInitialRelativePose(_parent->Pose());
    rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(rayShape);

    samplesStep = sdfPtr->Get<int>("samples");
    downSample = sdfPtr->Get<int>("downsample");
    if (downSample < 1) {
        downSample = 1;
    }
    rayShape->RayShapes().reserve(samplesStep / downSample);
    rayShape->Load(sdfPtr);
    rayShape->Init();

    auto offset = laserCollision->RelativePose();
    ignition::math::Vector3d start_point, end_point;
    for (int j = 0; j < samplesStep; j += downSample) {
        int index = j % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = RangeMin() * axis + offset.Pos();
        end_point = RangeMax() * axis + offset.Pos();
        rayShape->AddRay(start_point, end_point);
    }

    raySensor->SetActive(true);
}

void LivoxPointsPlugin::OnNewLaserScans() {

    if (rayShape) {
        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);
        rayShape->Update();

        msgs::Set(laserMsg.mutable_time(), world->SimTime());
        msgs::LaserScan *scan = laserMsg.mutable_scan();

        InitializeScan(scan);
        auto rayCount = RayCount();
        auto verticalRayCount = VerticalRayCount();
        auto angle_min = AngleMin().Radian();
        auto angle_incre = AngleResolution();
        auto verticle_min = VerticalAngleMin().Radian();
        auto verticle_incre = VerticalAngleResolution();

        sensor_msgs::PointCloud2 scan_point;
        scan_point.header.stamp = ros::Time::now();
        scan_point.header.frame_id = raySensor->Name();
        sensor_msgs::PointCloud2Modifier modifier(scan_point);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points_pair.size());
        sensor_msgs::PointCloud2Iterator<float> out_x(scan_point, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(scan_point, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(scan_point, "z");

        const size_t raySize = samplesStep;
        size_t countNoReturn {0};
        for (const auto [idx, rotateInfo] : points_pair) {
            auto range = RangeMin() + rayShape->GetRange(idx);
            auto intensity = rayShape->GetRetro(idx);
            if (range >= RangeMax()) {
                range = 0.0;
            } else if (range <= RangeMin()) {
                range = 0.0;
                // this one should could be a point with no return -> publish it in second cloud with zenith / azimuth
                countNoReturn++;
            }

            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotateInfo.zenith, rotateInfo.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point = range * axis;
            *out_x = point.X();
            *out_y = point.Y();
            *out_z = point.Z();
            ++out_x;
            ++out_y;
            ++out_z;
        }
        if (scanPub && scanPub->HasConnections()) scanPub->Publish(laserMsg);
        rosPointPub.publish(scan_point);

        // also publish all points with no return
        sensor_msgs::PointCloud2 scan_point_no_return;
        scan_point_no_return.header.stamp = ros::Time::now();
        scan_point_no_return.header.frame_id = raySensor->Name();
        sensor_msgs::PointCloud2Modifier modifier_no_return(scan_point_no_return);
        modifier_no_return.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                                   "y", 1, sensor_msgs::PointField::FLOAT32,
                                                   "z", 1, sensor_msgs::PointField::FLOAT32,
                                                   "yaw", 1, sensor_msgs::PointField::FLOAT32,
                                                   "pitch", 1, sensor_msgs::PointField::FLOAT32); 
        modifier_no_return.resize(countNoReturn);
        sensor_msgs::PointCloud2Iterator<float> out_no_return_x(scan_point_no_return, "x");
        sensor_msgs::PointCloud2Iterator<float> out_no_return_y(scan_point_no_return, "y");
        sensor_msgs::PointCloud2Iterator<float> out_no_return_z(scan_point_no_return, "z");

        sensor_msgs::PointCloud2Iterator<float> out_yaw(scan_point_no_return, "yaw");
        sensor_msgs::PointCloud2Iterator<float> out_pitch(scan_point_no_return, "pitch");
        visualization_msgs::MarkerArray marker_array_msg;

        visualization_msgs::Marker marker_msg;
        marker_msg.header.frame_id = raySensor->Name();
        marker_msg.header.stamp = scan_point_no_return.header.stamp;
        marker_msg.ns = "livox_simulation";
        size_t marker_id {0};
        marker_msg.id = marker_id;
        marker_msg.type = visualization_msgs::Marker::ARROW;
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 0.0;
        marker_msg.color.b = 1.0;
        marker_msg.color.a = 1.0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.05;
        marker_msg.scale.y = 0.05;
        marker_msg.scale.z = 0.05;
        marker_msg.pose.position.z = 0.0;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.action = visualization_msgs::Marker::MODIFY;
        geometry_msgs::Point point_zero_msg;
        for (const auto [idx, rotateInfo] : points_pair) {
            auto range = rayShape->GetRange(idx);
            auto retro = rayShape->GetRetro(idx);
            if (range == 0.0 && retro >= 0.13369 && retro <= 0.13371)
            {
                // set to fixed range to be able to show it in rViz
                range = 15.0;
                *out_yaw = rotateInfo.azimuth;
                *out_pitch = rotateInfo.zenith;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotateInfo.zenith, rotateInfo.azimuth));
                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;
                *out_no_return_x = point.X();
                *out_no_return_y = point.Y();
                *out_no_return_z = point.Z();
                ++out_no_return_x;
                ++out_no_return_y;
                ++out_no_return_z;
                ++out_yaw;
                ++out_pitch;
                marker_msg.points.emplace_back(point_zero_msg);

                // Marker
                geometry_msgs::Point point_msg;
                point_msg.x = static_cast<double>(point.X());
                point_msg.y = static_cast<double>(point.Y());
                point_msg.z = static_cast<double>(point.Z());
                marker_msg.points.emplace_back(point_msg);
                marker_msg.id = marker_id++;
                marker_array_msg.markers.push_back(marker_msg);
                marker_msg.points.clear();
            }
        }
        rosPointNoReturnPub.publish(scan_point_no_return);

        // publish no returns also as Marker
        rosMarkersPub.publish(marker_array_msg);

        ros::spinOnce();
    }
}

void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                       boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape) {
    auto &rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    auto offset = laserCollision->RelativePose();
    int64_t end_index = currStartIndex + samplesStep;
    int ray_index = 0;
    auto ray_size = rays.size();
    points_pair.reserve(samplesStep);
    for (int k = currStartIndex; k < end_index; k += downSample) {
        auto index = k % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = RangeMin() * axis + offset.Pos();
        end_point = RangeMax() * axis + offset.Pos();
        if (ray_index < ray_size) {
            rays[ray_index]->SetPoints(start_point, end_point);
            points_pair.emplace_back(ray_index, rotate_info);
        }
        ray_index++;
    }
    currStartIndex += samplesStep;
}

void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan) {
    // Store the latest laser scans into laserMsg
    msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
    scan->set_angle_min(AngleMin().Radian());
    scan->set_angle_max(AngleMax().Radian());
    scan->set_angle_step(AngleResolution());
    scan->set_count(RangeCount());

    scan->set_vertical_angle_min(VerticalAngleMin().Radian());
    scan->set_vertical_angle_max(VerticalAngleMax().Radian());
    scan->set_vertical_angle_step(VerticalAngleResolution());
    scan->set_vertical_count(VerticalRangeCount());

    scan->set_range_min(RangeMin());
    scan->set_range_max(RangeMax());

    scan->clear_ranges();
    scan->clear_intensities();

    unsigned int rangeCount = RangeCount();
    unsigned int verticalRangeCount = VerticalRangeCount();
    for (unsigned int j = 0; j < verticalRangeCount; ++j) {
        for (unsigned int i = 0; i < rangeCount; ++i) {
            scan->add_ranges(0);
            scan->add_intensities(0);
        }
    }
}

ignition::math::Angle LivoxPointsPlugin::AngleMin() const {
    if (rayShape)
        return rayShape->MinAngle();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::AngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->MaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

double LivoxPointsPlugin::RangeMin() const {
    if (rayShape)
        return rayShape->GetMinRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

double LivoxPointsPlugin::RangeMax() const {
    if (rayShape)
        return rayShape->GetMaxRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

double LivoxPointsPlugin::RangeResolution() const {
    if (rayShape)
        return rayShape->GetResRange();
    else
        return -1;
}

int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

int LivoxPointsPlugin::RayCount() const {
    if (rayShape)
        return rayShape->GetSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

int LivoxPointsPlugin::RangeCount() const {
    if (rayShape)
        return rayShape->GetSampleCount() * rayShape->GetScanResolution();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

int LivoxPointsPlugin::VerticalRayCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

int LivoxPointsPlugin::VerticalRangeCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
    } else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

double LivoxPointsPlugin::VerticalAngleResolution() const {
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
}

}
