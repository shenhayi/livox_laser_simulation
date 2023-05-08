#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_gpu_points_plugin.h"

#include <sensor_msgs/point_cloud2_iterator.h>

namespace gazebo
{

GZ_REGISTER_SENSOR_PLUGIN(LivoxGpuPointsPlugin)

LivoxGpuPointsPlugin::LivoxGpuPointsPlugin()
  : nh_(nullptr)
{
}

LivoxGpuPointsPlugin::~LivoxGpuPointsPlugin() {}

void
LivoxGpuPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Load plugin
  GpuRayPlugin::Load(_parent, _sdf);
  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init();

  auto curr_scan_topic = _sdf->Get<std::string>("ros_topic");

  // Create Node Handle
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  pub_ = nh_->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);

  raySensor = std::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);

  raySensor->SetActive(true);
  sub_ = gazebo_node_->Subscribe(raySensor->Topic(), &LivoxGpuPointsPlugin::OnNewLaserAnglesScans, this);
}

void
LivoxGpuPointsPlugin::OnNewLaserAnglesScans(ConstLaserScanAnglesStampedPtr& _msg)
{
  const double MIN_RANGE = raySensor->RangeMin();
  const double MAX_RANGE = raySensor->RangeMax();
  constexpr double MIN_INTENSITY = 0.0;
  const unsigned int sampleSize = raySensor->SampleSize();

  // Populate message fields
  const unsigned int POINT_STEP = 22;

  sensor_msgs::PointCloud2Ptr cloud = boost::make_shared<sensor_msgs::PointCloud2>();
  cloud->header.frame_id = raySensor->Name(); // laser_livox
  cloud->header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  cloud->fields.resize(6);
  cloud->fields[0].name = "x";
  cloud->fields[0].offset = 0;
  cloud->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud->fields[0].count = 1;
  cloud->fields[1].name = "y";
  cloud->fields[1].offset = 4;
  cloud->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud->fields[1].count = 1;
  cloud->fields[2].name = "z";
  cloud->fields[2].offset = 8;
  cloud->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud->fields[2].count = 1;
  cloud->fields[3].name = "intensity";
  cloud->fields[3].offset = 12;
  cloud->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  cloud->fields[3].count = 1;
  cloud->fields[4].name = "ring";
  cloud->fields[4].offset = 16;
  cloud->fields[4].datatype = sensor_msgs::PointField::UINT16;
  cloud->fields[4].count = 1;
  cloud->fields[5].name = "time";
  cloud->fields[5].offset = 18;
  cloud->fields[5].datatype = sensor_msgs::PointField::FLOAT32;
  cloud->fields[5].count = 1;
  cloud->data.resize(sampleSize * POINT_STEP);
  cloud->point_step = POINT_STEP;
  cloud->is_bigendian = false;
  cloud->width = sampleSize;
  cloud->height = 1;
  cloud->row_step = sampleSize * cloud->point_step;
  cloud->is_dense = true;

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cloud, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(*cloud, "ring");
  sensor_msgs::PointCloud2Iterator<float> iter_time(*cloud, "time");

  for (size_t i = 0; i < sampleSize; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_ring, ++iter_time)
  {
    double r = _msg->scan().ranges(i);
    double intensity = _msg->scan().intensities(i);

    // Get angles of ray to get xyz for point
    double yAngle = _msg->scan().azimuth(i);
    double pAngle = _msg->scan().zenith(i);

    // pAngle is rotated by yAngle:
    if ((MIN_RANGE < r) && (r < MAX_RANGE))
    {
      *iter_x = r * cos(pAngle) * cos(yAngle);
      *iter_y = r * cos(pAngle) * sin(yAngle);
      *iter_z = r * sin(pAngle);
      *iter_intensity = intensity;
      *iter_ring = 0;
      *iter_time = 0.0;
    }
  }
  // Publish output
  pub_.publish(cloud);
}

} // namespace gazebo
