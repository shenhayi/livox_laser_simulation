#include <livox_laser_simulation/livox_gpu_points_plugin.h>

#include "livox_laser_simulation/csv_reader.hpp"

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
  nh_ = new ros::NodeHandle();
  pub_ = nh_->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);

  raySensor = std::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);

  auto sensor_pose = raySensor->Pose();
  SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());

  raySensor->SetActive(true);
  sub_ = gazebo_node_->Subscribe(raySensor->Topic(), &LivoxGpuPointsPlugin::OnNewLaserAnglesScans, this);
}

void
LivoxGpuPointsPlugin::SendRosTf(const ignition::math::Pose3d& pose,
                                const std::string& /*father_frame*/,
                                const std::string& /*child_frame*/)
{
  if (!tfBroadcaster)
  {
    tfBroadcaster.reset(new tf::TransformBroadcaster);
  }
  tf::Transform tf;
  auto rot = pose.Rot();
  auto pos = pose.Pos();
  tf.setRotation(tf::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W()));
  tf.setOrigin(tf::Vector3(pos.X(), pos.Y(), pos.Z()));

  tfBroadcaster->sendTransform(tf::StampedTransform(tf, ros::Time::now(), raySensor->ParentName(), raySensor->Name()));
}

void
LivoxGpuPointsPlugin::OnNewLaserAnglesScans(ConstLaserScanAnglesStampedPtr& _msg)
{
  const double MIN_RANGE = raySensor->RangeMin();;
  const double MAX_RANGE = raySensor->RangeMax();;
  const double MIN_INTENSITY = 0.0;

  const unsigned int sampleSize = raySensor->SampleSize();

  // Populate message fields
  const unsigned int POINT_STEP = 22;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = raySensor->Name(); // laser_livox
  msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  msg.fields.resize(6);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 16;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.fields[5].name = "time";
  msg.fields[5].offset = 18;
  msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[5].count = 1;
  msg.data.resize(sampleSize * POINT_STEP);

  uint8_t* ptr = msg.data.data();
  for (unsigned int i = 0; i < sampleSize; i++)
  {
    double r = _msg->scan().ranges(i);
    double intensity = _msg->scan().intensities(i);

    // Get angles of ray to get xyz for point
    double yAngle = _msg->scan().azimuth(i);
    double pAngle = _msg->scan().zenith(i);

    // pAngle is rotated by yAngle:
    if ((MIN_RANGE < r) && (r < MAX_RANGE))
    {
      // ROS_INFO_STREAM("Range: " << r);
      *((float*)(ptr + 0)) = r * cos(pAngle) * cos(yAngle); // x
      *((float*)(ptr + 4)) = r * cos(pAngle) * sin(yAngle); // y
      *((float*)(ptr + 8)) = r * sin(pAngle); // z
      *((float*)(ptr + 12)) = intensity; // intensity
      *((uint16_t*)(ptr + 16)) = 0; // ring
      *((float*)(ptr + 18)) = 0.0; // time
      ptr += POINT_STEP;
    }
  }
  // Populate message with number of valid points
  msg.data.resize(ptr - msg.data.data()); // Shrink to actual size
  msg.point_step = POINT_STEP;
  msg.is_bigendian = false;
  msg.width = msg.data.size() / POINT_STEP;
  msg.height = 1;
  msg.row_step = msg.data.size();
  msg.is_dense = true;

  // Publish output
  pub_.publish(msg);
}

} // namespace gazebo
