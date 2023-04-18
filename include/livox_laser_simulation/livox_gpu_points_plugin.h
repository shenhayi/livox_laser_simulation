#ifndef LIVOX_GPU_POINTS_PLUGIN_H_
#define LIVOX_GPU_POINTS_PLUGIN_H_

#include <gazebo/plugins/GpuRayPlugin.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <ignition/math6/ignition/math.hh>

#include <sensor_msgs/PointCloud2.h>

#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <tf/transform_broadcaster.h>

#include <sdf/Param.hh>

#include <vector>

namespace gazebo
{

class LivoxGpuPointsPlugin : public GpuRayPlugin
{
public:
  LivoxGpuPointsPlugin();

  ~LivoxGpuPointsPlugin();

  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  void SendRosTf(const ignition::math::Pose3d& pose, const std::string& /*father_frame*/, const std::string& /*child_frame*/);

  void OnNewLaserAnglesScans(ConstLaserScanAnglesStampedPtr& _msg);

  ros::NodeHandle* nh_;
  ros::Publisher pub_;

  std::shared_ptr<tf::TransformBroadcaster> tfBroadcaster;

  gazebo::sensors::GpuRaySensorPtr raySensor;

  // Subscribe to gazebo laserscan
  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::SubscriberPtr sub_;
};
} // namespace gazebo

#endif /* LIVOX_GPU_POINTS_PLUGIN_H_ */
