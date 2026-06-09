#ifndef TEST_GENERIC_H
#define TEST_GENERIC_H

/* includes //{ */

#include <ros/ros.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <pairs_lib/subscribe_handler.h>
#include <pairs_lib/service_client_handler.h>
#include <pairs_lib/transformer.h>
#include <pairs_lib/attitude_converter.h>
#include <pairs_lib/geometry/cyclic.h>
#include <pairs_lib/param_loader.h>
#include <pairs_lib/publisher_handler.h>

#include <pairs_msgs/ControlManagerDiagnostics.h>
#include <pairs_msgs/UavManagerDiagnostics.h>
#include <pairs_msgs/HwApiStatus.h>
#include <pairs_msgs/EstimationDiagnostics.h>
#include <pairs_msgs/Vec4.h>
#include <pairs_msgs/Vec1.h>
#include <pairs_msgs/GainManagerDiagnostics.h>
#include <pairs_msgs/ConstraintManagerDiagnostics.h>
#include <pairs_msgs/String.h>
#include <pairs_msgs/UavState.h>
#include <pairs_msgs/PathSrv.h>
#include <pairs_msgs/Float64Stamped.h>
#include <pairs_msgs/TrackerCommand.h>
#include <pairs_msgs/DynamicsConstraints.h>
#include <pairs_msgs/String.h>
#include <pairs_msgs/TrajectoryReference.h>
#include <pairs_msgs/GetPathSrv.h>
#include <pairs_msgs/VelocityReferenceStamped.h>
#include <pairs_msgs/VelocityReferenceSrv.h>
#include <pairs_msgs/ReferenceStampedSrv.h>
#include <pairs_msgs/ValidateReference.h>
#include <pairs_msgs/ValidateReferenceArray.h>
#include <pairs_msgs/TransformReferenceSrv.h>
#include <pairs_msgs/TransformVector3Srv.h>
#include <pairs_msgs/TransformPoseSrv.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

//}

namespace pairs_uav_testing
{

using radians  = pairs_lib::geometry::radians;
using sradians = pairs_lib::geometry::sradians;

using namespace std;

/* class UAVHandler //{ */

class UAVHandler {

public:
  UAVHandler(std::string uav_name, std::shared_ptr<pairs_lib::SubscribeHandlerOptions> shopts, std::shared_ptr<pairs_lib::Transformer> transformer,
             bool use_hw_api = true);

  virtual void initialize(std::string uav_name, std::shared_ptr<pairs_lib::SubscribeHandlerOptions> shopts, std::shared_ptr<pairs_lib::Transformer> transformer,
                          bool use_hw_api = true);

  virtual tuple<bool, string> checkPreconditions(void);

  void sleep(const double &duration);

  // TODO: consider if we need to add initialization checks
  tuple<bool, string> takeoff(void);
  tuple<bool, string> land(void);
  tuple<bool, string> landHome(void);
  tuple<bool, string> activateMidAir(void);

  tuple<bool, string> gotoAbs(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> gotoRel(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> gotoFcu(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> setHeading(const double &setpoint);
  tuple<bool, string> setHeadingRelative(const double &hdg);
  tuple<bool, string> gotoAltitude(const double &z);
  tuple<bool, string> gotoTrajectoryStart();
  tuple<bool, string> startTrajectoryTracking();
  tuple<bool, string> resumeTrajectoryTracking();
  tuple<bool, string> stopTrajectoryTracking();

  tuple<bool, string> gotoRelativeService(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> gotoService(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> setReferenceTopic(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);
  tuple<bool, string> setReferenceService(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);
  tuple<bool, string> gotoFcuService(const double &x, const double &y, const double &z, const double &hdg);

  tuple<bool, string> gotoReference(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);
  tuple<bool, string> gotoReferenceTopic(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);
  tuple<bool, string> referenceService(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);
  tuple<bool, string> referenceTopic(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);

  bool hasGoal(void);
  bool isFlyingNormally(void);
  bool isOutputEnabled(void);
  bool isAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance, const std::string frame_id = "");
  bool isAtPosition(const double &x, const double &y, const double &hdg, const double &pos_tolerance, const std::string frame_id = "");
  bool isReferenceAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance);

  std::optional<bool> isStationary(void);

  std::optional<double>          getSpeed(void);
  std::optional<double>          getHeading(void);
  std::optional<Eigen::Vector3d> getVelocity(const std::string frame_id);

  std::string                                  getActiveTracker(void);
  std::string                                  getActiveController(void);
  std::string                                  getActiveEstimator(void);
  std::optional<pairs_msgs::TrackerCommand>      getTrackerCmd(void);
  std::optional<double>                        getHeightAgl(void);
  std::optional<pairs_msgs::DynamicsConstraints> getCurrentConstraints(void);

  tuple<bool, string> setPathSrv(const pairs_msgs::Path &path_in);
  tuple<bool, string> setPathTopic(const pairs_msgs::Path &path_in);
  tuple<bool, string> switchEstimator(const std::string &estimator);
  tuple<bool, string> switchController(const std::string &controller);
  tuple<bool, string> switchTracker(const std::string &tracker);
  tuple<bool, string> setGains(const std::string &gains);
  tuple<bool, string> setConstraints(const std::string &constraints);
  tuple<bool, string> hover();

  tuple<std::optional<pairs_msgs::TrajectoryReference>, std::optional<Eigen::VectorXd>, string> getPathSrv(const pairs_msgs::Path &path_in);

  bool mrsSystemReady(void);

  tuple<bool, string> validateReference(const pairs_msgs::ReferenceStamped &msg);

  tuple<bool, std::optional<pairs_msgs::ValidateReferenceArray::Response>> validateReferenceArray(const pairs_msgs::ValidateReferenceArray::Request &request);

  std::tuple<bool, std::optional<std::string>, std::optional<geometry_msgs::PoseStamped>> transformPose(const geometry_msgs::PoseStamped &msg,
                                                                                                        std::string                       target_frame);
  tuple<bool, std::optional<std::string>, std::optional<pairs_msgs::ReferenceStamped>>      transformReference(const pairs_msgs::ReferenceStamped &msg,
                                                                                                             std::string                       target_frame);
  tuple<bool, std::optional<std::string>, std::optional<geometry_msgs::Vector3Stamped>>   transformVector3(const geometry_msgs::Vector3Stamped &msg,
                                                                                                           std::string                          target_frame);

  pairs_lib::SubscribeHandler<pairs_msgs::ControlManagerDiagnostics>    sh_control_manager_diag_;
  pairs_lib::SubscribeHandler<pairs_msgs::DynamicsConstraints>          sh_current_constraints_;
  pairs_lib::SubscribeHandler<pairs_msgs::UavManagerDiagnostics>        sh_uav_manager_diag_;
  pairs_lib::SubscribeHandler<pairs_msgs::EstimationDiagnostics>        sh_estim_manager_diag_;
  pairs_lib::SubscribeHandler<pairs_msgs::GainManagerDiagnostics>       sh_gain_manager_diag_;
  pairs_lib::SubscribeHandler<pairs_msgs::ConstraintManagerDiagnostics> sh_constraint_manager_diag_;
  pairs_lib::SubscribeHandler<pairs_msgs::UavState>                     sh_uav_state_;
  pairs_lib::SubscribeHandler<pairs_msgs::TrackerCommand>               sh_tracker_cmd_;
  pairs_lib::SubscribeHandler<pairs_msgs::Float64Stamped>               sh_height_agl_;
  pairs_lib::SubscribeHandler<pairs_msgs::Float64Stamped>               sh_max_height_;
  pairs_lib::SubscribeHandler<pairs_msgs::Float64Stamped>               sh_speed_;

  pairs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming_;
  pairs_lib::ServiceClientHandler<std_srvs::Trigger> sch_offboard_;
  pairs_lib::ServiceClientHandler<std_srvs::Trigger> sch_midair_activation_;
  pairs_lib::ServiceClientHandler<std_srvs::Trigger> sch_land_;
  pairs_lib::ServiceClientHandler<std_srvs::Trigger> sch_land_home_;
  pairs_lib::ServiceClientHandler<pairs_msgs::String>  sch_switch_estimator_;
  pairs_lib::ServiceClientHandler<pairs_msgs::String>  sch_switch_controller_;
  pairs_lib::ServiceClientHandler<pairs_msgs::String>  sch_switch_tracker_;
  pairs_lib::ServiceClientHandler<pairs_msgs::String>  sch_set_gains_;
  pairs_lib::ServiceClientHandler<pairs_msgs::String>  sch_set_constraints_;

  pairs_lib::ServiceClientHandler<pairs_msgs::Vec4>                sch_goto_;
  pairs_lib::ServiceClientHandler<pairs_msgs::Vec4>                sch_goto_fcu_;
  pairs_lib::ServiceClientHandler<pairs_msgs::PathSrv>             sch_path_;
  pairs_lib::ServiceClientHandler<pairs_msgs::Vec4>                sch_goto_relative_;
  pairs_lib::ServiceClientHandler<pairs_msgs::Vec1>                sch_set_heading_;
  pairs_lib::ServiceClientHandler<pairs_msgs::Vec1>                sch_set_heading_relative_;
  pairs_lib::ServiceClientHandler<pairs_msgs::Vec1>                sch_goto_altitude_;
  pairs_lib::ServiceClientHandler<pairs_msgs::ReferenceStampedSrv> sch_reference_;

  pairs_lib::ServiceClientHandler<pairs_msgs::GetPathSrv> sch_get_path_;

  pairs_lib::ServiceClientHandler<std_srvs::Trigger> sch_start_trajectory_tracking_;
  pairs_lib::ServiceClientHandler<std_srvs::Trigger> sch_stop_trajectory_tracking_;
  pairs_lib::ServiceClientHandler<std_srvs::Trigger> sch_resume_trajectory_tracking_;
  pairs_lib::ServiceClientHandler<std_srvs::Trigger> sch_goto_trajectory_start_;

  pairs_lib::ServiceClientHandler<pairs_msgs::ValidateReference>      sch_validate_reference_;
  pairs_lib::ServiceClientHandler<pairs_msgs::ValidateReferenceArray> sch_validate_reference_array_;

  pairs_lib::ServiceClientHandler<pairs_msgs::TransformReferenceSrv> sch_tranform_reference_;
  pairs_lib::ServiceClientHandler<pairs_msgs::TransformVector3Srv>   sch_tranform_vector3_;
  pairs_lib::ServiceClientHandler<pairs_msgs::TransformPoseSrv>      sch_tranform_pose_;

  pairs_lib::ServiceClientHandler<std_srvs::Trigger> sch_hover_;

  pairs_lib::PublisherHandler<pairs_msgs::Path>                     ph_path_;
  pairs_lib::PublisherHandler<pairs_msgs::TrajectoryReference>      ph_trajectory_;
  pairs_lib::PublisherHandler<pairs_msgs::VelocityReferenceStamped> ph_velocity_reference_;
  pairs_lib::PublisherHandler<pairs_msgs::ReferenceStamped>         ph_reference_;

  pairs_lib::SubscribeHandler<pairs_msgs::HwApiStatus> sh_hw_api_status_;

  std::shared_ptr<pairs_lib::Transformer> transformer_;

protected:
  bool initialized_ = false;

  string _uav_name_;

  std::shared_ptr<pairs_lib::SubscribeHandlerOptions> shopts_;
  ros::NodeHandle                                   nh_;
  string                                            name_;
  bool                                              use_hw_api_ = true;
};

//}

/* class TestGeneric //{ */

class TestGeneric {

public:
  TestGeneric();

  void initialize(void);

  virtual bool test() = 0;

  std::shared_ptr<pairs_lib::ParamLoader> pl_;

  std::tuple<std::optional<std::shared_ptr<UAVHandler>>, string> getUAVHandler(const string &uav_name, const bool use_hw_api = true);

  void sleep(const double &duration);

protected:
  ros::NodeHandle                       nh_;
  std::shared_ptr<pairs_lib::Transformer> transformer_;

  std::shared_ptr<pairs_lib::SubscribeHandlerOptions> shopts_;

  string _uav_name_;  // TODO: remove, should be UAVHandler specific

  string _test_name_;
  string name_;

  bool initialized_ = false;

  bool mrsSystemReady(void);

private:
  shared_ptr<ros::AsyncSpinner> spinner_;
};

//}

}  // namespace pairs_uav_testing

#endif  // TEST_GENERIC_H
