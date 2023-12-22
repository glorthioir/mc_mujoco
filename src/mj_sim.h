#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <queue>

#include "mujoco.h"

#include "mj_configuration.h"

namespace mc_mujoco
{

struct MjSimImpl;

struct MjSim
{
public:
  /*! \brief Constructor
   *
   * Prepare to start a simulation.
   *
   * \param config Configuration for mc_mujoco
   *
   * \param mc_config Configuration file used by mc_rtc
   *
   */
  MjSim(const MjConfiguration & config);

  /*! \brief Destructor */
  ~MjSim();

  /** Reset the simulation and controller to the given state */
  void resetSimulation(const std::map<std::string, std::vector<double>> & reset_qs = {},
                       const std::map<std::string, sva::PTransformd> & reset_pos = {});

  /** Plays one step of physics simulation, should be called as often as possible
   *
   * \returns True if the controller fails and the simulation should stop
   */
  bool stepSimulation();

  /*! Stop the simulation */
  void stopSimulation();

  /*! Render the facial cameras and publish them to ROS */
  void publishCameraTopic(image_transport::CameraPublisher & pub_rgb_left, image_transport::CameraPublisher & pub_rgb_right, std::queue<cv::Mat> & buffer_left,
                          std::queue<cv::Mat> & buffer_right, std::chrono::duration<double> time_from_start, int framerate, double delay_seconds);

  /*! Prepare to render */
  void updateScene();

  /*! Update the GUI, no-op if visualization is disabled
   *
   * \returns False if the application should quit
   */
  bool render();

  /** The underlying global controller instance in the simulation
   *
   * nullptr if with_controller was false in MjConfiguration
   */
  mc_control::MCGlobalController * controller() noexcept;

  /** Return the MuJoCo model */
  mjModel & model() noexcept;

  /** Return the MuJoCo data */
  mjData & data() noexcept;

private:
  std::unique_ptr<MjSimImpl> impl;

};

} // namespace mc_mujoco
