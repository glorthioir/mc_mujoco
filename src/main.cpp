#include "mj_sim.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cmath>
#include <iostream>
#include <thread>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

bool render_state = true;
std::chrono::time_point<std::chrono::system_clock> timer;
std::chrono::duration<double> elapsed_time;
double refresheRate = 30;
double frequency = 1.0/refresheRate;

void simulate(mc_mujoco::MjSim & mj_sim)
{
  bool done = false;
  while(!done && render_state)
  {
    mj_sim.stepSimulation();
  }
}

int main(int argc, char * argv[])
{
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error("mc_mujoco was compiled with {} but mc_rtc is at version {}, you might "
                       "face subtle issues or unexpected crashes, please recompile mc_mujoco",
                       mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }

  mc_mujoco::MjConfiguration config;
  {
    po::options_description desc("mc_mujoco options");
    po::positional_options_description p;
    p.add("mc-config", 1);
    // clang-format off
    desc.add_options()
      ("help", "Show this help message")
      ("mc-config", po::value<std::string>(&config.mc_config), "Configuration given to mc_rtc")
      ("step-by-step", po::bool_switch(&config.step_by_step), "Start the simulation in step-by-step mode")
      ("torque-control", po::bool_switch(&config.torque_control), "Enable torque control")
      ("without-controller", po::bool_switch(), "Disable mc_rtc controller inside mc_mujoco")
      ("without-visualization", po::bool_switch(), "Disable mc_mujoco GUI")
      ("without-mc-rtc-gui", po::bool_switch(), "Disable mc_rtc GUI")
      ("with-collisions", po::bool_switch(), "Visualize collisions model")
      ("without-visuals", po::bool_switch(), "Disable visuals display")
      ("sync", po::bool_switch(&config.sync_real_time), "Synchronize mc_mujoco simulation time with real time");
    // clang-format on
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);
    if(vm.count("help"))
    {
      std::cout << desc << "\n";
      return 0;
    }
    config.with_controller = !vm["without-controller"].as<bool>();
    config.with_visualization = !vm["without-visualization"].as<bool>();
    config.with_mc_rtc_gui = !vm["without-mc-rtc-gui"].as<bool>();
    if(!vm["without-visuals"].defaulted())
    {
      config.visualize_visual = !vm["without-visuals"].as<bool>();
    }
    if(!vm["with-collisions"].defaulted())
    {
      config.visualize_collisions = vm["with-collisions"].as<bool>();
    }
  }
  mc_mujoco::MjSim mj_sim(config);

  ros::init(argc, argv, "mujoco_video_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher pub_rgb_left = it.advertiseCamera("/HRP4CR/ZED_MINI_LEFT_1080P/image_raw", 1);
  image_transport::CameraPublisher pub_rgb_right = it.advertiseCamera("/HRP4CR/ZED_MINI_RIGHT_1080P/image_raw", 1);
  timer = std::chrono::system_clock::now();

  std::thread simThread(simulate, std::ref(mj_sim));

  while(render_state)
  {
    if(ros::ok()){
      elapsed_time = std::chrono::system_clock::now() - timer;
      if (elapsed_time.count() > frequency){
        mj_sim.publishCameraTopic(pub_rgb_left, pub_rgb_right);
        timer = std::chrono::system_clock::now();
      }
    }
    mj_sim.updateScene();
    render_state = mj_sim.render();
  }

  simThread.join();
  mj_sim.stopSimulation();
  return 0;
}