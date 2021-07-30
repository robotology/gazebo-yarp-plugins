# Changelog
All notable changes to this project will be documented in this file.

The format of this document is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

### Added
- The multicamera plugin now implements the `yarp::dev::IRgbVisualParams` interface (https://github.com/robotology/gazebo-yarp-plugins/pull/558).
- The controlboard and multicamera plugins now handle the `yarpDeviceName` parameter (https://github.com/robotology/gazebo-yarp-plugins/pull/559).

### Changed
- `gazebo_yarp_multicamera`, `gazebo_yarp_lasersensor`, `gazebo_yarp_doublelaser`, `gazebo_yarp_controlboard` and `gazebo_yarp_depthCamera` plugins now log messages using the ["Log Components" YARP logging feature](http://www.yarp.it/git-master/yarp_logging.html).
- A cmake option (`GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS`) has been added, if this option is enabled then implicit wrappers present in`gazebo_yarp_multicamera`, `gazebo_yarp_lasersensor`, `gazebo_yarp_controlboard` and `gazebo_yarp_depthCamera` are removed, the new way to have them is to attach the new nws to gazebo devices via yarprobotinterface.
- `gazebo-yarp-plugins` now requires YARP 3.5 (https://github.com/robotology/gazebo-yarp-plugins/pull/562).

### Fixed
- Fixed the getRgbIntrinsicParam method in the depthCamera plugin when the distortion is not set (https://github.com/robotology/gazebo-yarp-plugins/pull/558).
- The property returned by `getRgbIntrinsicParam()`, now contains `rectificationMatrix` instead of `rectificationMatrix` (https://github.com/robotology/gazebo-yarp-plugins/pull/558, see also https://github.com/robotology/yarp/pull/2593).

## [3.6.1] - 2021-05-19

### Fixed 
- Fixed use of libYARP_robotinterface with YARP devices spawned by sensor plugins (https://github.com/robotology/gazebo-yarp-plugins/pull/544).
- Fixed compilation against YARP 3.5 by removing spurious print in WorldInterface plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/548).

## [3.6.0] - 2021-02-24

### Added
- Add `refSpeed` and `refAcceleration` options to the `TRAJECTORY_GENERATION` group of the `gazebo_yarp_controlboard` plugin
  configuration. They are expected to hold exactly n_joints values (an error is reported otherwise) describing reference speeds
  and accelerations, respectively, for use by the selected trajectory generator (if necessary).
- Add `trapezoidal_speed` as a new supported value for option `trajectory_type` of the `gazebo_yarp_controlboard` plugin
  configuration. This generator enables the trajectory to follow a trapezoidal speed profile in position control mode, limited
  by provided reference speed (saturation) and acceleration (both ramps) values. If already executing a trajectory in this manner,
  newly generated trajectories take into account previous joint velocities and update the motion accordingly.
- Add `gazebo_yarp_robotinterface` plugin, the documentation for it can be found at [plugins/robotinterface/README.md](plugins/robotinterface/README.md) (https://github.com/robotology/gazebo-yarp-plugins/pull/532).  
- The `gazebo_yarp_depthcamera` and `gazebo_yarp_doublesensor` now accept a `yarpDeviceName` parameter (https://github.com/robotology/gazebo-yarp-plugins/pull/532).

### Changed 
- The `deviceId` parameter of the `gazebo_yarp_lasersensor` is now named `yarpDeviceName` )https://github.com/robotology/gazebo-yarp-plugins/pull/532).
  
### Fixed
- Fix the support for running Gazebo itself with the `gazebo_yarp_clock` with YARP_CLOCK set, without Gazebo freezing at startup.  
  In particular, setting YARP_CLOCK is suggested to ensure that all the threads of YARP Network Wrapper Servers are executed with 
  the frequency correctly synchronized with the Gazebo simulation (https://github.com/robotology/gazebo-yarp-plugins/pull/537).

## [3.5.1] - 2020-10-05

### Added
- Add `velocityControlImplementationType` option to the `VELOCITY_CONTROL` group of the `gazebo_yarp_controlboard` plugin
  configuration. This option permits to switch between `direct_velocity_pid`, that is using a velocity PID for the Velocity Control Mode
  (what has been implemented until now) and `integrator_and_position_pid` that uses an integrator that integrates the velocity reference and then uses the position low level PID, similarly to what is implement on real YARP-powered robot such as iCub or R1.
  The setting is now optional and if not present will default to `direct_velocity_pid`, but it will be compulsory in
  gazebo-yarp-plugins 4.x (https://github.com/robotology/gazebo-yarp-plugins/pull/514).

### Fixed
- Fixed use of the `VOCAB_CM_MIXED` control mode when the physics timestep is different from 1 millisecond (https://github.com/robotology/gazebo-yarp-plugins/pull/514).
- Fixed missing initialization of a pointer in `gazebo_yarp_controlboard` . In some cases this was causing crashes when a model that contained a `gazebo_yarp_controlboard`
  plugin was removed from the simulation (https://github.com/robotology/gazebo-yarp-plugins/pull/514).

## [3.5.0] - 2020-08-26

### Added
- Add the implementation to import models in the gazebo world in two configuration. First, load it directly with the pose in the sdf-file or second, load it with a given pose. (https://github.com/robotology/gazebo-yarp-plugins/pull/467)
- Add external wrench smoothing feature for `externalwrench` plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/495)
- Add the possibility to define initial configuration for `linkattacher` plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/497)
- Add the possibility to specify limits for coupled joints for `gazebo_yarp_controlboard` plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/499)

### Fixed
- Fixed empty string return for creation of the cylinder object. (https://github.com/robotology/gazebo-yarp-plugins/pull/467)
- Fixed empty pointer error in the creation process of the objects (sphere, box, cylinder) and the frame. (https://github.com/robotology/gazebo-yarp-plugins/pull/467)
- Fixed wrong measurements feedback used for coupled joints in `gazebo_yarp_controlboard`(https://github.com/robotology/gazebo-yarp-plugins/pull/492).
- Fixed mismatched behavior, with respect to the real robot, of coupling handler FingersAbductionCouplingHandler in 'gazebo_yarp_controlboard' plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/499)
- Fixed mismatched behavior, with respect to the real robot, of mais analog sensors in 'gazebo_yarp_maissensor' plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/500)

## [3.4.2] - 2020-08-25

### Fixed
- Removed Windows end of lines (https://github.com/robotology/gazebo-yarp-plugins/pull/507).

## [3.4.1] - 2020-08-24

### Fixed
- Fixed compilation with Boost 1.73 (https://github.com/robotology/gazebo-yarp-plugins/pull/505).

## [3.4.0] - 2020-05-19

### Added
- Add the possibility to create different types of joints with the `linkattacher` plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/461).

### Fixed
- Fixed initialization of the local simulation time from Gazebo simTime on first call of onUpdate (https://github.com/robotology/gazebo-yarp-plugins/pull/478).

## [3.3.2] - 2020-05-08

- Fix linking on MSVC when not using the vcpkg's CMAKE_TOOLCHAIN_FILE (https://github.com/robotology/gazebo-yarp-plugins/pull/483).

## [3.3.1] - 2020-03-05

### Fixed
- Solve the problem of setting the interaction mode when the mode of actuator has been already set on the same requested interaction mode (https://github.com/robotology/gazebo-yarp-plugins/pull/464).
- Fix implementation of `yarp::dev::IPositionControl::relativeMove` in `gazebo_yarp_controlboard` (https://github.com/robotology/gazebo-yarp-plugins/pull/466).

## [3.3.0] - 2019-12-13

### Added
- Add the usage of `MultipleAnalogInterfaces`(https://www.yarp.it/group__dev__iface__multiple__analog.html)
  in the `gazebo_yarp_imu`.

### Fixed
- Fixed compilation on Gazebo 7 by disabling compilation of `externalwrench` plugin if the project is configured with Gazebo 7 (https://github.com/robotology/gazebo-yarp-plugins/pull/434).

### Removed
- Support for YARP <= 3.2 was removed. This release requires the use of YARP 3.3 .

## [3.2.0] - 2019-07-01
### Added
- Add `resetSimulation` method to reset simulation state and time time, in the `gazebo_yarp_clock` RPC interface (https://github.com/robotology/gazebo-yarp-plugins/pull/345).
- Add `resetSimulationState` method to reset simulation state, but not the time, in the `gazebo_yarp_clock` RPC interface (https://github.com/robotology/gazebo-yarp-plugins/pull/424).
- Add `gazebo_yarp_doublelaser` plugin to simulate a double laser device (https://github.com/robotology/gazebo-yarp-plugins/pull/419).
- Add `gazebo_yarp_configurationoverride` plugin that can be attached to a model in order to override the [`sdf`](http://sdformat.org/spec) configuration of any other plugin that is attached to the same model (https://github.com/robotology/gazebo-yarp-plugins/pull/401). This plugin is particularly useful to set the initial configuration  of a given model, and is documented in https://github.com/robotology/gazebo-yarp-plugins/blob/v3.2.0/plugins/configurationoverride/README.md .
- Add features in `gazebo_yarp_externalwrench` to apply multiple external wrenchs at the same time (https://github.com/robotology/gazebo-yarp-plugins/pull/418) .
- Add the  `yarpConfigurationString` to configure the YARP plugins with a single `yarpConfigurationString` embedded in the SDF code of the plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/396).
- Add a ChangeLog document based on the format defined in [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

### Fixed
- Fix invalid camera parameters in `gazebo_depthCamera` plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/408).
