# Changelog
All notable changes to this project will be documented in this file.

The format of this document is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).


## [Unreleased]

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
