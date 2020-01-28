# Changelog
All notable changes to this project will be documented in this file.

The format of this document is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

### Added
- Add the option for the external force orientation to be reperesented in the link frame with the `externalwrench` plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/463).
- Add the possibility to create different types of joints with the `linkattacher` plugin (https://github.com/robotology/gazebo-yarp-plugins/pull/461).

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


