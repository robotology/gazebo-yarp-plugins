This page is used as a starting point for contributing to the development of `gazebo_yarp_plugins`.

If you would like to contribute to the development of `gazebo_yarp_plugins`, please get in contact with the development team using [GitHub issues](https://github.com/robotology/gazebo_yarp_plugins/issues). 

The easiest way to contribute is to pick an issue and try to solve it. A list of issues a new developer can work on is available under the [Volunteer needed](https://github.com/robotology/gazebo_yarp_plugins/issues?labels=Volunteer+needed&page=1&state=open) tag.

If you need a new feature or to fix a bug, please [fill a GitHub issue](https://github.com/robotology/yarp/issues/new) describing the feature or the bugfix you would like to implement. 

## Code style
We adopt a consistent code style.
If you want to contribute to this project, we kindly ask you to conform to the code style guidelines.

We do not enforce the style (at least at the current stage) but we will probably offer scripts to ease the process in the future.

More precisely, we try to use the WebKit coding style (see [WebKit page](http://www.webkit.org/coding/coding-style.html)).

## Patches and features contribution
The contribution follows the classical GitHub stages:
* open an issue (so we can discuss the problem and possible solutions);
* develop the changes;
* fork the project and submit the pull request.

## Repository structure and releases management.
This part is mainly targeted to ''internal'' members.
The repository has two (2) permanent branches:
* `master`;
* `development`.

The `master` branch contains the stable code. Release versions will be tagged only in this branch.
The `development` branch contains unstable (but hopefully compilable) code.

Every effort expensive feature (i.e. it needs more than one commit to be developed) should be implemented in a separate branch (e.g. `feature/feature_name`). Once successfully implemented, the branch will be merged into `development` and  it will be deleted.
When we want to freeze the code, we fork `development` into a new branch named `release/release_version`. No new features are allowed in this branch. 
When the code is stable, the branch is merged into both `development` and `master`, and `master` is tagged accordingly.

Trivial fixes can be made directly into `development` or `master`, but do remember to propagate the changes to the right places. For more complex issues, once again, a new branch must be created (e.g. `fix/fix_name`).



