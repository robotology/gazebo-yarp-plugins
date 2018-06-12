/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_Pose
#define YARP_THRIFT_GENERATOR_STRUCT_Pose

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace GazeboYarpPlugins {
  class Pose;
}


class GazeboYarpPlugins::Pose : public yarp::os::idl::WirePortable {
public:
  // Fields
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  // Default constructor
  Pose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {
  }

  // Constructor with field values
  Pose(const double x,const double y,const double z,const double roll,const double pitch,const double yaw) : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {
  }

  // Copy constructor
  Pose(const Pose& __alt) : WirePortable(__alt)  {
    this->x = __alt.x;
    this->y = __alt.y;
    this->z = __alt.z;
    this->roll = __alt.roll;
    this->pitch = __alt.pitch;
    this->yaw = __alt.yaw;
  }

  // Assignment operator
  const Pose& operator = (const Pose& __alt) {
    this->x = __alt.x;
    this->y = __alt.y;
    this->z = __alt.z;
    this->roll = __alt.roll;
    this->pitch = __alt.pitch;
    this->yaw = __alt.yaw;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader) override;
  bool read(yarp::os::ConnectionReader& connection) override;
  bool write(const yarp::os::idl::WireWriter& writer) const override;
  bool write(yarp::os::ConnectionWriter& connection) const override;

private:
  bool write_x(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_x(const yarp::os::idl::WireWriter& writer) const;
  bool write_y(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_y(const yarp::os::idl::WireWriter& writer) const;
  bool write_z(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_z(const yarp::os::idl::WireWriter& writer) const;
  bool write_roll(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_roll(const yarp::os::idl::WireWriter& writer) const;
  bool write_pitch(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_pitch(const yarp::os::idl::WireWriter& writer) const;
  bool write_yaw(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_yaw(const yarp::os::idl::WireWriter& writer) const;
  bool read_x(yarp::os::idl::WireReader& reader);
  bool nested_read_x(yarp::os::idl::WireReader& reader);
  bool read_y(yarp::os::idl::WireReader& reader);
  bool nested_read_y(yarp::os::idl::WireReader& reader);
  bool read_z(yarp::os::idl::WireReader& reader);
  bool nested_read_z(yarp::os::idl::WireReader& reader);
  bool read_roll(yarp::os::idl::WireReader& reader);
  bool nested_read_roll(yarp::os::idl::WireReader& reader);
  bool read_pitch(yarp::os::idl::WireReader& reader);
  bool nested_read_pitch(yarp::os::idl::WireReader& reader);
  bool read_yaw(yarp::os::idl::WireReader& reader);
  bool nested_read_yaw(yarp::os::idl::WireReader& reader);

public:

  std::string toString() const;

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<GazeboYarpPlugins::Pose > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new Pose;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(Pose& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(Pose& obj, bool dirty = true) {
      if (obj_owned) delete this->obj;
      this->obj = &obj;
      obj_owned = false;
      dirty_flags(dirty);
      return true;
    }

    virtual ~Editor() {
    if (obj_owned) delete obj;
    }

    bool isValid() const {
      return obj!=0/*NULL*/;
    }

    Pose& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_x(const double x) {
      will_set_x();
      obj->x = x;
      mark_dirty_x();
      communicate();
      did_set_x();
    }
    void set_y(const double y) {
      will_set_y();
      obj->y = y;
      mark_dirty_y();
      communicate();
      did_set_y();
    }
    void set_z(const double z) {
      will_set_z();
      obj->z = z;
      mark_dirty_z();
      communicate();
      did_set_z();
    }
    void set_roll(const double roll) {
      will_set_roll();
      obj->roll = roll;
      mark_dirty_roll();
      communicate();
      did_set_roll();
    }
    void set_pitch(const double pitch) {
      will_set_pitch();
      obj->pitch = pitch;
      mark_dirty_pitch();
      communicate();
      did_set_pitch();
    }
    void set_yaw(const double yaw) {
      will_set_yaw();
      obj->yaw = yaw;
      mark_dirty_yaw();
      communicate();
      did_set_yaw();
    }
    double get_x() {
      return obj->x;
    }
    double get_y() {
      return obj->y;
    }
    double get_z() {
      return obj->z;
    }
    double get_roll() {
      return obj->roll;
    }
    double get_pitch() {
      return obj->pitch;
    }
    double get_yaw() {
      return obj->yaw;
    }
    virtual bool will_set_x() { return true; }
    virtual bool will_set_y() { return true; }
    virtual bool will_set_z() { return true; }
    virtual bool will_set_roll() { return true; }
    virtual bool will_set_pitch() { return true; }
    virtual bool will_set_yaw() { return true; }
    virtual bool did_set_x() { return true; }
    virtual bool did_set_y() { return true; }
    virtual bool did_set_z() { return true; }
    virtual bool did_set_roll() { return true; }
    virtual bool did_set_pitch() { return true; }
    virtual bool did_set_yaw() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection) override;
    bool write(yarp::os::ConnectionWriter& connection) const override;
  private:

    Pose *obj;

    bool obj_owned;
    int group;

    void communicate() {
      if (group!=0) return;
      if (yarp().canWrite()) {
        yarp().write(*this);
        clean();
      }
    }
    void mark_dirty() {
      is_dirty = true;
    }
    void mark_dirty_x() {
      if (is_dirty_x) return;
      dirty_count++;
      is_dirty_x = true;
      mark_dirty();
    }
    void mark_dirty_y() {
      if (is_dirty_y) return;
      dirty_count++;
      is_dirty_y = true;
      mark_dirty();
    }
    void mark_dirty_z() {
      if (is_dirty_z) return;
      dirty_count++;
      is_dirty_z = true;
      mark_dirty();
    }
    void mark_dirty_roll() {
      if (is_dirty_roll) return;
      dirty_count++;
      is_dirty_roll = true;
      mark_dirty();
    }
    void mark_dirty_pitch() {
      if (is_dirty_pitch) return;
      dirty_count++;
      is_dirty_pitch = true;
      mark_dirty();
    }
    void mark_dirty_yaw() {
      if (is_dirty_yaw) return;
      dirty_count++;
      is_dirty_yaw = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_x = flag;
      is_dirty_y = flag;
      is_dirty_z = flag;
      is_dirty_roll = flag;
      is_dirty_pitch = flag;
      is_dirty_yaw = flag;
      dirty_count = flag ? 6 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_x;
    bool is_dirty_y;
    bool is_dirty_z;
    bool is_dirty_roll;
    bool is_dirty_pitch;
    bool is_dirty_yaw;
  };
};

#endif
