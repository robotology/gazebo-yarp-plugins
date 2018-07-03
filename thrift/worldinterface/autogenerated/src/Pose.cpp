/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <Pose.h>

namespace GazeboYarpPlugins {
bool Pose::read_x(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(x)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::nested_read_x(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(x)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::read_y(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(y)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::nested_read_y(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(y)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::read_z(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(z)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::nested_read_z(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(z)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::read_roll(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(roll)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::nested_read_roll(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(roll)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::read_pitch(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(pitch)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::nested_read_pitch(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(pitch)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::read_yaw(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(yaw)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::nested_read_yaw(yarp::os::idl::WireReader& reader) {
  if (!reader.readFloat64(yaw)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Pose::read(yarp::os::idl::WireReader& reader) {
  if (!read_x(reader)) return false;
  if (!read_y(reader)) return false;
  if (!read_z(reader)) return false;
  if (!read_roll(reader)) return false;
  if (!read_pitch(reader)) return false;
  if (!read_yaw(reader)) return false;
  return !reader.isError();
}

bool Pose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(6)) return false;
  return read(reader);
}

bool Pose::write_x(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(x)) return false;
  return true;
}
bool Pose::nested_write_x(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(x)) return false;
  return true;
}
bool Pose::write_y(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(y)) return false;
  return true;
}
bool Pose::nested_write_y(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(y)) return false;
  return true;
}
bool Pose::write_z(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(z)) return false;
  return true;
}
bool Pose::nested_write_z(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(z)) return false;
  return true;
}
bool Pose::write_roll(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(roll)) return false;
  return true;
}
bool Pose::nested_write_roll(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(roll)) return false;
  return true;
}
bool Pose::write_pitch(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(pitch)) return false;
  return true;
}
bool Pose::nested_write_pitch(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(pitch)) return false;
  return true;
}
bool Pose::write_yaw(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(yaw)) return false;
  return true;
}
bool Pose::nested_write_yaw(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeFloat64(yaw)) return false;
  return true;
}
bool Pose::write(const yarp::os::idl::WireWriter& writer) const {
  if (!write_x(writer)) return false;
  if (!write_y(writer)) return false;
  if (!write_z(writer)) return false;
  if (!write_roll(writer)) return false;
  if (!write_pitch(writer)) return false;
  if (!write_yaw(writer)) return false;
  return !writer.isError();
}

bool Pose::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(6)) return false;
  return write(writer);
}
bool Pose::Editor::write(yarp::os::ConnectionWriter& connection) const {
  if (!isValid()) return false;
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(dirty_count+1)) return false;
  if (!writer.writeString("patch")) return false;
  if (is_dirty_x) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("x")) return false;
    if (!obj->nested_write_x(writer)) return false;
  }
  if (is_dirty_y) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("y")) return false;
    if (!obj->nested_write_y(writer)) return false;
  }
  if (is_dirty_z) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("z")) return false;
    if (!obj->nested_write_z(writer)) return false;
  }
  if (is_dirty_roll) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("roll")) return false;
    if (!obj->nested_write_roll(writer)) return false;
  }
  if (is_dirty_pitch) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("pitch")) return false;
    if (!obj->nested_write_pitch(writer)) return false;
  }
  if (is_dirty_yaw) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("yaw")) return false;
    if (!obj->nested_write_yaw(writer)) return false;
  }
  return !writer.isError();
}
bool Pose::Editor::read(yarp::os::ConnectionReader& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) return false;
  int len = reader.getLength();
  if (len==0) {
    yarp::os::idl::WireWriter writer(reader);
    if (writer.isNull()) return true;
    if (!writer.writeListHeader(1)) return false;
    writer.writeString("send: 'help' or 'patch (param1 val1) (param2 val2)'");
    return true;
  }
  std::string tag;
  if (!reader.readString(tag)) return false;
  if (tag=="help") {
    yarp::os::idl::WireWriter writer(reader);
    if (writer.isNull()) return true;
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("many",1, 0)) return false;
    if (reader.getLength()>0) {
      std::string field;
      if (!reader.readString(field)) return false;
      if (field=="x") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double x")) return false;
      }
      if (field=="y") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double y")) return false;
      }
      if (field=="z") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double z")) return false;
      }
      if (field=="roll") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double roll")) return false;
      }
      if (field=="pitch") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double pitch")) return false;
      }
      if (field=="yaw") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("double yaw")) return false;
      }
    }
    if (!writer.writeListHeader(7)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("x");
    writer.writeString("y");
    writer.writeString("z");
    writer.writeString("roll");
    writer.writeString("pitch");
    writer.writeString("yaw");
    return true;
  }
  bool nested = true;
  bool have_act = false;
  if (tag!="patch") {
    if ((len-1)%2 != 0) return false;
    len = 1 + ((len-1)/2);
    nested = false;
    have_act = true;
  }
  for (int i=1; i<len; i++) {
    if (nested && !reader.readListHeader(3)) return false;
    std::string act;
    std::string key;
    if (have_act) {
      act = tag;
    } else {
      if (!reader.readString(act)) return false;
    }
    if (!reader.readString(key)) return false;
    // inefficient code follows, bug paulfitz to improve it
    if (key == "x") {
      will_set_x();
      if (!obj->nested_read_x(reader)) return false;
      did_set_x();
    } else if (key == "y") {
      will_set_y();
      if (!obj->nested_read_y(reader)) return false;
      did_set_y();
    } else if (key == "z") {
      will_set_z();
      if (!obj->nested_read_z(reader)) return false;
      did_set_z();
    } else if (key == "roll") {
      will_set_roll();
      if (!obj->nested_read_roll(reader)) return false;
      did_set_roll();
    } else if (key == "pitch") {
      will_set_pitch();
      if (!obj->nested_read_pitch(reader)) return false;
      did_set_pitch();
    } else if (key == "yaw") {
      will_set_yaw();
      if (!obj->nested_read_yaw(reader)) return false;
      did_set_yaw();
    } else {
      // would be useful to have a fallback here
    }
  }
  reader.accept();
  yarp::os::idl::WireWriter writer(reader);
  if (writer.isNull()) return true;
  writer.writeListHeader(1);
  writer.writeVocab(yarp::os::createVocab('o','k'));
  return true;
}

std::string Pose::toString() const {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
} // namespace
