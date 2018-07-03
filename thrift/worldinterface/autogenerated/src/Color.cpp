/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <Color.h>

namespace GazeboYarpPlugins {
bool Color::read_r(yarp::os::idl::WireReader& reader) {
  if (!reader.readI16(r)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Color::nested_read_r(yarp::os::idl::WireReader& reader) {
  if (!reader.readI16(r)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Color::read_g(yarp::os::idl::WireReader& reader) {
  if (!reader.readI16(g)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Color::nested_read_g(yarp::os::idl::WireReader& reader) {
  if (!reader.readI16(g)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Color::read_b(yarp::os::idl::WireReader& reader) {
  if (!reader.readI16(b)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Color::nested_read_b(yarp::os::idl::WireReader& reader) {
  if (!reader.readI16(b)) {
    reader.fail();
    return false;
  }
  return true;
}
bool Color::read(yarp::os::idl::WireReader& reader) {
  if (!read_r(reader)) return false;
  if (!read_g(reader)) return false;
  if (!read_b(reader)) return false;
  return !reader.isError();
}

bool Color::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(3)) return false;
  return read(reader);
}

bool Color::write_r(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeI16(r)) return false;
  return true;
}
bool Color::nested_write_r(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeI16(r)) return false;
  return true;
}
bool Color::write_g(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeI16(g)) return false;
  return true;
}
bool Color::nested_write_g(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeI16(g)) return false;
  return true;
}
bool Color::write_b(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeI16(b)) return false;
  return true;
}
bool Color::nested_write_b(const yarp::os::idl::WireWriter& writer) const {
  if (!writer.writeI16(b)) return false;
  return true;
}
bool Color::write(const yarp::os::idl::WireWriter& writer) const {
  if (!write_r(writer)) return false;
  if (!write_g(writer)) return false;
  if (!write_b(writer)) return false;
  return !writer.isError();
}

bool Color::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  return write(writer);
}
bool Color::Editor::write(yarp::os::ConnectionWriter& connection) const {
  if (!isValid()) return false;
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(dirty_count+1)) return false;
  if (!writer.writeString("patch")) return false;
  if (is_dirty_r) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("r")) return false;
    if (!obj->nested_write_r(writer)) return false;
  }
  if (is_dirty_g) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("g")) return false;
    if (!obj->nested_write_g(writer)) return false;
  }
  if (is_dirty_b) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("b")) return false;
    if (!obj->nested_write_b(writer)) return false;
  }
  return !writer.isError();
}
bool Color::Editor::read(yarp::os::ConnectionReader& connection) {
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
      if (field=="r") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("std::int16_t r")) return false;
      }
      if (field=="g") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("std::int16_t g")) return false;
      }
      if (field=="b") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("std::int16_t b")) return false;
      }
    }
    if (!writer.writeListHeader(4)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("r");
    writer.writeString("g");
    writer.writeString("b");
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
    if (key == "r") {
      will_set_r();
      if (!obj->nested_read_r(reader)) return false;
      did_set_r();
    } else if (key == "g") {
      will_set_g();
      if (!obj->nested_read_g(reader)) return false;
      did_set_g();
    } else if (key == "b") {
      will_set_b();
      if (!obj->nested_read_b(reader)) return false;
      did_set_b();
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

std::string Color::toString() const {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
} // namespace
