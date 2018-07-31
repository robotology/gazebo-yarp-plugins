/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <WorldInterfaceServer.h>
#include <yarp/os/idl/WireTypes.h>

namespace GazeboYarpPlugins {


class WorldInterfaceServer_makeSphere : public yarp::os::Portable {
public:
  double radius;
  Pose pose;
  Color color;
  std::string frame_name;
  std::string object_name;
  bool gravity_enable;
  bool collision_enable;
  std::string _return;
  void init(const double radius, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_makeBox : public yarp::os::Portable {
public:
  double width;
  double height;
  double thickness;
  Pose pose;
  Color color;
  std::string frame_name;
  std::string object_name;
  bool gravity_enable;
  bool collision_enable;
  std::string _return;
  void init(const double width, const double height, const double thickness, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_makeCylinder : public yarp::os::Portable {
public:
  double radius;
  double length;
  Pose pose;
  Color color;
  std::string frame_name;
  std::string object_name;
  bool gravity_enable;
  bool collision_enable;
  std::string _return;
  void init(const double radius, const double length, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_makeFrame : public yarp::os::Portable {
public:
  double size;
  Pose pose;
  Color color;
  std::string frame_name;
  std::string object_name;
  bool gravity_enable;
  bool collision_enable;
  std::string _return;
  void init(const double size, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_changeColor : public yarp::os::Portable {
public:
  std::string id;
  Color color;
  bool _return;
  void init(const std::string& id, const Color& color);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_setPose : public yarp::os::Portable {
public:
  std::string id;
  Pose pose;
  std::string frame_name;
  bool _return;
  void init(const std::string& id, const Pose& pose, const std::string& frame_name);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_enableGravity : public yarp::os::Portable {
public:
  std::string id;
  bool enable;
  bool _return;
  void init(const std::string& id, const bool enable);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_enableCollision : public yarp::os::Portable {
public:
  std::string id;
  bool enable;
  bool _return;
  void init(const std::string& id, const bool enable);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_getPose : public yarp::os::Portable {
public:
  std::string id;
  std::string frame_name;
  Pose _return;
  void init(const std::string& id, const std::string& frame_name);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_loadModelFromFile : public yarp::os::Portable {
public:
  std::string filename;
  bool _return;
  void init(const std::string& filename);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_deleteObject : public yarp::os::Portable {
public:
  std::string id;
  bool _return;
  void init(const std::string& id);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_deleteAll : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_getList : public yarp::os::Portable {
public:
  std::vector<std::string>  _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_attach : public yarp::os::Portable {
public:
  std::string id;
  std::string link_name;
  bool _return;
  void init(const std::string& id, const std::string& link_name);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_detach : public yarp::os::Portable {
public:
  std::string id;
  bool _return;
  void init(const std::string& id);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class WorldInterfaceServer_rename : public yarp::os::Portable {
public:
  std::string old_name;
  std::string new_name;
  bool _return;
  void init(const std::string& old_name, const std::string& new_name);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

bool WorldInterfaceServer_makeSphere::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(15)) return false;
  if (!writer.writeTag("makeSphere",1,1)) return false;
  if (!writer.writeFloat64(radius)) return false;
  if (!writer.write(pose)) return false;
  if (!writer.write(color)) return false;
  if (!writer.writeString(frame_name)) return false;
  if (!writer.writeString(object_name)) return false;
  if (!writer.writeBool(gravity_enable)) return false;
  if (!writer.writeBool(collision_enable)) return false;
  return true;
}

bool WorldInterfaceServer_makeSphere::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_makeSphere::init(const double radius, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable) {
  _return = "";
  this->radius = radius;
  this->pose = pose;
  this->color = color;
  this->frame_name = frame_name;
  this->object_name = object_name;
  this->gravity_enable = gravity_enable;
  this->collision_enable = collision_enable;
}

bool WorldInterfaceServer_makeBox::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(17)) return false;
  if (!writer.writeTag("makeBox",1,1)) return false;
  if (!writer.writeFloat64(width)) return false;
  if (!writer.writeFloat64(height)) return false;
  if (!writer.writeFloat64(thickness)) return false;
  if (!writer.write(pose)) return false;
  if (!writer.write(color)) return false;
  if (!writer.writeString(frame_name)) return false;
  if (!writer.writeString(object_name)) return false;
  if (!writer.writeBool(gravity_enable)) return false;
  if (!writer.writeBool(collision_enable)) return false;
  return true;
}

bool WorldInterfaceServer_makeBox::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_makeBox::init(const double width, const double height, const double thickness, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable) {
  _return = "";
  this->width = width;
  this->height = height;
  this->thickness = thickness;
  this->pose = pose;
  this->color = color;
  this->frame_name = frame_name;
  this->object_name = object_name;
  this->gravity_enable = gravity_enable;
  this->collision_enable = collision_enable;
}

bool WorldInterfaceServer_makeCylinder::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(16)) return false;
  if (!writer.writeTag("makeCylinder",1,1)) return false;
  if (!writer.writeFloat64(radius)) return false;
  if (!writer.writeFloat64(length)) return false;
  if (!writer.write(pose)) return false;
  if (!writer.write(color)) return false;
  if (!writer.writeString(frame_name)) return false;
  if (!writer.writeString(object_name)) return false;
  if (!writer.writeBool(gravity_enable)) return false;
  if (!writer.writeBool(collision_enable)) return false;
  return true;
}

bool WorldInterfaceServer_makeCylinder::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_makeCylinder::init(const double radius, const double length, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable) {
  _return = "";
  this->radius = radius;
  this->length = length;
  this->pose = pose;
  this->color = color;
  this->frame_name = frame_name;
  this->object_name = object_name;
  this->gravity_enable = gravity_enable;
  this->collision_enable = collision_enable;
}

bool WorldInterfaceServer_makeFrame::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(15)) return false;
  if (!writer.writeTag("makeFrame",1,1)) return false;
  if (!writer.writeFloat64(size)) return false;
  if (!writer.write(pose)) return false;
  if (!writer.write(color)) return false;
  if (!writer.writeString(frame_name)) return false;
  if (!writer.writeString(object_name)) return false;
  if (!writer.writeBool(gravity_enable)) return false;
  if (!writer.writeBool(collision_enable)) return false;
  return true;
}

bool WorldInterfaceServer_makeFrame::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_makeFrame::init(const double size, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable) {
  _return = "";
  this->size = size;
  this->pose = pose;
  this->color = color;
  this->frame_name = frame_name;
  this->object_name = object_name;
  this->gravity_enable = gravity_enable;
  this->collision_enable = collision_enable;
}

bool WorldInterfaceServer_changeColor::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(5)) return false;
  if (!writer.writeTag("changeColor",1,1)) return false;
  if (!writer.writeString(id)) return false;
  if (!writer.write(color)) return false;
  return true;
}

bool WorldInterfaceServer_changeColor::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_changeColor::init(const std::string& id, const Color& color) {
  _return = false;
  this->id = id;
  this->color = color;
}

bool WorldInterfaceServer_setPose::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(9)) return false;
  if (!writer.writeTag("setPose",1,1)) return false;
  if (!writer.writeString(id)) return false;
  if (!writer.write(pose)) return false;
  if (!writer.writeString(frame_name)) return false;
  return true;
}

bool WorldInterfaceServer_setPose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_setPose::init(const std::string& id, const Pose& pose, const std::string& frame_name) {
  _return = false;
  this->id = id;
  this->pose = pose;
  this->frame_name = frame_name;
}

bool WorldInterfaceServer_enableGravity::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("enableGravity",1,1)) return false;
  if (!writer.writeString(id)) return false;
  if (!writer.writeBool(enable)) return false;
  return true;
}

bool WorldInterfaceServer_enableGravity::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_enableGravity::init(const std::string& id, const bool enable) {
  _return = false;
  this->id = id;
  this->enable = enable;
}

bool WorldInterfaceServer_enableCollision::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("enableCollision",1,1)) return false;
  if (!writer.writeString(id)) return false;
  if (!writer.writeBool(enable)) return false;
  return true;
}

bool WorldInterfaceServer_enableCollision::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_enableCollision::init(const std::string& id, const bool enable) {
  _return = false;
  this->id = id;
  this->enable = enable;
}

bool WorldInterfaceServer_getPose::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("getPose",1,1)) return false;
  if (!writer.writeString(id)) return false;
  if (!writer.writeString(frame_name)) return false;
  return true;
}

bool WorldInterfaceServer_getPose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_getPose::init(const std::string& id, const std::string& frame_name) {
  this->id = id;
  this->frame_name = frame_name;
}

bool WorldInterfaceServer_loadModelFromFile::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("loadModelFromFile",1,1)) return false;
  if (!writer.writeString(filename)) return false;
  return true;
}

bool WorldInterfaceServer_loadModelFromFile::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_loadModelFromFile::init(const std::string& filename) {
  _return = false;
  this->filename = filename;
}

bool WorldInterfaceServer_deleteObject::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("deleteObject",1,1)) return false;
  if (!writer.writeString(id)) return false;
  return true;
}

bool WorldInterfaceServer_deleteObject::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_deleteObject::init(const std::string& id) {
  _return = false;
  this->id = id;
}

bool WorldInterfaceServer_deleteAll::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("deleteAll",1,1)) return false;
  return true;
}

bool WorldInterfaceServer_deleteAll::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_deleteAll::init() {
  _return = false;
}

bool WorldInterfaceServer_getList::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getList",1,1)) return false;
  return true;
}

bool WorldInterfaceServer_getList::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  {
    _return.clear();
    uint32_t _size0;
    yarp::os::idl::WireState _etype3;
    reader.readListBegin(_etype3, _size0);
    _return.resize(_size0);
    uint32_t _i4;
    for (_i4 = 0; _i4 < _size0; ++_i4)
    {
      if (!reader.readString(_return[_i4])) {
        reader.fail();
        return false;
      }
    }
    reader.readListEnd();
  }
  return true;
}

void WorldInterfaceServer_getList::init() {
}

bool WorldInterfaceServer_attach::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("attach",1,1)) return false;
  if (!writer.writeString(id)) return false;
  if (!writer.writeString(link_name)) return false;
  return true;
}

bool WorldInterfaceServer_attach::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_attach::init(const std::string& id, const std::string& link_name) {
  _return = false;
  this->id = id;
  this->link_name = link_name;
}

bool WorldInterfaceServer_detach::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("detach",1,1)) return false;
  if (!writer.writeString(id)) return false;
  return true;
}

bool WorldInterfaceServer_detach::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_detach::init(const std::string& id) {
  _return = false;
  this->id = id;
}

bool WorldInterfaceServer_rename::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("rename",1,1)) return false;
  if (!writer.writeString(old_name)) return false;
  if (!writer.writeString(new_name)) return false;
  return true;
}

bool WorldInterfaceServer_rename::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void WorldInterfaceServer_rename::init(const std::string& old_name, const std::string& new_name) {
  _return = false;
  this->old_name = old_name;
  this->new_name = new_name;
}

WorldInterfaceServer::WorldInterfaceServer() {
  yarp().setOwner(*this);
}
std::string WorldInterfaceServer::makeSphere(const double radius, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable) {
  std::string _return = "";
  WorldInterfaceServer_makeSphere helper;
  helper.init(radius,pose,color,frame_name,object_name,gravity_enable,collision_enable);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string WorldInterfaceServer::makeSphere(const double radius, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string WorldInterfaceServer::makeBox(const double width, const double height, const double thickness, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable) {
  std::string _return = "";
  WorldInterfaceServer_makeBox helper;
  helper.init(width,height,thickness,pose,color,frame_name,object_name,gravity_enable,collision_enable);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string WorldInterfaceServer::makeBox(const double width, const double height, const double thickness, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string WorldInterfaceServer::makeCylinder(const double radius, const double length, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable) {
  std::string _return = "";
  WorldInterfaceServer_makeCylinder helper;
  helper.init(radius,length,pose,color,frame_name,object_name,gravity_enable,collision_enable);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string WorldInterfaceServer::makeCylinder(const double radius, const double length, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string WorldInterfaceServer::makeFrame(const double size, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable) {
  std::string _return = "";
  WorldInterfaceServer_makeFrame helper;
  helper.init(size,pose,color,frame_name,object_name,gravity_enable,collision_enable);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string WorldInterfaceServer::makeFrame(const double size, const Pose& pose, const Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool WorldInterfaceServer::changeColor(const std::string& id, const Color& color) {
  bool _return = false;
  WorldInterfaceServer_changeColor helper;
  helper.init(id,color);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::changeColor(const std::string& id, const Color& color)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool WorldInterfaceServer::setPose(const std::string& id, const Pose& pose, const std::string& frame_name) {
  bool _return = false;
  WorldInterfaceServer_setPose helper;
  helper.init(id,pose,frame_name);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::setPose(const std::string& id, const Pose& pose, const std::string& frame_name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool WorldInterfaceServer::enableGravity(const std::string& id, const bool enable) {
  bool _return = false;
  WorldInterfaceServer_enableGravity helper;
  helper.init(id,enable);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::enableGravity(const std::string& id, const bool enable)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool WorldInterfaceServer::enableCollision(const std::string& id, const bool enable) {
  bool _return = false;
  WorldInterfaceServer_enableCollision helper;
  helper.init(id,enable);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::enableCollision(const std::string& id, const bool enable)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
Pose WorldInterfaceServer::getPose(const std::string& id, const std::string& frame_name) {
  Pose _return;
  WorldInterfaceServer_getPose helper;
  helper.init(id,frame_name);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","Pose WorldInterfaceServer::getPose(const std::string& id, const std::string& frame_name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool WorldInterfaceServer::loadModelFromFile(const std::string& filename) {
  bool _return = false;
  WorldInterfaceServer_loadModelFromFile helper;
  helper.init(filename);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::loadModelFromFile(const std::string& filename)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool WorldInterfaceServer::deleteObject(const std::string& id) {
  bool _return = false;
  WorldInterfaceServer_deleteObject helper;
  helper.init(id);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::deleteObject(const std::string& id)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool WorldInterfaceServer::deleteAll() {
  bool _return = false;
  WorldInterfaceServer_deleteAll helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::deleteAll()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::vector<std::string>  WorldInterfaceServer::getList() {
  std::vector<std::string>  _return;
  WorldInterfaceServer_getList helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::vector<std::string>  WorldInterfaceServer::getList()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool WorldInterfaceServer::attach(const std::string& id, const std::string& link_name) {
  bool _return = false;
  WorldInterfaceServer_attach helper;
  helper.init(id,link_name);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::attach(const std::string& id, const std::string& link_name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool WorldInterfaceServer::detach(const std::string& id) {
  bool _return = false;
  WorldInterfaceServer_detach helper;
  helper.init(id);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::detach(const std::string& id)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool WorldInterfaceServer::rename(const std::string& old_name, const std::string& new_name) {
  bool _return = false;
  WorldInterfaceServer_rename helper;
  helper.init(old_name,new_name);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::rename(const std::string& old_name, const std::string& new_name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool WorldInterfaceServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  std::string tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "makeSphere") {
      double radius;
      Pose pose;
      Color color;
      std::string frame_name;
      std::string object_name;
      bool gravity_enable;
      bool collision_enable;
      if (!reader.readFloat64(radius)) {
        reader.fail();
        return false;
      }
      if (!reader.read(pose)) {
        reader.fail();
        return false;
      }
      if (!reader.read(color)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(frame_name)) {
        frame_name = "";
      }
      if (!reader.readString(object_name)) {
        object_name = "";
      }
      if (!reader.readBool(gravity_enable)) {
        gravity_enable = 0;
      }
      if (!reader.readBool(collision_enable)) {
        collision_enable = 1;
      }
      std::string _return;
      _return = makeSphere(radius,pose,color,frame_name,object_name,gravity_enable,collision_enable);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "makeBox") {
      double width;
      double height;
      double thickness;
      Pose pose;
      Color color;
      std::string frame_name;
      std::string object_name;
      bool gravity_enable;
      bool collision_enable;
      if (!reader.readFloat64(width)) {
        reader.fail();
        return false;
      }
      if (!reader.readFloat64(height)) {
        reader.fail();
        return false;
      }
      if (!reader.readFloat64(thickness)) {
        reader.fail();
        return false;
      }
      if (!reader.read(pose)) {
        reader.fail();
        return false;
      }
      if (!reader.read(color)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(frame_name)) {
        frame_name = "";
      }
      if (!reader.readString(object_name)) {
        object_name = "";
      }
      if (!reader.readBool(gravity_enable)) {
        gravity_enable = 0;
      }
      if (!reader.readBool(collision_enable)) {
        collision_enable = 1;
      }
      std::string _return;
      _return = makeBox(width,height,thickness,pose,color,frame_name,object_name,gravity_enable,collision_enable);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "makeCylinder") {
      double radius;
      double length;
      Pose pose;
      Color color;
      std::string frame_name;
      std::string object_name;
      bool gravity_enable;
      bool collision_enable;
      if (!reader.readFloat64(radius)) {
        reader.fail();
        return false;
      }
      if (!reader.readFloat64(length)) {
        reader.fail();
        return false;
      }
      if (!reader.read(pose)) {
        reader.fail();
        return false;
      }
      if (!reader.read(color)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(frame_name)) {
        frame_name = "";
      }
      if (!reader.readString(object_name)) {
        object_name = "";
      }
      if (!reader.readBool(gravity_enable)) {
        gravity_enable = 0;
      }
      if (!reader.readBool(collision_enable)) {
        collision_enable = 1;
      }
      std::string _return;
      _return = makeCylinder(radius,length,pose,color,frame_name,object_name,gravity_enable,collision_enable);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "makeFrame") {
      double size;
      Pose pose;
      Color color;
      std::string frame_name;
      std::string object_name;
      bool gravity_enable;
      bool collision_enable;
      if (!reader.readFloat64(size)) {
        reader.fail();
        return false;
      }
      if (!reader.read(pose)) {
        reader.fail();
        return false;
      }
      if (!reader.read(color)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(frame_name)) {
        frame_name = "";
      }
      if (!reader.readString(object_name)) {
        object_name = "";
      }
      if (!reader.readBool(gravity_enable)) {
        gravity_enable = 0;
      }
      if (!reader.readBool(collision_enable)) {
        collision_enable = 1;
      }
      std::string _return;
      _return = makeFrame(size,pose,color,frame_name,object_name,gravity_enable,collision_enable);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "changeColor") {
      std::string id;
      Color color;
      if (!reader.readString(id)) {
        reader.fail();
        return false;
      }
      if (!reader.read(color)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = changeColor(id,color);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setPose") {
      std::string id;
      Pose pose;
      std::string frame_name;
      if (!reader.readString(id)) {
        reader.fail();
        return false;
      }
      if (!reader.read(pose)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(frame_name)) {
        frame_name = "";
      }
      bool _return;
      _return = setPose(id,pose,frame_name);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "enableGravity") {
      std::string id;
      bool enable;
      if (!reader.readString(id)) {
        reader.fail();
        return false;
      }
      if (!reader.readBool(enable)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = enableGravity(id,enable);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "enableCollision") {
      std::string id;
      bool enable;
      if (!reader.readString(id)) {
        reader.fail();
        return false;
      }
      if (!reader.readBool(enable)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = enableCollision(id,enable);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getPose") {
      std::string id;
      std::string frame_name;
      if (!reader.readString(id)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(frame_name)) {
        frame_name = "";
      }
      Pose _return;
      _return = getPose(id,frame_name);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(6)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "loadModelFromFile") {
      std::string filename;
      if (!reader.readString(filename)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = loadModelFromFile(filename);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "deleteObject") {
      std::string id;
      if (!reader.readString(id)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = deleteObject(id);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "deleteAll") {
      bool _return;
      _return = deleteAll();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getList") {
      std::vector<std::string>  _return;
      _return = getList();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        {
          if (!writer.writeListBegin(BOTTLE_TAG_STRING, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::const_iterator _iter5;
          for (_iter5 = _return.begin(); _iter5 != _return.end(); ++_iter5)
          {
            if (!writer.writeString((*_iter5))) return false;
          }
          if (!writer.writeListEnd()) return false;
        }
      }
      reader.accept();
      return true;
    }
    if (tag == "attach") {
      std::string id;
      std::string link_name;
      if (!reader.readString(id)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(link_name)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = attach(id,link_name);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "detach") {
      std::string id;
      if (!reader.readString(id)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = detach(id);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "rename") {
      std::string old_name;
      std::string new_name;
      if (!reader.readString(old_name)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(new_name)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = rename(old_name,new_name);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT32, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    std::string next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> WorldInterfaceServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("makeSphere");
    helpString.push_back("makeBox");
    helpString.push_back("makeCylinder");
    helpString.push_back("makeFrame");
    helpString.push_back("changeColor");
    helpString.push_back("setPose");
    helpString.push_back("enableGravity");
    helpString.push_back("enableCollision");
    helpString.push_back("getPose");
    helpString.push_back("loadModelFromFile");
    helpString.push_back("deleteObject");
    helpString.push_back("deleteAll");
    helpString.push_back("getList");
    helpString.push_back("attach");
    helpString.push_back("detach");
    helpString.push_back("rename");
    helpString.push_back("help");
  }
  else {
    if (functionName=="makeSphere") {
      helpString.push_back("std::string makeSphere(const double radius, const Pose& pose, const Color& color, const std::string& frame_name = \"\", const std::string& object_name = \"\", const bool gravity_enable = 0, const bool collision_enable = 1) ");
      helpString.push_back("Make a sphere. ");
      helpString.push_back("@param radius radius of the sphere [m] ");
      helpString.push_back("@param pose pose of the sphere [m] ");
      helpString.push_back("@param color color of the sphere ");
      helpString.push_back("@param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used. ");
      helpString.push_back("@param object_name (optional) assigns a name to the object. ");
      helpString.push_back("@param gravity_enable (optional) enables gravity (default false) ");
      helpString.push_back("@param collision_enable (optional) enables collision (default true) ");
      helpString.push_back("@return returns a string that contains the name of the object in the world ");
    }
    if (functionName=="makeBox") {
      helpString.push_back("std::string makeBox(const double width, const double height, const double thickness, const Pose& pose, const Color& color, const std::string& frame_name = \"\", const std::string& object_name = \"\", const bool gravity_enable = 0, const bool collision_enable = 1) ");
      helpString.push_back("Make a box. ");
      helpString.push_back("@param width box width [m] ");
      helpString.push_back("@param height box height[m] ");
      helpString.push_back("@param thickness box thickness [m] ");
      helpString.push_back("@param pose pose of the box [m] ");
      helpString.push_back("@param color color of the box ");
      helpString.push_back("@param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used. ");
      helpString.push_back("@param object_name (optional) assigns a name to the object. ");
      helpString.push_back("@param gravity_enable (optional) enables gravity (default false) ");
      helpString.push_back("@param collision_enable (optional) enables collision (default true) ");
      helpString.push_back("@return returns a string that contains the name of the object in the world ");
    }
    if (functionName=="makeCylinder") {
      helpString.push_back("std::string makeCylinder(const double radius, const double length, const Pose& pose, const Color& color, const std::string& frame_name = \"\", const std::string& object_name = \"\", const bool gravity_enable = 0, const bool collision_enable = 1) ");
      helpString.push_back("Make a cylinder. ");
      helpString.push_back("@param radius radius of the cylinder [m] ");
      helpString.push_back("@param length lenght of the cylinder [m] ");
      helpString.push_back("@param pose pose of the cylinder [m] ");
      helpString.push_back("@param color color of the cylinder ");
      helpString.push_back("@param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used. ");
      helpString.push_back("@param object_name (optional) assigns a name to the object. ");
      helpString.push_back("@param gravity_enable (optional) enables gravity (default false) ");
      helpString.push_back("@param collision_enable (optional) enables collision (default true) ");
      helpString.push_back("@return returns a string that contains the name of the object in the world ");
    }
    if (functionName=="makeFrame") {
      helpString.push_back("std::string makeFrame(const double size, const Pose& pose, const Color& color, const std::string& frame_name = \"\", const std::string& object_name = \"\", const bool gravity_enable = 0, const bool collision_enable = 1) ");
      helpString.push_back("Make a reference frame. ");
      helpString.push_back("@param size size of the frame [m] ");
      helpString.push_back("@param pose pose of the frame [m] ");
      helpString.push_back("@param color color of the frame ");
      helpString.push_back("@param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used. ");
      helpString.push_back("@param object_name (optional) assigns a name to the object. ");
      helpString.push_back("@param gravity_enable (optional) enables gravity (default false) ");
      helpString.push_back("@param collision_enable (optional) enables collision (default true) ");
      helpString.push_back("@return returns a string that contains the name of the object in the world ");
    }
    if (functionName=="changeColor") {
      helpString.push_back("bool changeColor(const std::string& id, const Color& color) ");
      helpString.push_back("Change the color of an object ");
      helpString.push_back("@param id object id ");
      helpString.push_back("@param color color of the frame ");
      helpString.push_back("@return returns true or false on success failure ");
    }
    if (functionName=="setPose") {
      helpString.push_back("bool setPose(const std::string& id, const Pose& pose, const std::string& frame_name = \"\") ");
      helpString.push_back("Set new object pose. ");
      helpString.push_back("@param id object id ");
      helpString.push_back("@param pose new pose ");
      helpString.push_back("@param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used. ");
      helpString.push_back("@return returns true or false on success failure ");
    }
    if (functionName=="enableGravity") {
      helpString.push_back("bool enableGravity(const std::string& id, const bool enable) ");
      helpString.push_back("Enable/disables gravity for an object ");
      helpString.push_back("@param id object id ");
      helpString.push_back("@param enable 1 to enable gravity, 0 otherwise ");
      helpString.push_back("@return returns true or false on success failure ");
    }
    if (functionName=="enableCollision") {
      helpString.push_back("bool enableCollision(const std::string& id, const bool enable) ");
      helpString.push_back("Enable/disables collision detection for an object ");
      helpString.push_back("@param id object id ");
      helpString.push_back("@param enable 1 to enable collision detection, 0 otherwise ");
      helpString.push_back("@return returns true or false on success failure ");
    }
    if (functionName=="getPose") {
      helpString.push_back("Pose getPose(const std::string& id, const std::string& frame_name = \"\") ");
      helpString.push_back("Get object pose. ");
      helpString.push_back("@param id string that identifies object in gazebo (returned after creation) ");
      helpString.push_back("@param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used. ");
      helpString.push_back("@return returns value of the pose in the world reference frame ");
    }
    if (functionName=="loadModelFromFile") {
      helpString.push_back("bool loadModelFromFile(const std::string& filename) ");
      helpString.push_back("Load a model from file. ");
      helpString.push_back("@param id string that specifies the name of the model ");
      helpString.push_back("@return returns true/false on success failure. ");
    }
    if (functionName=="deleteObject") {
      helpString.push_back("bool deleteObject(const std::string& id) ");
      helpString.push_back("Delete an object. ");
      helpString.push_back("@param id string that identifies object in gazebo (returned after creation) ");
      helpString.push_back("@return returns true/false on success failure. ");
    }
    if (functionName=="deleteAll") {
      helpString.push_back("bool deleteAll() ");
      helpString.push_back("Delete all objects in the world. ");
    }
    if (functionName=="getList") {
      helpString.push_back("std::vector<std::string>  getList() ");
      helpString.push_back("List id of all objects that have been added to the world. ");
      helpString.push_back("@return return a list of string containing the id of the objects ");
    }
    if (functionName=="attach") {
      helpString.push_back("bool attach(const std::string& id, const std::string& link_name) ");
      helpString.push_back("Attach an object to a link of the robot. ");
      helpString.push_back("@param id string that identifies object in gazebo (returned after creation) ");
      helpString.push_back("@param link_name name of a link of the robot ");
      helpString.push_back("@return true if success, false otherwise ");
    }
    if (functionName=="detach") {
      helpString.push_back("bool detach(const std::string& id) ");
      helpString.push_back("Detach a previously attached object. ");
      helpString.push_back("@param id string that identifies object in gazebo (returned after creation) ");
      helpString.push_back("@return true if success, false otherwise ");
    }
    if (functionName=="rename") {
      helpString.push_back("bool rename(const std::string& old_name, const std::string& new_name) ");
      helpString.push_back("Change the names of an object. ");
      helpString.push_back("@param old_name string that identifies object in gazebo ");
      helpString.push_back("@param new_name string that will be used as new name ");
      helpString.push_back("@return true if success, false otherwise ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}
} // namespace


