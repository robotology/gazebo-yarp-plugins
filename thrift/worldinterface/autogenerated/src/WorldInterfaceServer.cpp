// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <WorldInterfaceServer.h>
#include <yarp/os/idl/WireTypes.h>

namespace GazeboYarpPlugins {


class WorldInterfaceServer_makeSphere : public yarp::os::Portable {
public:
  double radius;
  Pose pose;
  Color color;
  std::string _return;
  void init(const double radius, const Pose& pose, const Color& color);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_makeBox : public yarp::os::Portable {
public:
  double width;
  double height;
  double thickness;
  Pose pose;
  Color color;
  std::string _return;
  void init(const double width, const double height, const double thickness, const Pose& pose, const Color& color);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_makeCylinder : public yarp::os::Portable {
public:
  double radius;
  double length;
  Pose pose;
  Color color;
  std::string _return;
  void init(const double radius, const double length, const Pose& pose, const Color& color);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_makeFrame : public yarp::os::Portable {
public:
  double size;
  Pose pose;
  Color color;
  std::string _return;
  void init(const double size, const Pose& pose, const Color& color);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_changeColor : public yarp::os::Portable {
public:
  std::string id;
  Color color;
  bool _return;
  void init(const std::string& id, const Color& color);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_setPose : public yarp::os::Portable {
public:
  std::string id;
  Pose pose;
  bool _return;
  void init(const std::string& id, const Pose& pose);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_enableGravity : public yarp::os::Portable {
public:
  std::string id;
  bool enable;
  bool _return;
  void init(const std::string& id, const bool enable);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_enableCollision : public yarp::os::Portable {
public:
  std::string id;
  bool enable;
  bool _return;
  void init(const std::string& id, const bool enable);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_getPose : public yarp::os::Portable {
public:
  std::string id;
  Pose _return;
  void init(const std::string& id);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_loadModelFromFile : public yarp::os::Portable {
public:
  std::string filename;
  bool _return;
  void init(const std::string& filename);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_deleteObject : public yarp::os::Portable {
public:
  std::string id;
  bool _return;
  void init(const std::string& id);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_deleteAll : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_getList : public yarp::os::Portable {
public:
  std::vector<std::string>  _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_attach : public yarp::os::Portable {
public:
  std::string id;
  std::string link_name;
  bool _return;
  void init(const std::string& id, const std::string& link_name);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class WorldInterfaceServer_detach : public yarp::os::Portable {
public:
  std::string id;
  bool _return;
  void init(const std::string& id);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool WorldInterfaceServer_makeSphere::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(11)) return false;
  if (!writer.writeTag("makeSphere",1,1)) return false;
  if (!writer.writeDouble(radius)) return false;
  if (!writer.write(pose)) return false;
  if (!writer.write(color)) return false;
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

void WorldInterfaceServer_makeSphere::init(const double radius, const Pose& pose, const Color& color) {
  _return = "";
  this->radius = radius;
  this->pose = pose;
  this->color = color;
}

bool WorldInterfaceServer_makeBox::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(13)) return false;
  if (!writer.writeTag("makeBox",1,1)) return false;
  if (!writer.writeDouble(width)) return false;
  if (!writer.writeDouble(height)) return false;
  if (!writer.writeDouble(thickness)) return false;
  if (!writer.write(pose)) return false;
  if (!writer.write(color)) return false;
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

void WorldInterfaceServer_makeBox::init(const double width, const double height, const double thickness, const Pose& pose, const Color& color) {
  _return = "";
  this->width = width;
  this->height = height;
  this->thickness = thickness;
  this->pose = pose;
  this->color = color;
}

bool WorldInterfaceServer_makeCylinder::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(12)) return false;
  if (!writer.writeTag("makeCylinder",1,1)) return false;
  if (!writer.writeDouble(radius)) return false;
  if (!writer.writeDouble(length)) return false;
  if (!writer.write(pose)) return false;
  if (!writer.write(color)) return false;
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

void WorldInterfaceServer_makeCylinder::init(const double radius, const double length, const Pose& pose, const Color& color) {
  _return = "";
  this->radius = radius;
  this->length = length;
  this->pose = pose;
  this->color = color;
}

bool WorldInterfaceServer_makeFrame::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(11)) return false;
  if (!writer.writeTag("makeFrame",1,1)) return false;
  if (!writer.writeDouble(size)) return false;
  if (!writer.write(pose)) return false;
  if (!writer.write(color)) return false;
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

void WorldInterfaceServer_makeFrame::init(const double size, const Pose& pose, const Color& color) {
  _return = "";
  this->size = size;
  this->pose = pose;
  this->color = color;
}

bool WorldInterfaceServer_changeColor::write(yarp::os::ConnectionWriter& connection) {
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

bool WorldInterfaceServer_setPose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(8)) return false;
  if (!writer.writeTag("setPose",1,1)) return false;
  if (!writer.writeString(id)) return false;
  if (!writer.write(pose)) return false;
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

void WorldInterfaceServer_setPose::init(const std::string& id, const Pose& pose) {
  _return = false;
  this->id = id;
  this->pose = pose;
}

bool WorldInterfaceServer_enableGravity::write(yarp::os::ConnectionWriter& connection) {
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

bool WorldInterfaceServer_enableCollision::write(yarp::os::ConnectionWriter& connection) {
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

bool WorldInterfaceServer_getPose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("getPose",1,1)) return false;
  if (!writer.writeString(id)) return false;
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

void WorldInterfaceServer_getPose::init(const std::string& id) {
  this->id = id;
}

bool WorldInterfaceServer_loadModelFromFile::write(yarp::os::ConnectionWriter& connection) {
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

bool WorldInterfaceServer_deleteObject::write(yarp::os::ConnectionWriter& connection) {
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

bool WorldInterfaceServer_deleteAll::write(yarp::os::ConnectionWriter& connection) {
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

bool WorldInterfaceServer_getList::write(yarp::os::ConnectionWriter& connection) {
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

bool WorldInterfaceServer_attach::write(yarp::os::ConnectionWriter& connection) {
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

bool WorldInterfaceServer_detach::write(yarp::os::ConnectionWriter& connection) {
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

WorldInterfaceServer::WorldInterfaceServer() {
  yarp().setOwner(*this);
}
std::string WorldInterfaceServer::makeSphere(const double radius, const Pose& pose, const Color& color) {
  std::string _return = "";
  WorldInterfaceServer_makeSphere helper;
  helper.init(radius,pose,color);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string WorldInterfaceServer::makeSphere(const double radius, const Pose& pose, const Color& color)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string WorldInterfaceServer::makeBox(const double width, const double height, const double thickness, const Pose& pose, const Color& color) {
  std::string _return = "";
  WorldInterfaceServer_makeBox helper;
  helper.init(width,height,thickness,pose,color);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string WorldInterfaceServer::makeBox(const double width, const double height, const double thickness, const Pose& pose, const Color& color)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string WorldInterfaceServer::makeCylinder(const double radius, const double length, const Pose& pose, const Color& color) {
  std::string _return = "";
  WorldInterfaceServer_makeCylinder helper;
  helper.init(radius,length,pose,color);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string WorldInterfaceServer::makeCylinder(const double radius, const double length, const Pose& pose, const Color& color)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string WorldInterfaceServer::makeFrame(const double size, const Pose& pose, const Color& color) {
  std::string _return = "";
  WorldInterfaceServer_makeFrame helper;
  helper.init(size,pose,color);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string WorldInterfaceServer::makeFrame(const double size, const Pose& pose, const Color& color)");
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
bool WorldInterfaceServer::setPose(const std::string& id, const Pose& pose) {
  bool _return = false;
  WorldInterfaceServer_setPose helper;
  helper.init(id,pose);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool WorldInterfaceServer::setPose(const std::string& id, const Pose& pose)");
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
Pose WorldInterfaceServer::getPose(const std::string& id) {
  Pose _return;
  WorldInterfaceServer_getPose helper;
  helper.init(id);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","Pose WorldInterfaceServer::getPose(const std::string& id)");
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

bool WorldInterfaceServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "makeSphere") {
      double radius;
      Pose pose;
      Color color;
      if (!reader.readDouble(radius)) {
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
      std::string _return;
      _return = makeSphere(radius,pose,color);
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
      if (!reader.readDouble(width)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(height)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(thickness)) {
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
      std::string _return;
      _return = makeBox(width,height,thickness,pose,color);
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
      if (!reader.readDouble(radius)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(length)) {
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
      std::string _return;
      _return = makeCylinder(radius,length,pose,color);
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
      if (!reader.readDouble(size)) {
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
      std::string _return;
      _return = makeFrame(size,pose,color);
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
      if (!reader.readString(id)) {
        reader.fail();
        return false;
      }
      if (!reader.read(pose)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setPose(id,pose);
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
      if (!reader.readString(id)) {
        reader.fail();
        return false;
      }
      Pose _return;
      _return = getPose(id);
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
          std::vector<std::string> ::iterator _iter5;
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
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
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
    yarp::os::ConstString next_tag = reader.readTag();
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
    helpString.push_back("help");
  }
  else {
    if (functionName=="makeSphere") {
      helpString.push_back("std::string makeSphere(const double radius, const Pose& pose, const Color& color) ");
      helpString.push_back("Make a shpere. ");
      helpString.push_back("@param radius radius of the sphere [m] ");
      helpString.push_back("@param pose pose of the sphere [m] ");
      helpString.push_back("@param color color of the sphere ");
      helpString.push_back("@return returns a string that contains the name of the object in the world ");
    }
    if (functionName=="makeBox") {
      helpString.push_back("std::string makeBox(const double width, const double height, const double thickness, const Pose& pose, const Color& color) ");
      helpString.push_back("Make a shpere. ");
      helpString.push_back("@param width box width [m] ");
      helpString.push_back("@param height box height[m] ");
      helpString.push_back("@param thickness box thickness [m] ");
      helpString.push_back("@param pose pose of the box [m] ");
      helpString.push_back("@param color color of the box ");
      helpString.push_back("@return returns a string that contains the name of the object in the world ");
    }
    if (functionName=="makeCylinder") {
      helpString.push_back("std::string makeCylinder(const double radius, const double length, const Pose& pose, const Color& color) ");
      helpString.push_back("Make a cylinder. ");
      helpString.push_back("@param radius radius of the cylinder [m] ");
      helpString.push_back("@param length lenght of the cylinder [m] ");
      helpString.push_back("@param pose pose of the cylinder [m] ");
      helpString.push_back("@param color color of the cylinder ");
      helpString.push_back("@return returns a string that contains the name of the object in the world ");
    }
    if (functionName=="makeFrame") {
      helpString.push_back("std::string makeFrame(const double size, const Pose& pose, const Color& color) ");
      helpString.push_back("Make a reference frame. ");
      helpString.push_back("@param size size of the frame [m] ");
      helpString.push_back("@param pose pose of the frame [m] ");
      helpString.push_back("@param color color of the frame ");
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
      helpString.push_back("bool setPose(const std::string& id, const Pose& pose) ");
      helpString.push_back("Set new object pose. ");
      helpString.push_back("@param id object id ");
      helpString.push_back("@param pose new pose ");
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
      helpString.push_back("Pose getPose(const std::string& id) ");
      helpString.push_back("Get object pose. ");
      helpString.push_back("@param id string that identifies object in gazebo (returned after creation) ");
      helpString.push_back("@return returns value of the pose ");
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


