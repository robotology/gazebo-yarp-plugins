/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <LinkAttacherServer.h>
#include <yarp/os/idl/WireTypes.h>

namespace GazeboYarpPlugins {


class LinkAttacherServer_enableGravity : public yarp::os::Portable {
public:
  std::string model_name;
  bool enable;
  bool _return;
  void init(const std::string& model_name, const bool enable);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class LinkAttacherServer_attachUnscoped : public yarp::os::Portable {
public:
  std::string model_name;
  std::string model_link_name;
  std::string robot_name;
  std::string robot_link_name;
  bool _return;
  void init(const std::string& model_name, const std::string& model_link_name, const std::string& robot_name, const std::string& robot_link_name);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

class LinkAttacherServer_detachUnscoped : public yarp::os::Portable {
public:
  std::string model_name;
  std::string model_link_name;
  bool _return;
  void init(const std::string& model_name, const std::string& model_link_name);
  virtual bool write(yarp::os::ConnectionWriter& connection) const override;
  virtual bool read(yarp::os::ConnectionReader& connection) override;
};

bool LinkAttacherServer_enableGravity::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("enableGravity",1,1)) return false;
  if (!writer.writeString(model_name)) return false;
  if (!writer.writeBool(enable)) return false;
  return true;
}

bool LinkAttacherServer_enableGravity::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void LinkAttacherServer_enableGravity::init(const std::string& model_name, const bool enable) {
  _return = false;
  this->model_name = model_name;
  this->enable = enable;
}

bool LinkAttacherServer_attachUnscoped::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(5)) return false;
  if (!writer.writeTag("attachUnscoped",1,1)) return false;
  if (!writer.writeString(model_name)) return false;
  if (!writer.writeString(model_link_name)) return false;
  if (!writer.writeString(robot_name)) return false;
  if (!writer.writeString(robot_link_name)) return false;
  return true;
}

bool LinkAttacherServer_attachUnscoped::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void LinkAttacherServer_attachUnscoped::init(const std::string& model_name, const std::string& model_link_name, const std::string& robot_name, const std::string& robot_link_name) {
  _return = false;
  this->model_name = model_name;
  this->model_link_name = model_link_name;
  this->robot_name = robot_name;
  this->robot_link_name = robot_link_name;
}

bool LinkAttacherServer_detachUnscoped::write(yarp::os::ConnectionWriter& connection) const {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("detachUnscoped",1,1)) return false;
  if (!writer.writeString(model_name)) return false;
  if (!writer.writeString(model_link_name)) return false;
  return true;
}

bool LinkAttacherServer_detachUnscoped::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void LinkAttacherServer_detachUnscoped::init(const std::string& model_name, const std::string& model_link_name) {
  _return = false;
  this->model_name = model_name;
  this->model_link_name = model_link_name;
}

LinkAttacherServer::LinkAttacherServer() {
  yarp().setOwner(*this);
}
bool LinkAttacherServer::enableGravity(const std::string& model_name, const bool enable) {
  bool _return = false;
  LinkAttacherServer_enableGravity helper;
  helper.init(model_name,enable);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool LinkAttacherServer::enableGravity(const std::string& model_name, const bool enable)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool LinkAttacherServer::attachUnscoped(const std::string& model_name, const std::string& model_link_name, const std::string& robot_name, const std::string& robot_link_name) {
  bool _return = false;
  LinkAttacherServer_attachUnscoped helper;
  helper.init(model_name,model_link_name,robot_name,robot_link_name);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool LinkAttacherServer::attachUnscoped(const std::string& model_name, const std::string& model_link_name, const std::string& robot_name, const std::string& robot_link_name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool LinkAttacherServer::detachUnscoped(const std::string& model_name, const std::string& model_link_name) {
  bool _return = false;
  LinkAttacherServer_detachUnscoped helper;
  helper.init(model_name,model_link_name);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool LinkAttacherServer::detachUnscoped(const std::string& model_name, const std::string& model_link_name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool LinkAttacherServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  std::string tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "enableGravity") {
      std::string model_name;
      bool enable;
      if (!reader.readString(model_name)) {
        reader.fail();
        return false;
      }
      if (!reader.readBool(enable)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = enableGravity(model_name,enable);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "attachUnscoped") {
      std::string model_name;
      std::string model_link_name;
      std::string robot_name;
      std::string robot_link_name;
      if (!reader.readString(model_name)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(model_link_name)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(robot_name)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(robot_link_name)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = attachUnscoped(model_name,model_link_name,robot_name,robot_link_name);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "detachUnscoped") {
      std::string model_name;
      std::string model_link_name;
      if (!reader.readString(model_name)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(model_link_name)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = detachUnscoped(model_name,model_link_name);
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

std::vector<std::string> LinkAttacherServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("enableGravity");
    helpString.push_back("attachUnscoped");
    helpString.push_back("detachUnscoped");
    helpString.push_back("help");
  }
  else {
    if (functionName=="enableGravity") {
      helpString.push_back("bool enableGravity(const std::string& model_name, const bool enable) ");
      helpString.push_back("Enable/disables gravity for a model ");
      helpString.push_back("@param model_name name that identifies model in gazebo (that are already spawned in gazebo) ");
      helpString.push_back("@param enable 1 to enable gravity, 0 otherwise ");
      helpString.push_back("@return returns true or false on success failure ");
    }
    if (functionName=="attachUnscoped") {
      helpString.push_back("bool attachUnscoped(const std::string& model_name, const std::string& model_link_name, const std::string& robot_name, const std::string& robot_link_name) ");
      helpString.push_back("Attach any link of the models spawned in gazebo to a link of the robot. ");
      helpString.push_back("@param model_name name that identifies model in gazebo (that are already spawned in gazebo) ");
      helpString.push_back("@param model_link_name name of a the link in the model you want to attach to the robot ");
      helpString.push_back("@param robot_name name of the robot ");
      helpString.push_back("@param robot_link_name name of the robot link to which you want to attached the model link ");
      helpString.push_back("@return true if success, false otherwise ");
    }
    if (functionName=="detachUnscoped") {
      helpString.push_back("bool detachUnscoped(const std::string& model_name, const std::string& model_link_name) ");
      helpString.push_back("Detach a previously attached model link. ");
      helpString.push_back("@param model_name name that identifies model in gazebo (that are already spawned in gazebo) ");
      helpString.push_back("@param model_link_name name of a the link in the model that is attached to the robot ");
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


