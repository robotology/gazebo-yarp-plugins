// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <thrift/ObjectsServer.h>
#include <yarp/os/idl/WireTypes.h>

namespace gazebo {


class ObjectsServer_attach : public yarp::os::Portable {
public:
  std::string link_name;
  std::string object_name;
  double width;
  double height;
  double length;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(6)) return false;
    if (!writer.writeTag("attach",1,1)) return false;
    if (!writer.writeString(link_name)) return false;
    if (!writer.writeString(object_name)) return false;
    if (!writer.writeDouble(width)) return false;
    if (!writer.writeDouble(height)) return false;
    if (!writer.writeDouble(length)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class ObjectsServer_detach : public yarp::os::Portable {
public:
  std::string object_name;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("detach",1,1)) return false;
    if (!writer.writeString(object_name)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class ObjectsServer_deleteObject : public yarp::os::Portable {
public:
  std::string object_name;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("deleteObject",1,1)) return false;
    if (!writer.writeString(object_name)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

bool ObjectsServer::attach(const std::string& link_name, const std::string& object_name, const double width, const double height, const double length) {
  bool _return = false;
  ObjectsServer_attach helper;
  helper.link_name = link_name;
  helper.object_name = object_name;
  helper.width = width;
  helper.height = height;
  helper.length = length;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool ObjectsServer::attach(const std::string& link_name, const std::string& object_name, const double width, const double height, const double length)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool ObjectsServer::detach(const std::string& object_name) {
  bool _return = false;
  ObjectsServer_detach helper;
  helper.object_name = object_name;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool ObjectsServer::detach(const std::string& object_name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool ObjectsServer::deleteObject(const std::string& object_name) {
  bool _return = false;
  ObjectsServer_deleteObject helper;
  helper.object_name = object_name;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool ObjectsServer::deleteObject(const std::string& object_name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool ObjectsServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "attach") {
      std::string link_name;
      std::string object_name;
      double width;
      double height;
      double length;
      if (!reader.readString(link_name)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(object_name)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(width)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(height)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(length)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = attach(link_name,object_name,width,height,length);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "detach") {
      std::string object_name;
      if (!reader.readString(object_name)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = detach(object_name);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "deleteObject") {
      std::string object_name;
      if (!reader.readString(object_name)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = deleteObject(object_name);
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

std::vector<std::string> ObjectsServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("attach");
    helpString.push_back("detach");
    helpString.push_back("deleteObject");
    helpString.push_back("help");
  }
  else {
    if (functionName=="attach") {
      helpString.push_back("bool attach(const std::string& link_name, const std::string& object_name, const double width, const double height, const double length) ");
      helpString.push_back("Get attach an object to a link of the robot. ");
      helpString.push_back("@param link_name Name of the link ");
      helpString.push_back("@param object_name Name of the box to be operated ");
      helpString.push_back("@param width width of the box to be operated ");
      helpString.push_back("@param height height of the box to be operated ");
      helpString.push_back("@param length length of the box to be operated ");
      helpString.push_back("@return true if success, false otherwise ");
    }
    if (functionName=="detach") {
      helpString.push_back("bool detach(const std::string& object_name) ");
      helpString.push_back("Detaches an object to a link of the robot. ");
      helpString.push_back("@param object_name Name of the object ");
      helpString.push_back("@return true if success, false otherwise ");
    }
    if (functionName=="deleteObject") {
      helpString.push_back("bool deleteObject(const std::string& object_name) ");
      helpString.push_back("Delete an object from Gazebo environment. ");
      helpString.push_back("@param object_name Name of the object ");
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


