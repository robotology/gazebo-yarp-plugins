// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <thrift/ClockServer.h>
#include <yarp/os/idl/WireTypes.h>

namespace gazebo {


class ClockServer_pauseSimulation : public yarp::os::Portable {
public:
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ClockServer_continueSimulation : public yarp::os::Portable {
public:
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ClockServer_stepSimulation : public yarp::os::Portable {
public:
  int32_t numberOfSteps;
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ClockServer_stepSimulationAndWait : public yarp::os::Portable {
public:
  int32_t numberOfSteps;
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ClockServer_resetSimulationTime : public yarp::os::Portable {
public:
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ClockServer_getSimulationTime : public yarp::os::Portable {
public:
  double _return;
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class ClockServer_getStepSize : public yarp::os::Portable {
public:
  double _return;
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool ClockServer_pauseSimulation::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("pauseSimulation",1,1)) return false;
  return true;
}

bool ClockServer_pauseSimulation::read(yarp::os::ConnectionReader& connection) {
  YARP_UNUSED(connection);
  return true;
}

bool ClockServer_continueSimulation::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("continueSimulation",1,1)) return false;
  return true;
}

bool ClockServer_continueSimulation::read(yarp::os::ConnectionReader& connection) {
  YARP_UNUSED(connection);
  return true;
}

bool ClockServer_stepSimulation::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("stepSimulation",1,1)) return false;
  if (!writer.writeI32(numberOfSteps)) return false;
  return true;
}

bool ClockServer_stepSimulation::read(yarp::os::ConnectionReader& connection) {
  YARP_UNUSED(connection);
  return true;
}

bool ClockServer_stepSimulationAndWait::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("stepSimulationAndWait",1,1)) return false;
  if (!writer.writeI32(numberOfSteps)) return false;
  return true;
}

bool ClockServer_stepSimulationAndWait::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  return true;
}

bool ClockServer_resetSimulationTime::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("resetSimulationTime",1,1)) return false;
  return true;
}

bool ClockServer_resetSimulationTime::read(yarp::os::ConnectionReader& connection) {
  YARP_UNUSED(connection);
  return true;
}

bool ClockServer_getSimulationTime::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getSimulationTime",1,1)) return false;
  return true;
}

bool ClockServer_getSimulationTime::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

bool ClockServer_getStepSize::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getStepSize",1,1)) return false;
  return true;
}

bool ClockServer_getStepSize::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

ClockServer::ClockServer() {
  yarp().setOwner(*this);
}
void ClockServer::pauseSimulation() {
  ClockServer_pauseSimulation helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","void ClockServer::pauseSimulation()");
  }
  yarp().write(helper);
}
void ClockServer::continueSimulation() {
  ClockServer_continueSimulation helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","void ClockServer::continueSimulation()");
  }
  yarp().write(helper);
}
void ClockServer::stepSimulation(const int32_t numberOfSteps) {
  ClockServer_stepSimulation helper;
  helper.numberOfSteps = numberOfSteps;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","void ClockServer::stepSimulation(const int32_t numberOfSteps)");
  }
  yarp().write(helper);
}
void ClockServer::stepSimulationAndWait(const int32_t numberOfSteps) {
  ClockServer_stepSimulationAndWait helper;
  helper.numberOfSteps = numberOfSteps;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","void ClockServer::stepSimulationAndWait(const int32_t numberOfSteps)");
  }
  yarp().write(helper,helper);
}
void ClockServer::resetSimulationTime() {
  ClockServer_resetSimulationTime helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","void ClockServer::resetSimulationTime()");
  }
  yarp().write(helper);
}
double ClockServer::getSimulationTime() {
  double _return = (double)0;
  ClockServer_getSimulationTime helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","double ClockServer::getSimulationTime()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double ClockServer::getStepSize() {
  double _return = (double)0;
  ClockServer_getStepSize helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","double ClockServer::getStepSize()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool ClockServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "pauseSimulation") {
      pauseSimulation();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeOnewayResponse()) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "continueSimulation") {
      continueSimulation();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeOnewayResponse()) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stepSimulation") {
      int32_t numberOfSteps;
      if (!reader.readI32(numberOfSteps)) {
        numberOfSteps = 1;
      }
      stepSimulation(numberOfSteps);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeOnewayResponse()) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stepSimulationAndWait") {
      int32_t numberOfSteps;
      if (!reader.readI32(numberOfSteps)) {
        numberOfSteps = 1;
      }
      stepSimulationAndWait(numberOfSteps);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "resetSimulationTime") {
      resetSimulationTime();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeOnewayResponse()) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getSimulationTime") {
      double _return;
      _return = getSimulationTime();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getStepSize") {
      double _return;
      _return = getStepSize();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
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

std::vector<std::string> ClockServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("pauseSimulation");
    helpString.push_back("continueSimulation");
    helpString.push_back("stepSimulation");
    helpString.push_back("stepSimulationAndWait");
    helpString.push_back("resetSimulationTime");
    helpString.push_back("getSimulationTime");
    helpString.push_back("getStepSize");
    helpString.push_back("help");
  }
  else {
    if (functionName=="pauseSimulation") {
      helpString.push_back("void pauseSimulation() ");
      helpString.push_back("Pause the simulation if it was running ");
    }
    if (functionName=="continueSimulation") {
      helpString.push_back("void continueSimulation() ");
      helpString.push_back("Resume the simulation if it was paused ");
    }
    if (functionName=="stepSimulation") {
      helpString.push_back("void stepSimulation(const int32_t numberOfSteps = 1) ");
      helpString.push_back("Steps the simulation for the provided number of steps. ");
      helpString.push_back("The input paramter is the number of steps, not the time (Usually 1 step = 1ms but this is not guaranteed) ");
      helpString.push_back("@note: this function (will be) not blocking, i.e. it will return immediately ");
      helpString.push_back("@param numberOfSteps number of steps to simulate ");
    }
    if (functionName=="stepSimulationAndWait") {
      helpString.push_back("void stepSimulationAndWait(const int32_t numberOfSteps = 1) ");
      helpString.push_back("Steps the simulation for the provided number of steps. ");
      helpString.push_back("The input paramter is the number of steps, not the time (Usually 1 step = 1ms but this is not guaranteed) ");
      helpString.push_back("@note: this function is blocking ");
      helpString.push_back("@param numberOfSteps number of steps to simulate ");
    }
    if (functionName=="resetSimulationTime") {
      helpString.push_back("void resetSimulationTime() ");
      helpString.push_back("Reset the simulation time back to zero ");
    }
    if (functionName=="getSimulationTime") {
      helpString.push_back("double getSimulationTime() ");
      helpString.push_back("Get the current simulation time ");
      helpString.push_back("@return the simulation time. ");
    }
    if (functionName=="getStepSize") {
      helpString.push_back("double getStepSize() ");
      helpString.push_back("Get the current step size in seconds. ");
      helpString.push_back("@return the step size in seconds ");
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


