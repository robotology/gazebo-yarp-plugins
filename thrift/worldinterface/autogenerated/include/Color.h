// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_Color
#define YARP_THRIFT_GENERATOR_STRUCT_Color

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace GazeboYarpPlugins {
  class Color;
}


class GazeboYarpPlugins::Color : public yarp::os::idl::WirePortable {
public:
  // Fields
  int16_t r;
  int16_t g;
  int16_t b;

  // Default constructor
  Color() : r(0), g(0), b(0) {
  }

  // Constructor with field values
  Color(const int16_t r,const int16_t g,const int16_t b) : r(r), g(g), b(b) {
  }

  // Copy constructor
  Color(const Color& __alt) : WirePortable(__alt)  {
    this->r = __alt.r;
    this->g = __alt.g;
    this->b = __alt.b;
  }

  // Assignment operator
  const Color& operator = (const Color& __alt) {
    this->r = __alt.r;
    this->g = __alt.g;
    this->b = __alt.b;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);

private:
  bool write_r(yarp::os::idl::WireWriter& writer);
  bool nested_write_r(yarp::os::idl::WireWriter& writer);
  bool write_g(yarp::os::idl::WireWriter& writer);
  bool nested_write_g(yarp::os::idl::WireWriter& writer);
  bool write_b(yarp::os::idl::WireWriter& writer);
  bool nested_write_b(yarp::os::idl::WireWriter& writer);
  bool read_r(yarp::os::idl::WireReader& reader);
  bool nested_read_r(yarp::os::idl::WireReader& reader);
  bool read_g(yarp::os::idl::WireReader& reader);
  bool nested_read_g(yarp::os::idl::WireReader& reader);
  bool read_b(yarp::os::idl::WireReader& reader);
  bool nested_read_b(yarp::os::idl::WireReader& reader);

public:

  yarp::os::ConstString toString();

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<GazeboYarpPlugins::Color > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new Color;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(Color& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(Color& obj, bool dirty = true) {
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

    Color& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_r(const int16_t r) {
      will_set_r();
      obj->r = r;
      mark_dirty_r();
      communicate();
      did_set_r();
    }
    void set_g(const int16_t g) {
      will_set_g();
      obj->g = g;
      mark_dirty_g();
      communicate();
      did_set_g();
    }
    void set_b(const int16_t b) {
      will_set_b();
      obj->b = b;
      mark_dirty_b();
      communicate();
      did_set_b();
    }
    int16_t get_r() {
      return obj->r;
    }
    int16_t get_g() {
      return obj->g;
    }
    int16_t get_b() {
      return obj->b;
    }
    virtual bool will_set_r() { return true; }
    virtual bool will_set_g() { return true; }
    virtual bool will_set_b() { return true; }
    virtual bool did_set_r() { return true; }
    virtual bool did_set_g() { return true; }
    virtual bool did_set_b() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection);
    bool write(yarp::os::ConnectionWriter& connection);
  private:

    Color *obj;

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
    void mark_dirty_r() {
      if (is_dirty_r) return;
      dirty_count++;
      is_dirty_r = true;
      mark_dirty();
    }
    void mark_dirty_g() {
      if (is_dirty_g) return;
      dirty_count++;
      is_dirty_g = true;
      mark_dirty();
    }
    void mark_dirty_b() {
      if (is_dirty_b) return;
      dirty_count++;
      is_dirty_b = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_r = flag;
      is_dirty_g = flag;
      is_dirty_b = flag;
      dirty_count = flag ? 3 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_r;
    bool is_dirty_g;
    bool is_dirty_b;
  };
};

#endif
