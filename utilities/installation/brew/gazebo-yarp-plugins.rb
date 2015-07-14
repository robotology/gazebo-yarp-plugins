class GazeboYarpPlugins < Formula
  homepage "https://github.com/robotology/gazebo-yarp-plugins"

  stable do
    url "https://github.com/robotology/gazebo-yarp-plugins/archive/v0.1.2.tar.gz"
    sha256 "85fe3c1754b441c65e92f5c54149bc1dd1ed7c63365a413d2fad89236101d208"
  end

  head do
    url "https://github.com/robotology/gazebo-yarp-plugins.git", :branch => "master"
  end

  ## Dependencies
  depends_on "cmake" => :build
  depends_on "pkg-config" => :build
  option "without-yarp", "Don't use yarp from brew, but use the CMake facilities to located it (e.g. by manually specify the location with YARP_DIR environment variable)"
  depends_on "yarp" => :recommended
  depends_on "gazebo5"

  ## The install method.
  def install
    # Now the sources (from `url`) are downloaded, hash-checked and
    # Homebrew has changed into a temporary directory where the
    # archive has been unpacked or the repository has been cloned.

    # For Cmake, we have some necessary defaults in `std_cmake_args`:
    mkdir "build" do
      system "cmake", "..", *std_cmake_args
      system "cmake", "." # this fixes an issue with gazebo cmake-config
      system "make", "install"
    end
  end

  test do

    (testpath/"test.sh").write <<-EOS.undent
      #!/bin/bash
      yarp namespace /brew_test
      yarpserver &
      PID_yarpserver=$!

      export GAZEBO_MODEL_DATABASE_URI=""
      gzserver -slibgazebo_yarp_clock.so --verbose &
      PID_gzserver=$!
      sleep 2
      yarp exists /clock
      SUCCESS=$?
      if [ ${SUCCESS} -eq 0 ]
        then echo "Test successfull";
      else
        echo "Test failed";
      fi

      kill -9 ${PID_gzserver}
      kill -9 ${PID_yarpserver}

      exit ${SUCCESS}
    EOS
    # To capture the output of a command, we use backtics:
    system 'sh', (testpath/"test.sh")
    shell_output("echo $?")
    ohai "Success"
  end


end
