# TODO:
# - yarp can be installed not through brew?
# - which version of gazebo?

class GazeboYarpPlugins < Formula
  homepage "https://github.com/robotology/gazebo-yarp-plugins"

  stable do
    url "https://github.com/robotology/gazebo-yarp-plugins/archive/v0.1.1.tar.gz"
    sha1 "19a471bcfc2e735b9b7a58992ea3b22eec467f75"
  end

  head do
    url "https://github.com/robotology/gazebo-yarp-plugins.git"
  end

  # # Bottles are pre-built and added by the Homebrew maintainers for you.
  # # If you maintain your own repository, you can add your own bottle links.
  # # https://github.com/Homebrew/homebrew/blob/master/share/doc/homebrew/Bottles.md
  # bottle do
  #   root_url "http://mikemcquaid.com" # Optional root to calculate bottle URLs
  #   prefix "/opt/homebrew" # Optional HOMEBREW_PREFIX in which the bottles were built.
  #   cellar "/opt/homebrew/Cellar" # Optional HOMEBREW_CELLAR in which the bottles were built.
  #   revision 1 # Making the old bottle outdated without bumping the version of the formula.
  #   sha1 "d3d13fe6f42416765207503a946db01378131d7b" => :yosemite
  #   sha1 "cdc48e79de2dee796bb4ba1ad987f6b35ce1c1ee" => :mavericks
  #   sha1 "a19b544c8c645d7daad1d39a070a0eb86dfe9b9c" => :mountain_lion
  # end
  #
  # def pour_bottle?
  #   # Only needed if this formula has to check if using the pre-built
  #   # bottle is fine.
  #   true
  # end

  ## Dependencies
  depends_on "cmake" => :build
  depends_on "yarp" => :recommended
  depends_on "gazebo5"

  # Additional downloads can be defined as resources and accessed in the
  # install method. Resources can also be defined inside a stable, devel, or
  # head block. This mechanism replaces ad-hoc "subformula" classes.
  resource "iCub_models" do
    url "https://github.com/robotology-playground/icub-gazebo.git", :using => :git
#    sha1 "deadbeef7890123456789012345678901234567890"
  end
  ## The install method.

  def install
    # Now the sources (from `url`) are downloaded, hash-checked and
    # Homebrew has changed into a temporary directory where the
    # archive has been unpacked or the repository has been cloned.
  
    # For Cmake, we have some necessary defaults in `std_cmake_args`:
    system "cmake", ".", *std_cmake_args

    # If the arguments given to configure (or make or cmake) are depending
    # on options defined above, we usually make a list first and then
    # use the `args << if <condition>` to append to:
#    args = "-DCMAKE_BUILD_TYPE:STRING=RELEASE"

    # The `build.with?` and `build.without?` are smart enough to do the
    # right thing with respect to defaults defined via `:optional` and
    # `:recommended` dependencies.

    # If you need to give the path to lib/include of another brewed formula
    # please use the `opt_prefix` instead of the `prefix` of that other
    # formula. The reasoning behind this is that `prefix` has the exact
    # version number and if you update that other formula, things might
    # break if they remember that exact path. In contrast to that, the
    # `$(brew --prefix)/opt/formula` is the same path for all future
    # versions of the formula!
    
    # A general note: The commands here are executed line by line, so if
    # you change some variable or call a method like ENV.deparallelize, it
    # only affects the lines after that command.
    
    system "make", "install"

    # We are in a temporary directory and don't have to care about cleanup.

    # Instead of `system "cp"` or something, call `install` on the Pathname
    # objects as they are smarter with respect to correcting access rights.
    # (`install` is a Homebrew mixin into Ruby's Pathname)

    # Additional downloads can be defined as resources (see above).
    # The stage method will create a temporary directory and yield
    # to a block.
    #resource("additional_files").stage { bin.install "my/extra/tool" }

    # `name` and `version` are accessible too, if you need them.
  end


  ## Caveats
  ## Test (is optional but makes us happy)

  test do
    # `test do` will create, run in, and delete a temporary directory.
    #
    # # We are fine if the executable does not error out, so we know linking
    # # and building the software was ok.
    # system bin/"foobar", "--version"
    #
    # (testpath/"Test.file").write <<-EOS.undent
    #   writing some test file, if you need to
    # EOS
    # # To capture the output of a command, we use backtics:
    # assert_equal "OK", ` test.file`.strip
    #
    # # Need complete control over stdin, stdout?
    # require "open3"
    # Open3.popen3("#{bin}/example", "argument") do |stdin, stdout, _|
    #   stdin.write("some text")
    #   stdin.close
    #   assert_equal "result", stdout.read
    # end

    # The test will fail if it returns false, or if an exception is raised.
    # Failed assertions and failed `system` commands will raise exceptions.
  end


end
