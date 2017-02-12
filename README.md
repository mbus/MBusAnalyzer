MBusAnalyzer
============

This is the source for an Analyzer Module for the Saleae logic analyzer for MBus.


Usage
-----

This library ships with a built version of the analzyer module. For most users,
this should be all you need.

  1. Open Logic. Go to Options (top right), Preferences, Developer tab
  2. Set the search path for analyzer plugins to `release/[version]/[platform]/`
  3. Restart Logic software
  4. Under the Analyzers section, click the `+` icon, choose `Show more analyzers` and select MBus


For Saleae Plug-in Developers
-----------------------------

This library includes a git [submodule](https://github.com/blog/2104-working-with-submodules),
which links the [Saleae SDK](http://community.saleae.com/AnalyzerSdk). After cloning this
repository, verify that the AnalyzerSDK folder has files in `include` and `lib`, otherwise run

    git submodule update --init --recursive

On OS X / Linux, you should simply be able to run `python build_analyzer.py`. In Windows
environments, open the Visual Studio solution in `vs/MBusAnalyzer.sln`.

Things should be set up to build for a 64-bit enivonrment by default, though sometimes the
Visual Studio build environment seems to point back to the 32-bit version.

### Debugging

According to Saeleae, debugging on Windows is a tricky proposition, so use gdb/lldb on Linux/Mac.

Assuming Logic is already open, simply run `process attach --name Logic` to attach the debugger.
