MBusAnalyzer
============

This is the source for an Analyzer Module for the Saleae logic analyzer for MBus.

This is a very rough first draft, but basic things seem to be working ish.

Building
--------

To build you must first download the [Saleae SDK](http://community.saleae.com/AnalyzerSdk).
Checkout this repository inside the SDK folder, cd into the checked out module
folder and run `python build_analyzer.py` (Note: There is also a
`build_analyzer.py` in the SDK directory, do not run that).

If you're building on a 64-bit machine, you may have to go into the SDK/lib
folder and rename some things so saleae grabs the right library. Yes, I know
there are better ways to handle this, take it up with saleae.
