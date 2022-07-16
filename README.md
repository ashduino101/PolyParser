# PolyParser
A converter for Poly Bridge 2's .layout files, written in C++. Similar to PolyConverter, but is over 25x faster and does not require the game to be installed.

# How to use

1. Open Command Prompt or your favorite terminal.
2. Navigate to the directory you have PolyParser downloaded to.
3. Run `./PolyParser "<path to layout>"` (or `./PolyParser.exe "<path to layout>"` for Windows)
4. The converted layout should be in the same directory as the original.

# Build from source

Building from source requires CMake, a C++ compiler, and GNU Make.
All you have to do is execute `build.sh` on Linux, or `build.bat` on Windows.
To build for both platforms, run `python build.py`.

# License

This project and every dependency is licensed under the MIT License.
