# ArgMap

## Introduction

ArgMap is a command line argument parser. It grew out of rudimentary code from
the [HElib](https://github.com/homenc/HElib) library project.

This repo is essentially a spin-off for ArgMap to be its own project as it has
found use in other projects done by HElib contributors and users due to its
feature set and terseness.

## Integration
Argmap is header-only and [argmap.h](argmap.h) is the single required file. You
must add
```cpp
#include "argmap/argmap.h"
```
to any file you wish to use the argument parser in and enable C++17 (e.g.,
-std=c++17 for GCC and Clang).

## Usage
```c++
// Variables to be set by command line with default values.
long p = 2;
long m = 19;
bool t = false;
bool f = true;
std::string k = "Hello World";
std::string aliases = "Aliases";

argmap::ArgMap()                                    // (*) marks default.
  .required()                                       // set args to required.
  .positional()                                     // positional arguments appear
    .arg("p", p, "doc for p")                       //   in order of declaration.
    .arg("m", m, "doc for m", "undefined")          // special default info.
  .optional()                                       // swap to optional args (*).
  .named()                                          // named args (*) e.g.k=v.
  .separator(argmap::ArgMap::Separator::WHITESPACE) // change separator to
    .arg("-k", k, "doc for k", "")                  // whitespace ('=' is (*)).
    .arg({"-q", "--r", "--sos"}, aliases)           // {} is how to declare aliases.
    .note("an extra note")                          // no default value info.
  .toggle()                                         // add extra doc/note.
     .arg("-t", t, "doc for t", "")                 // toggle flag sets bool true.
  .toggle(false)                                    // toggle flag sets bool false.
     .arg("-f", f, "doc for f", "")                 //
  .helpArgs({"--myhelp"})                           // changes default help flags
  .parse(argc, argv);                               // (*) is {"-h", "--help"}.
                                                    // parses and overwrites values
```

## Running the tests
Tests are written in [Google Test](https://github.com/google/googletest) and
can be built using the provided `cmake` script.

```bash
cmake -S . -B build
cmake --build build -j
```

Note that the script will fetch google test for you local to the build
(standard `_deps` directory that `cmake` creates).

Run
```bash
./build/bin/test_argmap
```

### NTL tests
Because ArgMap spun-off from HElib, the tests also test whether ArgMap can read
in [NTL's](https://github.com/libntl/ntl) vector `Vec` type. To enable these
tests set the configuration step with the following flag.

```cmake
cmake -S . -B build -DNTL_TESTS=ON
```
