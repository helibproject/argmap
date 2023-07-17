# ArgMap

## Introduction

ArgMap is a command line argument parser. It grew out of rudimentary code from
the HElib library project, .

This repo is essentially a spin-off for ArgMap to be its own project as it has
found use in other projects done HElib contributors and users due its feature
set and terseness.

## Usage

```c++
// Variables to be set by command line.
long p = 2;                                 // default values.
long m = 19;
bool t = false;
bool f = true;
std::string k = "Hello World";
std::string aliases = "Aliases";

ArgMap()                                    // (*) marks default.
  .required()                               // set args to required.
  .positional()                             //
    .arg("p", p, "doc for p")               //
    .arg("m", m, "doc for m", "undefined")  // special default info.
  .optional()                               // swap to optional args (*).
  .named()                                  // named args (*) e.g.k=v.
  .separator(ArgMap::Separator::WHITESPACE) // change separator to
    .arg("-k", k, "doc for k", "")          // whitespace ('=' is (*)).
    .arg({"-q", "--r", "--sos"}, aliases)   //
    .note("an extra note")                  // no default value info.
  .toggle()                                 // add extra doc/note.
     .arg("-t", t, "doc for t", "")         // toggle flag sets bool true.
  .toggle(false)                            // toggle flag sets bool false.
     .arg("-f", f, "doc for f", "")         //
  .helpArgs({"--myhelp"})                   // changes default help flags
  .parse(argc, argv);                       // (*) is {"-h", "--help"}.
                                            // parses and overwrites values
```

## Running the tests
Tests are written in Google Test and can be built using the provided `cmake` script.

```bash
cmake -S . -B build
cmake --build build -j
```

Note that the script can 

### NTL tests
Because ArgMap spun-off from HElib, the tests also test whever ArgMap can read in NTL's vector `Vec` type. To anable these tests set the configuration step with the following flag.

```cmake
cmake -S . -B build -DNTL_TESTS=ON
```

