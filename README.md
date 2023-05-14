# Ament CMake Extension

DISCLAIMER: This project is EXPERIMENTAL. The package API is not stable and still work-in-progress. USE WITH CAUTION.

This package is a minimalist, opiniated and unofficial abstraction layer to ROS2 ament CMake build system. It provides a collection of functions that simplify the writing of ament cmake packages.

In CMake (and Ament CMake), building, installing and exporting targets are totally separate things so that users have total control over what the build system does. In practice however, such fine grained control is neither necessary, nor desired. For instance, if your ament package builds a shared library, you need to think about:

- fetching all the build dependencies, adding each dependency definitions, include directories, libraries and exporting required dependencies for downstream packages.
- including, installing and exporting your include directories.
- building, installing and exporting your targets.

There are a lot of things to think about and to write. It is easy to miss a few, although it is essentially the same thing over and over. The Ament CMake Extension package solves this issue with the `ament_ex` macros. With the `ament_ex` macros, to build is to install, is to export; to link a dependency is to export it; and so on.

## Why not `ament_auto`?

Although this package shares a similar purpose to the `ament_cmake_auto` package, it provides in my opinion a simpler, faster and more flexible abstraction of the base ament macros. The `ament_auto` macros are very convenient, but they come with a few drawbacks:

- The macros are very slow when there are a lot of recursive dependencies.
- Although they get rid of most of the boring cmake boilerplate, I find they do way too many other things at the same times. In most case, I just want to do something simple.
- Conversely, a few simple features are missing. For example there is no construct to simply include, install and export another include directory than "./include".
- In my opinion, they are bloated with too many not-so-useful or complex features. It is not necessarily a problem per se, but it makes the code quite difficult to read. At the end of the day, I find it difficult to understand what the macros actually do.

Don't get me wrong: these macros are great. I just find they are a bit too much... "magic", in the sense: "it works, but I don't understand why". On the other hand, `ament_cmake_extension` aims to be:

- a zero-cost abstraction: `ament_ex` macros are as fast as the base ament macros.
- minimalist: only get rid of the boring boilerplate, nothing else.
- easy to understand: straightforward API, no "magic", easy to read macros.
- foolproof: correctly build, install and export everything. No extra work is needed.

## How to use

Although `ament_auto` and `ament_ex` macros have very similar API, their behavior can be quite different. One key difference is that `ament_auto` works with old-style standard CMake variables, such as `_INCLUDE_DIRS`, `_LIBRARIES`, etc, while `ament_ex` macros work exclusively with modern CMake targets. From the user point-of-view it should not really matter, but in practice it is easier to mess up targets, as installing and exporting targets requires some extra care. If you follow the guidelines below, you should do just fine!

### package.xml

In your package.xml, add:

```xml
<buildtool_depend>ament_cmake_extension</buildtool_depend>
```

### Base CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.14) # whichever version is required

project(myRosPackage)

# import all ament_ex macros
find_package(ament_cmake_extension REQUIRED)

# automatically find_package() on all package dependencies
ament_ex_find_all_package_dependencies()

# add your libraries/executable/test here
# ament_ex_add_library(...)
# ament_ex_add_executable(...)

# automatically export targets/dependencies for downstream packages
ament_ex_package()
```

### Add a library/executable

```cmake
# simple wrapper around add_library()
# - include and install "./include"
# - include/link all package dependencies
# - install target library
ament_ex_add_library(MyLibrary
  src/fileA.cpp
  src/fileB.cpp
  #...
)

# simple wrapper around add_executable()
# - include and install "./include"
# - include/link all package dependencies
# - install target executable
ament_ex_add_executable(MyExecutable
  src/fileA.cpp
  src/fileB.cpp
  #...
)
```

### Add a non-ROS dependency

If the dependency define standard cmake variables, such as `_TARGETS`, `_LIBRARIES` or `_INCLUDE_DIRS`, then use `ament_target_dependencies()`. If the dependency defines targets, you can just use `target_link_libraries()`:

```cmake
# find and export the package downstream
ament_ex_find_package(PCL REQUIRED COMPONENTS common)
ament_ex_find_package(Eigen3)

# define your target...
# e.g. ament_ex_add_library(MyTarget ...)

# Using modern CMake target
target_link_libraries(MyTarget Eigen3::Eigen)

# Using old-style CMake variables, automatically add compile definitions, include directories and link libraries from dependency
ament_target_dependencies(MyTarget SYSTEM PCL)
```

## Advanced usage

The "How to use" section above should be enough for 99% use cases. Down there is a non-exhaustive list "tricky" situations, and how to handle them properly.

But first, a few DON'T:

- Don't use `_LIBRARIES`, `_INCLUDE_DIRS` and other cmake variables yourself.
- Don't use base ament macros yourself.
- Don't use the macros outside the root CMake file (this is limitation from `ament_cmake`)

### Create a header only package

Define an "empty" interface library target:

```cmake
# will properly include "./include", install and export dependencies
ament_ex_library(myHeaderOnlyPackage INTERFACE)
```

### Can't create target with ament_ex macros

For example CUDA libraries must use `cuda_add_library`:

```cmake
cuda_add_library(MyCudaLib
  lib/src/fileA.cu
  lib/src/fileB.cu
  #...
)
# (Optional) add all build dependencies:
ament_ex_target_add_package_dependencies(MyCudaLib BUILD_DEPS)
# install the library
ament_ex_install_targets(MyCudaLib)
```

### Need to include another directory than "./include"

If you define any target, the "./include" directory will be automatically included and installed. 99% of packages should not have to include anything else. But if you do, you have 2 options:

A. If the included directory does not need to be passed downstream, use cmake `target_include_directories` with the PRIVATE keyword:

```cmake
target_include_directories(myTarget PRIVATE lib/include)
```

B. If the include directory must be available in downstream packages, you can do:

```cmake
# Correctly include and install directory for downstream packages
ament_ex_target_include_directories(myTarget lib/include)
```

### Create a test

There are million ways to write tests, so this package does not provide any "smart" wrapper. I think the base ament macros are good enough for that purpose. For example:

```cmake
ament_ex_add_library(myLibrary
  # ...
)

if(BUILD_TESTING)
  ament_add_gtest(test_myLibrary
    # ...
  )
  # (Optional) add all <test_depend> dependencies:
  ament_ex_target_add_package_dependencies(test_myLibrary TEST_DEPS)
  # linking the target should work just fine
  target_link_library(test_myLibrary myLibrary)
endif()
```

## License

This package is heavily inspired from `ament_cmake_auto` package (<https://github.com/ament/ament_cmake/tree/rolling/ament_cmake_auto>), which is distributed under Apache License 2.0, and so is this project.
