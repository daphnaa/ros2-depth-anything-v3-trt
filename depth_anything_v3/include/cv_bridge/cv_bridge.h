#pragma once

// This repo ships include/cv_bridge/cv_bridge.h as a compatibility shim.
// Because it shadows the real header, it MUST NOT include <cv_bridge/cv_bridge.h>
// normally (it would include itself). We use <...hpp> variants or include_next.

#if __has_include(<cv_bridge/cv_bridge.hpp>)
  // Some ROS2 distros provide this
  #include <cv_bridge/cv_bridge.hpp>

#elif __has_include(<cv_bridge/cv_bridge/cv_bridge.hpp>)
  // Jazzy-style layout on some installs
  #include <cv_bridge/cv_bridge/cv_bridge.hpp>

#else
  // Humble often provides only cv_bridge/cv_bridge.h. Since this file shadows it,
  // we need include_next to pull the "next" header with the same name.
  #if defined(__GNUC__) || defined(__clang__)
    #include_next <cv_bridge/cv_bridge.h>
  #else
    #error "cv_bridge headers not found. Install ros-<distro>-cv-bridge and source setup.bash"
  #endif
#endif
