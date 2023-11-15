#ifndef YURS_ARM_CONTROLLER__VISIBILITY_CONTROL_H_
#define YURS_ARM_CONTROLLER__VISIBILITY_CONTROL_H_

#if defined(__WIN32)
  #if defined(YURS_ARM_CONTROLLER_BUILDING_DLL) || defined(YURS_ARM_CONTROLLER_EXPORTS)
    #define YURS_ARM_CONTROLLER_PUBLIC __declspec(dllexport)
    #define YURS_ARM_CONTROLLER_LOCAL
  #else
    #define YURS_ARM_CONTROLLER_PUBLIC __declspec(dllimport)
    #define YURS_ARM_CONTROLLER_LOCAL
  #endif
#elif defined(__linux__)
  #define YURS_ARM_CONTROLLER_PUBLIC __attribute__((visibility("default")))
  #define YURS_ARM_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define YURS_ARM_CONTROLLER_PUBLIC __attribute__((visibility("default")))
  #define YURS_ARM_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // YURS_ARM_CONTROLLER__VISIBILITY_CONTROL_H_
