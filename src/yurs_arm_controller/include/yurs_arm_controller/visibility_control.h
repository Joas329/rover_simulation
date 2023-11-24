#ifndef YURS_ARM_CONTROLLER__VISIBILITY_CONTROL_H_
#define YURS_ARM_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define YURS_ARM_CONTROLLER_EXPORT __attribute__((dllexport))
#define YURS_ARM_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define YURS_ARM_CONTROLLER_EXPORT __declspec(dllexport)
#define YURS_ARM_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef YURS_ARM_CONTROLLER_BUILDING_DLL
#define YURS_ARM_CONTROLLER_PUBLIC YURS_ARM_CONTROLLER_EXPORT
#else
#define YURS_ARM_CONTROLLER_PUBLIC YURS_ARM_CONTROLLER_IMPORT
#endif
#define YURS_ARM_CONTROLLER_PUBLIC_TYPE YURS_ARM_CONTROLLER_PUBLIC
#define YURS_ARM_CONTROLLER_LOCAL
#else
#define YURS_ARM_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define YURS_ARM_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define YURS_ARM_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define YURS_ARM_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define YURS_ARM_CONTROLLER_PUBLIC
#define YURS_ARM_CONTROLLER_LOCAL
#endif
#define YURS_ARM_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // YURS_ARM_CONTROLLER__VISIBILITY_CONTROL_H_