// Copyright (c) 2023, York University Robotics Society
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef YURS_ARM_CONTROLLER__VISIBILITY_CONTROL_H_
#define YURS_ARM_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define YURS_ARM_CONTROLLER__VISIBILITY_EXPORT __attribute__((dllexport))
#define YURS_ARM_CONTROLLER__VISIBILITY_IMPORT __attribute__((dllimport))
#else
#define YURS_ARM_CONTROLLER__VISIBILITY_EXPORT __declspec(dllexport)
#define YURS_ARM_CONTROLLER__VISIBILITY_IMPORT __declspec(dllimport)
#endif
#ifdef YURS_ARM_CONTROLLER__VISIBILITY_BUILDING_DLL
#define YURS_ARM_CONTROLLER__VISIBILITY_PUBLIC YURS_ARM_CONTROLLER__VISIBILITY_EXPORT
#else
#define YURS_ARM_CONTROLLER__VISIBILITY_PUBLIC YURS_ARM_CONTROLLER__VISIBILITY_IMPORT
#endif
#define YURS_ARM_CONTROLLER__VISIBILITY_PUBLIC_TYPE YURS_ARM_CONTROLLER__VISIBILITY_PUBLIC
#define YURS_ARM_CONTROLLER__VISIBILITY_LOCAL
#else
#define YURS_ARM_CONTROLLER__VISIBILITY_EXPORT __attribute__((visibility("default")))
#define YURS_ARM_CONTROLLER__VISIBILITY_IMPORT
#if __GNUC__ >= 4
#define YURS_ARM_CONTROLLER__VISIBILITY_PUBLIC __attribute__((visibility("default")))
#define YURS_ARM_CONTROLLER__VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#else
#define YURS_ARM_CONTROLLER__VISIBILITY_PUBLIC
#define YURS_ARM_CONTROLLER__VISIBILITY_LOCAL
#endif
#define YURS_ARM_CONTROLLER__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // YURS_ARM_CONTROLLER__VISIBILITY_CONTROL_H_
