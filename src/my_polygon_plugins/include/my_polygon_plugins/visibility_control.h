#ifndef MY_POLYGON_PLUGINS__VISIBILITY_CONTROL_H_
#define MY_POLYGON_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MY_POLYGON_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define MY_POLYGON_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define MY_POLYGON_PLUGINS_EXPORT __declspec(dllexport)
    #define MY_POLYGON_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MY_POLYGON_PLUGINS_BUILDING_LIBRARY
    #define MY_POLYGON_PLUGINS_PUBLIC MY_POLYGON_PLUGINS_EXPORT
  #else
    #define MY_POLYGON_PLUGINS_PUBLIC MY_POLYGON_PLUGINS_IMPORT
  #endif
  #define MY_POLYGON_PLUGINS_PUBLIC_TYPE MY_POLYGON_PLUGINS_PUBLIC
  #define MY_POLYGON_PLUGINS_LOCAL
#else
  #define MY_POLYGON_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define MY_POLYGON_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define MY_POLYGON_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define MY_POLYGON_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MY_POLYGON_PLUGINS_PUBLIC
    #define MY_POLYGON_PLUGINS_LOCAL
  #endif
  #define MY_POLYGON_PLUGINS_PUBLIC_TYPE
#endif

#endif  // MY_POLYGON_PLUGINS__VISIBILITY_CONTROL_H_
