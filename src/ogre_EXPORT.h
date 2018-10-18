#ifndef __ogre_EXPORT_h_INCLUDED__
#define __ogre_EXPORT_h_INCLUDED__

#if defined(_WIN32)
#  ifdef ogre_EXPORT_SYMBOLS
#    define ogre_EXPORT __declspec( dllexport )
#  else
#    define ogre_EXPORT __declspec( dllimport )
#  endif
#  define ogre_CDECL __cdecl
#else
#  define ogre_EXPORT
#  define ogre_CDECL
#endif //  defined(_WIN32)

#endif

// eof - ogre_EXPORT.h
