// Platform compatibility header for cross-platform builds
#pragma once

#ifdef _WIN32
    #include <io.h>
    #include <direct.h>
#else
    // POSIX equivalents for Unix-like systems
    #include <unistd.h>
    #include <sys/stat.h>
    #include <dirent.h>

    // Define Windows-specific types and functions for Unix
    #define _mkdir(path) mkdir(path, 0755)

    // File finding structures (basic compatibility)
    struct _finddata_t {
        char name[260];
        unsigned attrib;
        time_t time_write;
        long size;
    };

    #define _A_SUBDIR 0x10

    // Note: _findfirst/_findnext are not directly supported on Unix
    // FileAgent methods using these will need alternative implementations
    inline long _findfirst(const char* filespec, struct _finddata_t* fileinfo) {
        // Stub implementation - return -1 to indicate not found
        return -1;
    }

    inline int _findnext(long handle, struct _finddata_t* fileinfo) {
        // Stub implementation
        return -1;
    }

    inline int _findclose(long handle) {
        // Stub implementation
        return 0;
    }
#endif
