#pragma once
#include <NEPath/Curve.h>
// FileAgent is a class to read and write files.

namespace nepath
{
    class FileAgent
    {
    public:
        static path read_csv(char const *filename);                                                                                                             // read a path from a .csv file
        static void write_csv(const path &p, char const *filename);                                                                                             // write a path into a .csv file
        static void write_csv(const paths &ps, char const *filename_pre, char const *filename_post = NULL);                                                     // write paths into .csv files with the same perfix and suffix of filenames
        static std::vector<std::string> get_AllFiles(char const *path, bool folder = false, char const *filename_pre = NULL, char const *filename_post = NULL); // get all filenames in the folder
        // static void delete_AllFiles(const char *path); // delete all files in the folder
        static void mkdir(const char *path, bool clear = false); // create a directory
    private:
        static bool is_csv(char const *filename); // determine whether a file is a .csv file
    };
}