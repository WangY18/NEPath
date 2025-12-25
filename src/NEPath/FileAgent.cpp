#include <NEPath/FileAgent.h>
#include <string>
#include <cstring>
#include <filesystem>

// Platform-specific headers
#ifdef _WIN32
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
// #define _mkdir(path) mkdir(path, 0755)
// #define _rmdir(path) rmdir(path)

// File finding structures for cross-platform compatibility
struct _finddata_t
{
    char name[260];
    unsigned attrib;
    time_t time_write;
    long size;
};
#define _A_SUBDIR 0x10

// Stub implementations for Unix-like systems
inline long _findfirst(const char *, struct _finddata_t *) { return -1; }
inline int _findnext(long, struct _finddata_t *) { return -1; }
inline int _findclose(long) { return 0; }
#endif

namespace nepath
{
    // read a path from a .csv file
    // filename is the absolute path with filename of the input file
    // The .csv file has the following form:
    // x,y,
    // data_x1,data_y1,
    // data_x2,data_y2,
    // ...
    path FileAgent::read_csv(char const *filename)
    {
        if (!is_csv(filename))
        {
            printf("%s is not csv file.", filename);
            return path();
        }
        std::ifstream inFile(filename, std::ios::in);
        if (!inFile)
        {
            printf("File %s does not exist.\n", filename);
            return path();
        }
        std::vector<double> x;
        std::vector<double> y;
        while (!inFile.eof())
        {
            bool end = false;
            double t = readcsv_double(inFile, end);
            if (!end)
            {
                x.push_back(t);
                y.push_back(readcsv_double(inFile));
            }
        }
        return path(x.data(), y.data(), x.size());
    }

    // write a path into a .csv file
    // filename is the absolute path with filename of the output file
    // The .csv file has the following form:
    // x,y,
    // data_x1,data_y1,
    // data_x2,data_y2,
    // ...
    void FileAgent::write_csv(const path &p, char const *filename)
    {
        if (!is_csv(filename))
        {
            printf("%s is not csv file.", filename);
            return;
        }
        std::ofstream outFile(filename, std::ios::out);
        outFile << "x,y\n";
        for (int j = 0; j < p.length; ++j)
        {
            outFile << p.x[j] << ',' << p.y[j] << '\n';
        }
        outFile.close();
    }

    // determine whether a file is a .csv file
    bool FileAgent::is_csv(char const *filename)
    {
        return filename[strlen(filename) - 1] == 'v' && filename[strlen(filename) - 2] == 's' && filename[strlen(filename) - 3] == 'c' && filename[strlen(filename) - 4] == '.';
    }

    // write paths into .csv files with the same perfix and suffix of filenames
    // filename is the absolute path with filename of the output file
    // The .csv files ha the following form:
    // x,y,
    // data_x1,data_y1,
    // data_x2,data_y2,
    // ...
    void FileAgent::write_csv(const paths &ps, char const *filename_pre, char const *filename_post /*=NULL*/)
    {
        if (!ps.size())
        {
            return;
        }
        bool flag_new = !filename_post;
        if (flag_new)
        {
            filename_post = new char[5];
            filename_post = ".csv";
        }
        std::string filename(filename_pre);
        int num = 0;
        int n = ps.size();
        while (n)
        {
            ++num;
            n /= 10;
        }
        for (int i = 0; i < num; ++i)
        {
            filename += "0";
        }
        filename += filename_post;
        for (int i = 0; i < ps.size(); ++i)
        {
            int j = i;
            for (int k = 0; k < num; ++k)
            {
                filename[filename.size() - strlen(filename_post) - k - 1] = '0' + (j % 10);
                j /= 10;
            }
            write_csv(ps[i], filename.data());
        }
        if (flag_new)
        {
            delete[] filename_post;
            filename_post = NULL;
        }
    }

    // get all filenames in the folder
    std::vector<std::string> FileAgent::get_AllFiles(char const *path, bool folder /*=false*/, char const *filename_pre /*=NULL*/, char const *filename_post /*=NULL*/)
    {
        std::vector<std::string> files;
        std::string append_filename = "\\";
        if (filename_pre)
        {
            append_filename += filename_pre;
        }
        append_filename += "*";
        if (filename_post)
        {
            append_filename += filename_post;
        }

        long long hFile = 0;
        struct _finddata_t fileinfo;
        std::string p;
        if ((hFile = _findfirst(p.assign(path).append(append_filename).c_str(), &fileinfo)) != -1)
        {
            do
            {
                if (fileinfo.name[0] == '.')
                {
                    continue;
                }
                if (folder && !(fileinfo.attrib & _A_SUBDIR))
                {
                    continue;
                }
                files.push_back(p.assign(path).append("\\").append(fileinfo.name));
            } while (_findnext(hFile, &fileinfo) == 0);
            _findclose(hFile);
        }
        return files;
    }

    // delete all files in the folder
    // void FileAgent::delete_AllFiles(const char *path)
    // {
    // std::filesystem::remove_all(path);
    // std::filesystem::create_directory(path);

    // std::string dir(path);
    // std::string searchPath = dir + "\\*.*";

    // intptr_t handle;
    // struct _finddata_t fileinfo;
    // handle = _findfirst(searchPath.c_str(), &fileinfo);

    // if (handle == -1)
    //     return; // empty folder

    // do
    // {
    //     std::string fileName = fileinfo.name;

    //     if (fileinfo.attrib & _A_SUBDIR)
    //     {
    //         if (fileName == "." || fileName == "..")
    //             continue;

    //         std::string subdir = dir + "\\" + fileName;
    //         delete_AllFiles(subdir.c_str());
    //         _rmdir(subdir.c_str());
    //     }
    //     else
    //     {
    //         std::string filePath = dir + "\\" + fileName;
    //         remove(filePath.c_str());
    //     }

    // } while (_findnext(handle, &fileinfo) == 0);

    // _findclose(handle);
    // }

    // create a directory
    void FileAgent::mkdir(const char *path, bool clear /*=false*/)
    {
        std::filesystem::path dir(path);

        if (clear && std::filesystem::exists(dir))
        {
            // std::cout << "Deleting directory contents: " << dir << std::endl;
            std::filesystem::remove_all(dir); // Delete directory and its contents
        }

        // Ensure parent directory exists
        if (!std::filesystem::exists(dir.parent_path()))
        {
            std::filesystem::create_directories(dir.parent_path()); // Recursively create parent directories
        }

        // Create target directory, no error if it already exists
        if (!std::filesystem::exists(dir))
        {
            // std::cout << "Creating directory: " << dir << std::endl;
            std::filesystem::create_directory(dir);
        }
    }
}