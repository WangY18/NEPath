#pragma once
#include "Curve.h"

class FileAgent {
public:
	static path read_csv(char const* filename);
	static void write_csv(const path &p, char const* filename);
	static void write_csv(const paths& ps, char const* filename_pre, char const* filename_post = NULL);
	static vector<string> get_AllFiles(char const* path, bool folder = false, char const* filename_pre = NULL, char const* filename_post = NULL);
	static void delete_AllFiles(const char* path);
	static void mkdir(const char* path, bool clear = false);
private:
	static bool is_csv(char const* filename);
};