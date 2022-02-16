#ifndef _FILENAME_SUPPORT_FUNCTIONS_H_CREATE_BY_TONY_2017_05_30_
#define _FILENAME_SUPPORT_FUNCTIONS_H_CREATE_BY_TONY_2017_05_30_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
//#include <Windows.h>
#include <assert.h>
#include <vector>
#include <iostream>

#ifdef _WIN32
#ifdef FILESUPPORT_EXPORTS
#define  _fs_Lib_  __declspec(dllexport)
#else
#define  _fs_Lib_  __declspec(dllimport)
#endif

#ifndef FILESUPPORT_EXPORTS
#ifdef _DEBUG
#pragma comment(lib,"filesupportD.lib")
#else
#pragma comment(lib,"filesupport.lib")
#endif
#endif
#else
#  define _fs_Lib_
#endif


using namespace std;

//提取文件夹中指定后缀的文件
//bool _fs_Lib_ GetFilesInFolder(const char *pPath, char *pExterntion, vector <string> &filenames);

//从文件名中提取文件名称和后缀名, result_name带路径
void _fs_Lib_ GetFileName(const char *file_name, char *result_name, char *suffix_name);

//从文件名中提取文件名称和后缀名, result_name不带路径
void _fs_Lib_ GetPureFileName(const char *file_name, char *result_name, char *suffix_name);


//创建文件夹
bool _fs_Lib_ CreateMultipleDirectory(const char* szPath);


//将字符串中所有大写字符转换成小写
void _fs_Lib_ StrCaps2LowerCase(char *str);

//遍历文件夹下的指定文件
int _fs_Lib_ TraversFolderForCertainFiles(char *pPath, char *pExterntion, vector <string> &filenames);

//遍历所有文件夹
void _fs_Lib_ TraverseDirectory(char Dir[FILENAME_MAX], vector <string> &dirnames);




#endif

