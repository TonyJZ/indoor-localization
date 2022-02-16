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

//��ȡ�ļ�����ָ����׺���ļ�
//bool _fs_Lib_ GetFilesInFolder(const char *pPath, char *pExterntion, vector <string> &filenames);

//���ļ�������ȡ�ļ����ƺͺ�׺��, result_name��·��
void _fs_Lib_ GetFileName(const char *file_name, char *result_name, char *suffix_name);

//���ļ�������ȡ�ļ����ƺͺ�׺��, result_name����·��
void _fs_Lib_ GetPureFileName(const char *file_name, char *result_name, char *suffix_name);


//�����ļ���
bool _fs_Lib_ CreateMultipleDirectory(const char* szPath);


//���ַ��������д�д�ַ�ת����Сд
void _fs_Lib_ StrCaps2LowerCase(char *str);

//�����ļ����µ�ָ���ļ�
int _fs_Lib_ TraversFolderForCertainFiles(char *pPath, char *pExterntion, vector <string> &filenames);

//���������ļ���
void _fs_Lib_ TraverseDirectory(char Dir[FILENAME_MAX], vector <string> &dirnames);




#endif

