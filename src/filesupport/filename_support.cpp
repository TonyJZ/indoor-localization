#include "filesupport/filename_support.h"
#include <string.h>
//#include <windows.h>
#include <assert.h>
#include <vector>

#ifdef _WIN32
#include "io.h"
#else

#endif

#define _CRT_SECURE_NO_DEPRECATE  

// #include <cstdio>  
// #include <vector>  
// #include <iostream>  
// #include <fstream>  
// #include <cstring>  
// #include <cstdlib>  
// #include <cmath>  
// #include <algorithm>  



//////////////////////////////////////////////////////////////////////////
//函数名: GetFileName
//功  能: 从文件名中提取文件名称和后缀名
//示  例: file_name:   picture.jpg
//        result_name: picture
//		  suffix_name: .jpg
//////////////////////////////////////////////////////////////////////////
void GetFileName(const char *file_name, char *result_name, char *suffix_name)
{
	int i, n;
	n = strlen(file_name);
	for (i = n - 1; i >= 0; i--)
	{
		if (file_name[i] == '.')
			break;
	}
	strncpy(result_name, file_name, i);
	result_name[i] = '\0';

	strncpy(suffix_name, file_name + i, n - i);
	suffix_name[n - i] = '\0';
}

//this is a windows function
//#include "direct.h" //contain mkdir function
#ifdef _WIN32 
#include <direct.h> 
bool CreateMultipleDirectory(const char* szPath)
{
	// 	char *strDirPath=”D:/test/test1/test2/test.txt”;
	// 	if (strlen(strDirPath)>MAX_PATH)
	// 	{
	// 		return;
	// 	}
	int ipathLength = strlen(szPath);
	int ileaveLength = 0;
	int iCreatedLength = 0;
	char szPathTemp[FILENAME_MAX] = { 0 };
	for (int i = 0; (NULL != strchr(szPath + iCreatedLength, '\\')); i++)
	{
		ileaveLength = strlen(strchr(szPath + iCreatedLength, '\\')) - 1;
		iCreatedLength = ipathLength - ileaveLength;
		strncpy(szPathTemp, szPath, iCreatedLength);

		_mkdir(szPathTemp);
	}
	if (iCreatedLength < ipathLength)
	{
		_mkdir(szPathTemp);
	}

	return true;
}
#else
#include <sys/stat.h> 
#include<sys/types.h> 
#include <stdio.h>  
#include <unistd.h>  
bool CreateMultipleDirectory(const char* szPath)
{
	int i, len;
	char str[512];
	strncpy(str, szPath, 512);
	len = strlen(str);
	//printf("%s, %d\n", str, len);
	for (i = 1; i < len; i++)
	{
		if (str[i] == '/')
		{
			str[i] = '\0';
			//printf("%s\n", str);
			if (access(str, 0) != 0)
			{
				if (mkdir(str, 0777) == -1)
				{
					perror("mkdir   error");
					printf("%s\n", str);
					return   false;
				}
			}
			str[i] = '/';
		}
	}
	if (len > 0 && access(str, 0) != 0)
	{
		if (mkdir(str, 0777) == -1)
		{
			perror("mkdir   error");
			printf("%s\n", str);
			return   false;
		}
	}
	return true;
}
#endif



void GetPureFileName(const char *file_name, char *result_name, char *suffix_name)
{
	int i, j, n;
	n = strlen(file_name);
	for (i = n - 1; i >= 0; i--)
	{
		if (file_name[i] == '.')
			break;
	}
	for (j = i; j >= 0; j--)
	{
		if (file_name[j] == '/' || file_name[j] == '\\')
			break;
	}
	strncpy(result_name, file_name + j + 1, i - j - 1);
	result_name[i - j - 1] = '\0';



	strncpy(suffix_name, file_name + i, n - i);
	suffix_name[n - i] = '\0';
}

void StrCaps2LowerCase(char *str) 
{
	int i, n = strlen(str);
	int offset = 'a' - 'A';
	for (i = 0; i < n; i++) {
		if (str[i] >= 'A' && str[i] <= 'Z')
			str[i] += offset;
	}
}

#ifdef _WIN32
int TraversFolderForCertainFiles(char *pPath, char *pExterntion, vector <string> &filenames) 
{
	filenames.clear();

	int i, n = strlen(pPath);

	char pFixedpath[1024];
	bool needfix = false;

	strcpy(pFixedpath, pPath);

	i = n - 1;
	while (pPath[i] == '\0')
		i--;
	if (pPath[i] != '\\' || pPath[i] != '/')
		needfix = true;

	//strcat(pFixedpath, pPath);
	if (needfix)
	{
		pFixedpath[i+1] = '/';
		pFixedpath[i + 2] = '\0';
	}
		


	char path_ext_name[256];
	//先拼接得到带扩展名的路径名path_ext_name字符串
	strcpy(path_ext_name, pFixedpath);
	strcat(path_ext_name, "*.*");

	StrCaps2LowerCase(pExterntion);
	int nCount = 0;
	char fname[512], ext_name[512];

	_finddata_t file;
	size_t handle;
	if ((handle = _findfirst(path_ext_name, &file)) == -1l) 
	{
		printf("Can't find any file!\n");
		return 0;
	}
	else {
		do {
			if (strstr(file.name, ".")) 
			{
				GetFileName(file.name, fname, ext_name);
				StrCaps2LowerCase(ext_name);
				if (strcmp(ext_name, ".") != 0 && strcmp(ext_name, "..") != 0 && strstr(pExterntion, ext_name)) 
				{
					string afile;
					afile = pFixedpath;
					afile += file.name;
					filenames.push_back(afile);
					nCount++;
				}
			}
		} while (_findnext(handle, &file) == 0);
		//		printf("%s",file.name);	
	}
	assert(nCount == filenames.size());

	return filenames.size();
}
#else
#include <stdio.h>  
#include <string.h> 
#include <stdlib.h>  
#include <dirent.h>  
#include <sys/stat.h>  
#include <unistd.h>  
#include <sys/types.h> 
using namespace std;
int TraversFolderForCertainFiles(char *pPath, char *pExterntion, vector <string> &filenames)
{
	DIR *pDir;
	struct dirent *ent;
	int i = 0;
	char childpath[512];

	pDir = opendir(pPath);
	memset(childpath, 0, sizeof(childpath));

//	printf("%s\n", pPath);
	filenames.clear();
	while ((ent = readdir(pDir)) != NULL)
	{

		if (ent->d_type & DT_DIR)
		{
			continue;
// 			if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0)
// 				continue;
// 
// 			sprintf(childpath, "%s/%s", path, ent->d_name);
// 			printf("path:%s/n", childpath);
// 
// 			listDir(childpath);

		}
		else
		{
			char fname[512], ext_name[512];
			GetFileName(ent->d_name, fname, ext_name);
			StrCaps2LowerCase(ext_name);
			if (strcmp(ext_name, ".") != 0 && strcmp(ext_name, "..") != 0 && strstr(pExterntion, ext_name))
			{
				//printf("%s\n", ent->d_name);
				std::string fname = pPath;
				fname += '/';
				fname += ent->d_name;
				//printf("%s\n", fname.c_str());
				filenames.push_back(fname);
			}
			//cout << ent->d_name << endl;
		}
	}

//	printf("find file: %d\n", filenames.size());
	return filenames.size();
}
#endif

#ifdef _WIN32
#include <iostream>  
#include "windows.h"  
#include <string.h>  
#include <Strsafe.h>  

void TraverseDirectory_win(char Dir[FILENAME_MAX], vector <string> &dirnames)
{
	using namespace std;

	WIN32_FIND_DATA FindFileData;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	char DirSpec[FILENAME_MAX];                  //定义要遍历的文件夹的目录  
	DWORD dwError;

	strncpy(DirSpec, Dir, FILENAME_MAX);
	strcat(DirSpec, "\\*");
	// 	StringCchCopy(DirSpec, FILENAME_MAX, Dir);
	// 	StringCchCat(DirSpec, FILENAME_MAX, TEXT("\\*"));   //定义要遍历的文件夹的完整路径\*  

	hFind = FindFirstFile(DirSpec, &FindFileData);          //找到文件夹中的第一个文件  

	if (hFind == INVALID_HANDLE_VALUE)                               //如果hFind句柄创建失败，输出错误信息  
	{
		FindClose(hFind);
		return;
	}
	else
	{
		while (FindNextFile(hFind, &FindFileData) != 0)                            //当文件或者文件夹存在时  
		{
			if ((FindFileData.dwFileAttributes&FILE_ATTRIBUTE_DIRECTORY) != 0 && strcmp(FindFileData.cFileName, ".") == 0 || strcmp(FindFileData.cFileName, "..") == 0)        //判断是文件夹&&表示为"."||表示为"."  
			{
				continue;
			}
			if ((FindFileData.dwFileAttributes&FILE_ATTRIBUTE_DIRECTORY) != 0)      //判断如果是文件夹  
			{
				char DirAdd[FILENAME_MAX];
				strcpy(DirAdd, Dir);
				strcat(DirAdd, "\\");
				strcat(DirAdd, FindFileData.cFileName);       //拼接得到此文件夹的完整路径
				strcat(DirAdd, "\\");
				//TraverseDirectory(DirAdd);                                  //实现递归调用  

				string  ifv = DirAdd;

				// 				size_t len = strlen(DirAdd) + 1;
				// 				size_t converted = 0;
				// 				wcstombs_s(&converted, ifv.name, len, DirAdd, _TRUNCATE);

				dirnames.push_back(ifv);
			}
			if ((FindFileData.dwFileAttributes&FILE_ATTRIBUTE_DIRECTORY) == 0)    //如果不是文件夹  
			{
				//				wcout<<Dir<<"\\"<<FindFileData.cFileName<<endl;            //输出完整路径  
			}
		}
		FindClose(hFind);
	}
}
#else
#include <stdio.h>  
#include <dirent.h>    
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
void TraverseDirectory_linux(char Dir[FILENAME_MAX], vector <string> &dirnames)
{
	DIR *d;
	struct dirent *file;
	struct stat buf;
	if (!(d = opendir(Dir)))
	{
		printf("error opendir %s!!!\n", Dir);
	//	return -1;
	}
	chdir(Dir); //一定要打开文件夹，不打开会出现错误
	while ((file = readdir(d)) != NULL)
	{
		lstat(file->d_name, &buf);
		if (!(S_IFDIR&buf.st_mode))
		{
// 			printf("%*s%s\n", i, "", file->d_name);//printf特殊用法  格式输出
// 			printf("\t\tfile size=%d\n", buf.st_size);
// 			printf("\t\tfile last modify time=%s\n", asctime(gmtime(&buf.st_mtime)));//先转化成格林威治时间，然后返回tm结构，接着用asctime转化成标准时间（这里 不知到有没有更好的方法）
 		}
		else
		{
			if (strcmp(file->d_name, ".") == 0 || strcmp(file->d_name, "..") == 0)
				continue;
			dirnames.push_back(file->d_name);
// 			printf("%*s%s(dir)\n", i, "", file->d_name);
// 			printf("\t\tfile last modify time=%s\n", asctime(gmtime(&buf.st_mtime)));
// 			trave_dir(file->d_name, i + 2);
		}
	}
	chdir("..");
	closedir(d);
//	return 0;
}
#endif

void TraverseDirectory(char Dir[FILENAME_MAX], vector <string> &dirnames)
{
#ifdef _WIN32
	return TraverseDirectory_win(Dir, dirnames);
#else
	return TraverseDirectory_linux(Dir, dirnames);
#endif // _WIN32

}