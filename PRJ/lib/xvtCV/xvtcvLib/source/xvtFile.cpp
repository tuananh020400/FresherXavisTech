#include "xvtCV/xvtFile.h"
#include <locale>
#include <algorithm>
#include <WTypesbase.h>
#include <fstream>
#include <vector>
#include <io.h>
#include <tchar.h>
#include <atlconv.h>

#include <atlstr.h>

#pragma comment( lib, "Rpcrt4.lib" )

namespace xvt {
    
std::wstring RemoveFileSpec(std::wstring  lpszDirPath)
{
    WCHAR  strPath[MAX_PATH];
    ::_tcscpy_s(strPath, MAX_PATH - 1, lpszDirPath.c_str());
    ::PathRemoveFileSpec(strPath);
    return strPath;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ::PathCombine method
// Concatenates two strings that represent properly formed paths into one path
std::wstring CombinePath(std::wstring  lpszPath1, std::wstring  lpszPath2)
{
    WCHAR  strPath[MAX_PATH];
    ::PathCombine(strPath, lpszPath1.c_str(), lpszPath2.c_str());
    return strPath;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ::PathFileExist
// Determines whether a file or folder is valid.
bool IsPathExist(std::wstring  lpszPath)
{
    return ::PathFileExists(lpszPath.c_str());
}

// ::PathAddBackslash
// Adds a backslash to the end of a string 
std::wstring  AddBackslash(std::wstring  lpszPath)
{
    std::wstring strPath = lpszPath;
    if (!strPath.empty() && strPath.back() != '\\') {
        strPath += '\\';
    }
    return strPath;
}

std::wstring AddForwardSlash(std::wstring lpszPath)
{
    std::wstring strPath = lpszPath;
    if (!strPath.empty() && strPath.back() != L'/') {
        strPath += L'/';
    }
    return strPath;
}

// Removes a backslash from the end of a string 
std::wstring  RemoveBackslash(std::wstring lpszPath)
{
    std::wstring  strPath(lpszPath);
    WCHAR szCh = strPath.at(strPath.length() - 1);
    if (szCh == '\\')
        strPath.erase(strPath.length() - 1);

    return strPath;
}

std::wstring  RemoveForwardSlash(std::wstring lpszPath)
{
    std::wstring  strPath(lpszPath);
    WCHAR szCh = strPath.at(strPath.length() - 1);
    if (szCh == '/')
        strPath.erase(strPath.length() - 1);

    return strPath;
}

bool IsRelativePath(std::wstring  lpszPath)
{
    return ::PathIsRelative(lpszPath.c_str());
}

std::wstring  GetTempFolderPath(void)
{
    std::wstring  strTempDirectory;
    WCHAR  strPath[MAX_PATH*3];
    ::GetTempPath(MAX_PATH, strPath);
    strTempDirectory = strPath;
    return AddBackslash(strTempDirectory);
}

std::wstring  GetCurrentFolderPath(void)
{
    std::wstring  strCurrentDir;
    WCHAR  strPath[MAX_PATH * 3];
    ::GetCurrentDirectory(MAX_PATH, strPath);
    strCurrentDir = strPath;
    return AddBackslash(strCurrentDir);
}

std::wstring  GetCurrentProcessModulePath(void)
{
    std::wstring strCache = {};

    if (strCache.empty())
    {
        WCHAR tchModulePath[_MAX_PATH + 1];
        WCHAR tchFullPath[_MAX_PATH + 1];

        // get path
        ::GetModuleFileName(NULL, tchModulePath, _MAX_PATH);
        ::GetFullPathName(tchModulePath, _MAX_PATH, tchFullPath, nullptr);
        strCache = tchFullPath;
    }
    return strCache;

}

std::wstring  GetCurrentProcessFolderPath(void)
{
    std::wstring  strCache = {};

    if (strCache.empty())
    {
        WCHAR tchModulePath[_MAX_PATH + 1];

        // get path
        ::GetModuleFileName(NULL, tchModulePath, _MAX_PATH);

        // remove file name
        strCache = XVT_DIR_RemoveFileSpec(tchModulePath);
    }

    return strCache;
}

void SplitPath(std::wstring  lpszPath, std::wstring* pStrDrive, std::wstring* pStrDir, std::wstring* pStrFileTitle, std::wstring* pStrExt)
{
    if (lpszPath.empty())
        return;

    WCHAR lpDrive[_MAX_DRIVE + 1];
    WCHAR lpDir[_MAX_DIR + 1];
    WCHAR lpFile[_MAX_FNAME + 1];
    WCHAR lpExt[_MAX_EXT + 1];

    ::_tsplitpath_s(lpszPath.c_str(), lpDrive, lpDir, lpFile, lpExt);

    if (pStrDrive)
    {
        pStrDrive->clear();
        *pStrDrive = lpDrive;
    }

    if (pStrDir)
    {
        pStrExt->clear();
        *pStrDir = lpDir;
    }

    if (pStrFileTitle)
    {
        pStrFileTitle->clear();
        *pStrFileTitle = lpFile;
    }

    if (pStrExt)
    {
        pStrExt->clear();
        *pStrExt = lpExt;
    }
}

void SplitPath(std::wstring  lpszPath, std::wstring* pStrDirPath, std::wstring* pStrFileName)
{
    std::wstring  strDrive, strDir, strFileTitle, strExt;

    SplitPath(lpszPath, &strDrive, &strDir, &strFileTitle, &strExt);

    if (pStrDirPath)
        *pStrDirPath = strDrive + strDir;

    if (pStrFileName)
        *pStrFileName = strFileTitle + strExt;
}

std::wstring  CreateGUID(void)
{
    std::wstring  strGuid;
    UUID uuid;
    if (UuidCreate(&uuid) == RPC_S_OK)
    {
        WCHAR szIID[129] = { 0, };
        ::StringFromGUID2(uuid, szIID, 128);
        strGuid = szIID;
    }

    if (strGuid.empty())
        strGuid = L"{TEMP}";

    return strGuid;

}

// remove first slash character
std::wstring  RemoveFirstSlash(std::wstring lpszValue)
{
    if (!lpszValue.empty())
    {
        if (lpszValue.at(0) == ('/'))
            lpszValue.erase(0, 1);
    }

    return lpszValue;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// replace slash character to dot character
std::wstring  ReplaceSlashToDot(std::wstring lpszValue)
{
    std::replace(lpszValue.begin(), lpszValue.end(), '/', '.');
    return lpszValue;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// replace dot character to slash character
std::wstring  ReplaceDotToSlash(std::wstring  lpszValue)
{
    std::replace(lpszValue.begin(), lpszValue.end(), '.', '/');
    return lpszValue;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// xml path
std::wstring  MakeValidXmlPath(std::wstring  lpszPath)
{
    std::wstring  strResult;

    std::vector<std::wstring > arrItems;
    TokenizeString(lpszPath, '/', arrItems);

    const int nCount = static_cast<int>(arrItems.size());
    for (int i = 0; i < nCount; i++)
    {
        std::wstring  strTemp = arrItems[i];
        if (!strTemp.empty())
        {
            if (isdigit(strTemp.at(0)))
            {
                strTemp = L"Idx" + strTemp;
            }
        }
        strResult += strTemp;

        if (i != (nCount - 1))
            strResult += L"/";
    }

    return strResult;
}

// tokenize string
void TokenizeString(std::wstring  lpszSource, const char& lpszTokens, std::vector<std::wstring >& arrValues)
{
    std::wstring word = L"";
    arrValues.clear();

    for (auto x : lpszSource)
    {
        if (x  == lpszTokens)
        {
            arrValues.push_back(word);
            word = L"";
        }
        else
        {
            word = word + x;
        }
    }
    //push back final word
    arrValues.push_back(word);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IsDots(const TCHAR* str) {
    if (_tcscmp(str, L".") && _tcscmp(str, L"..")) return FALSE;
    return TRUE;
}

bool DeleteDirectory(std::wstring strPath) {
    HANDLE hFind;  // file handle
    WIN32_FIND_DATA FindFileData;

    WCHAR DirPath[MAX_PATH];
    WCHAR FileName[MAX_PATH];

    _tcscpy_s(DirPath, strPath.c_str());
    _tcscat_s(DirPath, L"\\*");    // searching all files
    _tcscpy_s(FileName, strPath.c_str());
    _tcscat_s(FileName, L"\\");

    hFind = FindFirstFile(DirPath, &FindFileData); // find the first file
    if (hFind == INVALID_HANDLE_VALUE) return FALSE;
    _tcscpy_s(DirPath, FileName);

    bool bSearch = true;
    while (bSearch) { // until we finds an entry
        if (FindNextFile(hFind, &FindFileData)) {
            if (IsDots(FindFileData.cFileName)) continue;
            _tcscat_s(FileName, FindFileData.cFileName);
            if ((FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {

                // we have found a directory, recurse
                if (!DeleteDirectory(FileName)) {
                    FindClose(hFind);
                    return FALSE; // directory couldn't be deleted
                }
                RemoveDirectory(FileName); // remove the empty directory
                _tcscpy_s(FileName, DirPath);
            }
            else {
                std::wstring FlName(FileName);
                std::string strFileName(FlName.begin(), FlName.end());

                if (FindFileData.dwFileAttributes & FILE_ATTRIBUTE_READONLY)
                    _chmod(strFileName.c_str(), _S_IWRITE); // change read-only file mode
                if (!DeleteFile(FileName)) {  // delete the file
                    FindClose(hFind);
                    return FALSE;
                }
                _tcscpy_s(FileName, DirPath);
            }
        }
        else {
            if (GetLastError() == ERROR_NO_MORE_FILES) // no more files there
                bSearch = false;
            else {
                // some error occured, close the handle and return FALSE
                FindClose(hFind);
                return FALSE;
            }

        }

    }
    FindClose(hFind);  // closing file handle

    return RemoveDirectory(strPath.c_str()); // remove the empty directory

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IsFileExist(const std::wstring& name)
{
    std::ifstream f(name.c_str());
    return f.good();
}

bool MakeDirectory(std::wstring  lpszDirPath)
{
    if (IsPathExist(lpszDirPath) == true)
        return true;

    WCHAR lpszTempPath[MAX_PATH] = { 0, };
    memset(lpszTempPath, 0, sizeof(lpszTempPath));
       
    _tcscpy_s(lpszTempPath, _countof(lpszTempPath), lpszDirPath.c_str());

    // Create parent directories
    for (TCHAR* p = _tcschr(lpszTempPath, _T('\\')); p; p = _tcschr(p + 1, _T('\\')))
    {
        *p PURE;
        ::CreateDirectory(lpszTempPath, NULL);  // may or may not already exist
        *p = _T('\\');
    }

    ::CreateDirectory(lpszTempPath, NULL);

    return IsPathExist(lpszTempPath);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MoveDirectory(std::wstring  lpszDirPath, std::wstring  lpszNewDirPath)
{
    if (!IsPathExist(lpszDirPath))
        return false;

    if (IsPathExist(lpszNewDirPath))
        return false;

    std::wstring lpDirPath = std::wstring(lpszDirPath.begin(), lpszDirPath.end());
    std::wstring lpNewDirPath = std::wstring(lpszNewDirPath.begin(), lpszNewDirPath.end());

    if (::MoveFileEx(lpDirPath.c_str(), lpNewDirPath.c_str(), MOVEFILE_REPLACE_EXISTING) == 0)
    {
        DWORD dwError = GetLastError();
        std::string  strError = "MoveFile Error : code =" + dwError;
        return false;
    }
    return IsPathExist(lpszNewDirPath);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int CopyDirectory(std::wstring  lpszSrcDirPath, std::wstring  lpszDstDirPath)
{
    std::wstring   strSrcDirPath = AddBackslash(lpszSrcDirPath);
    std::wstring   strDstDirPath = AddBackslash(lpszDstDirPath);

    WCHAR FileName[MAX_PATH * 5];
    memset(FileName, 0, sizeof(FileName));
    _tcscpy_s(FileName, _countof(FileName), strSrcDirPath.c_str());

    MakeDirectory(strDstDirPath);

    int nCount PURE;
    bool bResult = true;

    HANDLE hFind;  // file handle
    WIN32_FIND_DATA FindFileData;

    hFind = FindFirstFile(strSrcDirPath.c_str(), &FindFileData); // find the first file
    if (hFind == INVALID_HANDLE_VALUE)
    {
        return FALSE;
    }
    else
    {
        bResult = false;
    }

    while (bResult)
    {
        if (FindNextFile(hFind, &FindFileData))
        {
            if (IsDots(FindFileData.cFileName)) continue;
            _tcscat_s(FileName, FindFileData.cFileName);


            const std::wstring   strFilePath = FileName;
            const std::wstring   strFileName = FindFileData.cFileName;

            const std::wstring   strTargetFilePath = strDstDirPath + strFileName;


            if (FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
            {
                nCount += CopyDirectory(strFilePath, strTargetFilePath);
            }
            else
            {
                bResult = ::CopyFile(strFilePath.c_str(), strTargetFilePath.c_str(), FALSE) ? true : false;

                if (bResult)
                    nCount++;
            }
        }
        else
        {
            if (GetLastError() == ERROR_NO_MORE_FILES) // no more files there
                bResult = false;
            else {
                // some error occured, close the handle and return FALSE
                FindClose(hFind);
                return FALSE;
            }
        }
    }
    FindClose(hFind);  // closing file handle

    return nCount;
}

bool DeleteFolder(std::wstring lpszDirPath)
{
    SHFILEOPSTRUCT fos = { 0 };

    fos.wFunc = FO_DELETE;
    fos.pFrom = lpszDirPath.c_str();

    ::SHFileOperation(&fos);


    return true;
}

bool CopyFolder(std::wstring  lpszSrcDirPath, std::wstring  lpszDstDirPath)
{
    SHFILEOPSTRUCT fos = { 0 };

    fos.wFunc = FO_COPY;
    fos.pFrom = lpszSrcDirPath.c_str();
    fos.pTo = lpszDstDirPath.c_str();

    ::SHFileOperation(&fos);


    return true;
}

bool MoveFolder(std::wstring  lpszDirPath, std::wstring  lpszNewDirPath)
{
    SHFILEOPSTRUCT fos = { 0 };

    fos.wFunc = FO_MOVE;
    fos.pFrom = lpszDirPath.c_str();
    fos.pTo = lpszNewDirPath.c_str();

    ::SHFileOperation(&fos);


    return true;
}


auto GetFiles(std::wstring folder, std::wstring searchPattern) -> std::vector<std::wstring>
{
    auto fileList = std::vector<std::wstring>();
    if (!folder.empty())
    {
        if (searchPattern.empty())
        {
            searchPattern = L"*";
        }
        searchPattern = folder + L"/" + searchPattern;

        WIN32_FIND_DATAW fileData;
        HANDLE hFind = FindFirstFileW(searchPattern.c_str(), &fileData);
        bool isOK = hFind != INVALID_HANDLE_VALUE;
        while (isOK)
        {
            if (!(fileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
            {
                fileList.push_back(folder + L"/" + fileData.cFileName);
            }
            isOK = FindNextFileW(hFind, &fileData);
        }

        FindClose(hFind);
    }
    return fileList;
}

auto GetFolders(std::wstring const& folder) -> std::vector<std::wstring>
{
    std::vector<std::wstring> folderlist;
    WIN32_FIND_DATA findFileData;
    HANDLE hFind = FindFirstFile((folder + L"\\*").c_str(), &findFileData);

    if (hFind != INVALID_HANDLE_VALUE)
    {
        do
        {
            if (findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
            {
                if (std::wcscmp(findFileData.cFileName, L".") != 0 && std::wcscmp(findFileData.cFileName, L"..") != 0)
                {
                    folderlist.push_back(folder + L"/" + findFileData.cFileName);
                }
            }
        } while (FindNextFile(hFind, &findFileData) != 0);
    }

    FindClose(hFind);

    return folderlist;
}

auto GetFolderName(std::wstring const& path) -> std::wstring
{
    auto p = path.find_last_of(L"/\\");
    auto p_end = path.length();
    if (p == path.length() - 1)
    {
        p = path.find_last_of(L"/\\", p);
    }
    return p != std::wstring::npos ? path.substr(p + 1) : path;
}

auto DeleteFiles(std::wstring folder, std::wstring searchPattern) -> std::vector<std::wstring>
{
    std::vector<std::wstring> fileList = xvt::GetFiles(folder, searchPattern);
    std::vector<std::wstring> deletedList;
    deletedList.reserve(fileList.size());
    for (auto& f : fileList)
    {
        if (DeleteFile(f.c_str()))
        {
            deletedList.push_back(std::move(f));
        }

    }

    return deletedList;
}
}