/**
 * @example{lineno} test/xvtFileTest.cpp
 * This example demonstrates how to use xvtFile.h function.
 */

#include "pch.h"
#include "xvtCV/xvtFile.h"
#include "stringTestSet.h"
#include <atlstr.h>

template<class T>
const char* ConvertToConstChar(T str);

template <>
const char* ConvertToConstChar<CString>(CString str)
{
    return CT2A(str);
}

template <>
const char* ConvertToConstChar<std::string>(std::string str)
{
    return str.c_str();
}

template <>
const char* ConvertToConstChar<const char*>(const char* str)
{
    return str;
}

// Template specialization for wchar_t const*
template <>
const char* ConvertToConstChar<wchar_t const*>(wchar_t const* str)
{
    return CW2A(str);
}

template<class T>
void testRemoveGetExtension(T name, T ext)
{
    auto nameR = xvt::RemoveExtension(name + ext);
    EXPECT_EQ(nameR, name);

    auto extR = xvt::GetFileExtension(name + ext);
    EXPECT_EQ(extR, ext);
}

// Test xvt::GetFileName with std::string
TEST(GetFileNameTest, stdString)
{
    std::vector<std::string> filePath1 = {
         "D:\\test\\1\\"
        ,"path/to/folder/"
        ,"file/"
        ,".hidden/"
        ,"../folder/"
        ,"..\\folder\\"
    };

    std::vector<std::string> fileName1 = {
          "file"
        , ".file"
        , "file.txt"
        , ""
        , "."
        , ".."
    };

    for (auto const& dir : filePath1)
    {
        for (auto const& name : fileName1)
            EXPECT_EQ(xvt::GetFileName(dir + name), name);
    }

}

TEST(RemoveGetExtensionTest, stdString)
{
    std::vector<std::string> extVec = {
         ".txt"
       , "."
       , ""
    };

    std::vector < std::string> namevec = {
          "test"
        , "..\\folder\\"
    };

    for(auto const& name :namevec)
        for(auto const& ext:extVec)
            testRemoveGetExtension(name, ext);
}

TEST(RemoveGetExtensionTest, stdWString)
{
    std::wstring name = L"준비한";
    std::vector<std::wstring> extVec = {
         L".txt"
       , L"."
       , L""
    };

    std::vector < std::wstring> namevec = {
          L"준비한"
        , L"..\\folder\\"
    };

    for (auto const& name : namevec)
        for (auto const& ext : extVec)
            testRemoveGetExtension(name, ext);
}

// Test xvt::GetFileName with std::wstring
TEST(GetFileNameTest, stdWString)
{
    std::vector<std::wstring> filePath1 = {
         L"D:\\test\\1\\"
        ,L"path/to/folder/"
        ,L"file/"
        ,L".hidden/"
        ,L"../folder/"
        ,L"..\\folder\\"
    };

    std::vector<std::wstring> fileName1 = {
          L"file"
        , L".file"
        , L"file.txt"
        , L""
        , L"."
        , L".."
    };

    for (auto const& dir : filePath1)
    {
        for (auto const& name : fileName1)
            EXPECT_EQ(xvt::GetFileName(dir + name), name);
    }
}

// Test xvt::GetFileNameWithoutExtension
TEST(GetFileNameWithoutExtensionTest, stdString)
{
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(std::string("D:\\Path\\To\\Folder\\" + STRING_LOWERCASE + ".txt")), STRING_LOWERCASE);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(std::string("D:\\Path\\To\\Folder\\")), STRING_EMPTY);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(std::string("D:\\Path\\To\\Folder\\" + STRING_LOWERCASE)), STRING_LOWERCASE);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(std::string("D:\\Path\\To\\Folder\\" + STRING_LOWERCASE + ".hidden")), STRING_LOWERCASE);
}

// Test xvt::GetParentDirectory
TEST(GetParentDirectoryTest, stdString)
{
    std::string filePath1 = "D:\\test\\1\\file.txt";
    std::string filePath2 = "C:\\path\\to\\folder\\";
    std::string filePath3 = "folder\\";
    std::string filePath4 = "..\\folder\\";

    std::string parentDir1 = "D:\\test\\1";
    std::string parentDir2 = "C:\\path\\to\\folder";
    std::string parentDir3 = "folder";
    std::string parentDir4 = "..\\folder";

    EXPECT_EQ(xvt::GetParentDirectory(filePath1), parentDir1);
    EXPECT_EQ(xvt::GetParentDirectory(filePath2), parentDir2);
    EXPECT_EQ(xvt::GetParentDirectory(filePath3), parentDir3);
    EXPECT_EQ(xvt::GetParentDirectory(filePath4), parentDir4);
}

// Test xvt::GetFileExtension
TEST(GetFileExtensionTest, stdString)
{
    std::string filePath1 = "D:\\test\\1\\file.txt";
    std::string filePath2 = "C:\\path\\to\\folder\\";
    std::string filePath3 = "\\file";
    std::string filePath4 = "\\.hidden";

    std::string fileExt1 = ".txt";
    std::string fileExt4 = ".hidden";

    EXPECT_EQ(xvt::GetFileExtension(filePath1), fileExt1);
    EXPECT_EQ(xvt::GetFileExtension(filePath2), STRING_EMPTY);
    EXPECT_EQ(xvt::GetFileExtension(filePath2 + STRING_LOWERCASE), STRING_EMPTY);
    EXPECT_EQ(xvt::GetFileExtension(filePath4), fileExt4);
}

// Test xvt::GetFileNameWithoutExtension with std::wstring
TEST(GetFileNameWithoutExtensionTest, stdWString)
{
    std::wstring filePath1 = L"D:\\test\\1\\file.txt";
    std::wstring filePath2 = L"C:\\path\\to\\folder\\";
    std::wstring filePath3 = L"\\file";
    std::wstring filePath4 = L"\\.hidden";

    std::wstring fileName1 = L"file";
    std::wstring fileName3 = L"file";

    EXPECT_EQ(xvt::GetFileNameWithoutExtension(filePath1), fileName1);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(filePath2), WSTRING_EMPTY);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(filePath3), fileName3);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(filePath4), WSTRING_EMPTY);
}

// Test xvt::GetParentDirectory with std::wstring
TEST(GetParentDirectoryTest, stdWString)
{
    std::wstring filePath1 = L"D:\\test\\1\\file.txt";
    std::wstring filePath2 = L"C:\\path\\to\\folder\\";
    std::wstring filePath3 = L"\\file";
    std::wstring filePath4 = L"\\.hidden";

    std::wstring parentDir1 = L"D:\\test\\1";
    std::wstring parentDir2 = L"C:\\path\\to\\folder";

    EXPECT_EQ(xvt::GetParentDirectory(filePath1), parentDir1);
    EXPECT_EQ(xvt::GetParentDirectory(filePath2), parentDir2);
    //EXPECT_EQ(xvt::GetParentDirectory(W"..\\folder\\" + OTHERS_LANG), W"..\\folder\\");
    //EXPECT_EQ(xvt::GetParentDirectory(W"..\\folder\\" + W".hidden"), WSTRING_EMPTY);
}

// Test xvt::GetFileExtension with std::wstring
TEST(GetFileExtensionTest, stdWString)
{
    EXPECT_EQ(xvt::GetFileExtension(std::wstring(L"D:\\path\\to\\folder\\준비한.txt")), L".txt");
    EXPECT_EQ(xvt::GetFileExtension(std::wstring(L"D:\\Path\\To\\Folder\\")), WSTRING_EMPTY);
    EXPECT_EQ(xvt::GetFileExtension(L"D:\\Path\\To\\Folder\\" + WSTRING_LOWERCASE), WSTRING_EMPTY);
    EXPECT_EQ(xvt::GetFileExtension(std::wstring(L"D:\\Path\\To\\Folder\\.hidden")), L".hidden");
}

// Test xvt::GetFileNameWithoutExtension with '/' separator
TEST(GetFileNameWithoutExtensionTest, stdStringWithSlash)
{
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(std::string("D:\\Path\\To\\Folder\\abcdefghijklmnopqrstuvwxyz.txt")), STRING_LOWERCASE);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(std::string("D:\\Path\\To\\Folder\\")), "");
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(std::string("D:\\Path\\To\\Folder\\abcdefghijklmnopqrstuvwxyz")), STRING_LOWERCASE);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(std::string("D:\\Path\\To\\Folder\\") + STRING_LOWERCASE), STRING_LOWERCASE);
}

// Test xvt::GetParentDirectory with '/' separator
TEST(GetParentDirectoryTest, stdStringWithSlash)
{
    std::string filePath1 = "D:/test/1/file.txt";
    std::string filePath2 = "path/to/folder/";
    std::string filePath3 = "folder/";
    std::string filePath4 = "../folder/";

    std::string parentDir1 = "D:/test/1";
    std::string parentDir2 = "path/to/folder";
    std::string parentDir3 = "folder";
    std::string parentDir4 = "../folder";

    EXPECT_EQ(xvt::GetParentDirectory(filePath1), parentDir1);
    EXPECT_EQ(xvt::GetParentDirectory(filePath2), parentDir2);
    EXPECT_EQ(xvt::GetParentDirectory(filePath3), parentDir3);
    EXPECT_EQ(xvt::GetParentDirectory(filePath4), parentDir4);
}

// Test xvt::GetFileExtension with '/' separator
TEST(GetFileExtensionTest, stdStringWithSlash)
{
    std::string filePath1 = "D:/test/1/file.txt";
    std::string filePath2 = "path/to/folder/";
    std::string filePath3 = "/file";
    std::string filePath4 = "./folder/";

    std::string fileExt1 = ".txt";
    std::string fileExt2 = "";
    std::string fileExt3 = "";
    std::string fileExt4 = "";

    EXPECT_EQ(xvt::GetFileExtension(filePath1), fileExt1);
    EXPECT_EQ(xvt::GetFileExtension(filePath2), fileExt2);
    EXPECT_EQ(xvt::GetFileExtension(filePath3), fileExt3);
    EXPECT_EQ(xvt::GetFileExtension(filePath4), fileExt4);
}

// Test xvt::GetFileNameWithoutExtension with std::wstring and '/' separator
TEST(GetFileNameWithoutExtensionTest, stdWStringWithSlash)
{
    std::wstring filePath1 = L"D:/test/1/file.txt";
    std::wstring filePath2 = L"path/to/folder/";
    std::wstring filePath3 = L"/file";
    std::wstring filePath4 = L"./folder/file.txt";

    std::wstring fileName1 = L"file";
    std::wstring fileName2 = L"";
    std::wstring fileName3 = L"file";
    std::wstring fileName4 = L"file";

    EXPECT_EQ(xvt::GetFileNameWithoutExtension(filePath1), fileName1);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(filePath2), fileName2);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(filePath3), fileName3);
    EXPECT_EQ(xvt::GetFileNameWithoutExtension(filePath4), fileName4);
}

// Test xvt::GetParentDirectory with std::wstring and '/' separator
TEST(GetParentDirectoryTest, stdWStringWithSlash)
{
    std::wstring filePath1 = L"D:/test/1/file.txt";
    std::wstring filePath2 = L"path/to/folder/";
    std::wstring filePath3 = L"/file";
    std::wstring filePath4 = L"./folder/file";

    std::wstring parentDir1 = L"D:/test/1";
    std::wstring parentDir2 = L"path/to/folder";
    std::wstring parentDir3 = L"";
    std::wstring parentDir4 = L"./folder";

    EXPECT_EQ(xvt::GetParentDirectory(filePath1), parentDir1);
    EXPECT_EQ(xvt::GetParentDirectory(filePath2), parentDir2);
    EXPECT_EQ(xvt::GetParentDirectory(filePath3), parentDir3);
    EXPECT_EQ(xvt::GetParentDirectory(filePath4), parentDir4);
}

// Test xvt::GetFileExtension with std::wstring and '/' separator
TEST(GetFileExtensionTest, stdWStringWithSlash)
{
    EXPECT_EQ(xvt::GetFileExtension(std::wstring(L"D:\\path\\to\\folder\\준비한.txt")), std::wstring(L".txt"));
    EXPECT_EQ(xvt::GetFileExtension(std::wstring(L"..\\folder\\")), WSTRING_EMPTY);
    EXPECT_EQ(xvt::GetFileExtension(L"..\\folder\\" + WSTRING_LOWERCASE), WSTRING_EMPTY);
    EXPECT_EQ(xvt::GetFileExtension(std::wstring(L"..\\folder\\.hidden")), L".hidden");
}

TEST(xvtFile, WStrRemoveFileSpec)
{
    std::wstring folder = L"D:\\Path\\To\\Folder 2";
    std::wstring inputPath = folder  + L"\\" + OTHERS_LANG + L".txt";

    std::wstring result = xvt::RemoveFileSpec(inputPath);

    EXPECT_EQ(result, folder);
}

TEST(xvtFile, wStringCombinePathTest) 
{
    std::wstring path1 = L"D:\\Path\\To\\Folder\\";

    EXPECT_EQ(xvt::CombinePath(path1, L"..\\folder\\"), L"D:\\Path\\To\\folder\\");
    EXPECT_EQ(xvt::CombinePath(path1, L"folder3"     ), path1 + L"folder3");
    EXPECT_EQ(xvt::CombinePath(path1, L"folder3\\"   ), path1 + L"folder3\\");
    EXPECT_EQ(xvt::CombinePath(path1, WSTRING_EMPTY  ), path1);
}

TEST(xvtFile, IsPathExistTest)
{
    std::wstring path = L"D:\\Path\\To\\Folder\\";

    bool result = xvt::IsPathExist(path);
    EXPECT_FALSE(result);

    //path = L"D:\\abc.txt";
    //HANDLE hFile = CreateFile(path.c_str(), GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    //result = xvt::IsPathExist(path);
    //EXPECT_TRUE(result);


    path = L"D:/Path/To\\Folder\\";
    bool result2 = xvt::IsPathExist(path);
    EXPECT_FALSE(result2);

    //path = L"F:/Desktop\\New folder2";
    //hFile = CreateFile(path.c_str(), GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    //bool result3 = xvt::IsPathExist(path);
    //EXPECT_TRUE(result3);
}

TEST(xvtFile, AddBackslashTest) 
{
    std::wstring path = L"..\\folder";
    std::wstring expectResult = L"..\\folder\\";
    std::wstring result = xvt::AddBackslash(path);
    EXPECT_EQ(result, expectResult);

    path = L"D:\\Path\\To\\Folder\\";
    expectResult = L"D:\\Path\\To\\Folder\\";
    result = xvt::AddBackslash(path);
    EXPECT_EQ(result, expectResult);

    path = std::wstring(L"..\\folder\\준비한.txt");
    expectResult = std::wstring(L"..\\folder\\준비한.txt") + L"\\";
    result = xvt::AddBackslash(path);
    EXPECT_EQ(result, expectResult);
}

TEST(xvtFile, RemoveBackslashTest) 
{
    std::wstring path = L"..\\folder\\";
    std::wstring expectResult = L"..\\folder";
    std::wstring result = xvt::RemoveBackslash(path);
    EXPECT_EQ(result, expectResult);

    path = L"D:\\Path\\To\\Folder\\";
    expectResult = L"D:\\Path\\To\\Folder";
    result = xvt::RemoveBackslash(path);
    EXPECT_EQ(result, expectResult);

    path = std::wstring(L"..\\folder\\준비한.txt") + std::wstring(L"\\");
    expectResult = std::wstring(L"..\\folder\\준비한.txt");
    result = xvt::RemoveBackslash(path);
    EXPECT_EQ(result, expectResult);
}

TEST(xvtFile, IsRelativePathTest)
{
    std::wstring path = L"D:\\abc.txt";
    HANDLE hFile = CreateFile(path.c_str(), GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    bool result = xvt::IsRelativePath(path);

    EXPECT_FALSE(result);

    result = xvt::IsRelativePath(L"..\\Images\\photos\\photo\\");
    EXPECT_TRUE(result);

    bool result2 = xvt::IsRelativePath(L"\\Images/photo.png");
    EXPECT_FALSE(result2);

    bool result3 = xvt::IsRelativePath(L"D:/Images/photos/photo/");
    EXPECT_FALSE(result3);

    bool result4 = xvt::IsRelativePath(L"../Images\\photos/photo/");
    EXPECT_TRUE(result4);

    bool result5 = xvt::IsRelativePath(L"../Images/photos/photo/");
    EXPECT_TRUE(result5);

}

TEST(xvtFile, GetTempFolderPathTest) {
    std::wstring tempFolderPath = xvt::GetTempFolderPath();

    bool result = !tempFolderPath.empty();

    EXPECT_TRUE(result);
    EXPECT_TRUE(PathFileExists(tempFolderPath.c_str())); 
}

TEST(xvtFile, GetCurrentFolderPath) {
    std::wstring currentFolderPath = xvt::GetCurrentFolderPath();

    bool result = !currentFolderPath.empty();

    EXPECT_TRUE(result);
    EXPECT_TRUE(PathFileExists(currentFolderPath.c_str()));
}

TEST(xvtFile, GetCurrentProcessModulePath) {
    std::wstring currentModulePath = xvt::GetCurrentProcessModulePath();

    bool result = !currentModulePath.empty();

    EXPECT_TRUE(result);
    EXPECT_TRUE(PathFileExists(currentModulePath.c_str()));
}

TEST(xvtFile, GetCurrentProcessFolderPath) {
    std::wstring currentFolderPath = xvt::GetCurrentProcessFolderPath();

    bool result = !currentFolderPath.empty();

    EXPECT_TRUE(result);
    EXPECT_TRUE(PathFileExists(currentFolderPath.c_str()));
}

TEST(xvtFile, SplitPath1Test)
{
    std::wstring path = L"C:\\Users\\John\\Documents\\file.txt";
    std::wstring drive, dir, fileTitle, ext;
    xvt::SplitPath(path, &drive, &dir, &fileTitle, &ext);
    EXPECT_EQ(drive, L"C:");
    EXPECT_EQ(dir, L"\\Users\\John\\Documents\\");
    EXPECT_EQ(fileTitle, L"file");
    EXPECT_EQ(ext, L".txt");

    std::wstring path2 = L"C:/Users\\John\\/Documents\\";
    xvt::SplitPath(path2, &drive, &dir, &fileTitle, &ext);
    EXPECT_EQ(drive, L"C:");
    EXPECT_EQ(dir, L"/Users\\John\\/Documents\\");
    EXPECT_EQ(fileTitle, L"");
    EXPECT_EQ(ext, L"");

    std::wstring path3 = L"Users\\John\\Documents";
    xvt::SplitPath(path3, &drive, &dir, &fileTitle, &ext);
    EXPECT_EQ(drive, L"");
    EXPECT_EQ(dir, L"Users\\John\\");
    EXPECT_EQ(fileTitle, L"Documents");
    EXPECT_EQ(ext, L"");

    //std::wstring path4 = L"";
    //xvt::SplitPath(path4, &drive, &dir, &fileTitle, &ext);
    //EXPECT_EQ(drive, L"");
    //EXPECT_EQ(dir, L"");
    //EXPECT_EQ(fileTitle, L"");
    //EXPECT_EQ(ext, L"");

    path = L"C:/Users/John/Documents/file.txt";
    drive, dir, fileTitle, ext;
    xvt::SplitPath(path, &drive, &dir, &fileTitle, &ext);
    EXPECT_EQ(drive, L"C:");
    EXPECT_EQ(dir, L"/Users/John/Documents/");
    EXPECT_EQ(fileTitle, L"file");
    EXPECT_EQ(ext, L".txt");

    path2 = L"C:/Users/John//Documents/";
    xvt::SplitPath(path2, &drive, &dir, &fileTitle, &ext);
    EXPECT_EQ(drive, L"C:");
    EXPECT_EQ(dir, L"/Users/John//Documents/");
    EXPECT_EQ(fileTitle, L"");
    EXPECT_EQ(ext, L"");

    path3 = L"Users/John/Documents";
    xvt::SplitPath(path3, &drive, &dir, &fileTitle, &ext);
    EXPECT_EQ(drive, L"");
    EXPECT_EQ(dir, L"Users/John/");
    EXPECT_EQ(fileTitle, L"Documents");
    EXPECT_EQ(ext, L"");
}

TEST(xvtFile, SplitPath2Test)
{
    std::wstring path = L"C:\\Users\\John\\Documents\\file.txt";
    std::wstring dirPath, fileName;
    xvt::SplitPath(path, &dirPath, &fileName);
    EXPECT_EQ(dirPath, L"C:\\Users\\John\\Documents\\");
    EXPECT_EQ(fileName, L"file.txt");

    std::wstring path2 = L"";
    xvt::SplitPath(path2, &dirPath, &fileName);
    EXPECT_EQ(dirPath, L"");
    EXPECT_EQ(fileName, L"");

    std::wstring path3 = L"C:\\";
    xvt::SplitPath(path3, &dirPath, &fileName);
    EXPECT_EQ(dirPath, L"C:\\");
    EXPECT_EQ(fileName, L"");
}

TEST(xvtFile, CreateGUIDTest)
{
    std::wstring guid = xvt::CreateGUID();
    bool notEmpty = !guid.empty();
    EXPECT_TRUE(notEmpty);

    std::wstring tempGuid = L"{TEMP}";
    guid = xvt::CreateGUID();
    EXPECT_NE(guid, tempGuid);
}

TEST(xvtFile, RemoveFirstSlashTest)
{
    std::wstring path = L"/C:/Users/John/Documents/file.txt";
    std::wstring expectResult = L"C:/Users/John/Documents/file.txt";
    std::wstring result = xvt::RemoveFirstSlash(path);
    EXPECT_EQ(result, expectResult);

    path = L"D:\\Path\\To\\Folder\\";
    expectResult = L"D:\\Path\\To\\Folder\\";
    result = xvt::RemoveFirstSlash(path);
    EXPECT_EQ(result, expectResult);

    path = L"//" + std::wstring(L"..\\folder\\준비한.txt");
    expectResult = L"/" + std::wstring(L"..\\folder\\준비한.txt");
    result = xvt::RemoveFirstSlash(path);
    EXPECT_EQ(result, expectResult);

    path = L"/" ;
    expectResult = L"";
    result = xvt::RemoveFirstSlash(path);
    EXPECT_EQ(result, expectResult);

}

TEST(xvtFile, ReplaceSlashToDotTest) {
    std::wstring path = L"C:/Users/John/Documents/file.txt";
    std::wstring expectResult = L"C:.Users.John.Documents.file.txt";
    std::wstring result = xvt::ReplaceSlashToDot(path);
    EXPECT_EQ(result, expectResult);

    path = L"///C:/Users/John/Documents/file.txt";
    expectResult = L"...C:.Users.John.Documents.file.txt";
    result = xvt::ReplaceSlashToDot(path);
    EXPECT_EQ(result, expectResult);

    path = L"////";
    expectResult = L"....";
    result = xvt::ReplaceSlashToDot(path);
    EXPECT_EQ(result, expectResult);

    path = L"";
    std::wstring expectedResult4 = L"";
    std::wstring result4 = xvt::ReplaceDotToSlash(path);
    EXPECT_EQ(result4, expectedResult4);

}

TEST(xvtFile, ReplaceDotToSlashTest) {

    std::wstring path1 = L"C:.Users.John.Documents.file.txt";
    std::wstring expectedResult1 = L"C:/Users/John/Documents/file/txt";
    std::wstring result1 = xvt::ReplaceDotToSlash(path1);
    EXPECT_EQ(result1, expectedResult1);

    std::wstring path2 = L"...C:.Users.John.Documents.file.txt";
    std::wstring expectedResult2 = L"///C:/Users/John/Documents/file/txt";
    std::wstring result2 = xvt::ReplaceDotToSlash(path2);
    EXPECT_EQ(result2, expectedResult2);

    std::wstring path3 = L"....";
    std::wstring expectedResult3 = L"////";
    std::wstring result3 = xvt::ReplaceDotToSlash(path3);
    EXPECT_EQ(result3, expectedResult3);

    std::wstring path4 = L"";
    std::wstring expectedResult4 = L"";
    std::wstring result4 = xvt::ReplaceDotToSlash(path4);
    EXPECT_EQ(result4, expectedResult4);
}

TEST(xvtFile, MakeValidXmlPath)
{
    std::wstring input = L"abc/déf/ghi";
    std::wstring result = xvt::MakeValidXmlPath(input);
    EXPECT_EQ(result, input);

    input = L"abc/123/xyz";
    result = xvt::MakeValidXmlPath(input);
    EXPECT_EQ(result, L"abc/Idx123/xyz");

    input = L"123/xyz";
    result = xvt::MakeValidXmlPath(input);
    EXPECT_EQ(result, L"Idx123/xyz");

    input = L"123" + OTHERS_LANG;
    result = xvt::MakeValidXmlPath(input);
    EXPECT_EQ(result, (L"Idx123" + OTHERS_LANG));

    input = L"";
    result = xvt::MakeValidXmlPath(input);
    EXPECT_EQ(result, L"");
}

TEST(xvtFile, TokenizeStringTest) {

    std::wstring input = L"aabc/def/ghi";
    std::vector<std::wstring> result1;
    xvt::TokenizeString(input, 'm', result1);
    EXPECT_EQ(result1.size(), 1);
    EXPECT_EQ(result1[0], input);

    input = L"abc/123/xyz";
    std::vector<std::wstring> result2;
    xvt::TokenizeString(input, '/', result2);
    EXPECT_EQ(result2.size(), 3);
    EXPECT_EQ(result2[0], L"abc");
    EXPECT_EQ(result2[1], L"123");
    EXPECT_EQ(result2[2], L"xyz");

    input = L"";
    std::vector<std::wstring> result;
    xvt::TokenizeString(input, '/', result);
    EXPECT_EQ(result.size(), 1);
    EXPECT_EQ(result[0], L"");
}

TEST(xvtFile, MakeDirectoryTest) {
    std::wstring input = L"M:";
    bool result = xvt::MakeDirectory(input);
    EXPECT_EQ(result, false);

    input = L"";
    result = xvt::MakeDirectory(input);
    EXPECT_EQ(result, false);

    input = L"x64/test";

    result = xvt::IsPathExist(input);
    EXPECT_FALSE(result);

    result = xvt::MakeDirectory(input);
    EXPECT_EQ(result, true);

    result = xvt::IsPathExist(input);
    EXPECT_TRUE(result);

    result = xvt::DeleteDirectory(input);
    EXPECT_EQ(result, true);

    result = xvt::IsPathExist(input);
    EXPECT_FALSE(result);
}

TEST(xvtFile, GetFilesTest)
{
    //std::wstring folder = L"F:\\Desktop\\New folder";
    //std::wstring searchPattern = L"*.txt";
    //auto files = xvt::GetFiles(folder, searchPattern);
    //ASSERT_EQ(files.size(), 3);

    std::wstring folder2 = L"F:\\Desktop\\New folder23";
    std::wstring searchPattern2 = L"*.txt";
    auto files = xvt::GetFiles(folder2, searchPattern2);
    //ASSERT_EQ(files.size(), 0);

    folder2 = L"";
    searchPattern2 = L"";
    files = xvt::GetFiles(folder2, searchPattern2);
    //ASSERT_EQ(files.size(), 0);
}

TEST(xvtFile, GetFoldersTest)
{
    std::wstring folder = L"F:\\Desktop";
    auto files = xvt::GetFolders(folder);
    //ASSERT_EQ(files.size(), 16);

    folder = L"";
    files = xvt::GetFolders(folder);
    //ASSERT_EQ(files.size(), 0);

    folder = L"F:\\Desktop\\215";
    files = xvt::GetFolders(folder);
    //ASSERT_EQ(files.size(), 0);
}
