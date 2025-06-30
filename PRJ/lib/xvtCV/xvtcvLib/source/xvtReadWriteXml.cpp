#include "xvtCV/xvtReadWriteXml.h"
#include "xvtCV/xvtFile.h"
#include "xvtCV/xvtbase64.h"
#include "xvtCV/xvtConvert.h"
#include <algorithm>
#include <iostream>
#include <atlconv.h>
#include <fstream> 
//3 file includ nay phai o duoi atlconv.h va fstream
#include <mxml-private.h>
#include <mxml.h>
#include <zlib.h>



using namespace xvt;


#define XML_VERSION								"1.0"
#define EMPTY_STRING							L""
#define EMPTY_TSTRING							L""
#define PATH_TOKEN								'/'

#define BINIARY_TAG								L"|ExternalBinary|"
#define BINIARY_DATA_FOLDER_NAME				L".BinaryData"
#define EXTERNAL_BINARY_MINIMUM_LENGTH			4096

#define ATT_TYPE								L"type"
#define ATT_LENGTH								L"length"
#define ATT_VALUE_COMPRESSED					L"compressed"

namespace xvt 
{

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*
	* 'whitespace_cb()' - Let the mxmlSaveFile() function know when to insert
	*                     newlines and tabs...
	*/

	const char*				/* O - Whitespace string or NULL */
	whitespace_cb(mxml_node_t* node,	/* I - Element node */
		int         where)	/* I - Open or close tag? */
	{
		mxml_node_t* parent;		/* Parent node */
		int		level;				/* Indentation level */
		const char* name;			/* Name of element */
		static const char* tabs[] = { "\n\t",
									  "\n\t\t",
									  "\n\t\t\t",
									  "\n\t\t\t\t",
									  "\n\t\t\t\t\t",
									  "\n\t\t\t\t\t\t",
									  "\n\t\t\t\t\t\t\t",
									  "\n\t\t\t\t\t\t\t\t" };
		/* Tabs for indentation */

		/*
		* We can conditionally break to a new line before or after any element.
		* These are just common HTML elements...
		*/
		name = node->value.element.name;

		if (!strncmp(name, "?xml", 4))
		{
			if (where == MXML_WS_AFTER_OPEN)
				return ("\n");
			else
				return (NULL);
		}
		else if (where == MXML_WS_AFTER_CLOSE)
			return ("\n");
		else if (where == MXML_WS_AFTER_OPEN)
		{
			if (node->child && node->child->type == MXML_TEXT &&
				(node->child->value.text.string == nullptr || strlen(node->child->value.text.string) == 0))
				return ("\n");

			return (NULL);
		}
		else if (where == MXML_WS_BEFORE_OPEN || where == MXML_WS_BEFORE_CLOSE)
		{
			if (where == MXML_WS_BEFORE_CLOSE && node->child && node->child->type == MXML_TEXT
				&& node->child->value.text.string && strlen(node->child->value.text.string) != 0)
				return (NULL);

			for (level = -1, parent = node->parent;
				parent;
				level++, parent = parent->parent);

			if (level > 8)
				level = 8;
			else if (level <= 0)
				return (NULL);

			return tabs[level - 1];
		}


		/*
		* Return NULL for no added whitespace...
		*/

		return (NULL);
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read the value of a specific node.
	std::wstring  extractNodeValue(mxml_node_t* pNode)
	{
		std::wstring  strValue;

		if (pNode != nullptr)
		{
			if (pNode->type == MXML_ELEMENT)
				pNode = pNode->child;

			if (pNode == nullptr)
				return (L"");

			std::wstring  strOutput;
			// All data should be read based on space.
			mxml_node_t* pSibling = pNode;
			while (pSibling != nullptr)
			{
				int nWhiteSpace PURE;
				const char* pszCurrent = ::mxmlGetText(pSibling, &nWhiteSpace);

				if (pszCurrent != nullptr)
				{
					std::string  strValue = pszCurrent;
					strOutput += xvt::ToWString(strValue);

					// move to next
					pSibling = mxmlGetNextSibling(pSibling);

					if (pSibling != nullptr && pSibling->type == MXML_TEXT)
					{
						strOutput += (L" ");
					}
					else
					{
						break;
					}
				}
				else
				{
					pSibling = nullptr;
				}
			}

			strValue = strOutput;
			strValue = TrimSpace(strValue);
		}

		return strValue;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	 // Write the value of a specific node.
	bool writeNodeValue(mxml_node_t* pNode, std::string lpszValue)
	{
		if (pNode != nullptr)
		{
			if (pNode->type == MXML_TEXT)
				pNode = pNode->parent;

			if (pNode == nullptr || pNode->type != MXML_ELEMENT)
				return false;

			int nWhiteSpace PURE;
			if (::mxmlGetText(pNode, &nWhiteSpace) == nullptr)
			{
				nWhiteSpace PURE;
				::mxmlNewText(pNode, nWhiteSpace, lpszValue.c_str());
			}
			else
			{
				if (pNode->child != nullptr)
				{
					mxml_node_t* pSibling = pNode->child;
					while (pSibling != nullptr)
					{
						// move to next
						mxml_node_t* pNextSibling = ::mxmlGetNextSibling(pSibling);

						::mxmlRelease(pSibling);

						pSibling = pNextSibling;
					}

					nWhiteSpace PURE;
					::mxmlNewText(pNode, nWhiteSpace, lpszValue.c_str());
				}
				else
				{
					nWhiteSpace PURE;
					::mxmlSetText(pNode, nWhiteSpace, lpszValue.c_str());
				}
			}
			return true;
		}

		return false;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool isTrueString(std::wstring lpszValue)
	{
		std::wstring  strValue = lpszValue;

		strValue = ToLowerCase(strValue);

		strValue = TrimSpace(strValue);

		if (strValue == L"true" || strValue == L"yes" || strValue == L"1")
			return true;

		return false;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// constructor
	ReadWriteXml::ReadWriteXml(E_XmlStorageTypes nType)
		: _nType(nType)
	{
		Initialize();
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// destructor
	ReadWriteXml::~ReadWriteXml(void)
	{
		CleanUp();
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool ReadWriteXml::Initialize(void)
	{
		// xml root 
		_pCurrentXml = reinterpret_cast<LPVOID>(::mxmlNewXML(XML_VERSION));
		const bool bResult = InitializeTempDirectory();

		return (_pCurrentXml != nullptr) && bResult;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool ReadWriteXml::CleanUp(void)
	{
		::mxmlDelete(reinterpret_cast<mxml_node_t*>(_pCurrentXml));
		_pCurrentXml = NULL;

		std::wstring  strDeletedDir = _strTempDirectory;
		strDeletedDir.erase(_strTempDirectory.length() - 1,1);

		if (XVT_DIR_IsPathExist(strDeletedDir))
			XVT_DIR_DeleteDirectory(strDeletedDir);

		return _pCurrentXml == nullptr;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Loads the xml file in the lpszXmlFilePath path. It may fail if there is no valid xml.
	bool ReadWriteXml::Load(std::wstring lpszXmlFilePath)
	{
		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(_pCurrentXml);

		bool bLoaded = false;

		FILE* fp = NULL;

		if (::_wfopen_s(&fp, lpszXmlFilePath.c_str(), L"r") == 0)
		{
			// Avoid wrapping. (You can write without limiting the length of the line)
			::mxmlSetWrapMargin(0);
			::mxmlDelete(pCurrentXml);
			pCurrentXml = ::mxmlLoadFile(MXML_NO_PARENT, fp, MXML_TEXT_CALLBACK);

			_pCurrentXml = reinterpret_cast<LPVOID>(pCurrentXml);

			// clean up
			::fclose(fp);
			fp = nullptr;

			// set to be loaded 
			bLoaded = _pCurrentXml != nullptr;

			// set current xml file path 
			_strCurrentXmlFilePath = bLoaded ? lpszXmlFilePath : L"";

		}

		return bLoaded;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Save the current values to the lpszXmlFilePath path.
	bool ReadWriteXml::Save(std::wstring lpszXmlFilePath)
	{
		bool bSaved = false;

		// Converted and used to prevent the corresponding type from being exposed in the header.
		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(_pCurrentXml);

		USES_CONVERSION;

		if (IsPathExist(lpszXmlFilePath))
			::DeleteFile(lpszXmlFilePath.c_str());

		std::wstring  strBaseDir = RemoveFileSpec(lpszXmlFilePath);
		XVT_DIR_MakeDirectory(strBaseDir);

		FILE* fp = NULL;

		std::wstring widedrivestring = std::wstring(lpszXmlFilePath.begin(), lpszXmlFilePath.end());
		const wchar_t* szName = widedrivestring.c_str();

		if (::_wfopen_s(&fp, szName, L"w") == 0)
		{
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// xml object
			::mxmlSaveFile(pCurrentXml, fp, whitespace_cb);

			::fclose(fp);

			// If it has never been loaded or the repository has changed, you must copy the files.
			if (_strCurrentXmlFilePath.empty() || _strCurrentXmlFilePath.compare(lpszXmlFilePath) != 0)
			{
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Move saved binary files
				std::wstring  strTargetRootDirectory(lpszXmlFilePath);
				strTargetRootDirectory += BINIARY_DATA_FOLDER_NAME;

				XVT_DIR_MakeDirectory(strTargetRootDirectory);

				std::wstring  strSrcRootDirectory;

				// When the repository changes
				if (_strCurrentXmlFilePath.empty() == FALSE)
				{
					// Let's move the data in the existing folder.
					strSrcRootDirectory = _strCurrentXmlFilePath;
					RemoveFileSpec(strSrcRootDirectory);
				}
				else
				{
					// Let's move the data in the temporary folder.
					strSrcRootDirectory = _strTempDirectory;
				}

				if (strSrcRootDirectory[strSrcRootDirectory.length() - 1] == ('\\'))
					strSrcRootDirectory.erase(strSrcRootDirectory.length() - 1, 1);

				const int nCount = XVT_DIR_CopyDirectory(strSrcRootDirectory, strTargetRootDirectory);

				if (nCount == 0)
					XVT_DIR_DeleteDirectory(strTargetRootDirectory);


				_strCurrentXmlFilePath = lpszXmlFilePath;

			}

			// set to be saved
			bSaved = true;

		}

		return bSaved;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Is there a node in that path?
	bool ReadWriteXml::HasNode(std::wstring lpszPath) const
	{
		// Converted and used so that the corresponding type is not exposed in the header.
		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(_pCurrentXml);

		USES_CONVERSION;


		std::wstring  strPath(lpszPath);
		strPath = RemoveFirstSlash(strPath);

		//There is a "." in the middle of the path. If there is, create a subnode.
		strPath = ReplaceDotToSlash(strPath);

		strPath = MakeValidXmlPath(strPath);

		std::string str(strPath.begin(), strPath.end());
		// Find the node corresponding to lpszPath.
		mxml_node_t* pFoundNode = ::mxmlFindPath(pCurrentXml, str.c_str());

		if (pFoundNode != nullptr)
		{
			return true;
		}

		// failed to find the node of lpszPath
		return false;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reads a string from storage. If there is no node in that path, false is returned.
	bool ReadWriteXml::GetString(std::wstring lpszPath, std::wstring * pstrValue) const
	{
		if (pstrValue == nullptr)
			return false;

		// Converted and used to prevent the type from being exposed in the header
		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(_pCurrentXml);

		USES_CONVERSION;


		std::wstring  strPath(lpszPath);
		strPath = RemoveFirstSlash(strPath);

		// There is a "." in the middle of the path. If there is, create a subnode.
		strPath = ReplaceDotToSlash(strPath);

		strPath = MakeValidXmlPath(strPath);

		std::string str(strPath.begin(), strPath.end());
		// Find the node corresponding to lpszPath.
		mxml_node_t* pFoundNode = ::mxmlFindPath(pCurrentXml, str.c_str());

		if (pFoundNode != nullptr)
		{
			std::wstring  ss = extractNodeValue(pFoundNode);
			*pstrValue = ss;
			return true;
		}

		// failed to find the node of lpszPath

		return false;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Obtains the string value of a specific node.
	std::wstring  ReadWriteXml::ReadString(std::wstring lpszPath, std::wstring lpszDefaultValue) const
	{
		std::wstring  strValue;

		if (GetString(lpszPath, &strValue))
			return strValue;
		else
			return lpszDefaultValue;

	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sets the string value of a specific node.
	bool ReadWriteXml::WriteString(std::wstring lpszPath, std::wstring lpszValue)
	{

		// Converted and used to prevent the corresponding type from being exposed in the header.
		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(_pCurrentXml);

		bool bSet = false;

		USES_CONVERSION;

		std::wstring  strPath(lpszPath);
		strPath = RemoveFirstSlash(strPath);

		strPath = ReplaceDotToSlash(strPath);

		strPath = MakeValidXmlPath(strPath);

		std::string str(strPath.begin(), strPath.end());

		mxml_node_t* pFoundNode = ::mxmlFindPath(pCurrentXml, str.c_str());

		if (pFoundNode == nullptr)
		{

			mxml_node_t* pNewNode = pCurrentXml;

			std::wstring  strToken, strParentPath;
			int nStart PURE;

			std::vector<std::wstring > arrElements;
			TokenizeString(strPath, PATH_TOKEN, arrElements);
			int sizetok = arrElements.size();

			for(int i =0; i< sizetok;i++)
			{
				if (arrElements[i] != EMPTY_TSTRING)
				{
					std::string sToken(strToken.begin(), strToken.end());

					mxml_node_t* pTempNode = ::mxmlFindElement(pNewNode, pNewNode, sToken.c_str(),
						NULL, NULL,
						MXML_DESCEND_FIRST);
					if (pTempNode == nullptr)
					{
						pNewNode = ::mxmlNewElement(pNewNode, sToken.c_str());
					}
					else
					{
						pNewNode = pTempNode;
					}
				}

			}

			pFoundNode = pNewNode;

		}


		if (pFoundNode)
		{
			std::string sValue(lpszValue.begin(), lpszValue.end());

			writeNodeValue(pFoundNode, sValue);
			bSet = true;
		}

		return bSet;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// bool 
	bool ReadWriteXml::WriteBool(std::wstring lpszPath, bool bValue)
	{
		return WriteString(lpszPath, bValue ? L"true" : L"false");
	}

	// binary 
	bool ReadWriteXml::WriteBinary(std::wstring lpszPath, std::wstring &lpData, int dwDataLength)
	{
		// embedding
		if (dwDataLength <= EXTERNAL_BINARY_MINIMUM_LENGTH || GetXmlBinaryStoreType() == E_SimpleStorage)
		{
			std::wstring  strBase64Value = XVT_EncodeBase64(lpData.c_str());
			return WriteString(lpszPath, strBase64Value);
			
		}
		else if (GetXmlBinaryStoreType() == E_ComplexStorage || GetXmlBinaryStoreType() == E_ComplexStorageWithCompression)
		{
			const bool bCompressed = GetXmlBinaryStoreType() == E_ComplexStorageWithCompression;
	
			std::wstring  strFileName;
			if (SetExternalBinary(lpszPath, lpData, dwDataLength, bCompressed, &strFileName))
			{
				if (WriteString(lpszPath, BINIARY_TAG + strFileName))
				{
					HXMLELEMENT hElement = GetElement(lpszPath);
					if (hElement)
					{
						// set original length
						SetAttribute(hElement, ATT_LENGTH, std::to_wstring(dwDataLength));
	
						// set data type
						if (bCompressed)
						{
							SetAttribute(hElement, ATT_TYPE, ATT_VALUE_COMPRESSED);
						}
					}
	
					return true;
				}
	
			}
		}
	
		return false;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// integer 
	bool ReadWriteXml::WriteInteger(std::wstring lpszPath, int nValue)
	{
		return WriteString(lpszPath, std::to_wstring(nValue));
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// double 
	bool ReadWriteXml::WriteReal(std::wstring lpszPath, double dValue)
	{
		return WriteString(lpszPath, std::to_wstring(dValue));
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// binary 
	bool ReadWriteXml::SetExternalBinary(const std::wstring lpszHint, const std::wstring lpData, int dwDataLength,
		bool bCompressed, std::wstring * pstrFileName)
	{
		std::wstring  strFileName(lpszHint);
		strFileName = RemoveFirstSlash(strFileName);
		strFileName = ReplaceSlashToDot(strFileName);
	
		std::wstring  strTempPath = _strTempDirectory + strFileName;
	
		std::ofstream fsWrite;
		fsWrite.open(strTempPath, std::iostream::binary | std::iostream::out);
		if (fsWrite.is_open())
		{
			if (bCompressed)
	
			{
				DWORD dwBinaryDataLength = dwDataLength;
				LPBYTE pBinaryData = new BYTE[dwBinaryDataLength];
				const BYTE* p = reinterpret_cast<const BYTE*>(lpData.c_str());
				compress(pBinaryData, &dwBinaryDataLength, p, dwDataLength);
	
				if (dwBinaryDataLength > 0)
				{
					for (int i = 0; i < dwBinaryDataLength; i++)
						fsWrite.write((char*)&pBinaryData[i], sizeof(pBinaryData));
				}
				fsWrite.close();
	
				delete[] pBinaryData;
			}
			else
			{
				if (dwDataLength > 0)
				{
					for (int i = 0; i < dwDataLength; i++)
						fsWrite.write((char*)&lpData[i], sizeof(lpData));
				}
				fsWrite.close();
			}
	
			if (pstrFileName)
				*pstrFileName = strFileName;
	
			return true;
		}
	
		return false;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// integer 
	int ReadWriteXml::ReadInteger(std::wstring lpszPath, int nDefaultValue) const
	{
		std::wstring  strValue;

		if (GetString(lpszPath, &strValue))
		{
			int nValue = std::stoi(strValue);
			return nValue;
		}
		else
		{
			return nDefaultValue;
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// bool 
	bool ReadWriteXml::ReadBool(std::wstring lpszPath, bool bDefaultValue) const
	{
		std::wstring  strValue;

		if (GetString(lpszPath, &strValue))
		{
			return isTrueString(strValue);
		}
		else
		{
			return bDefaultValue;
		}
	}



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// double 
	double ReadWriteXml::ReadReal(std::wstring lpszPath, double dDefaultValue) const
	{
		std::wstring  strValue;

		if (GetString(lpszPath, &strValue))
		{
			double dValue = std::stof(strValue);
			return dValue;
		}
		else
		{
			return dDefaultValue;
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Read data in binary form from storage.
	// If the data is large, it may take time.
	// If you know the size of the binary data you want to read, allocate memory externally and use it.
	// The input dwDataLength must be equal to or larger than the size of the actual stored data. If it is larger, the actual size is applied to dwDataLength after the function returns.
	// If lpData is entered as nullptr, the data is alloced internally.
	// Allocated memory must be deleted using the FreeBinary() method after use.
	bool ReadWriteXml::GetBinary(std::wstring lpszHint, std::wstring &lpData, int dwDataLength) const
	{
		std::wstring  strHint(lpszHint);
	
		int npos = strHint.find(BINIARY_TAG);
		if (npos == 0)
		{
			std::wstring  strFileName(strHint);
			strFileName.replace(npos,sizeof( BINIARY_TAG), L"");
	
			const std::wstring  strXmlPath(GetCurrentXmlFilePath().begin(), GetCurrentXmlFilePath().end());
			std::wstring  strBinaryFullPath = CombinePath(strXmlPath + BINIARY_DATA_FOLDER_NAME, strFileName);
	
			if (strXmlPath.empty() || IsPathExist(strBinaryFullPath) == false)
			{
				strBinaryFullPath = _strTempDirectory + strFileName;
			}
	
			std::ifstream fsRead;
			fsRead.open(strBinaryFullPath, std::iostream::binary | std::iostream::in| std::iostream::ate);
			if (fsRead.is_open())
			{
				bool bResult = false;
	
				DWORD dwStoredDataLength = static_cast<DWORD>(fsRead.tellg());
	
				DWORD dwOriginalDataLength PURE;
				bool bIsCompressed = false;
	
				// get attribute
				HXMLELEMENT hElement = GetElement(strFileName);
				if (hElement)
				{
					const std::wstring  strLength = GetAttribute(hElement, ATT_LENGTH, nullptr);
					const std::wstring  strType = GetAttribute(hElement, ATT_TYPE, nullptr);
	
					dwOriginalDataLength = std::stoi(strLength);
					bIsCompressed = ToLowerCase(strType).compare(L"compressed") == 0;
				}
	
				if (dwOriginalDataLength == 0)
				{
					dwOriginalDataLength = dwStoredDataLength;
				}
	
				
				if (dwDataLength >= dwOriginalDataLength)
				{
					if (bIsCompressed)
					{
						DWORD dwCompressedLength = dwStoredDataLength;
						byte* pCompressed = new BYTE[dwCompressedLength];
	
						if (dwCompressedLength > 0)
						{
							for (int i = 0; i < dwCompressedLength; i++)
								fsRead.read((char*)&pCompressed[i], sizeof(pCompressed));
						}
						fsRead.close();
						Bytef* dest = {};
						DWORD dwLength = dwDataLength;
						bResult = uncompress(dest, &dwLength, pCompressed, dwCompressedLength) == Z_OK;
						lpData = (reinterpret_cast<wchar_t*>(dest), dwCompressedLength / sizeof(wchar_t));
						delete[] pCompressed;
	
					}
					else
					{
						dwDataLength = dwStoredDataLength;
	
						if (dwStoredDataLength > 0)
						{
							for (int i = 0; i < dwStoredDataLength; i++)
								fsRead.read((char*)&lpData[i], sizeof(lpData));
						}
						fsRead.close();
						bResult = true;
					}
	
					return bResult;
				}
				else
				{
					fsRead.close();
					return false;
				}
			}
		}
		else // embedding
		{
			lpData =  XVT_DecodeBase64(strHint.c_str());
			return true; 
		}
	
		return false;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//// binary 
	bool ReadWriteXml::ReadBinary(std::wstring lpszPath, std::wstring &lpData, int& dwDataLength) const
	{
		std::wstring  strValue;
		if (GetString(lpszPath, &strValue))
		{
			return GetBinary(strValue, lpData, dwDataLength);
		}
	
		return false;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ReadBinary() 
	void ReadWriteXml::FreeBinary(std::wstring lpData)
	{
		if (!lpData.empty())
		{
			lpData = nullptr;
		}
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Get xml file path
	const std::wstring & ReadWriteXml::GetCurrentXmlFilePath(void) const
	{
		return _strCurrentXmlFilePath;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool ReadWriteXml::InitializeTempDirectory(void)
	{
		_strTempDirectory = CombinePath(GetTempFolderPath(), L"xpUtils.temp.xml");

		_strTempDirectory = CombinePath(_strTempDirectory, XVT_DIR_CreateGUID());
		_strTempDirectory = AddBackslash(_strTempDirectory);

		const bool bResult = XVT_DIR_MakeDirectory(_strTempDirectory);

		return bResult;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// add element
	HXMLELEMENT ReadWriteXml::AddElement(std::wstring lpszPath)
	{
		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(_pCurrentXml);
		USES_CONVERSION;

		std::wstring  strPath(lpszPath);
		strPath = RemoveFirstSlash(strPath);

		strPath = ReplaceDotToSlash(strPath);

		strPath = MakeValidXmlPath(strPath);

		mxml_node_t* pNewNode = pCurrentXml;


		std::vector<std::wstring > arrElementNames;
		TokenizeString(strPath, PATH_TOKEN, arrElementNames);

		const int nElementDepth = static_cast<int>(arrElementNames.size());

		for (int i = 0; i < nElementDepth; i++)
		{
			const std::string  strElementName(arrElementNames[i].begin(), arrElementNames[i].end());

			mxml_node_t* pTempNode = ::mxmlFindElement(pNewNode, pNewNode, strElementName.c_str(),
				NULL, NULL,
				MXML_DESCEND_FIRST);

			if (i == (nElementDepth - 1) || pTempNode == nullptr)
			{
				pNewNode = ::mxmlNewElement(pNewNode, strElementName.c_str());
			}
			else
			{
				pNewNode = pTempNode;
			}

		}

		return reinterpret_cast<HXMLELEMENT>(pNewNode);

	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// add element
	HXMLELEMENT ReadWriteXml::AddElement(HXMLELEMENT hElement, std::wstring lpszPath)
	{
		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(hElement);

		USES_CONVERSION;
		std::wstring  strPath(lpszPath);
		strPath = RemoveFirstSlash(strPath);

		strPath = ReplaceDotToSlash(strPath);

		strPath = MakeValidXmlPath(strPath);

		mxml_node_t* pNewNode = pCurrentXml;


		std::vector<std::wstring > arrElementNames;
		TokenizeString(strPath, PATH_TOKEN, arrElementNames);

		const int nElementDepth = static_cast<int>(arrElementNames.size());

		for (int i = 0; i < nElementDepth; i++)
		{
			const std::string  strElementName (arrElementNames[i].begin(), arrElementNames[i].end());

			mxml_node_t* pTempNode = ::mxmlFindElement(pNewNode, pNewNode, strElementName.c_str(),
				NULL, NULL,
				MXML_DESCEND_FIRST);

			if (i == (nElementDepth - 1) || pTempNode == nullptr)
			{
				pNewNode = ::mxmlNewElement(pNewNode, strElementName.c_str());
			}
			else
			{
				pNewNode = pTempNode;
			}

		}

		return reinterpret_cast<HXMLELEMENT>(pNewNode);

	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// get element
	HXMLELEMENT ReadWriteXml::GetElement(std::wstring lpszPath, int nIndex) const
	{
		if (_pCurrentXml == nullptr)
			return nullptr;

		if (nIndex < 0 || nIndex >= GetElementCount(lpszPath))
			return nullptr;

		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(_pCurrentXml);

		int nCount PURE;
		USES_CONVERSION;

		std::wstring  strPath(lpszPath);
		strPath = RemoveFirstSlash(strPath);

		strPath = ReplaceDotToSlash(strPath);

		strPath = MakeValidXmlPath(strPath);

		std::wstring  strLastElementName;
		std::vector<std::wstring > arrElements;
		TokenizeString(strPath, PATH_TOKEN, arrElements);

		if (arrElements.size() > 0)
			strLastElementName = arrElements[arrElements.size() - 1];

		if (strLastElementName.empty())
			return nullptr;

		const int nElementCount = static_cast<int>(arrElements.size());
		mxml_node_t* pFoundNode = pCurrentXml;

		for (int i = 0; i < nElementCount; i++)
		{
			std::string str(arrElements[i].begin(), arrElements[i].end());

			pFoundNode = ::mxmlFindElement(pFoundNode,
				pFoundNode, str.c_str(),
				NULL, NULL,
				MXML_DESCEND_FIRST);

		}

		mxml_node_t* pSibling = pFoundNode;

		while (pSibling)
		{
			if (pSibling->type == MXML_ELEMENT)
			{
				std::string str(strLastElementName.begin(), strLastElementName.end());
				if (strcmp(str.c_str(), pSibling->value.element.name) == 0)
				{
					if (nIndex == nCount)
					{
						return reinterpret_cast<HXMLELEMENT>(pSibling);
					}
					nCount++;
				}
			}

			// move to next
			pSibling = mxmlGetNextSibling(pSibling);
		}

		return nullptr;
	}

	HXMLELEMENT ReadWriteXml::GetElement(HXMLELEMENT hElement, std::wstring lpszElementName, int nIndex) const
	{
		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(hElement);

		int nCount PURE;
		USES_CONVERSION;

		std::wstring  strPath(lpszElementName);
		strPath = RemoveFirstSlash(strPath);

		strPath = ReplaceDotToSlash(strPath);

		strPath = MakeValidXmlPath(strPath);

		std::wstring  strLastElementName;
		std::vector<std::wstring > arrElements;
		TokenizeString(strPath, PATH_TOKEN, arrElements);

		if (arrElements.size() > 0)
			strLastElementName = arrElements[arrElements.size() - 1];

		if (strLastElementName.empty())
			return nullptr;

		const int nElementCount = static_cast<int>(arrElements.size());
		mxml_node_t* pFoundNode = pCurrentXml;

		for (int i = 0; i < nElementCount; i++)
		{
			std::string str(arrElements[i].begin(), arrElements[i].end());
			pFoundNode = ::mxmlFindElement(pFoundNode,
				pFoundNode, str.c_str(),
				NULL, NULL,
				MXML_DESCEND_FIRST);

		}

		mxml_node_t* pSibling = pFoundNode;

		while (pSibling)
		{
			if (pSibling->type == MXML_ELEMENT)
			{
				std::string str(strLastElementName.begin(), strLastElementName.end());

				if (strcmp(str.c_str(), pSibling->value.element.name) == 0)
				{
					if (nIndex == nCount)
					{
						return reinterpret_cast<HXMLELEMENT>(pSibling);
					}
					nCount++;
				}
			}

			// move to next
			pSibling = mxmlGetNextSibling(pSibling);
		}

		return nullptr;
	}

	// get element count
	int ReadWriteXml::GetElementCount(HXMLELEMENT hElement, std::wstring lpszElementName) const
	{
		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(hElement);

		int nCount PURE;
		USES_CONVERSION;

		std::wstring  strPath(lpszElementName);
		strPath = RemoveFirstSlash(strPath);

		strPath = ReplaceDotToSlash(strPath);

		strPath = MakeValidXmlPath(strPath);

		std::wstring  strLastElementName;
		std::vector<std::wstring > arrElements;
		TokenizeString(strPath, PATH_TOKEN, arrElements);

		if (arrElements.size() > 0)
			strLastElementName = arrElements[arrElements.size() - 1];

		if (strLastElementName.empty())
			return -1;

		const int nElementCount = static_cast<int>(arrElements.size());
		mxml_node_t* pFoundNode = pCurrentXml;

		for (int i = 0; i < nElementCount; i++)
		{
			std::string str(arrElements[i].begin(), arrElements[i].end());
			pFoundNode = ::mxmlFindElement(pFoundNode,
				pFoundNode, str.c_str(),
				NULL, NULL,
				MXML_DESCEND_FIRST);

		}


		mxml_node_t* pSibling = pFoundNode;

		while (pSibling)
		{
			if (pSibling->type == MXML_ELEMENT)
			{
				std::string str(strLastElementName.begin(), strLastElementName.end());
				if (strcmp(str.c_str(), pSibling->value.element.name) == 0)
				{
					nCount++;
				}
			}

			// move to next
			pSibling = ::mxmlGetNextSibling(pSibling);
		}



		return nCount;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// get element count
	int ReadWriteXml::GetElementCount(std::wstring lpszPath) const
	{
		if (_pCurrentXml == nullptr)
			return -1;
		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(_pCurrentXml);

		USES_CONVERSION;

		int nCount PURE;

		std::wstring  strPath(lpszPath);
		strPath = RemoveFirstSlash(strPath);

		strPath = ReplaceDotToSlash(strPath);

		strPath = MakeValidXmlPath(strPath);

		std::wstring  strLastElementName;
		std::vector<std::wstring > arrElements;
		TokenizeString(strPath, PATH_TOKEN, arrElements);

		if (arrElements.size() > 0)
			strLastElementName = arrElements[arrElements.size() - 1];

		if (strLastElementName.empty())
			return -1;

		const int nElementCount = static_cast<int>(arrElements.size());
		mxml_node_t* pFoundNode = pCurrentXml;

		for (int i = 0; i < nElementCount; i++)
		{
			std::string str(arrElements[i].begin(), arrElements[i].end());
			pFoundNode = ::mxmlFindElement(pFoundNode,
				pFoundNode, str.c_str(),
				NULL, NULL,
				MXML_DESCEND_FIRST);

		}

		mxml_node_t* pSibling = pFoundNode;

		while (pSibling)
		{
			if (pSibling->type == MXML_ELEMENT)
			{
				std::wstring  strCurrentElementName = xvt::ToWString(pSibling->value.element.name);
				if (ToLowerCase(strLastElementName).compare(ToLowerCase(strCurrentElementName)) == 0)
				{
					nCount++;
				}
			}

			// move to next
			pSibling = mxmlGetNextSibling(pSibling);
		}

		return nCount;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// remove element
	bool ReadWriteXml::RemoveElement(std::wstring lpszPath, int nIndex)
	{
		HXMLELEMENT hElement = GetElement(lpszPath, nIndex);

		if (hElement != nullptr)
		{
			mxmlDelete(reinterpret_cast<mxml_node_t*>(hElement));
			return true;
		}

		return false;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// remove item. 
	bool ReadWriteXml::Remove(std::wstring lpszPath)
	{
		if (_pCurrentXml == nullptr)
			return false;

		mxml_node_t* pCurrentXml = reinterpret_cast<mxml_node_t*>(_pCurrentXml);

		USES_CONVERSION;

		std::wstring  strPath(lpszPath);
		strPath = RemoveFirstSlash(strPath);

		strPath = ReplaceDotToSlash(strPath);

		strPath = MakeValidXmlPath(strPath);

		std::vector<std::wstring > arrElements;
		TokenizeString(strPath, PATH_TOKEN, arrElements);

		const int nElementCount = static_cast<int>(arrElements.size());
		mxml_node_t* pFoundNode = pCurrentXml;

		for (int i = 0; i < nElementCount; i++)
		{
			std::string str(arrElements[i].begin(), arrElements[i].end());
			pFoundNode = ::mxmlFindElement(pFoundNode,
				pFoundNode, str.c_str(),
				NULL, NULL,
				MXML_DESCEND_FIRST);

		}

		if (pFoundNode)
		{
			::mxmlDelete(pFoundNode);
			return true;
		}

		return false;

	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// find element
	HXMLELEMENT ReadWriteXml::FindElement(HXMLELEMENT hElement, std::wstring lpszSubElementName, bool bCreate) const
	{
		mxml_node_t* pFoundNode = nullptr;

		if (!lpszSubElementName.empty())
		{
			std::wstring  strPath(lpszSubElementName);
			strPath = RemoveFirstSlash(strPath);

			strPath = ReplaceDotToSlash(strPath);

			strPath = MakeValidXmlPath(strPath);

			std::wstring  strLastElementName;
			std::vector<std::wstring > arrElements;
			TokenizeString(strPath, PATH_TOKEN, arrElements);

			pFoundNode = reinterpret_cast<mxml_node_t*>(hElement);

			const int nCount = static_cast<int>(arrElements.size());
			for (int i = 0; i < nCount; i++)
			{
				std::string str(arrElements[i].begin(), arrElements[i].end());
				mxml_node_t* pTempNode = ::mxmlFindElement(pFoundNode,
					pFoundNode, str.c_str(),
					NULL, NULL,
					MXML_DESCEND_FIRST);

				if (pTempNode == nullptr && bCreate)
				{
					pTempNode = ::mxmlNewElement(pFoundNode, str.c_str());
				}

				pFoundNode = pTempNode;
			}
		}
		else
		{
			pFoundNode = reinterpret_cast<mxml_node_t*>(hElement);
		}

		return pFoundNode;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// read string 
	std::wstring  ReadWriteXml::ReadString(HXMLELEMENT hElement, std::wstring lpszSubElementName, std::wstring lpszDefaultValue) const
	{
		if (hElement == nullptr)
			return L"";

		mxml_node_t* pFoundNode = reinterpret_cast<mxml_node_t*>(FindElement(hElement, lpszSubElementName));

		if (pFoundNode != nullptr)
		{
			return extractNodeValue(pFoundNode);
		}

		return lpszDefaultValue;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	int ReadWriteXml::ReadInteger(HXMLELEMENT hElement, std::wstring lpszSubElementName, int nDefaultValue) const
	{
		std::wstring  strValue = ReadString(hElement, lpszSubElementName, std::to_wstring(nDefaultValue));
		return std::stoi(strValue);
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// boolean 
	bool ReadWriteXml::ReadBool(HXMLELEMENT hElement, std::wstring lpszSubElementName, bool bValue) const
	{
		std::wstring  strDefault;
		strDefault = bValue ? L"Yes" : L"No";
		std::wstring  strValue = ReadString(hElement, lpszSubElementName, strDefault);
		return isTrueString(strValue);
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	double ReadWriteXml::ReadReal(HXMLELEMENT hElement, std::wstring lpszSubElementName, double dDefaultValue) const
	{
		std::wstring  strValue = ReadString(hElement, lpszSubElementName, std::to_wstring(dDefaultValue));
		return   std::stod(strValue);
	}

	bool xvt::ReadWriteXml::ReadBinary(HXMLELEMENT hElement, std::wstring lpszSubElementName, std::wstring &lpData) const
	{
		std::wstring  strValue = ReadString(hElement, lpszSubElementName, L"");
		if (strValue.empty() == FALSE)
			return GetBinary(strValue, lpData, strValue.length());
		else
			return false;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool ReadWriteXml::WriteString(HXMLELEMENT hElement, std::wstring lpszSubElementName, std::wstring lpszValue)
	{
		if (hElement == nullptr)
			return false;

		mxml_node_t* pFoundNode = reinterpret_cast<mxml_node_t*>(FindElement(hElement, lpszSubElementName, true));

		if (pFoundNode != nullptr)
		{
			std::string str(lpszValue.begin(), lpszValue.end());
			return writeNodeValue(pFoundNode, str);
		}

		return false;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// integer 
	bool ReadWriteXml::WriteInteger(HXMLELEMENT hElement, std::wstring lpszSubElementName, int nValue)
	{
		return WriteString(hElement, lpszSubElementName, std::to_wstring(nValue));
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// double 
	bool ReadWriteXml::WriteReal(HXMLELEMENT hElement, std::wstring lpszSubElementName, double dValue)
	{
		return WriteString(hElement, lpszSubElementName, std::to_wstring(dValue));
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// bool 
	bool ReadWriteXml::WriteBool(HXMLELEMENT hElement, std::wstring lpszSubElementName, bool bValue)
	{
		std::wstring  strValue;
		strValue = (bValue ==true) ? L"Yes" : L"No";
		return WriteString(hElement, lpszSubElementName, strValue);
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// binary 
	bool ReadWriteXml::WriteBinary(HXMLELEMENT hElement, std::wstring lpszSubElementName, std::wstring &lpData, int dwDataLength)
	{
		// embedding
		if (dwDataLength <= EXTERNAL_BINARY_MINIMUM_LENGTH || GetXmlBinaryStoreType() == E_SimpleStorage)
		{
			std::wstring  strBase64Value = XVT_EncodeBase64(lpData.c_str());
			return WriteString(hElement, lpszSubElementName, strBase64Value);
			
		}
		else if (GetXmlBinaryStoreType() == E_ComplexStorage || GetXmlBinaryStoreType() == E_ComplexStorageWithCompression)
		{
			std::wstring  strExternalFilePath;
			mxml_node_t* pNode = reinterpret_cast<mxml_node_t*>(hElement);
			while (pNode->parent != nullptr)
			{
				if (pNode->type == MXML_ELEMENT)
				{
					strExternalFilePath += xvt::ToWString(pNode->value.element.name) ;
				}
	
				pNode = mxmlGetParent(pNode);
	
				if (pNode)
				{
					strExternalFilePath += (L".");
				}
			}
	
			const bool bCompressed = GetXmlBinaryStoreType() == E_ComplexStorageWithCompression;
	
			std::wstring  strFileName;
			if (SetExternalBinary(strExternalFilePath, lpData, dwDataLength, bCompressed, &strFileName))
			{
				
				if (WriteString(hElement, lpszSubElementName, BINIARY_TAG + strFileName))
				{
					// set original length
					std::wstring  strLength;
					strLength = std::to_wstring(dwDataLength);
					SetAttribute(hElement, ATT_LENGTH, strLength);
	
					// set data type
					if (bCompressed)
					{
						SetAttribute(hElement, ATT_TYPE, ATT_VALUE_COMPRESSED);
					}
				}
			}
		}
	
		return false;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Get Attribute
	std::wstring  ReadWriteXml::GetAttribute(HXMLELEMENT hElement, std::wstring lpszAttributeName, std::wstring lpszDefaultValue)  const
	{
		if (hElement == nullptr)
			return lpszDefaultValue;

		mxml_node_t* pNode = reinterpret_cast<mxml_node_t*>(hElement);
		const char* pszValue = ::mxmlElementGetAttr(pNode,xvt::ToString(lpszAttributeName).c_str());

		if (pszValue)
		{
			std::string  strValue = pszValue;
			std::wstring strVal(strValue.begin(), strValue.end());
			return strVal;
		}

		return lpszDefaultValue;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Set Attribute
	bool ReadWriteXml::SetAttribute(HXMLELEMENT hElement, std::wstring lpszAttributeName, std::wstring lpszValue)
	{
		if (hElement == nullptr)
			return false;

		mxml_node_t* pNode = reinterpret_cast<mxml_node_t*>(hElement);

		std::string strName(lpszAttributeName.begin(), lpszAttributeName.end());
		std::string strVal(lpszValue.begin(), lpszValue.end());

		::mxmlElementSetAttr(pNode, strName.c_str(), strVal.c_str());

		return true;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// remove attribute
	bool ReadWriteXml::RemoveAttribute(HXMLELEMENT hElement, std::wstring lpszAttributeName)
	{
		if (hElement == nullptr)
			return false;

		mxml_node_t* pNode = reinterpret_cast<mxml_node_t*>(hElement);

		std::string strVal(lpszAttributeName.begin(), lpszAttributeName.end());
		::mxmlElementDeleteAttr(pNode, strVal.c_str());

		return true;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	std::wstring ReadWriteXml::ReadAttributeString(HXMLELEMENT hElement, std::wstring lpszAttributeName, std::wstring lpszDefaultValue) const
	{
		if (hElement == nullptr)
			return lpszDefaultValue;

		mxml_node_t* pNode = reinterpret_cast<mxml_node_t*>(hElement);
		std::string strName(lpszAttributeName.begin(), lpszAttributeName.end());
		const char* pszValue = ::mxmlElementGetAttr(pNode, strName.c_str());

		if (pszValue)
		{
			std::string  strValue = pszValue;
			std::wstring strVal(strValue.begin(), strValue.end());
			return strVal;
		}

		return lpszDefaultValue;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	int ReadWriteXml::ReadAttributeInteger(HXMLELEMENT hElement, std::wstring lpszSubElementName, int nDefaultValue) const
	{
		if (hElement == nullptr)
			return nDefaultValue;

		std::wstring  strValue = ReadAttributeString(hElement, lpszSubElementName, std::to_wstring(nDefaultValue));
		return std::stoi(strValue);
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	double ReadWriteXml::ReadAttributeReal(HXMLELEMENT hElement, std::wstring lpszSubElementName, double dDefaultValue) const
	{
		if (hElement == nullptr)
			return dDefaultValue;

		std::wstring  strValue = ReadAttributeString(hElement, lpszSubElementName, std::to_wstring(dDefaultValue));
		return std::stod(strValue);
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	float ReadWriteXml::ReadAttributeFloat(HXMLELEMENT hElement, std::wstring lpszSubElementName, float fDefaultValue) const
	{
		if (hElement == nullptr)
			return fDefaultValue;

		std::wstring  strValue = ReadAttributeString(hElement, lpszSubElementName, std::to_wstring(fDefaultValue));
		return std::stof(strValue);
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	bool ReadWriteXml::WriteAttributeString(HXMLELEMENT hElement, std::wstring lpszAttributeName, std::wstring lpszValue)
	{
		if (hElement == nullptr)
			return false;

		mxml_node_t* pNode = reinterpret_cast<mxml_node_t*>(hElement);

		std::string strName(lpszAttributeName.begin(), lpszAttributeName.end());
		std::string strVal(lpszValue.begin(), lpszValue.end());
		::mxmlElementSetAttr(pNode, strName.c_str(), strVal.c_str());

		return true;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	bool ReadWriteXml::WriteAttributeInteger(HXMLELEMENT hElement, std::wstring lpszSubElementName, int nValue)
	{
		if (hElement == nullptr)
			return false;
		WriteAttributeString(hElement, lpszSubElementName, std::to_wstring(nValue));

		return true;
	}

	bool ReadWriteXml::WriteAttributeBool(HXMLELEMENT hElement, std::wstring lpszSubElementName, bool bValue)
	{
		if (hElement == nullptr)
			return false;

		std::wstring  strValue;
		if (bValue)
		{
			strValue = L"true";
		}
		else
		{
			strValue = L"false";
		}
		WriteAttributeString(hElement, lpszSubElementName, strValue);

		return true;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	bool ReadWriteXml::WriteAttributeReal(HXMLELEMENT hElement, std::wstring lpszSubElementName, double dValue, bool bNoTrailZero)
	{
		if (hElement == nullptr)
			return false;
		WriteAttributeString(hElement, lpszSubElementName, std::to_wstring(dValue));

		return true;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	bool ReadWriteXml::WriteAttributeFloat(HXMLELEMENT hElement, std::wstring lpszSubElementName, float fValue, bool bNoTrailZero)
	{
		if (hElement == nullptr)
			return false;
		WriteAttributeString(hElement, lpszSubElementName, std::to_wstring(fValue));

		return true;
	}
}