#include "xvtCV/xvtbase64.h"
#include <tchar.h>
#include <vector>


namespace xvt 
{
	namespace base64
	{
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Base64 Encoding Table 
		const std::wstring B64chars = L"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// encode from binary to base64

		std::wstring EncodeBase64(std::wstring inputData)
		{
			std::vector<unsigned char> v(inputData.begin(), inputData.end());

			std::wstring result;
			size_t len = inputData.length();
			unsigned char* p = v.data();
			size_t j = 0, pad = len % 3;
			size_t last = len - pad;

			for (size_t i = 0; i < last; i += 3) {
				int n = (p[i] << 16) | (p[i + 1] << 8) | p[i + 2];
				result.push_back(B64chars[n >> 18]);
				result.push_back(B64chars[(n >> 12) & 0x3F]);
				result.push_back(B64chars[(n >> 6) & 0x3F]);
				result.push_back(B64chars[n & 0x3F]);
			}

			if (pad) {
				int n = (pad == 1 ? p[last] << 8 : (p[last] << 8) | p[last + 1]);
				result.push_back(B64chars[pad ? n >> 10 : n >> 2]);
				result.push_back(B64chars[pad ? (n >> 4) & 0x03F : (n << 4) & 0x3F]);
				result.push_back(pad ? B64chars[(n << 2) & 0x3F] : '=');
				result.push_back('=');
			}

			return result;
		}

		std::wstring DecodeBase64(std::wstring inputData)
		{
			size_t len = inputData.length();
			if (len == 0 || (len % 4 != 0)) return L"";  // Base64 string length must be a multiple of 4
			std::vector<unsigned char> result;
			size_t i = 0;

			while (i < len)
			{
				unsigned char a = B64chars.find(inputData[i++]);
				unsigned char b = B64chars.find(inputData[i++]);
				unsigned char c = B64chars.find(inputData[i++]);
				unsigned char d = B64chars.find(inputData[i++]);

				unsigned int n = (a << 18) | (b << 12) | (c << 6) | d;

				result.push_back((n >> 16) & 0xFF);
				if (c != 64) 
				{
					result.push_back((n >> 8) & 0xFF);
					if (d != 64)
					{
						result.push_back(n & 0xFF);
					}
				}
			}
			std::string astr(reinterpret_cast<char*>(&result[0]), result.size());

			return std::wstring(astr.begin(), astr.end());
		}

	}
}