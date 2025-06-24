#pragma once
#include "xvtCV/xvtDefine.h"
#include <opencv2/core/cvdef.h>
#include <string>
#include <vector>
#include <sstream>

namespace xvt
{
//! @addtogroup Converting
//! @{

/**
 * @name Angle Converting
 * @{
*/

//! Convert from Radian to Degree
inline constexpr
double Rad2Deg(double rad)
{
    return (180.0 / CV_PI) * rad;
}

//! Convert from Degree to Radian
inline constexpr
double Deg2Rad(double deg)
{
    return (CV_PI / 180.0) * deg;
}

/**
 * @} end of Angle Converting
*/

/**
 * @name Convert to string 
 * @{
*/

/**
 * @brief Convert from std::string to std::wstring representation 
 * @param value input std::string
 * @return std::wstring
*/
XVT_EXPORTS
auto ToWString(std::string value)->std::wstring;

/**
 * @brief Convert from std::wstring to std::string representation 
 * @param value input std::wstring
 * @return std::string
*/
XVT_EXPORTS
auto ToString(std::wstring value)->std::string;

/**
 * @overload
 * @brief Convert a int number to string representation 
 * @param a int Value
 * @return string value
*/
inline
auto ToString(int a)->std::string
{
    return std::to_string(a);
}

/**
 * @overload
 * @brief Convert a float number to string representation 
 * @param value a float Value
 * @return string value
*/
inline
auto ToString(float value)->std::string
{
    return std::to_string(value);
}

/**
 * @overload
 * @brief Convert a double number to string representation 
 * @param value a double Value
 * @return string value
*/
inline
auto ToString(double value)->std::string
{
    return std::to_string(value);
}

/**
* @overload
* @brief It is needed when using in template funtion
*/
inline
auto ToString(std::string const& value)->std::string const&
{
    return value;
}

/**
 * @overload
 * @brief It is needed when using in template funtion
*/
inline
auto ToString(std::string& value)->std::string&
{
    return value;
}

/**
 * @overload
 * @brief It is needed when using in template funtion
*/
inline
auto ToString(std::string&& value)->std::string
{
    return value;
}

/**
 * @brief Convert a double to a string with a specified precision.
 *
 * This function converts a double value to a string representation with a given number
 * of decimal places (precision). The precision determines the number of digits after
 * the decimal point.
 *
 * @param a The double value to convert to a string.
 * @param precision The number of decimal places (precision) to include in the string representation.
 *
 * @return std::string The string representation of the double value with the specified precision.
 *
 * @note If precision is set to a negative value, the default behavior of the underlying
 *       stringstream formatting will be used, which could vary depending on the system.
 */
inline
auto ToString(const double a, int precision)->std::string
{
    std::ostringstream oss;
    if (precision <= 0)
    {
        oss << (int)std::round(a);
    }
    else
    {
        if (precision > 6) precision = 6;
        int power = std::pow(10, precision); // Calculate the power of 10
        double value = std::round(a * power) / power;
        oss.precision(precision);
        oss << std::fixed << value;
    }
    return std::move(oss).str();
}

/**
 * @} end of Convert to string
*/

/**
 * @name String Handling
 * @{
*/

/**
 * @brief Convert to lower case
*/
XVT_EXPORTS
auto ToLowerCase(std::string s)->std::string;

/**
 * @brief Convert to lower case
*/
XVT_EXPORTS
auto ToLowerCase(std::wstring s)->std::wstring;

/**
 * @brief Remove the space at the begining and ending of a string
*/
XVT_EXPORTS
auto TrimSpace (const std::string& s)->std::string;

/**
 * @brief Remove the space at the begining and ending of a string
*/
XVT_EXPORTS
auto TrimSpace(const std::wstring& s)->std::wstring;

/**
 * @brief Remove the space at the begining of a string
*/
XVT_EXPORTS
auto TrimLeadingSpace(std::string const& str)->std::string;

/**
 * @brief Remove the space at the begining of a string
*/
XVT_EXPORTS
auto TrimLeadingSpace(std::wstring const& str)->std::wstring;

/**
 * @brief Remove the space at the ending of a string
*/
XVT_EXPORTS
auto TrimTrailingSpace(std::string const& str)->std::string;

/**
 * @brief Remove the space at the ending of a string
*/
XVT_EXPORTS
auto TrimTrailingSpace(std::wstring const& str)->std::wstring;

/**
 * @brief Split a string by a specified separator into a list of substrings.
 *
 * This function takes a string and splits it into a vector of substrings wherever the
 * specified separator character is found. The default separator is a comma (`,`).
 *
 * @param str The input string to be split.
 * @param separator The character used to separate the string into substrings. Default is `,`.
 *
 * @return std::vector<std::string> A vector of substrings resulting from the split operation.
 *
 * @note If the separator character is not found in the string, the entire string is returned
 *       as a single element in the resulting vector.
 * 
 * **Examples**  
 * How to use the function
 * @code
 * std::string input = "apple,banana,grape";
 * auto result = Split(input);  // result: {"apple", "banana", "grape"}
 *
 * input = "apple|banana|grape";
 * auto result2 = Split(input, '|');  // result: {"apple", "banana", "grape"}
 * @endcode
 */
XVT_EXPORTS
auto Split(std::string const& str, char separator = ',')->std::vector<std::string>;

/**
 * @overload 
*/
XVT_EXPORTS
auto Split(std::wstring const& str, char separator = ',')->std::vector<std::wstring>;

/**
 * @} end of String Handling
*/

/**
 * @name Convert from string representation to type
 * @{
*/

/// Convert value from string representation to a type
template<typename T>
inline
auto To_(const std::wstring& str) -> T = delete;

/// Convert value from string representation to a std::wstring
template<>
inline
auto To_<std::wstring>(const std::wstring& str)->std::wstring
{
    return str;
}

/// Convert value from string representation to a const wchar_t
template<>
inline
auto To_<const wchar_t*>(const std::wstring& str)->const wchar_t*
{
    return str.c_str();
}

/// Convert value from string representation to a std::string
template<>
inline
auto To_<std::string>(const std::wstring& str)->std::string
{
    return xvt::ToString(str);
}

/// Convert value from string representation to int type
template<>
inline
auto To_<int>(const std::wstring& str)->int
{
    return std::stoi(str);
}

/// Convert value from string representation to a long
template<>
inline
auto To_<long>(const std::wstring& str)->long
{
    return std::stol(str);
}

/// Convert value from string representation to float
template<>
inline
auto To_<float>(const std::wstring& str)->float
{
    return std::stof(str);
}

/// Convert value from string representation to double
template<>
inline
auto To_<double>(const std::wstring& str)->double
{
    return std::stod(str);
}

/// Convert value from string representation to long double
template<>
inline
auto To_<long double>(const std::wstring& str)->long double
{
    return std::stold(str);
}

/// Convert value from string representation to bool
template<>
inline
auto To_<bool>(const std::wstring& str)->bool
{
    return (bool)std::stoi(str);
}

/// Convert vector of value to vector of string representation
template<typename T>
auto To_(std::vector<T> const& data)->std::vector<std::string>
{
    using namespace xvt;
    std::vector<std::string> vec;
    for (auto&& d : data)
        vec.emplace_back(ToString(d));
    return vec;
}

/// Convert vector of pair key, value to vector of  pair key and string representation
template<typename T>
auto To_(std::vector<std::pair<std::string, T>> const& data, std::vector<std::pair<std::string, std::string>>& out, std::string prefix="")->void
{
    using namespace xvt;
    size_t targetCapacity = out.size() + data.size() * 2;
    if (out.capacity() < targetCapacity)
        out.reserve(targetCapacity);

    if(prefix.empty())
    {
        for (auto&& d : data)
            out.emplace_back(d.first, ToString(d.second));
    }
    else
    {
        for (auto&& d : data)
            out.emplace_back(prefix + d.first, ToString(d.second));
    }
}

/**
 * @} end of Convert from string representation to type
*/

//! @} end of group Converting

}