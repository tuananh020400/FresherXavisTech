#pragma once
#include "pch.h"
const char COMMA_SEPARATOR = ',';

const std::string STRING_EMPTY                    = "";
const std::wstring WSTRING_EMPTY                  = L"";

const std::string STRING_SPACE                    = "   	";
const std::wstring WSTRING_SPACE                  = L"   	";

const std::string STRING_ESCAPE_SEQUENCE          = "\a\b\f\n\r\t\v\'\"\?\001\x12\\";
const std::string STRING_UPPERCASE                = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const std::string STRING_LOWERCASE                = "abcdefghijklmnopqrstuvwxyz";
const std::string STRING_NUMBER                   = "0123456789";
const std::string STRING_SPECIAL                  = "!\"#$%&'()*+,-./:;<=>?@[\\]^_{|}~";

const std::string STRING_ASCII                    = STRING_UPPERCASE + STRING_LOWERCASE + STRING_NUMBER + STRING_SPECIAL + STRING_ESCAPE_SEQUENCE;
 
const std::string STRING_ASCII_SPACE              = STRING_SPACE + STRING_ASCII + STRING_SPACE;
const std::string STRING_ASCII_LEADING_SPACE      = STRING_SPACE + STRING_ASCII;
const std::string STRING_ASCII_TRAILING_SPACE     = STRING_ASCII + STRING_SPACE;

const std::wstring WSTRING_ESCAPE_SEQUENCE        = L"\a\b\f\n\r\t\v\'\"\?\001\x12\\";
const std::wstring WSTRING_UPPERCASE              = L"ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const std::wstring WSTRING_LOWERCASE              = L"abcdefghijklmnopqrstuvwxyz";
const std::wstring WSTRING_NUMBER                 = L"0123456789";
const std::wstring WSTRING_SPECIAL_ASCII          = L"!\"#$%&'()*+,-./:;<=>?@[\\]^_{|}~";
const std::wstring WSTRING_SPECIAL_UNICODE        = L"!\"#$%&'()*+,-./:;<=>?@[\\]^_{|}~¡¢£¤¥¦§¨©ª«¬®¯°±²³´µ¶·¸¹º»¼½¾¿ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞßàáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ";
const std::wstring SYMBOL                         = L"🌟🍎🚀";
const std::wstring OTHERS_LANG                    = L"준비한";

const std::wstring WSTRING_ASCII                  = WSTRING_UPPERCASE + WSTRING_LOWERCASE + WSTRING_NUMBER + WSTRING_SPECIAL_ASCII + WSTRING_ESCAPE_SEQUENCE;
const std::wstring WSTRING_UNICODE                = WSTRING_UPPERCASE + WSTRING_LOWERCASE + WSTRING_NUMBER + WSTRING_SPECIAL_UNICODE + SYMBOL + OTHERS_LANG + WSTRING_ESCAPE_SEQUENCE;

const std::wstring WSTRING_UNICODE_SPACE          = WSTRING_SPACE + WSTRING_UNICODE + WSTRING_SPACE;
const std::wstring WSTRING_UNICODE_LEADING_SPACE  = WSTRING_SPACE + WSTRING_UNICODE;
const std::wstring WSTRING_UNICODE_TRAILING_SPACE = WSTRING_UNICODE + WSTRING_SPACE;

