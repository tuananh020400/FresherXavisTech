#pragma once
#include "xvtCV/xvtDefine.h"
#include <chrono>
#include <string>
#include <iostream>
#include <iomanip>

namespace xvt {
//! @addtogroup Utils
//! @{

/**
 * @brief A utility class for measuring the time duration of code execution.
 *
 * ScopeTimer automatically starts when an instance is created and stops when
 * the instance goes out of scope (destructor called). It provides methods for
 * calculating elapsed time between intervals, as well as the total elapsed time.
 *
 * This class is useful for performance profiling by measuring how long specific
 * sections of code take to execute.  
 * **Examples**
 * @code
 * void Func()
 * {
 *    // Create an instace of ScopeTimer t, it will automatically start the timer and print the 
 *    // elapsed time when goes out of function.
 *    ScopeTimer t("Func");
 *    //do somthing
 * }
 * @endcode
 */
class ScopeTimer
{
public:
    using Miliseconds = std::chrono::duration<double, std::milli>; //!< Type alias for milliseconds representation
    using Clock = std::chrono::steady_clock; //!< Type alias for steady clock
    using TimePoint = Clock::time_point; //!< Type alias for time points

    /**
     * @brief Constructs a ScopeTimer and starts the timer.
     *
     * The timer starts upon instantiation and stores the provided name as a wide string.
     *
     * @param str Name associated with the timer (in UTF-8 format).
     */
    ScopeTimer(std::string const& str)
    {
        mName = std::wstring(str.begin(), str.end());
        Start();
    }

    /**
     * @brief Constructs a ScopeTimer and starts the timer.
     *
     * The timer starts upon instantiation and stores the provided name as a wide string.
     *
     * @param str Name associated with the timer (in wide string format).
     */
    ScopeTimer(std::wstring const& str)
    {
        mName = str;
        Start();
    }

    /**
    * @brief Destructor that stops the timer if it's still running.
    *
    * The destructor automatically stops the timer and prints the total time unless
    * it was already stopped manually.
    */
    ~ScopeTimer()
    {
        if (!mIsStopped) Stop();
    }

    /**
    * @brief Starts the timer.
    *
    * Resets the start time to the current time and initializes other internal state.
    *
    * @return TimePoint The start time of the timer.
    */
    inline TimePoint Start()
    {
        mStart = Clock::now();
        mStopPoint = mStart;
        mIsStopped = false;
        return mStart;
    }

    /**
     * @brief Gets the elapsed time since the last checkpoint or start.
     *
     * This method measures the time between the current call and the last time the timer was started or a checkpoint was made.
     * Optionally, the elapsed time is displayed along with a sub-name to distinguish multiple measurements.
     *
     * @param subName Optional sub-name to identify the measurement (default is an empty string).
     * @param isDisplay Whether to display the time or not (default is true).
     * @return Miliseconds The elapsed time in milliseconds.
     */
    inline Miliseconds GetElapsedTime(std::wstring subName = L"", bool isDisplay=true)
    {
        if (!mIsStopped)
        {
            mEnd = Clock::now();
            mStopCount++;
        }
        auto res = std::chrono::duration_cast<Miliseconds>(mEnd - mStopPoint);
        mStopPoint = mEnd;

        PrintTime(res, subName, isDisplay);

        return res;
    }

    /**
     * @brief Gets the total elapsed time since the timer was started.
     *
     * Measures the time from when the timer was started to the current call.
     * Optionally, the total elapsed time is displayed along with a sub-name.
     *
     * @param subName Optional sub-name to identify the measurement (default is an empty string).
     * @param isDisplay Whether to display the time or not (default is true).
     * @return Miliseconds The total elapsed time in milliseconds.
     */
    inline Miliseconds GetTotalElapsedTime(std::wstring subName = L"", bool isDisplay=true)
    {
        if (!mIsStopped)
        {
            mEnd = Clock::now();
        }

        auto res = std::chrono::duration_cast<Miliseconds>(mEnd - mStart);
        PrintTime(res, subName, isDisplay);
        return res;
    }

    /**
     * @brief Stops the timer and optionally displays the total elapsed time.
     *
     * The total time between the timer's start and stop is measured and returned.
     * The timer is marked as stopped, and no further measurements will be made unless restarted.
     *
     * @param isDisplay Whether to display the time or not (default is true).
     * @return Miliseconds The total elapsed time in milliseconds.
     */
    inline Miliseconds Stop(bool isDisplay = true)
    {
        auto res = GetTotalElapsedTime(L"", isDisplay);
        mIsStopped = true;
        return   res;
    }

    // Deleting copy and move constructors/assignment operators to ensure unique ownership.
    ScopeTimer(const ScopeTimer&) = delete;
    ScopeTimer(ScopeTimer&&) = delete;
    ScopeTimer& operator=(const ScopeTimer&) = delete;
    ScopeTimer& operator=(ScopeTimer&&) = delete;

private:
    /**
     * @brief Prints the elapsed time in milliseconds.
     *
     * Outputs the elapsed time along with the timer's name and optional sub-name.
     * This method handles the display logic depending on the build configuration (console or debug).
     *
     * @param res The elapsed time to be printed (in milliseconds).
     * @param subName An optional sub-name to distinguish between different checkpoints.
     * @param isDisplay Whether to actually display the time (default is true).
     */
    inline void PrintTime(Miliseconds res, std::wstring subName = L"", bool isDisplay=true)
    {
        if(isDisplay)
        {
            subName = subName.empty() ? L":" : L"-" + subName + L":";
            std::wstring str = mStopCount > 0 ? mName + std::to_wstring(mStopCount) + subName : mName + subName;
            std::wstring str2 = (str + std::to_wstring(res.count()) + L"ms\n");

        #ifndef _AFX
            // Outputs to console for non-MFC builds.
            std::wcout << str2;
        #else
            // Outputs to the debug console for MFC builds.
            ::OutputDebugString(CString(str2.c_str()));
        #endif // !MFC
        }
    }

private:
    bool mIsStopped = true; //!< Whether the timer has been stopped.
    int  mStopCount = 0;    //!< The number of times a checkpoint has been measured.
    TimePoint mStart;       //!< The time when the timer started.
    TimePoint mEnd;         //!< The last time when a checkpoint or stop was called.
    TimePoint mStopPoint;   //!< The last checkpoint for elapsed time measurement.
    std::wstring mName;     //!< Name of the timer for identification in output.
};

/**
 * @brief Get the current time formatted as a string.
 *
 * This function retrieves the current time and formats it according to the provided
 * format string. It uses the standard C++ time formatting specifiers.
 *
 * Common format specifiers:
 * - `%F` -> Date in the form of `YYYY-MM-DD` (e.g., 2000-11-01).
 * - `%T` -> Time in the form of `HH:MM:SS` (e.g., 14:10:43).
 *
 * If no format is provided, the default format is `%F %T` which corresponds to
 * `YYYY-MM-DD HH:MM:SS`.
 *
 * @param format Optional. The format string that defines how the current time should be formatted.
 *               Default value is `"%F %T"`.
 *
 * @return std::string The current time formatted according to the specified format.
 *
 * @note The function uses the C++ standard time library to retrieve the system time.
 * **Examples**
 * @code
 * std::string currentTime   = GetCurrentTime();                       // Output: 2023-07-14 16:23:54
 * std::string customFormat1 = GetCurrentTime("%Y/%m/%d %I:%M:%S %p"); // Output: 2023/07/14 04:23:54 PM
 * std::string customFormat2 = GetCurrentTime("%Y%m%d_%HH%MM%SS");     // Output: 20230714_162354
 * @endcode
 */
XVT_EXPORTS
auto GetCurrentTime(std::string format="%F %T")->std::string;

/**
 * @brief Write performance timing information to a log.
 *
 * This function logs performance data including the file name, line number, function name,
 * and the elapsed time (in milliseconds). The function can be used to trace and log the
 * performance of different parts of the program for profiling purposes.
 *
 * @param srcFileName The source file name where the performance logging is being triggered.
 * @param line The line number where the performance logging is being triggered.
 * @param funcName The name of the function being logged for performance.
 * @param time The time (in milliseconds) that represents the duration or performance metric to log.
 *
 * @note Ensure that the logging mechanism is properly set up to handle performance data.  
 * 
 * **Examples**  
 * @code
 * WritePerformance(L"main.cpp", 42, L"main", 10.5f);  // Logs performance at line 42 in main()
 * //Another way to automatic add the file name and line number, function name
 * WritePerformance(std::wstring(__FILEW__), __LINE__, __FUNCTIONW__, 10.5f);
 * @endcode
 */
XVT_EXPORTS
void WritePerformance(std::wstring const& srcFileName, int line, std::wstring const& funcName, float time);

template<typename T, typename... Types>
void TestPerformance(std::wstring srcFileName, int line, std::wstring funcName, T funcT, Types... args)
{

    xvt::ScopeTimer timer(funcName);
    timer.Start();
    funcT(args...);
    auto t = timer.Stop();
    WritePerformance(srcFileName, line, funcName, t.count());
    return;
}

class XVT_EXPORTS PerfomanceTest: ScopeTimer
{
public:
    PerfomanceTest(std::wstring srcFileName, int line, std::wstring funcName) : ScopeTimer(funcName)
        , mSrcFileName{ srcFileName }
        , mLine{ line }
        , mFuncName{ funcName }
    {
        Start();
    }

    ~PerfomanceTest()
    {
        auto t = Stop();
        WritePerformance(mSrcFileName, mLine, mFuncName, t.count());
    }

private:
    std::wstring mSrcFileName;
    int mLine;
    std::wstring mFuncName;
};

#define XVT_PERF_FUNCNAME_FUNC( name ) L#name, name
#define XVT_PERF_TEMPNAME1(x) __xvt_Perf_temp##x
#define XVT_PERF_TEMPNAME(x) XVT_PERF_TEMPNAME1(x)
#ifdef __PERFORMANCE_ANALYSIS__
#define XVT_MESUARE_PERFORMANCE_F(func, ...) xvt::TestPerformance(std::wstring(__FILEW__), __LINE__, XVT_PERF_FUNCNAME_FUNC(func), __VA_ARGS__)
#define XVT_MESUARE_PERFORMANCE() xvt::PerfomanceTest XVT_PERF_TEMPNAME(__LINE__)(std::wstring(__FILEW__), __LINE__, __FUNCTIONW__)
#else
#define XVT_MESUARE_PERFORMANCE_F(func, ...) 
#define XVT_MESUARE_PERFORMANCE()
#endif // __PERFORMANCE_ANALYSIS__

//! @} end of group Utils

}
