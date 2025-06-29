#pragma once
#include "xvtCV/xvtDefine.h"
#include <string>
#include <vector>

namespace xvt {
//! @addtogroup Interface
//! @{

#define XVT_VERSION_MAJOR 0
#define XVT_VERSION_MINOR 0
#define XVT_VERSION_PATCH 0
#define XVT_VERSION_NUMBER 0
#define XVT_VERSION_COMIT 
#define XVT_VERSION_DIRTY 
#define XVT_BRANCH 

class VersionInfo;

/**
 * @brief Type alias for a list of constant pointers to VersionInfo objects.
 * 
 * You have to create and instace of VersionInfo object then push the reference to the vector.
 * Don't create dynamic object of VersionInfo and push it to vector. it can cause the
 * memory leakages.
 * 
 * @code
 * xvt::VersionInfo ainfo;
 * VersionInfoList list;
 * list.push_back(&ainfo);
 * @endcode
 */
using VersionInfoList = std::vector<xvt::VersionInfo const*>;

/**
 * @brief Class to handle version information and dependencies.
 *
 * The VersionInfo class is designed to store and provide version details such as major, minor,
 * patch numbers, commit, branch, and other information. It also handles dependencies on other modules.
 * 
 */
class XVTCV_EXPORTS VersionInfo
{
public:
    VersionInfo(  std::string name
                , std::string major
                , std::string minor
                , std::string patch
                , std::string number = ""
                , std::string commit = ""
                , std::string dirty  = ""
                , std::string branch = ""
                , VersionInfoList depend = VersionInfoList()
    )   : mName{ name }
        , mMajor{ major }, mMinor{ minor }, mPatch{ patch }
        , mNumber{ number }, mCommit{ commit }, mDirty{ dirty }, mBranch{branch}
        , mDependencies{ depend }
    {}

    /**
      * @brief Retrieves version information in a formatted string.
      *
      * This function will generate and return the version information of the current module
      * and optionally, if a list is provided, appends the version information of other modules to it.
      *
      * @param dependInfo Optional. Whether to get the dependencies information.
      * @return std::string A formatted string representing the version information.
      */
    auto GetVersionInfo(bool dependInfo = true)const->std::string;

    /**
     * @brief Retrieves dependency information in a formatted string.
     *
     * This function will generate and return the information of any module dependencies,
     * if they exist. Optionally, appends the dependencies' version information to the provided list.
     *
     * @param dependInfo Optional. Whether to get the dependencies information.
     * @return std::string A formatted string representing the dependency information.
     */
    auto GetDependenciesInfo(bool dependInfo = true)const->std::string;

    std::string mName;      //<! Module or project name
    std::string mMajor;     //<! Major number
    std::string mMinor;     //<! Minor number
    std::string mPatch;     //<! Patch version number
    std::string mNumber;    //<! Build number or additional version number
    std::string mCommit;    //<! Commit hash of the current version in version control (e.g., Git).
    std::string mDirty;     //<! Indicator if the version has uncommitted changes
    std::string mBranch;    //<! Branch name in version control system

    /**
     * @brief List of dependencies for the current module.
     *
     * This contains pointers to VersionInfo objects of the dependencies required for this module.
     */
    VersionInfoList mDependencies{}; //<! Pointer to submodule dependencies

private:
    /**
     * @brief Retrieves version information in a formatted string.
     *
     * This function will generate and return the version information of the current module
     * and optionally, if a list is provided, appends the version information of other modules to it.
     *
     * @param list Optional. A pointer to a vector that will hold the version info of other modules, if provided.
     * @param dependInfo Optional. Whether to get the dependencies information.
     * @return std::string A formatted string representing the version information.
     */
    auto GetVersionInfo(VersionInfoList& list, bool dependInfo = true)const->std::string;

    /**
     * @brief Retrieves dependency information in a formatted string.
     *
     * This function will generate and return the information of any module dependencies,
     * if they exist. Optionally, appends the dependencies' version information to the provided list.
     *
     * @param list Optional. A pointer to a vector that will hold the version info of dependent modules, if provided.
     * @param dependInfo Optional. Whether to get the dependencies information.
     * @return std::string A formatted string representing the dependency information.
     */
    auto GetDependenciesInfo(VersionInfoList& list, bool dependInfo = true)const->std::string;
private:
    /**
     * @brief Flag to check if the version information has been printed.
     *
     * This mutable flag can be changed even in const functions to track if
     * the version info has already been printed.
     */
    mutable bool mIsPrinted = false;
};

/**
 * @brief Extern declaration of the xvtCV module version info.
 */
extern XVTCV_EXPORTS VersionInfo const xvtCV_VersionInfo;
/**
 * @brief Extern declaration of the OpenCV module version info.
 */
extern XVTCV_EXPORTS VersionInfo const opencv_VersionInfo;
/**
 * @brief Extern declaration of the zlib module version info.
 */
extern XVTCV_EXPORTS VersionInfo const zlib_VersionInfo;

/**@}*/ //end of group Interface

}
