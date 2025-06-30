#pragma once
#include <string>
#include <xvtCV/xvtEnumTemplate.h>
#include <xvtCV/xvtProperty.h>
#include "xvtBattery/CTBatteryInspectorUpper.h"
#include "xvtBattery/CTBatteryInspectorLower.h"

enum class CylinderType
{
    PROJECT_2D      = 1<<1,
    PROJECT_2D_JR   = 1<<2,
    SLICE_3D        = 1<<3,
    SLICE_CT        = SLICE_3D,
};
DEFINE_CONVERT_ENUM_FUNCS(CylinderType);

namespace xvt {

template<>
class TypedProperty<xvt::battery::BatteryInspectorBase> : public PropertyList
{
public:
    using Type_Trait = xvt::battery::BatteryInspectorBase;
    using Type_Trait_Ref = std::reference_wrapper<Type_Trait>;
    explicit TypedProperty(Type_Trait& data, double &pixelSize);
    explicit TypedProperty(Type_Trait&& data) = delete;

private:
    std::wstring mName;
    Type_Trait_Ref mData;
    Type_Trait mDefaultValue;
};

template<>
class TypedProperty<xvt::battery::BatteryInspectorBeading> : public PropertyList
{
public:
    using Type_Trait = xvt::battery::BatteryInspectorBeading;
    using Type_Trait_Ref = std::reference_wrapper<Type_Trait>;
    explicit TypedProperty(Type_Trait& data, double& pixelSize);
    explicit TypedProperty(Type_Trait&& data) = delete;

private:
    std::wstring mName;
    Type_Trait_Ref mData;
    Type_Trait mDefaultValue;
};

template<>
class TypedProperty<xvt::battery::BatteryInspectorJR> : public PropertyList
{
public:
    using Type_Trait = xvt::battery::BatteryInspectorJR;
    using Type_Trait_Ref = std::reference_wrapper<Type_Trait>;
    explicit TypedProperty(Type_Trait& data, double& pixelSize);
    explicit TypedProperty(Type_Trait&& data) = delete;

private:
    std::wstring mName;
    Type_Trait_Ref mData;
    Type_Trait mDefaultValue;
};

template<>
class TypedProperty<xvt::battery::BatteryInspectorPole> : public PropertyList
{
public:
    using Type_Trait = xvt::battery::BatteryInspectorPole;
    using Type_Trait_Ref = std::reference_wrapper<Type_Trait>;
    explicit TypedProperty(Type_Trait& data, double& pixelSize);
    explicit TypedProperty(Type_Trait&& data) = delete;

private:
    std::wstring mName;
    Type_Trait_Ref mData;
    Type_Trait mDefaultValue;
};

template<>
class TypedProperty<xvt::battery::BatteryInspectorCathode> : public PropertyList
{
public:
    using Type_Trait = xvt::battery::BatteryInspectorCathode;
    using Type_Trait_Ref = std::reference_wrapper<Type_Trait>;
    explicit TypedProperty(Type_Trait& data);
    explicit TypedProperty(Type_Trait&& data) = delete;

private:
    std::wstring mName;
    Type_Trait_Ref mData;
};

template<>
class TypedProperty<xvt::battery::BatteryInspectorAnode> : public PropertyList
{
public:
    using Type_Trait = xvt::battery::BatteryInspectorAnode;
    using Type_Trait_Ref = std::reference_wrapper<Type_Trait>;
    explicit TypedProperty(Type_Trait& data);
    explicit TypedProperty(Type_Trait&& data) = delete;

private:
    std::wstring mName;
    Type_Trait_Ref mData;
};

template<>
class XVT_EXPORTS TypedProperty<xvt::battery::CTBatteryInspector> : public PropertyList
{
public:
    using Type_Trait = xvt::battery::CTBatteryInspector;
    using Type_Trait_Ref = std::reference_wrapper<Type_Trait>;
    explicit TypedProperty(Type_Trait& data);
    explicit TypedProperty(Type_Trait&& data) = delete;

private:
    std::wstring mName;
    Type_Trait_Ref mData;
    Type_Trait mDefaultValue;
};

using CTPropeties = TypedProperty<xvt::battery::CTBatteryInspector>;

#if 0
template<>
class XVT_EXPORTS TypedProperty<xvt::battery::CTBatteryInspectorUpper> : public PropertyList
{
public:
    using Type_Trait = xvt::battery::CTBatteryInspectorUpper;
    using Type_Trait_Ref = std::reference_wrapper<Type_Trait>;
    explicit TypedProperty(Type_Trait& data);
    explicit TypedProperty(Type_Trait&& data) = delete;

private:
    std::wstring mName;
    Type_Trait_Ref mData;
    Type_Trait mDefaultValue;
};

template<>
class XVT_EXPORTS TypedProperty<xvt::battery::CTBatteryInspectorLower> : public PropertyList
{
public:
    using Type_Trait = xvt::battery::CTBatteryInspectorLower;
    using Type_Trait_Ref = std::reference_wrapper<Type_Trait>;
    explicit TypedProperty(Type_Trait& data);
    explicit TypedProperty(Type_Trait&& data) = delete;

private:
    std::wstring mName;
    Type_Trait_Ref mData;
    Type_Trait mDefaultValue;
};

using CTPropetiesUpper = TypedProperty<xvt::battery::CTBatteryInspectorUpper>;
using CTPropetiesLower = TypedProperty<xvt::battery::CTBatteryInspectorLower>;
#endif

}//namespace xvt