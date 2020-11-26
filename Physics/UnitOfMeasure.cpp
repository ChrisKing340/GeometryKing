#include "UnitOfMeasure.h"

using namespace King;
using namespace King::UnitOfMeasure;
using namespace std;

// globals
const King::UnitOfMeasure::Accel King::UnitOfMeasure::gravity { 9.80665_mPerSecSq };
const King::UnitOfMeasure::Speed King::UnitOfMeasure::speedOfSoundInAir { 1118.0_ftPerSec };

// statics for one instance per class type
const string King::UnitOfMeasure::Mass::_unit = "kg"s;
const string King::UnitOfMeasure::Inertia::_unit = "kg m^2"s;
const string King::UnitOfMeasure::Angle::_unit = "rad"s;
const string King::UnitOfMeasure::Length::_unit = "m"s;
const string King::UnitOfMeasure::Area::_unit = "m^2"s;
const string King::UnitOfMeasure::Volume::_unit = "m^3"s;
const string King::UnitOfMeasure::Energy::_unit = "J"s;
const string King::UnitOfMeasure::Power::_unit = "w"s;
const string King::UnitOfMeasure::Strength::_unit = "N"s;
const string King::UnitOfMeasure::AngularStrength::_unit = "N m"s;
const string King::UnitOfMeasure::Accel::_unit = "m/s^2"s;
const string King::UnitOfMeasure::AngularAccel::_unit = "rad/s^2"s;
const string King::UnitOfMeasure::Speed::_unit = "m/s"s;
const string King::UnitOfMeasure::AngularSpeed::_unit = "rad/s"s;
const string King::UnitOfMeasure::Motion::_unit = "kg m/s"s;
const string King::UnitOfMeasure::AngularMotion::_unit = "kg rad/s"s;
const string King::UnitOfMeasure::Temperature::_unit = "K"s;
const string King::UnitOfMeasure::Time::_unit = "s"s;

const wstring King::UnitOfMeasure::Mass::_wunit = L"kg";
const wstring King::UnitOfMeasure::Inertia::_wunit = L"kg m^2"; // I = m * r^2
const wstring King::UnitOfMeasure::Angle::_wunit = L"rad";
const wstring King::UnitOfMeasure::Length::_wunit = L"m";
const wstring King::UnitOfMeasure::Area::_wunit = L"m^2";
const wstring King::UnitOfMeasure::Volume::_wunit = L"m^3";
const wstring King::UnitOfMeasure::Energy::_wunit = L"J";
const wstring King::UnitOfMeasure::Power::_wunit = L"w";
const wstring King::UnitOfMeasure::Strength::_wunit = L"N";
const wstring King::UnitOfMeasure::AngularStrength::_wunit = L"N m"; // r * F
const wstring King::UnitOfMeasure::Accel::_wunit = L"m/s^2"; // F = m * a
const wstring King::UnitOfMeasure::AngularAccel::_wunit = L"rad/s^2";  //  r * F = m * a
const wstring King::UnitOfMeasure::Speed::_wunit = L"m/s";
const wstring King::UnitOfMeasure::AngularSpeed::_wunit = L"rad/s";
const wstring King::UnitOfMeasure::Motion::_wunit = L"kg m/s";
const wstring King::UnitOfMeasure::AngularMotion::_wunit = L"kg rad/s";
const wstring King::UnitOfMeasure::Temperature::_wunit = L"K";
const wstring King::UnitOfMeasure::Time::_wunit = L"s";

// itermediate squared types
const string King::UnitOfMeasure::MassSq::_unit = "g^2"s;
const string King::UnitOfMeasure::AngleSq::_unit = "rad^2"s;
const string King::UnitOfMeasure::AreaSq::_unit = "m^4"s;
const string King::UnitOfMeasure::VolumeSq::_unit = "m^6"s;
const string King::UnitOfMeasure::EnergySq::_unit = "J^2"s;
const string King::UnitOfMeasure::PowerSq::_unit = "w^2"s;
const string King::UnitOfMeasure::StrengthSq::_unit = "N^2"s;
const string King::UnitOfMeasure::AccelSq::_unit = "m^2/s^4"s;
const string King::UnitOfMeasure::AngularAccelSq::_unit = "rad^2/s^4"s;
const string King::UnitOfMeasure::SpeedSq::_unit = "m^2/s^2"s;
const string King::UnitOfMeasure::AngularSpeedSq::_unit = "rad^2/s^2"s;
const string King::UnitOfMeasure::TemperatureSq::_unit = "K^2"s;
const string King::UnitOfMeasure::TimeSq::_unit = "s^2"s;

const wstring King::UnitOfMeasure::MassSq::_wunit = L"g^2";
const wstring King::UnitOfMeasure::AngleSq::_wunit = L"rad^2";
const wstring King::UnitOfMeasure::AreaSq::_wunit = L"m^4";
const wstring King::UnitOfMeasure::VolumeSq::_wunit = L"m^6";
const wstring King::UnitOfMeasure::EnergySq::_wunit = L"J^2";
const wstring King::UnitOfMeasure::PowerSq::_wunit = L"w^2";
const wstring King::UnitOfMeasure::StrengthSq::_wunit = L"N^2";
const wstring King::UnitOfMeasure::AccelSq::_wunit = L"m^2/s^4";
const wstring King::UnitOfMeasure::AngularAccelSq::_wunit = L"rad^2/s^4";
const wstring King::UnitOfMeasure::SpeedSq::_wunit = L"m^2/s^2";
const wstring King::UnitOfMeasure::AngularSpeedSq::_wunit = L"rad^2/s^2";
const wstring King::UnitOfMeasure::TemperatureSq::_wunit = L"K^2";
const wstring King::UnitOfMeasure::TimeSq::_wunit = L"s^2";

float King::UnitOfMeasure::AreaCrossSectionalSphereFtSq(const float &diameterFtIn) { return 0.785398163f * diameterFtIn * diameterFtIn; } // ft^2

/******************************************************************************
*   Mass
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Mass &in)
{
    return os << "{ Mass " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Mass &in)
{
    return os << L"{ Mass " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream & King::UnitOfMeasure::operator>> (std::istream &is, King::UnitOfMeasure::Mass &out)
{
    return is >> out._value; // binary in
}
std::wistream & King::UnitOfMeasure::operator>> (std::wistream &is, King::UnitOfMeasure::Mass &out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json & j, const King::UnitOfMeasure::Mass & from) { j = json{ {"Mass", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json & j, King::UnitOfMeasure::Mass & to) { j.at("Mass").get_to(to._value); }
// operators
UnitOfMeasure::MassSq King::UnitOfMeasure::operator*(const UnitOfMeasure::Mass& t1, const UnitOfMeasure::Mass& t2) { return MassSq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::Mass King::UnitOfMeasure::operator/(const UnitOfMeasure::MassSq& num, const UnitOfMeasure::Mass& dem) { return Mass(static_cast<float>(num) / static_cast<float>(dem)); }
UnitOfMeasure::Mass King::UnitOfMeasure::operator/(const UnitOfMeasure::Energy& num, const UnitOfMeasure::SpeedSq& dem) { return UnitOfMeasure::Mass(static_cast<float>(num) / static_cast<float>(dem)); }

/******************************************************************************
*   Angle
******************************************************************************/
// streams
std::ostream& King::UnitOfMeasure::operator<< (std::ostream& os, const King::UnitOfMeasure::Angle& in)
{
    return os << "{ Angle " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream& King::UnitOfMeasure::operator<< (std::wostream& os, const King::UnitOfMeasure::Angle& in)
{
    return os << L"{ Angle " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream& King::UnitOfMeasure::operator>> (std::istream& is, King::UnitOfMeasure::Angle& out)
{
    return is >> out._value; // binary in
}
std::wistream& King::UnitOfMeasure::operator>> (std::wistream& is, King::UnitOfMeasure::Angle& out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json& j, const King::UnitOfMeasure::Angle& from) { j = json{ {"Angle", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json& j, King::UnitOfMeasure::Angle& to) { j.at("Angle").get_to(to._value); }
// operators
Angle King::UnitOfMeasure::operator*(const Time& t, const AngularSpeed& s) { return Angle(static_cast<float>(t)* static_cast<float>(s)); }
Angle King::UnitOfMeasure::operator*(const AngularSpeed& s, const Time& t) { return Angle(static_cast<float>(s)* static_cast<float>(t)); }
AngleSq King::UnitOfMeasure::operator*(const Angle& in1, const Angle& in2)
{
    return AngleSq(static_cast<float>(in1)* static_cast<float>(in2));
}
/******************************************************************************
*   Length
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Length &in)
{
    return os << "{ Length " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Length &in)
{
    return os << L"{ Length " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream & King::UnitOfMeasure::operator>> (std::istream &is, King::UnitOfMeasure::Length &out)
{
    return is >> out._value; // binary in
}
std::wistream & King::UnitOfMeasure::operator>> (std::wistream &is, King::UnitOfMeasure::Length &out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json & j, const King::UnitOfMeasure::Length & from) { j = json{ {"Length", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json & j, King::UnitOfMeasure::Length & to) { j.at("Length").get_to(to._value); }
// operators
Length King::UnitOfMeasure::operator*(const Time& t, const Speed& s) { return Length(static_cast<float>(t) * static_cast<float>(s)); }
Length King::UnitOfMeasure::operator*(const Speed& s, const Time& t) { return Length(static_cast<float>(s) * static_cast<float>(t)); }
Length King::UnitOfMeasure::operator/(const UnitOfMeasure::Area& num, const UnitOfMeasure::Length& dem) { return Length(static_cast<float>(num) / static_cast<float>(dem)); }
Length King::UnitOfMeasure::operator/(const UnitOfMeasure::Volume& num, const UnitOfMeasure::Area& dem) { return Length(static_cast<float>(num) / static_cast<float>(dem)); }
Length King::UnitOfMeasure::operator/(const UnitOfMeasure::SpeedSq& num, const UnitOfMeasure::Accel& dem) { return Length(static_cast<float>(num) / static_cast<float>(dem)); }

/******************************************************************************
*   Area
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Area &in)
{
    return os << "{ Area " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Area &in)
{
    return os << L"{ Area " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream & King::UnitOfMeasure::operator>> (std::istream &is, King::UnitOfMeasure::Area &out)
{
    return is >> out._value; // binary in
}
std::wistream & King::UnitOfMeasure::operator>> (std::wistream &is, King::UnitOfMeasure::Area &out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json & j, const King::UnitOfMeasure::Area & from) { j = json{ {"Area", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json & j, King::UnitOfMeasure::Area & to) { j.at("Area").get_to(to._value); }
// operators
UnitOfMeasure::Area King::UnitOfMeasure::operator*(const Length& l1, const Length& l2) { return Area(static_cast<float>(l1) * static_cast<float>(l2)); }

UnitOfMeasure::AreaSq King::UnitOfMeasure::operator*(const UnitOfMeasure::Area& t1, const UnitOfMeasure::Area& t2) { return AreaSq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::Area King::UnitOfMeasure::operator/(const UnitOfMeasure::AreaSq& num, const UnitOfMeasure::Area& dem) { return Area(static_cast<float>(num) / static_cast<float>(dem)); }

/******************************************************************************
*   Volume
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Volume &in)
{
    return os << "{ Volume " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Volume &in)
{
    return os << L"{ Volume " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream & King::UnitOfMeasure::operator>> (std::istream &is, King::UnitOfMeasure::Volume &out)
{
    return is >> out._value; // binary in
}
std::wistream & King::UnitOfMeasure::operator>> (std::wistream &is, King::UnitOfMeasure::Volume &out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json & j, const King::UnitOfMeasure::Volume & from) { j = json{ {"Volume", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json & j, King::UnitOfMeasure::Volume & to) { j.at("Volume").get_to(to._value); }
// operators
Volume King::UnitOfMeasure::operator*(const Length& l, const Area& a) { return Volume(static_cast<float>(a) * static_cast<float>(l)); }
Volume King::UnitOfMeasure::operator*(const Area& a, const Length& l) { return Volume(static_cast<float>(l) * static_cast<float>(a)); }

UnitOfMeasure::VolumeSq King::UnitOfMeasure::operator*(const UnitOfMeasure::Volume& t1, const UnitOfMeasure::Volume& t2) { return VolumeSq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::Volume King::UnitOfMeasure::operator/(const UnitOfMeasure::VolumeSq& num, const UnitOfMeasure::Volume& dem) { return Volume(static_cast<float>(num) / static_cast<float>(dem)); }

/******************************************************************************
*   Energy
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Energy &in)
{
    return os << "{ Energy " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Energy &in)
{
    return os << L"{ Energy " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream & King::UnitOfMeasure::operator>> (std::istream &is, King::UnitOfMeasure::Energy &out)
{
    return is >> out._value; // binary in
}
std::wistream & King::UnitOfMeasure::operator>> (std::wistream &is, King::UnitOfMeasure::Energy &out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json & j, const King::UnitOfMeasure::Energy & from) { j = json{ {"Energy", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json & j, King::UnitOfMeasure::Energy & to) { j.at("Energy").get_to(to._value); }
// operators
UnitOfMeasure::EnergySq King::UnitOfMeasure::operator*(const UnitOfMeasure::Energy& t1, const UnitOfMeasure::Energy& t2) { return EnergySq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::Energy King::UnitOfMeasure::operator/(const UnitOfMeasure::EnergySq& num, const UnitOfMeasure::Energy& dem) { return Energy(static_cast<float>(num) / static_cast<float>(dem)); }
UnitOfMeasure::Energy King::UnitOfMeasure::operator*(const UnitOfMeasure::Mass& m, const UnitOfMeasure::SpeedSq& sSq) { return Energy(static_cast<float>(m) * static_cast<float>(sSq)) * 0.5f; }
//UnitOfMeasure::Energy King::UnitOfMeasure::operator*(const UnitOfMeasure::Inertia& I, const UnitOfMeasure::AngularSpeedSq& asSq) { return Energy(static_cast<float>(I) * static_cast<float>(asSq)) * 0.5f; }
UnitOfMeasure::Energy King::UnitOfMeasure::operator*(const UnitOfMeasure::AngularStrength& t1, const UnitOfMeasure::Angle& t2) { return Energy(static_cast<float>(t1)* static_cast<float>(t2)); } // angular work = τ*θ
UnitOfMeasure::Energy King::UnitOfMeasure::operator*(const UnitOfMeasure::Strength& t1, const UnitOfMeasure::Length& t2) { return Energy(static_cast<float>(t1)* static_cast<float>(t2)); } // work
/******************************************************************************
*   Power 
*       watts; 1 w = 1 J / s = Nm/s
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Power &in)
{
    return os << "{ Power " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Power &in)
{
    return os << L"{ Power " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream & King::UnitOfMeasure::operator>> (std::istream &is, King::UnitOfMeasure::Power &out)
{
    return is >> out._value; // binary in
}
std::wistream & King::UnitOfMeasure::operator>> (std::wistream &is, King::UnitOfMeasure::Power &out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json & j, const King::UnitOfMeasure::Power & from) { j = json{ {"Power", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json & j, King::UnitOfMeasure::Power & to) { j.at("Power").get_to(to._value); }
// operators
UnitOfMeasure::PowerSq King::UnitOfMeasure::operator*(const UnitOfMeasure::Power& t1, const UnitOfMeasure::Power& t2) { return PowerSq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::Power King::UnitOfMeasure::operator/(const UnitOfMeasure::PowerSq& num, const UnitOfMeasure::Power& dem) { return Power(static_cast<float>(num) / static_cast<float>(dem)); }
/******************************************************************************
*   Strength
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Strength &in)
{
    return os << "{ Strength " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Strength &in)
{
    return os << L"{ Strength " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream & King::UnitOfMeasure::operator>> (std::istream &is, King::UnitOfMeasure::Strength &out)
{
    return is >> out._value; // binary in
}
std::wistream & King::UnitOfMeasure::operator>> (std::wistream &is, King::UnitOfMeasure::Strength &out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json & j, const King::UnitOfMeasure::Strength & from) { j = json{ {"Strength", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json & j, King::UnitOfMeasure::Strength & to) { j.at("Strength").get_to(to._value); }
// operators
UnitOfMeasure::StrengthSq King::UnitOfMeasure::operator*(const UnitOfMeasure::Strength& t1, const UnitOfMeasure::Strength& t2) { return StrengthSq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::Strength King::UnitOfMeasure::operator/(const UnitOfMeasure::StrengthSq& num, const UnitOfMeasure::Strength& dem) { return Strength(static_cast<float>(num) / static_cast<float>(dem)); }
/******************************************************************************
*   AngularStrength
******************************************************************************/
// streams
std::ostream& King::UnitOfMeasure::operator<< (std::ostream& os, const King::UnitOfMeasure::AngularStrength& in)
{
    return os << "{ AngularStrength " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream& King::UnitOfMeasure::operator<< (std::wostream& os, const King::UnitOfMeasure::AngularStrength& in)
{
    return os << L"{ AngularStrength " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream& King::UnitOfMeasure::operator>> (std::istream& is, King::UnitOfMeasure::AngularStrength& out)
{
    return is >> out._value; // binary in
}
std::wistream& King::UnitOfMeasure::operator>> (std::wistream& is, King::UnitOfMeasure::AngularStrength& out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json& j, const King::UnitOfMeasure::AngularStrength& from) { j = json{ {"AngularStrength", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json& j, King::UnitOfMeasure::AngularStrength& to) { j.at("AngularStrength").get_to(to._value); }
// operators
// first two below conflict with work = Force * Distance
//UnitOfMeasure::AngularStrength King::UnitOfMeasure::operator*(const UnitOfMeasure::Strength& s, const UnitOfMeasure::Length& l) { return AngularStrength(static_cast<float>(s) * static_cast<float>(l)); }
//UnitOfMeasure::AngularStrength King::UnitOfMeasure::operator*(const UnitOfMeasure::Length& l, const UnitOfMeasure::Strength& s) { return AngularStrength(static_cast<float>(s) * static_cast<float>(l)); }
UnitOfMeasure::Length King::UnitOfMeasure::operator/(const UnitOfMeasure::AngularStrength& num, const UnitOfMeasure::Strength& dem) { return Length(static_cast<float>(num) / static_cast<float>(dem)); }
UnitOfMeasure::Strength King::UnitOfMeasure::operator/(const UnitOfMeasure::AngularStrength& num, const UnitOfMeasure::Length& dem) { return Strength(static_cast<float>(num) / static_cast<float>(dem)); }

/******************************************************************************
*   Accel
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Accel &in)
{
    return os << "{ Accel " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Accel &in)
{
    return os << L"{ Accel " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream & King::UnitOfMeasure::operator>> (std::istream &is, King::UnitOfMeasure::Accel &out)
{
    return is >> out._value; // binary in
}
std::wistream & King::UnitOfMeasure::operator>> (std::wistream &is, King::UnitOfMeasure::Accel &out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json & j, const King::UnitOfMeasure::Accel & from) { j = json{ {"Accel", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json & j, King::UnitOfMeasure::Accel & to) { j.at("Accel").get_to(to._value); }
// operators
UnitOfMeasure::AccelSq King::UnitOfMeasure::operator*(const UnitOfMeasure::Accel& t1, const UnitOfMeasure::Accel& t2) { return AccelSq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::Accel King::UnitOfMeasure::operator/(const UnitOfMeasure::AccelSq& num, const UnitOfMeasure::Accel& dem) { return Accel(static_cast<float>(num) / static_cast<float>(dem)); }
UnitOfMeasure::Accel King::UnitOfMeasure::operator/(const UnitOfMeasure::Length& num, const UnitOfMeasure::TimeSq& dem) { return Accel(static_cast<float>(num) / static_cast<float>(dem)); }

/******************************************************************************
*   AngularAccel
******************************************************************************/
// streams
std::ostream& King::UnitOfMeasure::operator<< (std::ostream& os, const King::UnitOfMeasure::AngularAccel& in)
{
    return os << "{ AngularAccel " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream& King::UnitOfMeasure::operator<< (std::wostream& os, const King::UnitOfMeasure::AngularAccel& in)
{
    return os << L"{ AngularAccel " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream& King::UnitOfMeasure::operator>> (std::istream& is, King::UnitOfMeasure::AngularAccel& out)
{
    return is >> out._value; // binary in
}
std::wistream& King::UnitOfMeasure::operator>> (std::wistream& is, King::UnitOfMeasure::AngularAccel& out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json& j, const King::UnitOfMeasure::AngularAccel& from) { j = json{ {"AngularAccel", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json& j, King::UnitOfMeasure::AngularAccel& to) { j.at("AngularAccel").get_to(to._value); }
// operators
UnitOfMeasure::AngularAccelSq King::UnitOfMeasure::operator*(const UnitOfMeasure::AngularAccel& t1, const UnitOfMeasure::AngularAccel& t2) { return AngularAccelSq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::AngularAccel King::UnitOfMeasure::operator/(const UnitOfMeasure::AngularAccelSq& num, const UnitOfMeasure::AngularAccel& dem) { return AngularAccel(static_cast<float>(num) / static_cast<float>(dem)); }
UnitOfMeasure::AngularAccel King::UnitOfMeasure::operator/(const UnitOfMeasure::Angle& num, const UnitOfMeasure::TimeSq& dem) { return AngularAccel(static_cast<float>(num) / static_cast<float>(dem)); } // rad/s^2

/******************************************************************************
*   AngularSpeed
******************************************************************************/
// streams
std::ostream& King::UnitOfMeasure::operator<< (std::ostream& os, const King::UnitOfMeasure::AngularSpeed& in)
{
    return os << "{ AngularSpeed " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream& King::UnitOfMeasure::operator<< (std::wostream& os, const King::UnitOfMeasure::AngularSpeed& in)
{
    return os << L"{ AngularSpeed " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream& King::UnitOfMeasure::operator>> (std::istream& is, King::UnitOfMeasure::AngularSpeed& out)
{
    return is >> out._value; // binary in
}
std::wistream& King::UnitOfMeasure::operator>> (std::wistream& is, King::UnitOfMeasure::AngularSpeed& out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json& j, const King::UnitOfMeasure::AngularSpeed& from) { j = json{ {"AngularSpeed", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json& j, King::UnitOfMeasure::AngularSpeed& to) { j.at("AngularSpeed").get_to(to._value); }
// operators
UnitOfMeasure::AngularSpeed King::UnitOfMeasure::operator*(const Time& t, const AngularAccel& a) { return AngularSpeed(static_cast<float>(t)* static_cast<float>(a)); }
UnitOfMeasure::AngularSpeed King::UnitOfMeasure::operator*(const AngularAccel& a, const Time& t) { return AngularSpeed(static_cast<float>(a)* static_cast<float>(t)); }
UnitOfMeasure::AngularSpeedSq King::UnitOfMeasure::operator*(const UnitOfMeasure::AngularSpeed& t1, const UnitOfMeasure::AngularSpeed& t2) { return AngularSpeedSq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::AngularSpeed King::UnitOfMeasure::operator/(const UnitOfMeasure::AngularSpeedSq& num, const UnitOfMeasure::AngularSpeed& dem) { return AngularSpeed(static_cast<float>(num) / static_cast<float>(dem)); }
UnitOfMeasure::AngularSpeed King::UnitOfMeasure::operator/(const UnitOfMeasure::Angle& num, const UnitOfMeasure::Time& dem) { return AngularSpeed(static_cast<float>(num) / static_cast<float>(dem)); } // rad/s
/******************************************************************************
*   Motion
******************************************************************************/
// streams
std::ostream& King::UnitOfMeasure::operator<< (std::ostream& os, const King::UnitOfMeasure::Motion& in)
{
    return os << "{ Motion " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream& King::UnitOfMeasure::operator<< (std::wostream& os, const King::UnitOfMeasure::Motion& in)
{
    return os << L"{ Motion " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream& King::UnitOfMeasure::operator>> (std::istream& is, King::UnitOfMeasure::Motion& out)
{
    return is >> out._value; // binary in
}
std::wistream& King::UnitOfMeasure::operator>> (std::wistream& is, King::UnitOfMeasure::Motion& out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json& j, const King::UnitOfMeasure::Motion& from) { j = json{ {"Motion", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json& j, King::UnitOfMeasure::Motion& to) { j.at("Motion").get_to(to._value); }
// operators
UnitOfMeasure::Motion King::UnitOfMeasure::operator*(const UnitOfMeasure::Speed& v, const UnitOfMeasure::Mass& m)
{
    return UnitOfMeasure::Motion(static_cast<float>(v) * static_cast<float>(m));
}

UnitOfMeasure::Motion King::UnitOfMeasure::operator*(const UnitOfMeasure::Mass& m, const UnitOfMeasure::Speed& v)
{
    return UnitOfMeasure::Motion(static_cast<float>(v) * static_cast<float>(m));
}
/******************************************************************************
*   AngularMotion
******************************************************************************/
// streams
std::ostream& King::UnitOfMeasure::operator<< (std::ostream& os, const King::UnitOfMeasure::AngularMotion& in)
{
    return os << "{ AngularMotion " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream& King::UnitOfMeasure::operator<< (std::wostream& os, const King::UnitOfMeasure::AngularMotion& in)
{
    return os << L"{ AngularMotion " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream& King::UnitOfMeasure::operator>> (std::istream& is, King::UnitOfMeasure::AngularMotion& out)
{
    return is >> out._value; // binary in
}
std::wistream& King::UnitOfMeasure::operator>> (std::wistream& is, King::UnitOfMeasure::AngularMotion& out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json& j, const King::UnitOfMeasure::AngularMotion& from) { j = json{ {"AngularMotion", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json& j, King::UnitOfMeasure::AngularMotion& to) { j.at("AngularMotion").get_to(to._value); }
// operators
UnitOfMeasure::AngularMotion King::UnitOfMeasure::operator*(const UnitOfMeasure::AngularSpeed& v, const UnitOfMeasure::Inertia& I)
{
    return UnitOfMeasure::AngularMotion(static_cast<float>(v)* static_cast<float>(I));
}

UnitOfMeasure::AngularMotion King::UnitOfMeasure::operator*(const UnitOfMeasure::Inertia& I, const UnitOfMeasure::AngularSpeed& v)
{
    return UnitOfMeasure::AngularMotion(static_cast<float>(v)* static_cast<float>(I));
}

/******************************************************************************
*   Speed
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Speed &in)
{
    return os << "{ Speed " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Speed &in)
{
    return os << L"{ Speed " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream & King::UnitOfMeasure::operator>> (std::istream &is, King::UnitOfMeasure::Speed &out)
{
    return is >> out._value; // binary in
}
std::wistream & King::UnitOfMeasure::operator>> (std::wistream &is, King::UnitOfMeasure::Speed &out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json & j, const King::UnitOfMeasure::Speed & from) { j = json{ {"Speed", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json & j, King::UnitOfMeasure::Speed & to) { j.at("Speed").get_to(to._value); }
// operators
UnitOfMeasure::Speed King::UnitOfMeasure::operator*(const Time& t, const Accel& a) { return Speed(static_cast<float>(t) * static_cast<float>(a)); }
UnitOfMeasure::Speed King::UnitOfMeasure::operator*(const Accel& a, const Time& t) { return Speed(static_cast<float>(a) * static_cast<float>(t)); }
UnitOfMeasure::SpeedSq King::UnitOfMeasure::operator*(const UnitOfMeasure::Speed& t1, const UnitOfMeasure::Speed& t2) { return SpeedSq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::SpeedSq King::UnitOfMeasure::operator/(const UnitOfMeasure::Energy& num, const UnitOfMeasure::Mass& dem) { return UnitOfMeasure::SpeedSq(static_cast<float>(num) / static_cast<float>(dem)); }
UnitOfMeasure::Speed King::UnitOfMeasure::operator/(const UnitOfMeasure::SpeedSq& num, const UnitOfMeasure::Speed& dem) { return Speed(static_cast<float>(num) / static_cast<float>(dem)); }

/******************************************************************************
*   Temperature
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Temperature &in)
{
    return os << "{ Temperature " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Temperature &in)
{
    return os << L"{ Temperature " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream & King::UnitOfMeasure::operator>> (std::istream &is, King::UnitOfMeasure::Temperature &out)
{
    return is >> out._value; // binary in
}
std::wistream & King::UnitOfMeasure::operator>> (std::wistream &is, King::UnitOfMeasure::Temperature &out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json & j, const King::UnitOfMeasure::Temperature & from) { j = json{ {"Temperature", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json & j, King::UnitOfMeasure::Temperature & to) { j.at("Temperature").get_to(to._value); }
// operators
UnitOfMeasure::TemperatureSq King::UnitOfMeasure::operator*(const UnitOfMeasure::Temperature& t1, const UnitOfMeasure::Temperature& t2) { return TemperatureSq(static_cast<float>(t1)* static_cast<float>(t2)); }
UnitOfMeasure::Temperature King::UnitOfMeasure::operator/(const UnitOfMeasure::TemperatureSq& num, const UnitOfMeasure::Temperature& dem) { return Temperature(static_cast<float>(num) / static_cast<float>(dem)); }

/******************************************************************************
*   Time
******************************************************************************/
// streams
std::ostream& King::UnitOfMeasure::operator<< (std::ostream& os, const King::UnitOfMeasure::Time& in)
{
    return os << "{ Time " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream& King::UnitOfMeasure::operator<< (std::wostream& os, const King::UnitOfMeasure::Time& in)
{
    return os << L"{ Time " << in._value << L" " << in.UnitW() << L" }"; // text out
}
std::istream& King::UnitOfMeasure::operator>> (std::istream& is, King::UnitOfMeasure::Time& out)
{
    return is >> out._value; // binary in
}
std::wistream& King::UnitOfMeasure::operator>> (std::wistream& is, King::UnitOfMeasure::Time& out)
{
    return is >> out._value; // binary in
}
// json
void King::UnitOfMeasure::to_json(json& j, const King::UnitOfMeasure::Time& from) { j = json{ {"Time", from._value}, {"U", from._unit} }; }
void King::UnitOfMeasure::from_json(const json& j, King::UnitOfMeasure::Time& to) { j.at("Time").get_to(to._value); }
// operators
UnitOfMeasure::Time King::UnitOfMeasure::operator/(const UnitOfMeasure::Speed& s, const UnitOfMeasure::Accel& a) { return Time(static_cast<float>(s) / static_cast<float>(a)); }

UnitOfMeasure::TimeSq King::UnitOfMeasure::operator*(const UnitOfMeasure::Time& t1, const UnitOfMeasure::Time& t2) { return TimeSq(static_cast<float>(t1) * static_cast<float>(t2)); }
UnitOfMeasure::Time King::UnitOfMeasure::operator/(const UnitOfMeasure::TimeSq& num, const UnitOfMeasure::Time& dem) { return Time(static_cast<float>(num) / static_cast<float>(dem)); }