#include "UnitOfMeasure.h"

using namespace King;
using namespace King::UnitOfMeasure;
using namespace std;

// globals
const King::UnitOfMeasure::Accel gravity { 9.80665_mPerSecSq };
const King::UnitOfMeasure::Speed speedOfSoundInAir { 1118.0_ftPerSec };

// statics for one instance per class type
const string King::UnitOfMeasure::Mass::_unit = "g"s;
const string King::UnitOfMeasure::Length::_unit = "m"s;
const string King::UnitOfMeasure::Area::_unit = "m^2"s;
const string King::UnitOfMeasure::Volume::_unit = "m^3"s;
const string King::UnitOfMeasure::Energy::_unit = "J"s;
const string King::UnitOfMeasure::Power::_unit = "w"s;
const string King::UnitOfMeasure::Strength::_unit = "N"s;
const string King::UnitOfMeasure::Accel::_unit = "m/s^2"s;
const string King::UnitOfMeasure::Speed::_unit = "m/s"s;
const string King::UnitOfMeasure::Temperature::_unit = "K"s;
const string King::UnitOfMeasure::Time::_unit = "s"s;

const wstring King::UnitOfMeasure::Mass::_wunit = L"g";
const wstring King::UnitOfMeasure::Length::_wunit = L"m";
const wstring King::UnitOfMeasure::Area::_wunit = L"m^2";
const wstring King::UnitOfMeasure::Volume::_wunit = L"m^3";
const wstring King::UnitOfMeasure::Energy::_wunit = L"J";
const wstring King::UnitOfMeasure::Power::_wunit = L"w";
const wstring King::UnitOfMeasure::Strength::_wunit = L"N";
const wstring King::UnitOfMeasure::Accel::_wunit = L"m/s^2";
const wstring King::UnitOfMeasure::Speed::_wunit = L"m/s";
const wstring King::UnitOfMeasure::Temperature::_wunit = L"K";
const wstring King::UnitOfMeasure::Time::_wunit = L"s";

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

/******************************************************************************
*   Force
******************************************************************************/
// streams
std::ostream & King::UnitOfMeasure::operator<< (std::ostream &os, const King::UnitOfMeasure::Strength &in)
{
    return os << "{ Force " << in._value << " " << in.Unit() << " }"; // text out
}
std::wostream & King::UnitOfMeasure::operator<< (std::wostream &os, const King::UnitOfMeasure::Strength &in)
{
    return os << L"{ Force " << in._value << L" " << in.UnitW() << L" }"; // text out
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

// Math Functions
Speed King::UnitOfMeasure::operator*(const Time &t, const Accel & a) { return Speed(static_cast<float>(t) * static_cast<float>(a)); }
Speed King::UnitOfMeasure::operator*(const Accel & a, const Time &t) { return Speed(static_cast<float>(a) * static_cast<float>(t)); }
Length King::UnitOfMeasure::operator*(const Time &t, const Speed & s) { return Length(static_cast<float>(t) * static_cast<float>(s)); }
Length King::UnitOfMeasure::operator*(const Speed & s, const Time &t) { return Length(static_cast<float>(s) * static_cast<float>(t)); }
Area King::UnitOfMeasure::operator*(const Length &l1, const Length & l2) { return Area(static_cast<float>(l1) * static_cast<float>(l2)); }
Volume King::UnitOfMeasure::operator*(const Length &l, const Area & a) { return Volume(static_cast<float>(a) * static_cast<float>(l)); }
Volume King::UnitOfMeasure::operator*(const Area & a, const Length &l) { return Volume(static_cast<float>(l) * static_cast<float>(a)); }

