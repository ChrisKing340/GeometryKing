/******************************************************************************
MIT License

Copyright (c) 2019 and 2020 Christopher H. King

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

HISTORY:

RELEASE VERSION MAJOR 1.0:
Constants for our physics modeling thus defines our game world interactions.
Not used in the GameObject class much but is applied heavily in our
MovableObject class that manages forces and momentum.  Conversions are
provide to convert from SI unit to English standard units.  As a
mechanical engineer I had to learn both systems and non-engineers I work
with are unable to calculate and relate to both SI and English units in their 
daily work (horse power, lbs force, etc).  

RELEASE VERSION MAJOR 2.0:
1) Added classes for each type of major measurement that make use of constexpr
and literals so that units of measure can be included when defining the 
measurement and automatically converted to the internal storage unit (SI).
This allows more systems of measurements that rely on formulas and not just
multiplication factors (e.g. temperature). Works great for common types (like 
time) where second is the internal and minutes or hours can be litteraled on 
input.
2) Added json support for data serialization and transfer of class data with
unit of measures.

RELEASE VERSION MINOR 2.1:
Functions for recognizing unit type and returning the correct change in type.
(ex. Length * Length = Area)

RELEASE VERSION MINOR 2.2:
Additional types added for square values typicaly used as intermediate
values in calculations.

RELEASE VERSION MAJOR 3.0:
Completed custom data types in support of a full physics simulation of ridgid 
bodies in motion.  That code realized performance increase when converting 
(ex: Velocity, Momentum, etc.) to this code base since those types keep
a scalar using the classes here plus a unit vector.  The engine is no longer
calculating a scalar several times (as magnitude) in different parts of the 
code per time step.  Work I started as error prevention with units has 
unexpectedly improved our performance.
1) Conversions factors added for momentum
2) Classes added to support linear and angular momentum.
3) Classes added to support angular motion
******************************************************************************/
#pragma once

#ifndef __cplusplus
#error UnitOfMeasure requires C++
#endif

#if defined(_MSC_VER) && (_MSC_VER < 1920)
#error requires Visual C++ 2019 or later.
#endif

#define KING_UNITOFMEASURE_VERSION_MAJOR 3
#define KING_UNITOFMEASURE_VERSION_MINOR 0
#define KING_UNITOFMEASURE_VERSION_PATCH 0

#include <iomanip>

#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

namespace King {
namespace UnitOfMeasure
{
    // Forwards
    class Mass; // scalar
    class Length; // scalar
    class Area; // scalar
    class Volume; // scalar
    class Energy; // scalar
    class Power; // scalar
    class Strength; // scalar part of force
    class Accel; // scalar part of acceleration
    class Speed; // scalar part of velocity
    class Motion; // scalar part of momentum
    class Temperature; // scalar
    class Time; // scalar

    class Inertia; // scalar
    class Angle; // scalar in radians
    class AngularStrength; // scalar part of torqe
    class AngularAccel; // scalar part of angular acceleration in radians
    class AngularSpeed; // scalar part of angular velocity in radians
    class AngularMotion; // scalar part of angular momentum in radians

    extern const Accel gravity;
    extern const Speed speedOfSoundInAir;

    /******************************************************************************
    * Conversion factors for both SI (Standard International) 
    * and EN (English Standard) measurements
    ******************************************************************************/
    /*
        SI System
        Measurement         Unit                Abbreviation
        Length              meter                   m
        Mass                kilogram                kg
        Abs.Temperature     Kelvin                  K
        Rel.Temperature     degree Celcius          °C
        Volume              liter                   L
        Time                second                  s
        Force               Newton                  N
        Energy              Joule                   J
        Power               watt                    w
    */
    /*
        EN System
        Measurement         Unit                Abbreviation
        Length              foot                    ft
        Mass                pound mass              lbm
        Abs.Temperature     degree Rankin           °R
        Rel.Temperature     degree Fahrenheit       °F
        Volume              gallon                  gal
        Time                second                  s
        Force               pound force             lbf
        Energy              British thermal unit    BTU
        Power               horse power             hp
    */
    /*
        British Engineering System
        Measurement         Unit                Abbreviation
        Mass                slug                    slug ; mass that is accelerated by 1 ft/s2 when a force of one pound (lbf) is exerted on it.
    */
    
    // Gravity
    const float gravityEN = 32.174049f; // ft / s^2
    const float gravitySI = 9.80665f; // m / s^2
    // English pound
    // the pound, lb, does not describe if it is mass or force.  This term is used for both, which makes english units ambiguous.  We must specify which (by adding m or f)
    // 1 lbf = F = m * a = 32.174049 lbm * 1 (ft / s^2); thus, 1 lbf is the force of one unit of gravitational force applied to lbm
    // 1 lbm = 1 lbf / (32.174049 * (ft / s ^ 2)) = (1 lbf * s ^ 2) / (32.174049 ft) =  slug / 32.174049; 32.174049 * lbm * ft / s^2 = lbf
    // conversions when lbm also multiplied with Ft/(s^2 lbf) lbm to lbf.  That happens when we are multiplying with gravity
    const float lbmFtPerSecSqTolbf = 1.0f / gravityEN; // lbf
    const float lbfTolbmFtPerSecSq = gravityEN; // lbm * ft / s^2 ; m = w/g 
    const float gravityENToSI = gravitySI / gravityEN;
    // MASS
    const float lbm = 1.0f;
    const float slug = 1.0f;
    const float kg = 1.0f;
    const float kgTog = 1000.0f;
    const float kgTolbm = 2.2046226218488f; // lbm
    const float lbmTokg = 1.0f / kgTolbm; // kg
    const float slugTolbfSecSqPerFt = 1.0f; // 1 slug = 1 lbf * s^2 / ft
    const float slugTolbm = gravityEN; // (lbf * s^2 / ft) * (32.174049)(ft / s^2) * (lbm/lbf) = 32.174049 lbm = 1 slug
    const float lbmToslug = 1.0f / gravityEN; // 32.174049 lbm = 1 slug
    const float lbmTolbfSecSqPerFt = 1.0f / gravityEN * 1.0f; // 32.174049 * lbm * ft / s^2 = lbf ; and, work = lbf * ft = 32.174049 * lbm * ft^2 / s^2 
    const float kgToslug = kgTolbm * lbmToslug; // slug
    const float slugTokg = 1.0f / kgToslug; // slug
    const float tonsTolbm = 2000.f; // lbm
    const float lbmTotons = 1.f / tonsTolbm; // tons
    const float tonsToslug = tonsTolbm * lbmToslug; // slug
    const float tonsTokg = tonsTolbm * lbmTokg; // slug
    // ANGLE
    const float PI = 3.141592f; // highest precision for 32 bit floating point value
    const float degTorad = PI / 180.f; // rad
    const float radTodeg = 180.f / PI; // deg  
    // DENSITY
    const float kgPerMeterCubedTolbmPerFtCubed = kgTolbm / 35.3147f; // lbm / ft^3
    // FORCE
    const float lbf = 1.0f;
    const float N = 1.0f; // kg * m / s^2 = definition of a Newton, N
    const float lbfToN = lbmTokg * gravitySI; // 4.44822 N
    const float Ntolbf = 1.0f / lbfToN; // lbf
    const float lbfTolbm = 1.0f; // lbm ; l lbm * (1 slug / 32.174049 lbm) * 32.174049 ft/s^2 = 1 lbf ; F = m * g = slug * ft/s^2 = lbf ; weight lbm = lbf
    const float lbmTolbf = 1.0f; // lbf
    // LENGTH
    const float ft = 1.0f;
    const float m = 1.0f;
    const float kmTom = 1000.0f;
    const float cmToft = 0.032808399f; // ft
    const float mToft = 3.2808399f; // ft
    const float ftTom = 1.0f / mToft; // m
    const float mileToFeet = 5280.f; // ft
    const float ftTomile = 1.0f / mileToFeet; // miles
    const float mileTom = mileToFeet * ftTom; // m
    const float nmTom = 1852.f; // m
    const float mTonm = 1.0f / nmTom;// nautical mile
    const float nmToft = nmTom * mToft; // ft; (1 nm ~= 6076 ft)
    const float ftTonm = 1.0f / nmToft; // nautical mile
    // VELOCITY
    const float ftPerSec = 1.0f; // ft/s
    const float mPerSec = 1.0f; // m/s
    const float mPerSecToftPerSec = 3.281f; // ft/s
    const float ftPerSecTomPerSec = 1.0f / mPerSecToftPerSec; // m/s
    const float machToftPerSec = 1116.0f; // ft/s
    const float mphToftPerSec = 1.4667f; // ft/s
    const float ftPerSecTomph = 1.0f / mphToftPerSec; // mph
    const float mphTokph = 1.60934f; // kph
    const float kphTomph = 1.f / mphTokph; // mph
    // ACCELERATION
    const float mPerSecSq = 1.0f; // m/s^2
    const float ftPerSecSq = 1.0f; // ft/s^2
    const float mPerSecSqToftPerSecSq = 3.281f; // ft/s^2
    // MOMENTUM
    const float kgMPerSec = 1.0f; // kg⋅m/s for SI linear momentum
    const float slugFtPerSec = 1.0f; // slug⋅ft/s for EN linear momentum
    const float kgMPerSecToslugFtPerSec = kgToslug * mToft; // slug⋅ft/s
    const float slugFtPerSecTokgMPerSec = slugTokg * ftTom; // kg⋅m/s 
    const float kgMSqPerSec = 1.0f; // kg⋅m^2/s for SI angular momentum
    const float slugFtSqPerSec = 1.0f; // slug⋅ft^2/s for EN angular momentum
    const float kgMSqPerSecToslugFtSqPerSec = kgToslug * mToft * mToft; // slug⋅ft^2/s
    const float slugFtSqPerSecTokgMSqPerSec = slugTokg * ftTom * ftTom; // kg⋅m^2/s
    // ENERGY
    const float Btu = 1.0f;
    const float therm = 1.0f;
    const float J = 1.0f;
    const float BtuTotherm = 1.f / 100000.f * therm / Btu; // @ 59 deg F
    const float thermToBtu = 1.f / BtuTotherm;
    const float MBtuToBtu = 1000.f * Btu;
    const float BtuToMBtu = 1.f / MBtuToBtu;
    const float MMBtuToBtu = 1000.f * 1000.f * Btu;
    const float BtuToMMBtu = 1.f / MMBtuToBtu;
    const float BtuTolbfFt = 778.16f; // lbf * ft
    const float lbfFtToBtu = 1.f / BtuTolbfFt; // Btu
    const float lbfFtToJ = 1.3558f; // J
    const float lbfFtTolbmSecSqPerSecSq = gravityEN;// work = lbf * ft = 32.174049 * lbm * ft^2 / s^2 
    const float BtuToJ = BtuTolbfFt * lbfFtToJ * J / Btu; // J
    const float JtoBtu = 1.f / BtuToJ;
    const float JToNm = 1.f; // N * m
    const float NmToJ = 1.f / JToNm; // J
    // POWER (energy per unit time)
    const float hp = 1.0f;
    const float kw = 1.0f;
    const float hpTolbfFtPerSec = 550.000037f; // lbf * ft / s = 1 HP = 550 lb * ft * 1 rad / 1 sec; note power is force * velocity 
    const float hpTolbfFtRotationsPerSec = hpTolbfFtPerSec / 6.283185307f; // or as torque * rpm (when not is straight line)
    const float hpTolbfFtRpm = hpTolbfFtRotationsPerSec * 60.f;// 1 HP = (550 lb * ft * 1 rad / 1 sec) * (1 rev / (2 * PI rad)) * 60 sec / 1 min) = 5252 lb * ft * 1 rev / 1 min  = 5252 lb * ft * rpm
    const float lbfFtPerSecTohp = 1.f / hpTolbfFtPerSec; // hp
    const float hpTokw = 0.7457f; // kw
    const float kwTohp = 1.f / hpTokw; // hp
    const float JPerSecTokw = 1.0f / 1000.f; // 1 w = 1 J / s
    const float kwToJPerSec = 1000.f; // J / s
    const float hpToBtuPerH = hpTolbfFtPerSec / BtuTolbfFt * 60.f * 60.f; // 2545 Btu / h
    const float BtuPerHTohp = 1.0f / hpToBtuPerH; // hp
    // TIME
    const float hrTosec = 360.f;
    const float hrTomin = 60.f;
    const float minTosec = 60.f;
    const float secTohr = 1.f / hrTosec;
    const float minTohr = 1.f / hrTomin;
    const float secTomin = 1.f / minTosec;
    // PHYSICAL PROPERTIES
    float AreaCrossSectionalSphereFtSq(const float &diameterFtIn);

    namespace DensityEN { // slug/ft^3 
        const float Helium = 0.179f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
        const float Air = 0.0765f * lbmToslug;
        const float Wood = 700.0f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
        const float Ice = 916.0f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
        const float Water = 62.3f * lbmToslug;
        const float Plastic = 1175.0f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
        const float Concrete = 2000.0f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
        const float Aluminium = 2700.0f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
        const float Iron = 7870.0f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
        const float Copper = 8940.0f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
        const float Silver = 10500.0f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
        const float Lead = 11340.0f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
        const float Gold = 19320.0f * kgPerMeterCubedTolbmPerFtCubed * lbmToslug;
    }
    namespace DragCoefficient {    // unitless
        const float Sphere = 0.20f;
        const float FlatPlate = 1.28f;
        const float Bullet = 0.295f;
        const float Wing = 0.045f;
    }
    namespace DynamicFrictionCoefficient {    // unitless
        const float BilliardBallOnCushionRail = 0.818f;
        const float BilliardBallOnCueTip = 0.92f;
    }
    
/******************************************************************************
*   MEASUREMENT CLASSES
*       Each has base math functionality, IO streaming, and json serialization
*       Custom literals are defined to provide unit conversions at compile time
******************************************************************************/
#define MAKE_SCALAR_CLASS( Type ) \
    class alignas(16) Type \
    { \
    private: \
        float _value; \
        static const std::string _unit; \
        static const std::wstring _wunit; \
    public: \
        constexpr Type() : _value(0.f) {} \
        explicit constexpr Type(const long double h) : _value(static_cast<float>(h)) {} \
        const std::string Unit() const { return _unit; } \
        const std::wstring UnitW() const { return _wunit; } \
        inline operator float() const { return _value; } \
        inline Type operator- () const { return Type(-1 * _value); } \
        inline Type operator+ (const Type & in) const { return Type(_value + in._value); } \
        inline Type operator- (const Type & in) const { return Type(_value - in._value); } \
        inline Type operator* (const float & in) const { return Type(_value * in); } \
        inline Type operator/ (const float & in) const { return Type(_value / in); } \
        inline Type & operator= (const float & in) { _value = in; return *this; } \
        inline Type & operator= (const Type & in) { _value = in._value; return *this; } \
        inline Type & operator+= (const Type & in) { _value = _value + in._value; return *this; } \
        inline Type & operator-= (const Type & in) { _value = _value - in._value; return *this; } \
        friend std::ostream& operator<< (std::ostream &os, const Type &in); \
        friend std::wostream& operator<< (std::wostream &os, const Type &in); \
        friend std::istream& operator>> (std::istream &is, Type &in); \
        friend std::wistream& operator>> (std::wistream &is, Type &in); \
        friend void to_json(json& j, const Type& from); \
        friend void from_json(const json& j, Type& to); \
    }; \
    std::ostream& operator<< (std::ostream &os, const Type &in); \
    std::wostream& operator<< (std::wostream &os, const Type &in); \
    std::istream& operator>> (std::istream &is, Type &in); \
    std::wistream& operator>> (std::wistream &is, Type &in); \
    void to_json(json& j, const Type& from); \
    void from_json(const json& j, Type& to);

    // Types
    MAKE_SCALAR_CLASS(Mass);
    MAKE_SCALAR_CLASS(Inertia);
    MAKE_SCALAR_CLASS(Angle);
    MAKE_SCALAR_CLASS(Length);
    MAKE_SCALAR_CLASS(Area);
    MAKE_SCALAR_CLASS(Volume);
    MAKE_SCALAR_CLASS(Energy);
    MAKE_SCALAR_CLASS(Power);
    MAKE_SCALAR_CLASS(Strength);
    MAKE_SCALAR_CLASS(AngularStrength);
    MAKE_SCALAR_CLASS(Accel);
    MAKE_SCALAR_CLASS(AngularAccel);
    MAKE_SCALAR_CLASS(Speed);
    MAKE_SCALAR_CLASS(AngularSpeed);
    MAKE_SCALAR_CLASS(Motion);
    MAKE_SCALAR_CLASS(AngularMotion);
    MAKE_SCALAR_CLASS(Temperature);
    MAKE_SCALAR_CLASS(Time);
    
#undef MAKE_SCALAR_CLASS
// Intermediate types for squared values
#define MAKE_SCALAR_CLASS( Type ) \
    class alignas(16) Type \
    { \
    private: \
        float _value; \
        static const std::string _unit; \
        static const std::wstring _wunit; \
    public: \
        constexpr Type() : _value(0.f) {} \
        explicit constexpr Type(const long double h) : _value(static_cast<float>(h)) {} \
        const std::string Unit() const { return _unit; } \
        const std::wstring UnitW() const { return _wunit; } \
        inline operator float() const { return _value; } \
        inline Type operator- () const { return Type(-1 * _value); } \
        inline Type operator+ (const Type & in) const { return Type(_value + in._value); } \
        inline Type operator- (const Type & in) const { return Type(_value - in._value); } \
        inline Type operator* (const float & in) const { return Type(_value * in); } \
        inline Type operator/ (const float & in) const { return Type(_value / in); } \
        inline Type & operator= (const float & in) { _value = in; return *this; } \
        inline Type & operator+= (const Type & in) { _value = _value + in._value; return *this; } \
        inline Type & operator-= (const Type & in) { _value = _value - in._value; return *this; } };

    MAKE_SCALAR_CLASS(MassSq);
    MAKE_SCALAR_CLASS(AngleSq);
    MAKE_SCALAR_CLASS(AreaSq);
    MAKE_SCALAR_CLASS(VolumeSq);
    MAKE_SCALAR_CLASS(EnergySq);
    MAKE_SCALAR_CLASS(PowerSq);
    MAKE_SCALAR_CLASS(StrengthSq);
    MAKE_SCALAR_CLASS(AccelSq);
    MAKE_SCALAR_CLASS(SpeedSq);
    MAKE_SCALAR_CLASS(AngularAccelSq);
    MAKE_SCALAR_CLASS(AngularSpeedSq);
    MAKE_SCALAR_CLASS(TemperatureSq);
    MAKE_SCALAR_CLASS(TimeSq);

#undef MAKE_SCALAR_CLASS

    // Square Math Operators
    MassSq operator*(const Mass& in1, const Mass& in2);
    AngleSq operator*(const Angle& in1, const Angle& in2);
    AreaSq operator*(const Area& in1, const Area& in2);
    VolumeSq operator*(const Volume& in1, const Volume& in2);
    EnergySq operator*(const Energy& in1, const Energy& in2);
    PowerSq operator*(const Power& in1, const Power& in2);
    StrengthSq operator*(const Strength& in1, const Strength& in2);
    AccelSq operator*(const Accel& in1, const Accel& in2);
    SpeedSq operator*(const Speed& in1, const Speed& in2);
    AngularAccelSq operator*(const AngularAccel& in1, const AngularAccel& in2);
    AngularSpeedSq operator*(const AngularSpeed& in1, const AngularSpeed& in2);
    TemperatureSq operator*(const Temperature& in1, const Temperature& in2);
    TimeSq operator*(const Time& in1, const Time& in2);

    // Math Operators
    UnitOfMeasure::Speed operator*(const UnitOfMeasure::Time &t, const UnitOfMeasure::Accel & a);
    UnitOfMeasure::Speed operator*(const UnitOfMeasure::Accel & a, const UnitOfMeasure::Time &t);

    UnitOfMeasure::AngularSpeed operator*(const UnitOfMeasure::Time& t, const UnitOfMeasure::AngularAccel& a);
    UnitOfMeasure::AngularSpeed operator*(const UnitOfMeasure::AngularAccel& a, const UnitOfMeasure::Time& t);

    UnitOfMeasure::Angle operator*(const UnitOfMeasure::Time& t, const UnitOfMeasure::AngularSpeed& s);
    UnitOfMeasure::Angle operator*(const UnitOfMeasure::AngularSpeed& s, const UnitOfMeasure::Time& t);

    UnitOfMeasure::Length operator*(const UnitOfMeasure::Time &t, const UnitOfMeasure::Speed & s);
    UnitOfMeasure::Length operator*(const UnitOfMeasure::Speed & s, const UnitOfMeasure::Time &t);
    UnitOfMeasure::Length operator/(const UnitOfMeasure::Area& num, const UnitOfMeasure::Length& dem);
    UnitOfMeasure::Length operator/(const UnitOfMeasure::Volume& num, const UnitOfMeasure::Area& dem);
    UnitOfMeasure::Length operator/(const UnitOfMeasure::SpeedSq& num, const UnitOfMeasure::Accel& dem);
    UnitOfMeasure::Length operator/(const UnitOfMeasure::AngularStrength& num, const UnitOfMeasure::Strength& dem);

    UnitOfMeasure::Volume operator*(const UnitOfMeasure::Length &l, const UnitOfMeasure::Area & a); 
    UnitOfMeasure::Volume operator*(const UnitOfMeasure::Area & a, const UnitOfMeasure::Length &l);

    UnitOfMeasure::Time operator/(const UnitOfMeasure::Speed& s, const UnitOfMeasure::Accel& a);

    UnitOfMeasure::Mass operator/(const UnitOfMeasure::MassSq& num, const UnitOfMeasure::Mass& dem);
    UnitOfMeasure::Mass operator/(const UnitOfMeasure::Energy& num, const UnitOfMeasure::SpeedSq& dem);

    UnitOfMeasure::Area operator*(const UnitOfMeasure::Length& l1, const UnitOfMeasure::Length& l2);

    UnitOfMeasure::Area operator/(const UnitOfMeasure::AreaSq& num, const UnitOfMeasure::Area& dem);

    UnitOfMeasure::Volume operator/(const UnitOfMeasure::VolumeSq& num, const UnitOfMeasure::Volume& dem);

    UnitOfMeasure::Energy operator/(const UnitOfMeasure::EnergySq& num, const UnitOfMeasure::Energy& dem);
    UnitOfMeasure::Energy operator*(const UnitOfMeasure::Mass& m, const UnitOfMeasure::SpeedSq& sSq);
    // UnitOfMeasure::Energy operator*(const UnitOfMeasure::Inertia& I, const UnitOfMeasure::AngularSpeedSq& asSq); // we do not have r^2 to accomplish this since AngularSpeedSq is rad^2
    UnitOfMeasure::Energy operator*(const UnitOfMeasure::AngularStrength& t1, const UnitOfMeasure::Angle& t2); // W = τ*θ = N * r * θ = N m = F*l ; F is force and l is arc length; l = r * θ
    
    UnitOfMeasure::Energy operator*(const UnitOfMeasure::Strength& t1, const UnitOfMeasure::Length& t2); // work (which conflicts with torqe

    UnitOfMeasure::Power operator/(const UnitOfMeasure::PowerSq& num, const UnitOfMeasure::Power& dem);

    UnitOfMeasure::StrengthSq operator*(const UnitOfMeasure::Strength& t1, const UnitOfMeasure::Strength& t2);
    
    //UnitOfMeasure::AngularStrength operator*(const UnitOfMeasure::Strength& s, const UnitOfMeasure::Length& l);
    //UnitOfMeasure::AngularStrength operator*(const UnitOfMeasure::Length& l, const UnitOfMeasure::Strength& s);
    
    UnitOfMeasure::Strength operator/(const UnitOfMeasure::AngularStrength& num, const UnitOfMeasure::Length& dem);
    UnitOfMeasure::Strength operator/(const UnitOfMeasure::StrengthSq& num, const UnitOfMeasure::Strength& dem);

    UnitOfMeasure::Accel operator/(const UnitOfMeasure::AccelSq& num, const UnitOfMeasure::Accel& dem);
    UnitOfMeasure::Accel operator/(const UnitOfMeasure::Length& num, const UnitOfMeasure::TimeSq& dem);

    UnitOfMeasure::AngularAccel operator/(const UnitOfMeasure::Angle& num, const UnitOfMeasure::TimeSq& dem);
    UnitOfMeasure::AngularAccel operator/(const UnitOfMeasure::AngularAccelSq& num, const UnitOfMeasure::AngularAccel& dem);
    
    UnitOfMeasure::SpeedSq operator*(const UnitOfMeasure::Speed& t1, const UnitOfMeasure::Speed& t2);
    UnitOfMeasure::SpeedSq operator/(const UnitOfMeasure::Energy& num, const UnitOfMeasure::Mass& dem);
    UnitOfMeasure::Speed operator/(const UnitOfMeasure::SpeedSq& num, const UnitOfMeasure::Speed& dem);

    UnitOfMeasure::AngularSpeed operator/(const UnitOfMeasure::AngularSpeedSq& num, const UnitOfMeasure::AngularSpeed& dem);
    UnitOfMeasure::AngularSpeed operator/(const UnitOfMeasure::Angle& num, const UnitOfMeasure::Time& dem);
    
    UnitOfMeasure::Motion operator*(const UnitOfMeasure::Speed& v, const UnitOfMeasure::Mass& m);
    UnitOfMeasure::Motion operator*(const UnitOfMeasure::Mass& m, const UnitOfMeasure::Speed& v);

    UnitOfMeasure::AngularMotion operator*(const UnitOfMeasure::AngularSpeed& v, const UnitOfMeasure::Inertia& m);
    UnitOfMeasure::AngularMotion operator*(const UnitOfMeasure::Inertia& m, const UnitOfMeasure::AngularSpeed& v);

    UnitOfMeasure::Temperature operator/(const UnitOfMeasure::TemperatureSq& num, const UnitOfMeasure::Temperature& dem);

    UnitOfMeasure::Time operator/(const UnitOfMeasure::TimeSq& num, const UnitOfMeasure::Time& dem);
    
    // since c++14
    // literals ex: Mass m1(10_g), m2(2_lbm); Mass m3 = m1 + m2;
    constexpr UnitOfMeasure::Mass operator"" _g(long double in) { return UnitOfMeasure::Mass{ in * 1000 }; }
    constexpr UnitOfMeasure::Mass operator"" _kg(long double in) { return UnitOfMeasure::Mass{ in }; }
    constexpr UnitOfMeasure::Mass operator"" _slug(long double in) { return UnitOfMeasure::Mass{ in * slugTokg }; }
    constexpr UnitOfMeasure::Mass operator"" _lbm(long double in) { return UnitOfMeasure::Mass{ in * lbmTokg }; }
    constexpr UnitOfMeasure::Mass operator"" _tons(long double in) { return UnitOfMeasure::Mass{ in * tonsTokg }; }

    constexpr UnitOfMeasure::Angle operator"" _rev(long double in) { return UnitOfMeasure::Angle{ in * 2.f * PI }; } // 1 revolution = 2 PI
    constexpr UnitOfMeasure::Angle operator"" _rad(long double in) { return UnitOfMeasure::Angle{ in }; }
    constexpr UnitOfMeasure::Angle operator"" _deg(long double in) { return UnitOfMeasure::Angle{ in * degTorad }; }

    constexpr UnitOfMeasure::Length operator"" _m(long double in) { return UnitOfMeasure::Length{ in }; }
    constexpr UnitOfMeasure::Length operator"" _km(long double in) { return UnitOfMeasure::Length{ in * kmTom }; }
    constexpr UnitOfMeasure::Length operator"" _ft(long double in) { return UnitOfMeasure::Length{ in * ftTom }; }
    constexpr UnitOfMeasure::Length operator"" _mile(long double in) { return UnitOfMeasure::Length{ in * mileTom }; }
    constexpr UnitOfMeasure::Length operator"" _nmile(long double in) { return UnitOfMeasure::Length{ in * nmTom }; }

    constexpr UnitOfMeasure::Area operator"" _mSq(long double in) { return UnitOfMeasure::Area{ in }; }
    constexpr UnitOfMeasure::Area operator"" _ftSq(long double in) { return UnitOfMeasure::Area{ in * ftTom * ftTom }; }

    constexpr UnitOfMeasure::Volume operator"" _mCubed(long double in) { return UnitOfMeasure::Volume{ in }; }
    constexpr UnitOfMeasure::Volume operator"" _ftCubed(long double in) { return UnitOfMeasure::Volume{ in * ftTom * ftTom * ftTom }; }
    constexpr UnitOfMeasure::Volume operator"" _L(long double in) { return UnitOfMeasure::Volume{ in * 0.001 }; }

    constexpr UnitOfMeasure::Energy operator"" _J(long double in) { return UnitOfMeasure::Energy{ in }; }
    constexpr UnitOfMeasure::Energy operator"" _Nm(long double in) { return UnitOfMeasure::Energy{ in }; }
    constexpr UnitOfMeasure::Energy operator"" _Btu(long double in) { return UnitOfMeasure::Energy{ in * BtuToJ }; }
    constexpr UnitOfMeasure::Energy operator"" _MBtu(long double in) { return UnitOfMeasure::Energy{ in * MBtuToBtu * BtuToJ }; }
    constexpr UnitOfMeasure::Energy operator"" _therm(long double in) { return UnitOfMeasure::Energy{ in * thermToBtu * BtuToJ }; }
    constexpr UnitOfMeasure::Energy operator"" _lbfFt(long double in) { return UnitOfMeasure::Energy{ in * lbfFtToJ }; }

    constexpr UnitOfMeasure::Power operator"" _w(long double in) { return UnitOfMeasure::Power{ in }; }
    constexpr UnitOfMeasure::Power operator"" _kw(long double in) { return UnitOfMeasure::Power{ in * kwToJPerSec }; }
    constexpr UnitOfMeasure::Power operator"" _JPerSec(long double in) { return UnitOfMeasure::Power{ in }; }
    constexpr UnitOfMeasure::Power operator"" _hp(long double in) { return UnitOfMeasure::Power{ in * hpTokw * kwToJPerSec }; }
    constexpr UnitOfMeasure::Power operator"" _BTUh(long double in) { return UnitOfMeasure::Power{ in * BtuPerHTohp * hpTokw * kwToJPerSec }; } // btu per hour
    constexpr UnitOfMeasure::Power operator"" _lbfFtPerSec(long double in) { return UnitOfMeasure::Power{ in * lbfFtPerSecTohp * hpTokw * kwToJPerSec }; }
    constexpr UnitOfMeasure::Power operator"" _lbfFtRpm(long double in) { return UnitOfMeasure::Power{ in * (1.0 / hpTolbfFtRpm) * hpTokw * kwToJPerSec }; }
    constexpr UnitOfMeasure::Power operator"" _NmPerSec(long double in) { return UnitOfMeasure::Power{ in }; }

    constexpr UnitOfMeasure::Strength operator"" _N(long double in) { return UnitOfMeasure::Strength{ in }; }
    constexpr UnitOfMeasure::Strength operator"" _lbf(long double in) { return UnitOfMeasure::Strength{ in * lbfToN }; }

    constexpr UnitOfMeasure::Accel operator"" _mPerSecSq(long double in) { return UnitOfMeasure::Accel{ in }; }
    constexpr UnitOfMeasure::Accel operator"" _ftPerSecSq(long double in) { return UnitOfMeasure::Accel{ in * (1.0 / mPerSecSqToftPerSecSq) }; }

    constexpr UnitOfMeasure::Speed operator"" _mPerSec(long double in) { return UnitOfMeasure::Speed{ in }; }
    constexpr UnitOfMeasure::Speed operator"" _kph(long double in) { return UnitOfMeasure::Speed{ in * kmTom * hrTosec }; } // 1000m/sec * 360 sec/1 hr
    constexpr UnitOfMeasure::Speed operator"" _ftPerSec(long double in) { return UnitOfMeasure::Speed{ in * ftPerSecTomPerSec }; }
    constexpr UnitOfMeasure::Speed operator"" _mph(long double in) { return UnitOfMeasure::Speed{ in * mphToftPerSec * ftPerSecTomPerSec }; }
    constexpr UnitOfMeasure::Speed operator"" _mach(long double in) { return UnitOfMeasure::Speed{ in * machToftPerSec * ftPerSecTomPerSec }; }

    constexpr UnitOfMeasure::AngularSpeed operator"" _rpm(long double in) { return UnitOfMeasure::AngularSpeed{ in / 30 }; } // 2 PI in 60 sec
    constexpr UnitOfMeasure::AngularSpeed operator"" _radPerSec(long double in) { return UnitOfMeasure::AngularSpeed{ in }; }
    constexpr UnitOfMeasure::AngularSpeed operator"" _degPerSec(long double in) { return UnitOfMeasure::AngularSpeed{ in * degTorad }; }

    constexpr UnitOfMeasure::Temperature operator"" _K(long double in) { return UnitOfMeasure::Temperature{ in }; }
    constexpr UnitOfMeasure::Temperature operator"" _degR(long double in) { return UnitOfMeasure::Temperature{ in * 5/9 }; }
    constexpr UnitOfMeasure::Temperature operator"" _degF(long double in) { return UnitOfMeasure::Temperature{ (in-32)*5/9 + 273.15 }; }
    constexpr UnitOfMeasure::Temperature operator"" _degC(long double in) { return UnitOfMeasure::Temperature{ in + 273.15 }; }

    constexpr UnitOfMeasure::Time operator"" _s(long double in) { return UnitOfMeasure::Time{ in }; }
    constexpr UnitOfMeasure::Time operator"" _min(long double in) { return UnitOfMeasure::Time{ in * 60 }; }
    constexpr UnitOfMeasure::Time operator"" _hr(long double in) { return UnitOfMeasure::Time{ in * 360 }; }

}
}