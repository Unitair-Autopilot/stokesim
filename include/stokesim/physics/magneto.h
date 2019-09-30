/** @file
Declaration of the Magneto class.
*/
#pragma once
#include "stokesim/common.h"

namespace stokesim {
namespace physics {
class Env;
struct WMM;

//////////////////////////////////////////////////

/// Magnetosphere. Manages a geomagnetic model for
/// computing the magnetic B-field at any point.
/// Defaults to NOAA WMM-2020.
class Magneto {
public:
    /// @name Provided
    //@{
    Env const* env; ///< Pointer to the owner of this magneto
    Str const  wmm_cof; ///< String containing the exact content of a standard NOAA WMM.COF file (DEFAULT: WMM-2020)
    //@}

    /// @name Derived
    //@{
    std::shared_ptr<WMM> const wmm; ///< Memoized WMM prep calculations
    //@}

    /// @name Utilities
    //@{
    Evec3 bfield_at_lla(LLA const& lla, bool use_degrees=true) const; ///< Computes the magnetic B-field at the given LLA (altitude in meters, field in teslas ENU)
    Evec3 magdecinc_from_enu(Evec3 const& v, bool use_degrees=true) const; ///< Computes the magnitude, declination, and inclination angles for the given ENU field vector
    //@}

    /// @name Types
    //@{
    struct Cmd {inline void merge(Cmd const& other) {}};
    //@}

private:
    /// @name Core
    //@{
    friend class Env;
    Magneto(YAML::Node const& config, Env const* container);
    inline void update(Cmd const& cmd) {}
    //@}

    NO_COPY(Magneto)
};

//////////////////////////////////////////////////
/// @cond DOCUMENT_WMMLIB
/// @name NOAA WMM Library (Magneto Backend)
//@{

/* THIS IS AN ABRIDGED VERSION OF THE NOAA WORLD MAGNETIC MODEL C SOFTWARE HERE:
 *   https://www.ngdc.noaa.gov/geomag/WMM/soft.shtml
 *
 * *****************************************************************************
 *
 *   The purpose of Geomagnetism Library is primarily to support the World
 *   Magnetic Model (WMM). It however is built to be used for spherical harmonic
 *   models of the Earth's magnetic field generally and supports models even
 *   with a large (>>12) number of degrees. It is also used in many other
 *   geomagnetic models distributed by NCEI.
 *
 * REUSE NOTES
 *
 *   Geomagnetism Library is intended for reuse by any application that requires
 *   Computation of Geomagnetic field from a spherical harmonic model.
 *
 * REFERENCES
 *
 *   Further information on Geoid can be found in the WMM Technical Documents.
 *   https://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml
 *
 * LICENSES
 *
 *   The WMM source code is in the public domain and not licensed or under
 *   copyright. The information and software may be used freely by the public.
 *   As required by 17 U.S.C. 403, third parties producing copyrighted works
 *   consisting predominantly of the material produced by U.S. government
 *   agencies must provide notice with such work(s) identifying the U.S.
 *   Government material incorporated and stating that such material is not
 *   subject to copyright protection.
 *
 * RESTRICTIONS
 *
 *   Geomagnetism library has no restrictions.
 *
 * ENVIRONMENT
 *
 *   Geomagnetism library was tested in the following environments
 *   1. Red Hat Linux with GCC Compiler
 *   2. MS Windows 7 with MinGW compiler
 *   3. Sun Solaris with GCC Compiler
 *   4. Ubuntu with GCC Compiler (MITLL)
 *
 * National Centers for Environmental Information
 * NOAA E/NE42, 325 Broadway
 * Boulder, CO 80305 USA
 * Attn: Arnaud Chulliat
 * Phone: (303) 497-6522
 * Email: Arnaud.Chulliat@noaa.gov
 *
 * Software and Model Support
 * National Centers for Environmental Information
 * NOAA E/NE42
 * 325 Broadway
 * Boulder, CO 80305 USA
 * Attn: Adam Woods or Manoj Nair
 * Phone: (303) 497-6640 or -4642
 * Email: geomag.models@noaa.gov
 * URL: http://www.ngdc.noaa.gov/Geomagnetic/WMM/DoDWMM.shtml
 *
 * For more details on the subroutines, please consult the WMM
 * Technical Documentations at
 * http://www.ngdc.noaa.gov/Geomagnetic/WMM/DoDWMM.shtml
 *
 * Nov 23, 2009
 * Written by Manoj C Nair and Adam Woods
 * Manoj.C.Nair@noaa.Gov
 * Adam.Woods@noaa.gov
 *
 *             (Abridged by MITLL)
 */

////////////////////////////////////////////////// ALIASES

#define TRUE  ((int)1)
#define FALSE ((int)0)

#define RAD2DEG(rad) ((rad)*(180.0L/M_PI))
#define DEG2RAD(deg) ((deg)*(M_PI/180.0L))
#define ATanH(x)     (0.5 * log((1 + x) / (1 - x)))

#define MAG_PS_MIN_LAT_DEGREE  -55    /* Minimum Latitude for  Polar Stereographic projection in degrees */
#define MAG_PS_MAX_LAT_DEGREE   55    /* Maximum Latitude for Polar Stereographic projection in degrees */
#define MAG_UTM_MIN_LAT_DEGREE -80.5  /* Minimum Latitude for UTM projection in degrees */
#define MAG_UTM_MAX_LAT_DEGREE  84.5  /* Maximum Latitude for UTM projection in degrees */

#define MAG_GEO_POLE_TOLERANCE  1e-5
#define MAG_USE_GEOID   1    /* 1 Geoid - Ellipsoid difference should be corrected, 0 otherwise */

#define MAG_ORDER 12 // Standard use-case (MITLL)
#define MAG_NUMTERMS 90 // Derived from MAG_ORDER (MITLL)

////////////////////////////////////////////////// TYPES

struct MAGtype_MagneticModel {
    Escal EditionDate;
    Escal epoch; /*Base time of Geomagnetic model epoch (yrs)*/
    char ModelName[32];
    Escal *Main_Field_Coeff_G; /* C - Gauss coefficients of main geomagnetic model (nT) Index is (n * (n + 1) / 2 + m) */
    Escal *Main_Field_Coeff_H; /* C - Gauss coefficients of main geomagnetic model (nT) */
    Escal *Secular_Var_Coeff_G; /* CD - Gauss coefficients of secular geomagnetic model (nT/yr) */
    Escal *Secular_Var_Coeff_H; /* CD - Gauss coefficients of secular geomagnetic model (nT/yr) */
    int nMax; /* Maximum degree of spherical harmonic model */
    int nMaxSecVar; /* Maximum degree of spherical harmonic secular model */
    int SecularVariationUsed; /* Whether or not the magnetic secular variation vector will be needed by program*/
    Escal CoefficientFileEndDate;
};

struct MAGtype_Ellipsoid {
    Escal a; /*semi-major axis of the ellipsoid*/
    Escal b; /*semi-minor axis of the ellipsoid*/
    Escal fla; /* flattening */
    Escal epssq; /*first eccentricity squared */
    Escal eps; /* first eccentricity */
    Escal re; /* mean radius of  ellipsoid*/
};

struct MAGtype_CoordGeodetic {
    Escal lambda; /* longitude */
    Escal phi; /* geodetic latitude */
    Escal HeightAboveEllipsoid; /* height above the ellipsoid (HaE) */
    Escal HeightAboveGeoid; /* (height above the EGM96 geoid model ) */
    int UseGeoid;
};

struct MAGtype_CoordSpherical {
    Escal lambda; /* longitude*/
    Escal phig; /* geocentric latitude*/
    Escal r; /* distance from the center of the ellipsoid*/
};

struct MAGtype_Date {
    int Year;
    int Month;
    int Day;
    Escal DecimalYear; /* decimal years */
};

struct MAGtype_LegendreFunction {
    Escal *Pcup; /* Legendre Function */
    Escal *dPcup; /* Derivative of Legendre fcn */
};

struct MAGtype_MagneticResults {
    Escal Bx; /* North */
    Escal By; /* East */
    Escal Bz; /* Down */
};

struct MAGtype_SphericalHarmonicVariables {
    Escal *RelativeRadiusPower; /* [earth_reference_radius_km / sph. radius ]^n  */
    Escal *cos_mlambda; /*cp(m)  - cosine of (m*spherical coord. longitude)*/
    Escal *sin_mlambda; /* sp(m)  - sine of (m*spherical coord. longitude) */
};

struct MAGtype_GeoMagneticElements {
    Escal Decl; /* 1. Angle between the magnetic field vector and true north, positive east*/
    Escal Incl; /*2. Angle between the magnetic field vector and the horizontal plane, positive down*/
    Escal F; /*3. Magnetic Field Strength*/
    Escal H; /*4. Horizontal Magnetic Field Strength*/
    Escal X; /*5. Northern component of the magnetic field vector*/
    Escal Y; /*6. Eastern component of the magnetic field vector*/
    Escal Z; /*7. Downward component of the magnetic field vector*/
    Escal GV; /*8. The Grid Variation*/
    Escal Decldot; /*9. Yearly Rate of change in declination*/
    Escal Incldot; /*10. Yearly Rate of change in inclination*/
    Escal Fdot; /*11. Yearly rate of change in Magnetic field strength*/
    Escal Hdot; /*12. Yearly rate of change in horizontal field strength*/
    Escal Xdot; /*13. Yearly rate of change in the northern component*/
    Escal Ydot; /*14. Yearly rate of change in the eastern component*/
    Escal Zdot; /*15. Yearly rate of change in the downward component*/
    Escal GVdot; /*16. Yearly rate of change in grid variation*/
};

struct MAGtype_Geoid {
    int NumbGeoidCols; /* 360 degrees of longitude at 15 minute spacing */
    int NumbGeoidRows; /* 180 degrees of latitude  at 15 minute spacing */
    int NumbHeaderItems; /* min, max lat, min, max long, lat, long spacing*/
    int ScaleFactor; /* 4 grid cells per degree at 15 minute spacing  */
    float *GeoidHeightBuffer;
    int NumbGeoidElevs;
    int Geoid_Initialized; /* indicates successful initialization */
    int UseGeoid; /*Is the Geoid being used?*/
};

struct MAGtype_UTMParameters {
    Escal Easting; /* (X) in meters*/
    Escal Northing; /* (Y) in meters */
    int Zone; /*UTM Zone*/
    char HemiSphere;
    Escal CentralMeridian;
    Escal ConvergenceOfMeridians;
    Escal PointScale;
};

enum COEFFICIENTS {
    IE,
    N,
    M,
    GNM,
    HNM,
    DGNM,
    DHNM
};

struct WMM { // (MITLL)
    MAGtype_Ellipsoid ellip;
    MAGtype_Geoid geoid;
    MAGtype_MagneticModel * model;
    WMM(Str const& wmm_cof, Escal date);
    ~WMM();
};

////////////////////////////////////////////////// FUNCTIONS

MAGtype_LegendreFunction *MAG_AllocateLegendreFunctionMemory(int NumTerms);
MAGtype_MagneticModel *MAG_AllocateModelMemory(int NumTerms);
MAGtype_SphericalHarmonicVariables* MAG_AllocateSphVarMemory(int nMax);
int MAG_FreeMagneticModelMemory(MAGtype_MagneticModel *MagneticModel);
int MAG_FreeLegendreMemory(MAGtype_LegendreFunction *LegendreFunction);
int MAG_FreeSphVarMemory(MAGtype_SphericalHarmonicVariables *SphVar);
int MAG_readMagneticModel(Str const& datastring, MAGtype_MagneticModel * MagneticModel); // Edited to take string not file (MITLL)

int MAG_Geomag(MAGtype_Ellipsoid Ellip, MAGtype_CoordSpherical CoordSpherical, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_MagneticModel *TimedMagneticModel, MAGtype_GeoMagneticElements *GeoMagneticElements);
int MAG_SetDefaults(MAGtype_Ellipsoid *Ellip, MAGtype_Geoid *Geoid);

int MAG_Error(int control);
int MAG_Warnings(int control, Escal value, MAGtype_MagneticModel *MagneticModel);

int MAG_CalculateGeoMagneticElements(MAGtype_MagneticResults *MagneticResultsGeo, MAGtype_GeoMagneticElements *GeoMagneticElements);
int MAG_CalculateGridVariation(MAGtype_CoordGeodetic location, MAGtype_GeoMagneticElements *elements);
int MAG_CalculateGradientElements(MAGtype_MagneticResults GradResults, MAGtype_GeoMagneticElements MagneticElements, MAGtype_GeoMagneticElements *GradElements);

int MAG_CalculateSecularVariationElements(MAGtype_MagneticResults MagneticVariation, MAGtype_GeoMagneticElements *MagneticElements);
int MAG_CartesianToGeodetic(MAGtype_Ellipsoid Ellip, Escal x, Escal y, Escal z, MAGtype_CoordGeodetic *CoordGeodetic);
MAGtype_CoordGeodetic MAG_CoordGeodeticAssign(MAGtype_CoordGeodetic CoordGeodetic);
int MAG_GeodeticToSpherical(MAGtype_Ellipsoid Ellip, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_CoordSpherical *CoordSpherical);
MAGtype_GeoMagneticElements MAG_GeoMagneticElementsAssign(MAGtype_GeoMagneticElements Elements);
MAGtype_GeoMagneticElements MAG_GeoMagneticElementsScale(MAGtype_GeoMagneticElements Elements, Escal factor);
MAGtype_GeoMagneticElements MAG_GeoMagneticElementsSubtract(MAGtype_GeoMagneticElements minuend, MAGtype_GeoMagneticElements subtrahend);
int MAG_GetTransverseMercator(MAGtype_CoordGeodetic CoordGeodetic, MAGtype_UTMParameters *UTMParameters);
int MAG_GetUTMParameters(Escal Latitude, Escal Longitude, int *Zone, char *Hemisphere, Escal *CentralMeridian);
int MAG_isNaN(Escal d);
int MAG_RotateMagneticVector(MAGtype_CoordSpherical CoordSpherical, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_MagneticResults MagneticResultsSph, MAGtype_MagneticResults *MagneticResultsGeo);
int MAG_SphericalToCartesian(MAGtype_CoordSpherical CoordSpherical, Escal *x, Escal *y, Escal *z);
int MAG_SphericalToGeodetic(MAGtype_Ellipsoid Ellip, MAGtype_CoordSpherical CoordSpherical, MAGtype_CoordGeodetic *CoordGeodetic);
int MAG_TMfwd4(Escal Eps, Escal Epssq, Escal K0R4, Escal K0R4oa, Escal Acoeff[], Escal Lam0, Escal K0, Escal falseE, Escal falseN, int XYonly, Escal Lambda, Escal Phi, Escal *X, Escal *Y, Escal *pscale, Escal *CoM);

int MAG_AssociatedLegendreFunction(MAGtype_CoordSpherical CoordSpherical, int nMax, MAGtype_LegendreFunction *LegendreFunction);
int MAG_CheckGeographicPole(MAGtype_CoordGeodetic *CoordGeodetic);
int MAG_ComputeSphericalHarmonicVariables(MAGtype_Ellipsoid Ellip, MAGtype_CoordSpherical CoordSpherical, int nMax, MAGtype_SphericalHarmonicVariables *SphVariables);
int MAG_GradY(MAGtype_Ellipsoid Ellip, MAGtype_CoordSpherical CoordSpherical, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_MagneticModel *TimedMagneticModel, MAGtype_GeoMagneticElements GeoMagneticElements, MAGtype_GeoMagneticElements *GradYElements);
int MAG_GradYSummation(MAGtype_LegendreFunction *LegendreFunction, MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *GradY);
int MAG_PcupHigh(Escal *Pcup, Escal *dPcup, Escal x, int nMax);
int MAG_PcupLow(Escal *Pcup, Escal *dPcup, Escal x, int nMax);
int MAG_SecVarSummation(MAGtype_LegendreFunction *LegendreFunction, MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults);
int MAG_SecVarSummationSpecial(MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults);
int MAG_Summation(MAGtype_LegendreFunction *LegendreFunction, MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults);
int MAG_SummationSpecial(MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults);
int MAG_TimelyModifyMagneticModel(MAGtype_Date UserDate, MAGtype_MagneticModel *MagneticModel, MAGtype_MagneticModel *TimedMagneticModel);

int MAG_ConvertGeoidToEllipsoidHeight(MAGtype_CoordGeodetic *CoordGeodetic, MAGtype_Geoid *Geoid);
int MAG_GetGeoidHeight(Escal Latitude, Escal Longitude, Escal *DeltaHeight, MAGtype_Geoid *Geoid);
int MAG_EquivalentLatLon(Escal lat, Escal lon, Escal *repairedLat, Escal  *repairedLon);

////////////////////////////////////////////////// CONFIG

static Str const WMM_COF_DEFAULT = // (MITLL)
R"( 2020.0            WMM-2020        12/10/2019
  1  0  -29404.5       0.0        6.7        0.0
  1  1   -1450.7    4652.9        7.7      -25.1
  2  0   -2500.0       0.0      -11.5        0.0
  2  1    2982.0   -2991.6       -7.1      -30.2
  2  2    1676.8    -734.8       -2.2      -23.9
  3  0    1363.9       0.0        2.8        0.0
  3  1   -2381.0     -82.2       -6.2        5.7
  3  2    1236.2     241.8        3.4       -1.0
  3  3     525.7    -542.9      -12.2        1.1
  4  0     903.1       0.0       -1.1        0.0
  4  1     809.4     282.0       -1.6        0.2
  4  2      86.2    -158.4       -6.0        6.9
  4  3    -309.4     199.8        5.4        3.7
  4  4      47.9    -350.1       -5.5       -5.6
  5  0    -234.4       0.0       -0.3        0.0
  5  1     363.1      47.7        0.6        0.1
  5  2     187.8     208.4       -0.7        2.5
  5  3    -140.7    -121.3        0.1       -0.9
  5  4    -151.2      32.2        1.2        3.0
  5  5      13.7      99.1        1.0        0.5
  6  0      65.9       0.0       -0.6        0.0
  6  1      65.6     -19.1       -0.4        0.1
  6  2      73.0      25.0        0.5       -1.8
  6  3    -121.5      52.7        1.4       -1.4
  6  4     -36.2     -64.4       -1.4        0.9
  6  5      13.5       9.0       -0.0        0.1
  6  6     -64.7      68.1        0.8        1.0
  7  0      80.6       0.0       -0.1        0.0
  7  1     -76.8     -51.4       -0.3        0.5
  7  2      -8.3     -16.8       -0.1        0.6
  7  3      56.5       2.3        0.7       -0.7
  7  4      15.8      23.5        0.2       -0.2
  7  5       6.4      -2.2       -0.5       -1.2
  7  6      -7.2     -27.2       -0.8        0.2
  7  7       9.8      -1.9        1.0        0.3
  8  0      23.6       0.0       -0.1        0.0
  8  1       9.8       8.4        0.1       -0.3
  8  2     -17.5     -15.3       -0.1        0.7
  8  3      -0.4      12.8        0.5       -0.2
  8  4     -21.1     -11.8       -0.1        0.5
  8  5      15.3      14.9        0.4       -0.3
  8  6      13.7       3.6        0.5       -0.5
  8  7     -16.5      -6.9        0.0        0.4
  8  8      -0.3       2.8        0.4        0.1
  9  0       5.0       0.0       -0.1        0.0
  9  1       8.2     -23.3       -0.2       -0.3
  9  2       2.9      11.1       -0.0        0.2
  9  3      -1.4       9.8        0.4       -0.4
  9  4      -1.1      -5.1       -0.3        0.4
  9  5     -13.3      -6.2       -0.0        0.1
  9  6       1.1       7.8        0.3       -0.0
  9  7       8.9       0.4       -0.0       -0.2
  9  8      -9.3      -1.5       -0.0        0.5
  9  9     -11.9       9.7       -0.4        0.2
 10  0      -1.9       0.0        0.0        0.0
 10  1      -6.2       3.4       -0.0       -0.0
 10  2      -0.1      -0.2       -0.0        0.1
 10  3       1.7       3.5        0.2       -0.3
 10  4      -0.9       4.8       -0.1        0.1
 10  5       0.6      -8.6       -0.2       -0.2
 10  6      -0.9      -0.1       -0.0        0.1
 10  7       1.9      -4.2       -0.1       -0.0
 10  8       1.4      -3.4       -0.2       -0.1
 10  9      -2.4      -0.1       -0.1        0.2
 10 10      -3.9      -8.8       -0.0       -0.0
 11  0       3.0       0.0       -0.0        0.0
 11  1      -1.4      -0.0       -0.1       -0.0
 11  2      -2.5       2.6       -0.0        0.1
 11  3       2.4      -0.5        0.0        0.0
 11  4      -0.9      -0.4       -0.0        0.2
 11  5       0.3       0.6       -0.1       -0.0
 11  6      -0.7      -0.2        0.0        0.0
 11  7      -0.1      -1.7       -0.0        0.1
 11  8       1.4      -1.6       -0.1       -0.0
 11  9      -0.6      -3.0       -0.1       -0.1
 11 10       0.2      -2.0       -0.1        0.0
 11 11       3.1      -2.6       -0.1       -0.0
 12  0      -2.0       0.0        0.0        0.0
 12  1      -0.1      -1.2       -0.0       -0.0
 12  2       0.5       0.5       -0.0        0.0
 12  3       1.3       1.3        0.0       -0.1
 12  4      -1.2      -1.8       -0.0        0.1
 12  5       0.7       0.1       -0.0       -0.0
 12  6       0.3       0.7        0.0        0.0
 12  7       0.5      -0.1       -0.0       -0.0
 12  8      -0.2       0.6        0.0        0.1
 12  9      -0.5       0.2       -0.0       -0.0
 12 10       0.1      -0.9       -0.0       -0.0
 12 11      -1.1      -0.0       -0.0        0.0
 12 12      -0.3       0.5       -0.1       -0.1
999999999999999999999999999999999999999999999999
999999999999999999999999999999999999999999999999)";

//@}
/// @endcond
//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
