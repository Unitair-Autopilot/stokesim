/** @file
Implementation of magneto.h
*/
#include "stokesim/physics/magneto.h"
#include "stokesim/physics/world.h"

namespace stokesim {
namespace physics {

//////////////////////////////////////////////////

Evec3 Magneto::bfield_at_lla(LLA const& lla, bool use_degrees/*=true*/) const {
    // Convert geodetic coordinates to spherical coordinates
    MAGtype_CoordGeodetic gcoord;
    gcoord.phi = lla.lat;
    gcoord.lambda = lla.lon;
    gcoord.HeightAboveGeoid = lla.alt / 1000; // model assumes kilometers
    if(!use_degrees) { // model assumes degrees
        gcoord.phi *= RAD2DEG;
        gcoord.lambda *= RAD2DEG;
    }
    MAGtype_CoordSpherical scoord;
    MAG_GeodeticToSpherical(wmm->ellip, gcoord, &scoord);
    // Do WMM calculation
    MAGtype_GeoMagneticElements elements;
    MAG_Geomag(wmm->ellip, scoord, gcoord, wmm->model, &elements);
    MAG_CalculateGridVariation(gcoord, &elements);
    // Return field in teslas ENU (model assumes nanotesla NED)
    return Evec3(elements.Y, elements.X, -elements.Z) * 1e-9;
}

//////////////////////////////////////////////////

Evec3 Magneto::magdecinc_from_enu(Evec3 const& v, bool use_degrees/*=true*/) const {
    Escal h2 = v(0)*v(0) + v(1)*v(1);
    Evec3 magdecinc(sqrt(h2 + v(2)*v(2)),
                    atan2(v(0), v(1)),
                    atan2(-v(2), sqrt(h2)));
    if(use_degrees) {
        magdecinc(1) *= RAD2DEG;
        magdecinc(2) *= RAD2DEG;
    }
    return magdecinc;
}

//////////////////////////////////////////////////

Magneto::Magneto(YAML::Node const& config, Env const* container) :
    env(container),
    wmm_cof(yget("wmm_cof", config, WMM_COF_DEFAULT).as<Str>()),
    wmm(new WMM(wmm_cof, env->world->clock.date)) {
}

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


WMM::WMM(Str const& wmm_cof, Escal date) { // (MITLL)
    // Prepare geoid and ellipsoid with WMM defaults
    MAG_SetDefaults(&ellip, &geoid);
    // Create temporary base model using given coefficients
    MAGtype_MagneticModel * model0 = MAG_AllocateModelMemory(MAG_NUMTERMS);
    model0->nMax = MAG_ORDER;
    model0->nMaxSecVar = MAG_ORDER;
    MAG_readMagneticModel(wmm_cof, model0);
    // Build time-extrapolated model using date and base model
    MAGtype_Date wmm_date;
    wmm_date.DecimalYear = date;
    model = MAG_AllocateModelMemory(MAG_NUMTERMS);
    MAG_TimelyModifyMagneticModel(wmm_date, model0, model);
    // Release temporary base model memory
    MAG_FreeMagneticModelMemory(model0);
}


WMM::~WMM() { // (MITLL)
    // Release model memory
    MAG_FreeMagneticModelMemory(model);
}


/******************************************************************************
 ********************************Memory and File Processing********************
 * This grouping consists of functions that read coefficient files into the
 * memory, allocate memory, free memory or print models into coefficient files.
 ******************************************************************************/


MAGtype_LegendreFunction *MAG_AllocateLegendreFunctionMemory(int NumTerms)

/* Allocate memory for Associated Legendre Function data types.
   Should be called before computing Associated Legendre Functions.

 INPUT: NumTerms : int : Total number of spherical harmonic coefficients in the model


 OUTPUT:    Pointer to data structure MAGtype_LegendreFunction with the following elements
                        Escal *Pcup;  (  pointer to store Legendre Function  )
                        Escal *dPcup; ( pointer to store  Derivative of Legendre function )

                        FALSE: Failed to allocate memory

CALLS : none

 */
{
    MAGtype_LegendreFunction *LegendreFunction;

    LegendreFunction = (MAGtype_LegendreFunction *) calloc(1, sizeof (MAGtype_LegendreFunction));

    if(!LegendreFunction)
    {
        MAG_Error(1);
        return NULL;
    }
    LegendreFunction->Pcup = (Escal *) malloc((NumTerms + 1) * sizeof ( Escal));
    if(LegendreFunction->Pcup == 0)
    {
        MAG_Error(1);
        return NULL;
    }
    LegendreFunction->dPcup = (Escal *) malloc((NumTerms + 1) * sizeof ( Escal));
    if(LegendreFunction->dPcup == 0)
    {
        MAG_Error(1);
        return NULL;
    }
    return LegendreFunction;
} /*MAGtype_LegendreFunction*/


MAGtype_MagneticModel *MAG_AllocateModelMemory(int NumTerms)

/* Allocate memory for WMM Coefficients
 * Should be called before reading the model file *

  INPUT: NumTerms : int : Total number of spherical harmonic coefficients in the model


 OUTPUT:    Pointer to data structure MAGtype_MagneticModel with the following elements
                        Escal EditionDate;
                        Escal epoch;       Base time of Geomagnetic model epoch (yrs)
                        char  ModelName[20];
                        Escal *Main_Field_Coeff_G;          C - Gauss coefficients of main geomagnetic model (nT)
                        Escal *Main_Field_Coeff_H;          C - Gauss coefficients of main geomagnetic model (nT)
                        Escal *Secular_Var_Coeff_G;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
                        Escal *Secular_Var_Coeff_H;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
                        int nMax;  Maximum degree of spherical harmonic model
                        int nMaxSecVar; Maxumum degree of spherical harmonic secular model
                        int SecularVariationUsed; Whether or not the magnetic secular variation vector will be needed by program

                        FALSE: Failed to allocate memory
CALLS : none
 */
{
    MAGtype_MagneticModel *MagneticModel;
    int i;


    MagneticModel = (MAGtype_MagneticModel *) calloc(1, sizeof (MAGtype_MagneticModel));

    if(MagneticModel == NULL)
    {
        MAG_Error(2);
        return NULL;
    }

    MagneticModel->Main_Field_Coeff_G = (Escal *) malloc((NumTerms + 1) * sizeof ( Escal));

    if(MagneticModel->Main_Field_Coeff_G == NULL)
    {
        MAG_Error(2);
        return NULL;
    }

    MagneticModel->Main_Field_Coeff_H = (Escal *) malloc((NumTerms + 1) * sizeof ( Escal));

    if(MagneticModel->Main_Field_Coeff_H == NULL)
    {
        MAG_Error(2);
        return NULL;
    }
    MagneticModel->Secular_Var_Coeff_G = (Escal *) malloc((NumTerms + 1) * sizeof ( Escal));
    if(MagneticModel->Secular_Var_Coeff_G == NULL)
    {
        MAG_Error(2);
        return NULL;
    }
    MagneticModel->Secular_Var_Coeff_H = (Escal *) malloc((NumTerms + 1) * sizeof ( Escal));
    if(MagneticModel->Secular_Var_Coeff_H == NULL)
    {
        MAG_Error(2);
        return NULL;
    }
    MagneticModel->CoefficientFileEndDate = 0;
    MagneticModel->EditionDate = 0;
    strcpy(MagneticModel->ModelName, "");
    MagneticModel->SecularVariationUsed = 0;
    MagneticModel->epoch = 0;
    MagneticModel->nMax = 0;
    MagneticModel->nMaxSecVar = 0;

    for(i=0; i<NumTerms; i++) {
        MagneticModel->Main_Field_Coeff_G[i] = 0;
        MagneticModel->Main_Field_Coeff_H[i] = 0;
        MagneticModel->Secular_Var_Coeff_G[i] = 0;
        MagneticModel->Secular_Var_Coeff_H[i] = 0;
    }

    return MagneticModel;

} /*MAG_AllocateModelMemory*/


MAGtype_SphericalHarmonicVariables* MAG_AllocateSphVarMemory(int nMax)
{
    MAGtype_SphericalHarmonicVariables* SphVariables;
    SphVariables  = (MAGtype_SphericalHarmonicVariables*) calloc(1, sizeof(MAGtype_SphericalHarmonicVariables));
    SphVariables->RelativeRadiusPower = (Escal *) malloc((nMax + 1) * sizeof ( Escal));
    SphVariables->cos_mlambda = (Escal *) malloc((nMax + 1) * sizeof (Escal));
    SphVariables->sin_mlambda = (Escal *) malloc((nMax + 1) * sizeof (Escal));
    return SphVariables;
} /*MAG_AllocateSphVarMemory*/


int MAG_FreeMagneticModelMemory(MAGtype_MagneticModel *MagneticModel)

/* Free the magnetic model memory used by WMM functions.
INPUT :  MagneticModel  pointer to data structure with the following elements

                        Escal EditionDate;
                        Escal epoch;       Base time of Geomagnetic model epoch (yrs)
                        char  ModelName[20];
                        Escal *Main_Field_Coeff_G;          C - Gauss coefficients of main geomagnetic model (nT)
                        Escal *Main_Field_Coeff_H;          C - Gauss coefficients of main geomagnetic model (nT)
                        Escal *Secular_Var_Coeff_G;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
                        Escal *Secular_Var_Coeff_H;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
                        int nMax;  Maximum degree of spherical harmonic model
                        int nMaxSecVar; Maxumum degree of spherical harmonic secular model
                        int SecularVariationUsed; Whether or not the magnetic secular variation vector will be needed by program

OUTPUT  none
CALLS : none

 */
{
    if(MagneticModel->Main_Field_Coeff_G)
    {
        free(MagneticModel->Main_Field_Coeff_G);
        MagneticModel->Main_Field_Coeff_G = NULL;
    }
    if(MagneticModel->Main_Field_Coeff_H)
    {
        free(MagneticModel->Main_Field_Coeff_H);
        MagneticModel->Main_Field_Coeff_H = NULL;
    }
    if(MagneticModel->Secular_Var_Coeff_G)
    {
        free(MagneticModel->Secular_Var_Coeff_G);
        MagneticModel->Secular_Var_Coeff_G = NULL;
    }
    if(MagneticModel->Secular_Var_Coeff_H)
    {
        free(MagneticModel->Secular_Var_Coeff_H);
        MagneticModel->Secular_Var_Coeff_H = NULL;
    }
    if(MagneticModel)
    {
        free(MagneticModel);
        MagneticModel = NULL;
    }

    return TRUE;
} /*MAG_FreeMagneticModelMemory */


int MAG_FreeLegendreMemory(MAGtype_LegendreFunction *LegendreFunction)

/* Free the Legendre Coefficients memory used by the WMM functions.
INPUT : LegendreFunction Pointer to data structure with the following elements
                                                Escal *Pcup;  (  pointer to store Legendre Function  )
                                                Escal *dPcup; ( pointer to store  Derivative of Lagendre function )

OUTPUT: none
CALLS : none

 */
{
    if(LegendreFunction->Pcup)
    {
        free(LegendreFunction->Pcup);
        LegendreFunction->Pcup = NULL;
    }
    if(LegendreFunction->dPcup)
    {
        free(LegendreFunction->dPcup);
        LegendreFunction->dPcup = NULL;
    }
    if(LegendreFunction)
    {
        free(LegendreFunction);
        LegendreFunction = NULL;
    }

    return TRUE;
} /*MAG_FreeLegendreMemory */


int MAG_FreeSphVarMemory(MAGtype_SphericalHarmonicVariables *SphVar)

/* Free the Spherical Harmonic Variable memory used by the WMM functions.
INPUT : LegendreFunction Pointer to data structure with the following elements
                                                Escal *RelativeRadiusPower
                                                Escal *cos_mlambda
                                                Escal *sin_mlambda
 OUTPUT: none
 CALLS : none
 */
{
    if(SphVar->RelativeRadiusPower)
    {
        free(SphVar->RelativeRadiusPower);
        SphVar->RelativeRadiusPower = NULL;
    }
    if(SphVar->cos_mlambda)
    {
        free(SphVar->cos_mlambda);
        SphVar->cos_mlambda = NULL;
    }
    if(SphVar->sin_mlambda)
    {
        free(SphVar->sin_mlambda);
        SphVar->sin_mlambda = NULL;
    }
    if(SphVar)
    {
        free(SphVar);
        SphVar = NULL;
    }

    return TRUE;
} /*MAG_FreeSphVarMemory*/


int MAG_readMagneticModel(Str const& datastring, MAGtype_MagneticModel * MagneticModel) // Edited to take string not file (MITLL)
{

    /* READ WORLD MAGNETIC MODEL SPHERICAL HARMONIC COEFFICIENTS
       INPUT :  datastring: One string containing the entirety of a standard WMM.COF file
            MagneticModel : Pointer to the data structure with the following fields required as inputs
                                    nMax :  Number of static coefficients
       UPDATES : MagneticModel : Pointer to the data structure with the following fields populated
                                    char  *ModelName;
                                    Escal epoch;       Base time of Geomagnetic model epoch (yrs)
                                    Escal *Main_Field_Coeff_G;          C - Gauss coefficients of main geomagnetic model (nT)
                                    Escal *Main_Field_Coeff_H;          C - Gauss coefficients of main geomagnetic model (nT)
                                    Escal *Secular_Var_Coeff_G;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
                                    Escal *Secular_Var_Coeff_H;  CD - Gauss coefficients of secular geomagnetic model (nT/yr)
            CALLS : none
    */

    char c_str[81], c_new[5]; /*these strings are used to read a line from coefficient file*/
    int i, icomp, m, n, EOF_Flag = 0, index;
    Escal epoch, gnm, hnm, dgnm, dhnm;
    std::stringstream datastream(datastring); // Acts similarly to a FILE* with fgets (MITLL)

    MagneticModel->Main_Field_Coeff_H[0] = 0.0;
    MagneticModel->Main_Field_Coeff_G[0] = 0.0;
    MagneticModel->Secular_Var_Coeff_H[0] = 0.0;
    MagneticModel->Secular_Var_Coeff_G[0] = 0.0;
    datastream.getline(c_str, 80);
    sscanf(c_str, "%lf%s", &epoch, MagneticModel->ModelName);
    MagneticModel->epoch = epoch;
    while(EOF_Flag == 0)
    {
        datastream.getline(c_str, 80);
        /* CHECK FOR LAST LINE IN FILE */
        for(i = 0; i < 4 && (c_str[i] != '\0'); i++)
        {
            c_new[i] = c_str[i];
            c_new[i + 1] = '\0';
        }
        icomp = strcmp("9999", c_new);
        if(icomp == 0)
        {
            EOF_Flag = 1;
            break;
        }
        /* END OF FILE NOT ENCOUNTERED, GET VALUES */
        sscanf(c_str, "%d%d%lf%lf%lf%lf", &n, &m, &gnm, &hnm, &dgnm, &dhnm);
        if(m <= n)
        {
            index = (n * (n + 1) / 2 + m);
            MagneticModel->Main_Field_Coeff_G[index] = gnm;
            MagneticModel->Secular_Var_Coeff_G[index] = dgnm;
            MagneticModel->Main_Field_Coeff_H[index] = hnm;
            MagneticModel->Secular_Var_Coeff_H[index] = dhnm;
        }
    }

    return TRUE;
} /*MAG_readMagneticModel*/


/*End of Memory and File Processing functions*/


/******************************************************************************
 ************************************Wrapper***********************************
 * This grouping consists of functions call groups of other functions to do a
 * complete calculation of some sort.  For example, the MAG_Geomag function
 * does everything necessary to compute the geomagnetic elements from a given
 * geodetic point in space and magnetic model adjusted for the appropriate
 * date. These functions are the external functions necessary to create a
 * program that uses or calculates the magnetic field.
 ******************************************************************************
 ******************************************************************************/


int MAG_Geomag(MAGtype_Ellipsoid Ellip, MAGtype_CoordSpherical CoordSpherical, MAGtype_CoordGeodetic CoordGeodetic,
        MAGtype_MagneticModel *TimedMagneticModel, MAGtype_GeoMagneticElements *GeoMagneticElements)
/*
The main subroutine that calls a sequence of WMM sub-functions to calculate the magnetic field elements for a single point.
The function expects the model coefficients and point coordinates as input and returns the magnetic field elements and
their rate of change. Though, this subroutine can be called successively to calculate a time series, profile or grid
of magnetic field, these are better achieved by the subroutine MAG_Grid.

INPUT: Ellip
              CoordSpherical
              CoordGeodetic
              TimedMagneticModel

OUTPUT : GeoMagneticElements

CALLS:      MAG_AllocateLegendreFunctionMemory(NumTerms);  ( For storing the ALF functions )
                     MAG_ComputeSphericalHarmonicVariables( Ellip, CoordSpherical, TimedMagneticModel->nMax, &SphVariables); (Compute Spherical Harmonic variables  )
                     MAG_AssociatedLegendreFunction(CoordSpherical, TimedMagneticModel->nMax, LegendreFunction);    Compute ALF
                     MAG_Summation(LegendreFunction, TimedMagneticModel, SphVariables, CoordSpherical, &MagneticResultsSph);  Accumulate the spherical harmonic coefficients
                     MAG_SecVarSummation(LegendreFunction, TimedMagneticModel, SphVariables, CoordSpherical, &MagneticResultsSphVar); Sum the Secular Variation Coefficients
                     MAG_RotateMagneticVector(CoordSpherical, CoordGeodetic, MagneticResultsSph, &MagneticResultsGeo); Map the computed Magnetic fields to Geodetic coordinates
                     MAG_CalculateGeoMagneticElements(&MagneticResultsGeo, GeoMagneticElements);   Calculate the Geomagnetic elements
                     MAG_CalculateSecularVariationElements(MagneticResultsGeoVar, GeoMagneticElements); Calculate the secular variation of each of the Geomagnetic elements

 */
{
    MAGtype_LegendreFunction *LegendreFunction;
    MAGtype_SphericalHarmonicVariables *SphVariables;
    int NumTerms;
    MAGtype_MagneticResults MagneticResultsSph, MagneticResultsGeo, MagneticResultsSphVar, MagneticResultsGeoVar;

    NumTerms = ((TimedMagneticModel->nMax + 1) * (TimedMagneticModel->nMax + 2) / 2);
    LegendreFunction = MAG_AllocateLegendreFunctionMemory(NumTerms); /* For storing the ALF functions */
    SphVariables = MAG_AllocateSphVarMemory(TimedMagneticModel->nMax);
    MAG_ComputeSphericalHarmonicVariables(Ellip, CoordSpherical, TimedMagneticModel->nMax, SphVariables); /* Compute Spherical Harmonic variables  */
    MAG_AssociatedLegendreFunction(CoordSpherical, TimedMagneticModel->nMax, LegendreFunction); /* Compute ALF  */
    MAG_Summation(LegendreFunction, TimedMagneticModel, *SphVariables, CoordSpherical, &MagneticResultsSph); /* Accumulate the spherical harmonic coefficients*/
    MAG_SecVarSummation(LegendreFunction, TimedMagneticModel, *SphVariables, CoordSpherical, &MagneticResultsSphVar); /*Sum the Secular Variation Coefficients  */
    MAG_RotateMagneticVector(CoordSpherical, CoordGeodetic, MagneticResultsSph, &MagneticResultsGeo); /* Map the computed Magnetic fields to Geodeitic coordinates  */
    MAG_RotateMagneticVector(CoordSpherical, CoordGeodetic, MagneticResultsSphVar, &MagneticResultsGeoVar); /* Map the secular variation field components to Geodetic coordinates*/
    MAG_CalculateGeoMagneticElements(&MagneticResultsGeo, GeoMagneticElements); /* Calculate the Geomagnetic elements, Equation 19 , WMM Technical report */
    MAG_CalculateSecularVariationElements(MagneticResultsGeoVar, GeoMagneticElements); /*Calculate the secular variation of each of the Geomagnetic elements*/

    MAG_FreeLegendreMemory(LegendreFunction);
    MAG_FreeSphVarMemory(SphVariables);

    return TRUE;
} /*MAG_Geomag*/


int MAG_SetDefaults(MAGtype_Ellipsoid *Ellip, MAGtype_Geoid *Geoid)

/*
        Sets default values for WMM subroutines.

        UPDATES : Ellip
                        Geoid

        CALLS : none
 */
{

    /* Sets WGS-84 parameters */
    Ellip->a = 6378.137; /*semi-major axis of the ellipsoid in */
    Ellip->b = 6356.7523142; /*semi-minor axis of the ellipsoid in */
    Ellip->fla = 1 / 298.257223563; /* flattening */
    Ellip->eps = sqrt(1 - (Ellip->b * Ellip->b) / (Ellip->a * Ellip->a)); /*first eccentricity */
    Ellip->epssq = (Ellip->eps * Ellip->eps); /*first eccentricity squared */
    Ellip->re = 6371.2; /* Earth's radius */

    /* Sets EGM-96 model file parameters */
    Geoid->NumbGeoidCols = 1441; /* 360 degrees of longitude at 15 minute spacing */
    Geoid->NumbGeoidRows = 721; /* 180 degrees of latitude  at 15 minute spacing */
    Geoid->NumbHeaderItems = 6; /* min, max lat, min, max long, lat, long spacing*/
    Geoid->ScaleFactor = 4; /* 4 grid cells per degree at 15 minute spacing  */
    Geoid->NumbGeoidElevs = Geoid->NumbGeoidCols * Geoid->NumbGeoidRows;
    Geoid->Geoid_Initialized = 0; /*  Geoid will be initialized only if this is set to zero */
    Geoid->UseGeoid = MAG_USE_GEOID;

    return TRUE;
} /*MAG_SetDefaults */


/*End of Wrapper Functions*/


/******************************************************************************
 ********************************User Interface********************************
 * This grouping consists of functions which interact with the directly with
 * the user and are generally specific to the XXX_point.c, XXX_grid.c, and
 * XXX_file.c programs. They deal with input from and output to the user.
 ******************************************************************************/


int MAG_Error(int control)

/*This prints WMM errors.
INPUT     control     Error look up number
OUTPUT    none
CALLS : none

 */
{
    switch(control) {
        case 1:
            printf("\nError allocating in MAG_LegendreFunctionMemory.\n");
            break;
        case 2:
            printf("\nError allocating in MAG_AllocateModelMemory.\n");
            break;
        case 3:
            printf("\nError allocating in MAG_InitializeGeoid\n");
            break;
        case 4:
            printf("\nError in setting default values.\n");
            break;
        case 5:
            printf("\nError initializing Geoid.\n");
            break;
        case 6:
            printf("\nError opening WMM.COF\n.");
            break;
        case 7:
            printf("\nError opening WMMSV.COF\n.");
            break;
        case 8:
            printf("\nError reading Magnetic Model.\n");
            break;
        case 9:
            printf("\nError printing Command Prompt introduction.\n");
            break;
        case 10:
            printf("\nError converting from geodetic co-ordinates to spherical co-ordinates.\n");
            break;
        case 11:
            printf("\nError in time modifying the Magnetic model\n");
            break;
        case 12:
            printf("\nError in Geomagnetic\n");
            break;
        case 13:
            printf("\nError printing user data\n");\
            break;
        case 14:
            printf("\nError allocating in MAG_SummationSpecial\n");
            break;
        case 15:
            printf("\nError allocating in MAG_SecVarSummationSpecial\n");
            break;
        case 16:
            printf("\nError in opening EGM9615.BIN file\n");
            break;
        case 17:
            printf("\nError: Latitude OR Longitude out of range in MAG_GetGeoidHeight\n");
            break;
        case 18:
            printf("\nError allocating in MAG_PcupHigh\n");
            break;
        case 19:
            printf("\nError allocating in MAG_PcupLow\n");
            break;
        case 20:
            printf("\nError opening coefficient file\n");
            break;
        case 21:
            printf("\nError: UnitDepth too large\n");
            break;
        case 22:
            printf("\nYour system needs Big endian version of EGM9615.BIN.  \n");
            printf("Please download this file from http://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml.  \n");
            printf("Replace the existing EGM9615.BIN file with the downloaded one\n");
            break;
    }
    return TRUE;
} /*MAG_Error*/


int MAG_Warnings(int control, Escal value, MAGtype_MagneticModel *MagneticModel)

/*Return value 0 means end program, Return value 1 means get new data, Return value 2 means continue.
  This prints a warning to the screen determined by the control integer. It also takes the value of the parameter causing the warning as a Escal.  This is unnecessary for some warnings.
  It requires the MagneticModel to determine the current epoch.

 INPUT control :int : (Warning number)
                value  : Escal: Magnetic field strength
                MagneticModel
OUTPUT : none
CALLS : none

 */
{
    char ans[20];
    strcpy(ans, "");

    switch(control) {
        case 1:/* Horizontal Field strength low */
            do {
                printf("\nCaution: location is approaching the blackout zone around the magnetic pole as\n");
                printf("      defined by the WMM military specification \n");
                printf("      (https://www.ngdc.noaa.gov/geomag/WMM/data/MIL-PRF-89500B.pdf). Compass\n");
                printf("      accuracy may be degraded in this region.\n");
                printf("Press enter to continue...\n");
            } while(NULL == fgets(ans, 20, stdin));
            break;
        case 2:/* Horizontal Field strength very low */
            do {
                printf("\nWarning: location is in the blackout zone around the magnetic pole as defined\n");
                printf("      by the WMM military specification \n");
                printf("      (https://www.ngdc.noaa.gov/geomag/WMM/data/MIL-PRF-89500B.pdf). Compass\n");
                printf("      accuracy is highly degraded in this region.\n");
            } while(NULL == fgets(ans, 20, stdin));
            break;
        case 3:/* Elevation outside the recommended range */
            printf("\nWarning: The value you have entered of %.1f km for the elevation is outside of the recommended range.\n Elevations above -10.0 km are recommended for accurate results. \n", value);
            while(1)
            {
                printf("\nPlease press 'C' to continue, 'G' to get new data or 'X' to exit...\n");
                while( NULL == fgets(ans, 20, stdin)) {
                    printf("\nInvalid input\n");
                }
                switch(ans[0]) {
                    case 'X':
                    case 'x':
                        return 0;
                    case 'G':
                    case 'g':
                        return 1;
                    case 'C':
                    case 'c':
                        return 2;
                    default:
                        printf("\nInvalid input %c\n", ans[0]);
                        break;
                }
            }
            break;

        case 4:/*Date outside the recommended range*/
            printf("\nWARNING - TIME EXTENDS BEYOND INTENDED USAGE RANGE\n CONTACT NCEI FOR PRODUCT UPDATES:\n");
            printf("    National Centers for Environmental Information\n");
            printf("    NOAA E/NE42\n");
            printf("    325 Broadway\n");
            printf("\n  Boulder, CO 80305 USA");
            printf("    Attn: Manoj Nair or Arnaud Chulliat\n");
            printf("    Phone:  (303) 497-4642 or -6522\n");
            printf("    Email:  geomag.models@noaa.gov\n");
            printf("    Web: http://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml\n");
            printf("\n VALID RANGE  = %d - %d\n", (int) MagneticModel->epoch, (int) MagneticModel->CoefficientFileEndDate);
            printf(" TIME   = %f\n", value);
            while(1)
            {
                printf("\nPlease press 'C' to continue, 'N' to enter new data or 'X' to exit...\n");
                while (NULL ==fgets(ans, 20, stdin)){
                    printf("\nInvalid input\n");
                }
                switch(ans[0]) {
                    case 'X':
                    case 'x':
                        return 0;
                    case 'N':
                    case 'n':
                        return 1;
                    case 'C':
                    case 'c':
                        return 2;
                    default:
                        printf("\nInvalid input %c\n", ans[0]);
                        break;
                }
            }
            break;
        case 5:/*Elevation outside the allowable range*/
            printf("\nError: The value you have entered of %f km for the elevation is outside of the recommended range.\n Elevations above -10.0 km are recommended for accurate results. \n", value);
            while(1)
            {
                printf("\nPlease press 'C' to continue, 'G' to get new data or 'X' to exit...\n");
                while (NULL ==fgets(ans, 20, stdin)){
                    printf("\nInvalid input\n");
                }
                switch(ans[0]) {
                    case 'X':
                    case 'x':
                        return 0;
                    case 'G':
                    case 'g':
                        return 1;
                    case 'C':
                    case 'c':
                        return 2;
                    default:
                        printf("\nInvalid input %c\n", ans[0]);
                        break;
                }
            }
            break;
    }
    return 2;
} /*MAG_Warnings*/


/*End of User Interface functions*/


/******************************************************************************
 *************Conversions, Transformations, and other Calculations**************
 * This grouping consists of functions that perform unit conversions, coordinate
 * transformations and other simple or straightforward calculations that are
 * usually easily replicable with a typical scientific calculator.
 ******************************************************************************/


int MAG_CalculateGeoMagneticElements(MAGtype_MagneticResults *MagneticResultsGeo, MAGtype_GeoMagneticElements *GeoMagneticElements)

/* Calculate all the Geomagnetic elements from X,Y and Z components
INPUT     MagneticResultsGeo   Pointer to data structure with the following elements
                        Escal Bx;    ( North )
                        Escal By;    ( East )
                        Escal Bz;    ( Down )
OUTPUT    GeoMagneticElements    Pointer to data structure with the following elements
                        Escal Decl; (Angle between the magnetic field vector and true north, positive east)
                        Escal Incl; Angle between the magnetic field vector and the horizontal plane, positive down
                        Escal F; Magnetic Field Strength
                        Escal H; Horizontal Magnetic Field Strength
                        Escal X; Northern component of the magnetic field vector
                        Escal Y; Eastern component of the magnetic field vector
                        Escal Z; Downward component of the magnetic field vector
CALLS : none
 */
{
    GeoMagneticElements->X = MagneticResultsGeo->Bx;
    GeoMagneticElements->Y = MagneticResultsGeo->By;
    GeoMagneticElements->Z = MagneticResultsGeo->Bz;

    // Removing these calculations because they get masked by the wrapping Magneto class (MITLL)
    //GeoMagneticElements->H = sqrt(MagneticResultsGeo->Bx * MagneticResultsGeo->Bx + MagneticResultsGeo->By * MagneticResultsGeo->By);
    //GeoMagneticElements->F = sqrt(GeoMagneticElements->H * GeoMagneticElements->H + MagneticResultsGeo->Bz * MagneticResultsGeo->Bz);
    //GeoMagneticElements->Decl = RAD2DEG(atan2(GeoMagneticElements->Y, GeoMagneticElements->X));
    //GeoMagneticElements->Incl = RAD2DEG(atan2(GeoMagneticElements->Z, GeoMagneticElements->H));

    return TRUE;
} /*MAG_CalculateGeoMagneticElements */


int MAG_CalculateGridVariation(MAGtype_CoordGeodetic location, MAGtype_GeoMagneticElements *elements)

/*Computes the grid variation for |latitudes| > MAG_MAX_LAT_DEGREE

Grivation (or grid variation) is the angle between grid north and
magnetic north. This routine calculates Grivation for the Polar Stereographic
projection for polar locations (Latitude => |55| deg). Otherwise, it computes the grid
variation in UTM projection system. However, the UTM projection codes may be used to compute
the grid variation at any latitudes.

INPUT    location    Data structure with the following elements
                Escal lambda; (longitude)
                Escal phi; ( geodetic latitude)
                Escal HeightAboveEllipsoid; (height above the ellipsoid (HaE) )
                Escal HeightAboveGeoid;(height above the Geoid )
OUTPUT  elements Data  structure with the following elements updated
                Escal GV; ( The Grid Variation )
CALLS : MAG_GetTransverseMercator

 */
{
    MAGtype_UTMParameters UTMParameters;
    if(location.phi >= MAG_PS_MAX_LAT_DEGREE)
    {
        elements->GV = elements->Decl - location.lambda;
        return 1;
    } else if(location.phi <= MAG_PS_MIN_LAT_DEGREE)
    {
        elements->GV = elements->Decl + location.lambda;
        return 1;
    } else
    {
        MAG_GetTransverseMercator(location, &UTMParameters);
        elements->GV = elements->Decl - UTMParameters.ConvergenceOfMeridians;
    }
    return 0;
} /*MAG_CalculateGridVariation*/


int MAG_CalculateGradientElements(MAGtype_MagneticResults GradResults, MAGtype_GeoMagneticElements MagneticElements, MAGtype_GeoMagneticElements *GradElements)
{
    GradElements->X = GradResults.Bx;
    GradElements->Y = GradResults.By;
    GradElements->Z = GradResults.Bz;

    GradElements->H = (GradElements->X * MagneticElements.X + GradElements->Y * MagneticElements.Y) / MagneticElements.H;
    GradElements->F = (GradElements->X * MagneticElements.X + GradElements->Y * MagneticElements.Y + GradElements->Z * MagneticElements.Z) / MagneticElements.F;
    GradElements->Decl = 180.0 / M_PI * (MagneticElements.X * GradElements->Y - MagneticElements.Y * GradElements->X) / (MagneticElements.H * MagneticElements.H);
    GradElements->Incl = 180.0 / M_PI * (MagneticElements.H * GradElements->Z - MagneticElements.Z * GradElements->H) / (MagneticElements.F * MagneticElements.F);
    GradElements->GV = GradElements->Decl;

    return TRUE;
}


int MAG_CalculateSecularVariationElements(MAGtype_MagneticResults MagneticVariation, MAGtype_GeoMagneticElements *MagneticElements)
/*This takes the Magnetic Variation in x, y, and z and uses it to calculate the secular variation of each of the Geomagnetic elements.
        INPUT     MagneticVariation   Data structure with the following elements
                                Escal Bx;    ( North )
                                Escal By;    ( East )
                                Escal Bz;    ( Down )
        OUTPUT   MagneticElements   Pointer to the data  structure with the following elements updated
                        Escal Decldot; Yearly Rate of change in declination
                        Escal Incldot; Yearly Rate of change in inclination
                        Escal Fdot; Yearly rate of change in Magnetic field strength
                        Escal Hdot; Yearly rate of change in horizontal field strength
                        Escal Xdot; Yearly rate of change in the northern component
                        Escal Ydot; Yearly rate of change in the eastern component
                        Escal Zdot; Yearly rate of change in the downward component
                        Escal GVdot;Yearly rate of chnage in grid variation
        CALLS : none

 */
{
    MagneticElements->Xdot = MagneticVariation.Bx;
    MagneticElements->Ydot = MagneticVariation.By;
    MagneticElements->Zdot = MagneticVariation.Bz;
    MagneticElements->Hdot = (MagneticElements->X * MagneticElements->Xdot + MagneticElements->Y * MagneticElements->Ydot) / MagneticElements->H; /* See equation 19 in the WMM technical report */
    MagneticElements->Fdot = (MagneticElements->X * MagneticElements->Xdot + MagneticElements->Y * MagneticElements->Ydot + MagneticElements->Z * MagneticElements->Zdot) / MagneticElements->F;
    MagneticElements->Decldot = 180.0 / M_PI * (MagneticElements->X * MagneticElements->Ydot - MagneticElements->Y * MagneticElements->Xdot) / (MagneticElements->H * MagneticElements->H);
    MagneticElements->Incldot = 180.0 / M_PI * (MagneticElements->H * MagneticElements->Zdot - MagneticElements->Z * MagneticElements->Hdot) / (MagneticElements->F * MagneticElements->F);
    MagneticElements->GVdot = MagneticElements->Decldot;
    return TRUE;
} /*MAG_CalculateSecularVariationElements*/


int MAG_CartesianToGeodetic(MAGtype_Ellipsoid Ellip, Escal x, Escal y, Escal z, MAGtype_CoordGeodetic *CoordGeodetic)
{
    /*This converts the Cartesian x, y, and z coordinates to Geodetic Coordinates
     x is defined as the direction pointing out of the core toward the point defined
     * by 0 degrees latitude and longitude.
     y is defined as the direction from the core toward 90 degrees east longitude along
     * the equator
     z is defined as the direction from the core out the geographic north pole*/

    Escal modified_b,r,e,f,p,q,d,v,g,t,zlong,rlat;

/*
 *   1.0 compute semi-minor axis and set sign to that of z in order
 *       to get sign of Phi correct
 */

  if (z < 0.0) modified_b = -Ellip.b;
  else  modified_b = Ellip.b;

/*
 *   2.0 compute intermediate values for latitude
 */
        r= sqrt( x*x + y*y );
        e= ( modified_b*z - (Ellip.a*Ellip.a - modified_b*modified_b) ) / ( Ellip.a*r );
        f= ( modified_b*z + (Ellip.a*Ellip.a - modified_b*modified_b) ) / ( Ellip.a*r );
/*
 *   3.0 find solution to:
 *       t^4 + 2*E*t^3 + 2*F*t - 1 = 0
 */
        p= (4.0 / 3.0) * (e*f + 1.0);
        q= 2.0 * (e*e - f*f);
        d= p*p*p + q*q;

        if( d >= 0.0 )
          {
            v= pow( (sqrt( d ) - q), (1.0 / 3.0) )
              - pow( (sqrt( d ) + q), (1.0 / 3.0) );
          }
        else
          {
            v= 2.0 * sqrt( -p )
              * cos( acos( q/(p * sqrt( -p )) ) / 3.0 );
          }
/*
 *   4.0 improve v
 *       NOTE: not really necessary unless point is near pole
 */
        if( v*v < fabs(p) ) {
                v= -(v*v*v + 2.0*q) / (3.0*p);
        }
        g = (sqrt( e*e + v ) + e) / 2.0;
        t = sqrt( g*g  + (f - v*g)/(2.0*g - e) ) - g;

        rlat =atan( (Ellip.a*(1.0 - t*t)) / (2.0*modified_b*t) );
        CoordGeodetic->phi = RAD2DEG(rlat);

/*
 *   5.0 compute height above ellipsoid
 */
        CoordGeodetic->HeightAboveEllipsoid = (r - Ellip.a*t) * cos(rlat) + (z - modified_b) * sin(rlat);
/*
 *   6.0 compute longitude east of Greenwich
 */
        zlong = atan2( y, x );
        if( zlong < 0.0 )
                zlong= zlong + 2*M_PI;

        CoordGeodetic->lambda = RAD2DEG(zlong);
        while(CoordGeodetic->lambda > 180)
        {
            CoordGeodetic->lambda-=360;
        }

    return TRUE;
}


MAGtype_CoordGeodetic MAG_CoordGeodeticAssign(MAGtype_CoordGeodetic CoordGeodetic)
{
    MAGtype_CoordGeodetic Assignee;
    Assignee.phi = CoordGeodetic.phi;
    Assignee.lambda = CoordGeodetic.lambda;
    Assignee.HeightAboveEllipsoid = CoordGeodetic.HeightAboveEllipsoid;
    Assignee.HeightAboveGeoid = CoordGeodetic.HeightAboveGeoid;
    Assignee.UseGeoid = CoordGeodetic.UseGeoid;
    return Assignee;
}


int MAG_GeodeticToSpherical(MAGtype_Ellipsoid Ellip, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_CoordSpherical *CoordSpherical)

/* Converts Geodetic coordinates to Spherical coordinates

  INPUT   Ellip  data  structure with the following elements
                        Escal a; semi-major axis of the ellipsoid
                        Escal b; semi-minor axis of the ellipsoid
                        Escal fla;  flattening
                        Escal epssq; first eccentricity squared
                        Escal eps;  first eccentricity
                        Escal re; mean radius of  ellipsoid

                CoordGeodetic  Pointer to the  data  structure with the following elements updates
                        Escal lambda; ( longitude )
                        Escal phi; ( geodetic latitude )
                        Escal HeightAboveEllipsoid; ( height above the WGS84 ellipsoid (HaE) )
                        Escal HeightAboveGeoid; (height above the EGM96 Geoid model )

 OUTPUT     CoordSpherical  Pointer to the data structure with the following elements
                        Escal lambda; ( longitude)
                        Escal phig; ( geocentric latitude )
                        Escal r;     ( distance from the center of the ellipsoid)

CALLS : none

 */
{
    Escal CosLat, SinLat, rc, xp, zp; /*all local variables */

    /*
     ** Convert geodetic coordinates, (defined by the WGS-84
     ** reference ellipsoid), to Earth Centered Earth Fixed Cartesian
     ** coordinates, and then to spherical coordinates.
     */

    CosLat = cos(DEG2RAD(CoordGeodetic.phi));
    SinLat = sin(DEG2RAD(CoordGeodetic.phi));

    /* compute the local radius of curvature on the WGS-84 reference ellipsoid */

    rc = Ellip.a / sqrt(1.0 - Ellip.epssq * SinLat * SinLat);

    /* compute ECEF Cartesian coordinates of specified point (for longitude=0) */

    xp = (rc + CoordGeodetic.HeightAboveEllipsoid) * CosLat;
    zp = (rc * (1.0 - Ellip.epssq) + CoordGeodetic.HeightAboveEllipsoid) * SinLat;

    /* compute spherical radius and angle lambda and phi of specified point */

    CoordSpherical->r = sqrt(xp * xp + zp * zp);
    CoordSpherical->phig = RAD2DEG(asin(zp / CoordSpherical->r)); /* geocentric latitude */
    CoordSpherical->lambda = CoordGeodetic.lambda; /* longitude */

    return TRUE;
}/*MAG_GeodeticToSpherical*/


MAGtype_GeoMagneticElements MAG_GeoMagneticElementsAssign(MAGtype_GeoMagneticElements Elements)
{
    MAGtype_GeoMagneticElements Assignee;
    Assignee.X = Elements.X;
    Assignee.Y = Elements.Y;
    Assignee.Z = Elements.Z;
    Assignee.H = Elements.H;
    Assignee.F = Elements.F;
    Assignee.Decl = Elements.Decl;
    Assignee.Incl = Elements.Incl;
    Assignee.GV = Elements.GV;
    Assignee.Xdot = Elements.Xdot;
    Assignee.Ydot = Elements.Ydot;
    Assignee.Zdot = Elements.Zdot;
    Assignee.Hdot = Elements.Hdot;
    Assignee.Fdot = Elements.Fdot;
    Assignee.Decldot = Elements.Decldot;
    Assignee.Incldot = Elements.Incldot;
    Assignee.GVdot = Elements.GVdot;
    return Assignee;
}


MAGtype_GeoMagneticElements MAG_GeoMagneticElementsScale(MAGtype_GeoMagneticElements Elements, Escal factor)
{
    /*This function scales all the geomagnetic elements to scale a vector use
     MAG_MagneticResultsScale*/
    MAGtype_GeoMagneticElements product;
    product.X = Elements.X * factor;
    product.Y = Elements.Y * factor;
    product.Z = Elements.Z * factor;
    product.H = Elements.H * factor;
    product.F = Elements.F * factor;
    product.Incl = Elements.Incl * factor;
    product.Decl = Elements.Decl * factor;
    product.GV = Elements.GV * factor;
    product.Xdot = Elements.Xdot * factor;
    product.Ydot = Elements.Ydot * factor;
    product.Zdot = Elements.Zdot * factor;
    product.Hdot = Elements.Hdot * factor;
    product.Fdot = Elements.Fdot * factor;
    product.Incldot = Elements.Incldot * factor;
    product.Decldot = Elements.Decldot * factor;
    product.GVdot = Elements.GVdot * factor;
    return product;
}


MAGtype_GeoMagneticElements MAG_GeoMagneticElementsSubtract(MAGtype_GeoMagneticElements minuend, MAGtype_GeoMagneticElements subtrahend)
{
    /*This algorithm does not result in the difference of F being derived from
     the Pythagorean theorem.  This function should be used for computing residuals
     or changes in elements.*/
    MAGtype_GeoMagneticElements difference;
    difference.X = minuend.X - subtrahend.X;
    difference.Y = minuend.Y - subtrahend.Y;
    difference.Z = minuend.Z - subtrahend.Z;

    difference.H = minuend.H - subtrahend.H;
    difference.F = minuend.F - subtrahend.F;
    difference.Decl = minuend.Decl - subtrahend.Decl;
    difference.Incl = minuend.Incl - subtrahend.Incl;

    difference.Xdot = minuend.Xdot - subtrahend.Xdot;
    difference.Ydot = minuend.Ydot - subtrahend.Ydot;
    difference.Zdot = minuend.Zdot - subtrahend.Zdot;

    difference.Hdot = minuend.Hdot - subtrahend.Hdot;
    difference.Fdot = minuend.Fdot - subtrahend.Fdot;
    difference.Decldot = minuend.Decldot - subtrahend.Decldot;
    difference.Incldot = minuend.Incldot - subtrahend.Incldot;

    difference.GV = minuend.GV - subtrahend.GV;
    difference.GVdot = minuend.GVdot - subtrahend.GVdot;

    return difference;
}


int MAG_GetTransverseMercator(MAGtype_CoordGeodetic CoordGeodetic, MAGtype_UTMParameters *UTMParameters)
/* Gets the UTM Parameters for a given Latitude and Longitude.

INPUT: CoordGeodetic : Data structure MAGtype_CoordGeodetic.
OUTPUT : UTMParameters : Pointer to data structure MAGtype_UTMParameters with the following elements
                     Escal Easting;  (X) in meters
                     Escal Northing; (Y) in meters
                     int Zone; UTM Zone
                     char HemiSphere ;
                     Escal CentralMeridian; Longitude of the Central Meridian of the UTM Zone
                     Escal ConvergenceOfMeridians;  Convergence of Meridians
                     Escal PointScale;
 */
{

    Escal Eps, Epssq;
    Escal Acoeff[8];
    Escal Lam0, K0, falseE, falseN;
    Escal K0R4, K0R4oa;
    Escal Lambda, Phi;
    int XYonly;
    Escal X, Y, pscale, CoM;
    int Zone;
    char Hemisphere;



    /*   Get the map projection  parameters */

    Lambda = DEG2RAD(CoordGeodetic.lambda);
    Phi = DEG2RAD(CoordGeodetic.phi);

    MAG_GetUTMParameters(Phi, Lambda, &Zone, &Hemisphere, &Lam0);
    K0 = 0.9996;



    if(Hemisphere == 'n' || Hemisphere == 'N')
    {
        falseN = 0;
    }
    if(Hemisphere == 's' || Hemisphere == 'S')
    {
        falseN = 10000000;
    }

    falseE = 500000;


    /* WGS84 ellipsoid */

    Eps = 0.081819190842621494335;
    Epssq = 0.0066943799901413169961;
    K0R4 = 6367449.1458234153093*K0;
    K0R4oa = K0R4/6378137;


    Acoeff[0] = 8.37731820624469723600E-04;
    Acoeff[1] = 7.60852777357248641400E-07;
    Acoeff[2] = 1.19764550324249124400E-09;
    Acoeff[3] = 2.42917068039708917100E-12;
    Acoeff[4] = 5.71181837042801392800E-15;
    Acoeff[5] = 1.47999793137966169400E-17;
    Acoeff[6] = 4.10762410937071532000E-20;
    Acoeff[7] = 1.21078503892257704200E-22;

    /* WGS84 ellipsoid */


    /*   Execution of the forward T.M. algorithm  */

    XYonly = 0;

    MAG_TMfwd4(Eps, Epssq, K0R4, K0R4oa, Acoeff,
            Lam0, K0, falseE, falseN,
            XYonly,
            Lambda, Phi,
            &X, &Y, &pscale, &CoM);

    /*   Report results  */

    UTMParameters->Easting = X; /* UTM Easting (X) in meters*/
    UTMParameters->Northing = Y; /* UTM Northing (Y) in meters */
    UTMParameters->Zone = Zone; /*UTM Zone*/
    UTMParameters->HemiSphere = Hemisphere;
    UTMParameters->CentralMeridian = RAD2DEG(Lam0); /* Central Meridian of the UTM Zone */
    UTMParameters->ConvergenceOfMeridians = RAD2DEG(CoM); /* Convergence of meridians of the UTM Zone and location */
    UTMParameters->PointScale = pscale;

    return 0;
} /*MAG_GetTransverseMercator*/


int MAG_GetUTMParameters(Escal Latitude,
        Escal Longitude,
        int *Zone,
        char *Hemisphere,
        Escal *CentralMeridian)
{
    /*
     * The function MAG_GetUTMParameters converts geodetic (latitude and
     * longitude) coordinates to UTM projection parameters (zone, hemisphere and central meridian)
     * If any errors occur, the error code(s) are returned
     * by the function, otherwise TRUE is returned.
     *
     *    Latitude          : Latitude in radians                 (input)
     *    Longitude         : Longitude in radians                (input)
     *    Zone              : UTM zone                            (output)
     *    Hemisphere        : North or South hemisphere           (output)
     *    CentralMeridian   : Central Meridian of the UTM Zone in radians      (output)
     */

    long Lat_Degrees;
    long Long_Degrees;
    long temp_zone;
    int Error_Code = 0;



    if((Latitude < DEG2RAD(MAG_UTM_MIN_LAT_DEGREE)) || (Latitude > DEG2RAD(MAG_UTM_MAX_LAT_DEGREE)))
    { /* Latitude out of range */
        MAG_Error(23);
        Error_Code = 1;
    }
    if((Longitude < -M_PI) || (Longitude > (2 * M_PI)))
    { /* Longitude out of range */
        MAG_Error(24);
        Error_Code = 1;
    }
    if(!Error_Code)
    { /* no errors */
        if(Longitude < 0)
            Longitude += (2 * M_PI) + 1.0e-10;
        Lat_Degrees = (long) (Latitude * 180.0 / M_PI);
        Long_Degrees = (long) (Longitude * 180.0 / M_PI);

        if(Longitude < M_PI)
            temp_zone = (long) (31 + ((Longitude * 180.0 / M_PI) / 6.0));
        else
            temp_zone = (long) (((Longitude * 180.0 / M_PI) / 6.0) - 29);
        if(temp_zone > 60)
            temp_zone = 1;
        /* UTM special cases */
        if((Lat_Degrees > 55) && (Lat_Degrees < 64) && (Long_Degrees > -1)
                && (Long_Degrees < 3))
            temp_zone = 31;
        if((Lat_Degrees > 55) && (Lat_Degrees < 64) && (Long_Degrees > 2)
                && (Long_Degrees < 12))
            temp_zone = 32;
        if((Lat_Degrees > 71) && (Long_Degrees > -1) && (Long_Degrees < 9))
            temp_zone = 31;
        if((Lat_Degrees > 71) && (Long_Degrees > 8) && (Long_Degrees < 21))
            temp_zone = 33;
        if((Lat_Degrees > 71) && (Long_Degrees > 20) && (Long_Degrees < 33))
            temp_zone = 35;
        if((Lat_Degrees > 71) && (Long_Degrees > 32) && (Long_Degrees < 42))
            temp_zone = 37;

        if(!Error_Code)
        {
            if(temp_zone >= 31)
                *CentralMeridian = (6 * temp_zone - 183) * M_PI / 180.0;
            else
                *CentralMeridian = (6 * temp_zone + 177) * M_PI / 180.0;
            *Zone = temp_zone;
            if(Latitude < 0) *Hemisphere = 'S';
            else *Hemisphere = 'N';
        }
    } /* END OF if (!Error_Code) */
    return (Error_Code);
} /* MAG_GetUTMParameters */


int MAG_isNaN(Escal d)
{
    return d != d;
}


int MAG_RotateMagneticVector(MAGtype_CoordSpherical CoordSpherical, MAGtype_CoordGeodetic CoordGeodetic, MAGtype_MagneticResults MagneticResultsSph, MAGtype_MagneticResults *MagneticResultsGeo)
/* Rotate the Magnetic Vectors to Geodetic Coordinates
Manoj Nair, June, 2009 Manoj.C.Nair@Noaa.Gov
Equation 16, WMM Technical report

INPUT : CoordSpherical : Data structure MAGtype_CoordSpherical with the following elements
                        Escal lambda; ( longitude)
                        Escal phig; ( geocentric latitude )
                        Escal r;     ( distance from the center of the ellipsoid)

                CoordGeodetic : Data structure MAGtype_CoordGeodetic with the following elements
                        Escal lambda; (longitude)
                        Escal phi; ( geodetic latitude)
                        Escal HeightAboveEllipsoid; (height above the ellipsoid (HaE) )
                        Escal HeightAboveGeoid;(height above the Geoid )

                MagneticResultsSph : Data structure MAGtype_MagneticResults with the following elements
                        Escal Bx;      North
                        Escal By;      East
                        Escal Bz;      Down

OUTPUT: MagneticResultsGeo Pointer to the data structure MAGtype_MagneticResults, with the following elements
                        Escal Bx;      North
                        Escal By;      East
                        Escal Bz;      Down

CALLS : none

 */
{
    Escal Psi;
    /* Difference between the spherical and Geodetic latitudes */
    Psi = (M_PI / 180) * (CoordSpherical.phig - CoordGeodetic.phi);

    /* Rotate spherical field components to the Geodetic system */
    MagneticResultsGeo->Bz = MagneticResultsSph.Bx * sin(Psi) + MagneticResultsSph.Bz * cos(Psi);
    MagneticResultsGeo->Bx = MagneticResultsSph.Bx * cos(Psi) - MagneticResultsSph.Bz * sin(Psi);
    MagneticResultsGeo->By = MagneticResultsSph.By;
    return TRUE;
} /*MAG_RotateMagneticVector*/


int MAG_SphericalToCartesian(MAGtype_CoordSpherical CoordSpherical, Escal *x, Escal *y, Escal *z)
{
    Escal radphi;
    Escal radlambda;

    radphi = CoordSpherical.phig * (M_PI / 180);
    radlambda = CoordSpherical.lambda * (M_PI / 180);

    *x = CoordSpherical.r * cos(radphi) * cos(radlambda);
    *y = CoordSpherical.r * cos(radphi) * sin(radlambda);
    *z = CoordSpherical.r * sin(radphi);

    return TRUE;
}


int MAG_SphericalToGeodetic(MAGtype_Ellipsoid Ellip, MAGtype_CoordSpherical CoordSpherical, MAGtype_CoordGeodetic *CoordGeodetic)
{
    /*This converts spherical coordinates back to geodetic coordinates.  It is not used in the WMM but
     may be necessary for some applications, such as geomagnetic coordinates*/
     Escal x,y,z;

   MAG_SphericalToCartesian(CoordSpherical, &x,&y,&z);
   MAG_CartesianToGeodetic(Ellip, x,y,z,CoordGeodetic);

   return TRUE;
}


int MAG_TMfwd4(Escal Eps, Escal Epssq, Escal K0R4, Escal K0R4oa,
        Escal Acoeff[], Escal Lam0, Escal K0, Escal falseE,
        Escal falseN, int XYonly, Escal Lambda, Escal Phi,
        Escal *X, Escal *Y, Escal *pscale, Escal *CoM)
{

    /*  Transverse Mercator forward equations including point-scale and CoM
            =--------- =------- =--=--= ---------

       Algorithm developed by: C. Rollins   August 7, 2006
       C software written by:  K. Robins


            Constants fixed by choice of ellipsoid and choice of projection parameters
            ---------------

              Eps          Eccentricity (epsilon) of the ellipsoid
              Epssq        Eccentricity squared
            ( R4           Meridional isoperimetric radius   )
            ( K0           Central scale factor              )
              K0R4         K0 times R4
              K0R4oa       K0 times Ratio of R4 over semi-major axis
              Acoeff       Trig series coefficients, omega as a function of chi
              Lam0         Longitude of the central meridian in radians
              K0           Central scale factor, for example, 0.9996 for UTM
              falseE       False easting, for example, 500000 for UTM
              falseN       False northing

       Processing option
       ---------- ------

              XYonly       If one (1), then only X and Y will be properly
                                       computed.  Values returned for point-scale
                                       and CoM will merely be the trivial values for
                                       points on the central meridian

       Input items that identify the point to be converted
       ----- -----

              Lambda       Longitude (from Greenwich) in radians
              Phi          Latitude in radians

       Output items
       ------ -----

              X            X coordinate (Easting) in meters
              Y            Y coordinate (Northing) in meters
              pscale       point-scale (dimensionless)
          CoM          Convergence-of-meridians in radians
     */

    Escal Lam, CLam, SLam, CPhi, SPhi;
    Escal P, part1, part2, denom, CChi, SChi;
    Escal U, V;
    Escal T, Tsq, denom2;
    Escal c2u, s2u, c4u, s4u, c6u, s6u, c8u, s8u;
    Escal c2v, s2v, c4v, s4v, c6v, s6v, c8v, s8v;
    Escal Xstar, Ystar;
    Escal sig1, sig2, comroo;

    /*
       Ellipsoid to sphere
       --------- -- ------

       Convert longitude (Greenwhich) to longitude from the central meridian
       It is unnecessary to find the (-Pi, Pi] equivalent of the result.
       Compute its cosine and sine.
     */

    Lam = Lambda - Lam0;
    CLam = cos(Lam);
    SLam = sin(Lam);

    /*   Latitude  */

    CPhi = cos(Phi);
    SPhi = sin(Phi);

    /*   Convert geodetic latitude, Phi, to conformal latitude, Chi
         Only the cosine and sine of Chi are actually needed.        */

    P = exp(Eps * ATanH(Eps * SPhi));
    part1 = (1 + SPhi) / P;
    part2 = (1 - SPhi) * P;
    denom = 1 / (part1 + part2);
    CChi = 2 * CPhi * denom;
    SChi = (part1 - part2) * denom;

    /*
       Sphere to first plane
       ------ -- ----- -----

       Apply spherical theory of transverse Mercator to get (u,v) coordinates
       Note the order of the arguments in Fortran's version of ArcTan, i.e.
                 atan2(y, x) = ATan(y/x)
       The two argument form of ArcTan is needed here.
     */

    T = CChi * SLam;
    U = ATanH(T);
    V = atan2(SChi, CChi * CLam);

    /*
       Trigonometric multiple angles
       ------------- -------- ------

       Compute Cosh of even multiples of U
       Compute Sinh of even multiples of U
       Compute Cos  of even multiples of V
       Compute Sin  of even multiples of V
     */

    Tsq = T * T;
    denom2 = 1 / (1 - Tsq);
    c2u = (1 + Tsq) * denom2;
    s2u = 2 * T * denom2;
    c2v = (-1 + CChi * CChi * (1 + CLam * CLam)) * denom2;
    s2v = 2 * CLam * CChi * SChi * denom2;

    c4u = 1 + 2 * s2u * s2u;
    s4u = 2 * c2u * s2u;
    c4v = 1 - 2 * s2v * s2v;
    s4v = 2 * c2v * s2v;

    c6u = c4u * c2u + s4u * s2u;
    s6u = s4u * c2u + c4u * s2u;
    c6v = c4v * c2v - s4v * s2v;
    s6v = s4v * c2v + c4v * s2v;

    c8u = 1 + 2 * s4u * s4u;
    s8u = 2 * c4u * s4u;
    c8v = 1 - 2 * s4v * s4v;
    s8v = 2 * c4v * s4v;


    /*   First plane to second plane
         ----- ----- -- ------ -----

         Accumulate terms for X and Y
     */

    Xstar = Acoeff[3] * s8u * c8v;
    Xstar = Xstar + Acoeff[2] * s6u * c6v;
    Xstar = Xstar + Acoeff[1] * s4u * c4v;
    Xstar = Xstar + Acoeff[0] * s2u * c2v;
    Xstar = Xstar + U;

    Ystar = Acoeff[3] * c8u * s8v;
    Ystar = Ystar + Acoeff[2] * c6u * s6v;
    Ystar = Ystar + Acoeff[1] * c4u * s4v;
    Ystar = Ystar + Acoeff[0] * c2u * s2v;
    Ystar = Ystar + V;

    /*   Apply isoperimetric radius, scale adjustment, and offsets  */

    *X = K0R4 * Xstar + falseE;
    *Y = K0R4 * Ystar + falseN;


    /*  Point-scale and CoM
        ----- ----- --- ---  */

    if(XYonly == 1)
    {
        *pscale = K0;
        *CoM = 0;
    } else
    {
        sig1 = 8 * Acoeff[3] * c8u * c8v;
        sig1 = sig1 + 6 * Acoeff[2] * c6u * c6v;
        sig1 = sig1 + 4 * Acoeff[1] * c4u * c4v;
        sig1 = sig1 + 2 * Acoeff[0] * c2u * c2v;
        sig1 = sig1 + 1;

        sig2 = 8 * Acoeff[3] * s8u * s8v;
        sig2 = sig2 + 6 * Acoeff[2] * s6u * s6v;
        sig2 = sig2 + 4 * Acoeff[1] * s4u * s4v;
        sig2 = sig2 + 2 * Acoeff[0] * s2u * s2v;

        /*    Combined square roots  */
        comroo = sqrt((1 - Epssq * SPhi * SPhi) * denom2 *
                (sig1 * sig1 + sig2 * sig2));

        *pscale = K0R4oa * 2 * denom * comroo;
        *CoM = atan2(SChi * SLam, CLam) + atan2(sig2, sig1);
    }

    return TRUE;
} /*MAG_TMfwd4*/


/******************************************************************************
 ********************************Spherical Harmonics***************************
 * This grouping consists of functions that together take gauss coefficients
 * and return a magnetic vector for an input location in spherical coordinates
 ******************************************************************************/


int MAG_AssociatedLegendreFunction(MAGtype_CoordSpherical CoordSpherical, int nMax, MAGtype_LegendreFunction *LegendreFunction)

/* Computes  all of the Schmidt-semi normalized associated Legendre
functions up to degree nMax. If nMax <= 16, function MAG_PcupLow is used.
Otherwise MAG_PcupHigh is called.
INPUT  CoordSpherical   A data structure with the following elements
                                                Escal lambda; ( longitude)
                                                Escal phig; ( geocentric latitude )
                                                Escal r;     ( distance from the center of the ellipsoid)
                nMax            integer      ( Maxumum degree of spherical harmonic secular model)
                LegendreFunction Pointer to data structure with the following elements
                                                Escal *Pcup;  (  pointer to store Legendre Function  )
                                                Escal *dPcup; ( pointer to store  Derivative of Lagendre function )

OUTPUT  LegendreFunction  Calculated Legendre variables in the data structure

 */
{
    Escal sin_phi;
    int FLAG = 1;

    sin_phi = sin(DEG2RAD(CoordSpherical.phig)); /* sin  (geocentric latitude) */

    if(nMax <= 16 || (1 - fabs(sin_phi)) < 1.0e-10) /* If nMax is less tha 16 or at the poles */
        FLAG = MAG_PcupLow(LegendreFunction->Pcup, LegendreFunction->dPcup, sin_phi, nMax);
    else FLAG = MAG_PcupHigh(LegendreFunction->Pcup, LegendreFunction->dPcup, sin_phi, nMax);
    if(FLAG == 0) /* Error while computing  Legendre variables*/
        return FALSE;


    return TRUE;
} /*MAG_AssociatedLegendreFunction */


int MAG_CheckGeographicPole(MAGtype_CoordGeodetic *CoordGeodetic)

/* Check if the latitude is equal to -90 or 90. If it is,
offset it by 1e-5 to aint division by zero. This is not currently used in the Geomagnetic
main function. This may be used to aint calling MAG_SummationSpecial.
The function updates the input data structure.

INPUT   CoordGeodetic Pointer to the  data  structure with the following elements
                Escal lambda; (longitude)
                Escal phi; ( geodetic latitude)
                Escal HeightAboveEllipsoid; (height above the ellipsoid (HaE) )
                Escal HeightAboveGeoid;(height above the Geoid )
OUTPUT  CoordGeodetic  Pointer to the  data  structure with the following elements updates
                Escal phi; ( geodetic latitude)
CALLS : none

 */
{
    CoordGeodetic->phi = CoordGeodetic->phi < (-90.0 + MAG_GEO_POLE_TOLERANCE) ? (-90.0 + MAG_GEO_POLE_TOLERANCE) : CoordGeodetic->phi;
    CoordGeodetic->phi = CoordGeodetic->phi > (90.0 - MAG_GEO_POLE_TOLERANCE) ? (90.0 - MAG_GEO_POLE_TOLERANCE) : CoordGeodetic->phi;
    return TRUE;
} /*MAG_CheckGeographicPole*/


int MAG_ComputeSphericalHarmonicVariables(MAGtype_Ellipsoid Ellip, MAGtype_CoordSpherical CoordSpherical, int nMax, MAGtype_SphericalHarmonicVariables *SphVariables)

/* Computes Spherical variables
       Variables computed are (a/r)^(n+2), cos_m(lamda) and sin_m(lambda) for spherical harmonic
       summations. (Equations 10-12 in the WMM Technical Report)
       INPUT   Ellip  data  structure with the following elements
                             Escal a; semi-major axis of the ellipsoid
                             Escal b; semi-minor axis of the ellipsoid
                             Escal fla;  flattening
                             Escal epssq; first eccentricity squared
                             Escal eps;  first eccentricity
                             Escal re; mean radius of  ellipsoid
                     CoordSpherical     A data structure with the following elements
                             Escal lambda; ( longitude)
                             Escal phig; ( geocentric latitude )
                             Escal r;        ( distance from the center of the ellipsoid)
                     nMax   integer      ( Maxumum degree of spherical harmonic secular model)\

     OUTPUT  SphVariables  Pointer to the   data structure with the following elements
             Escal RelativeRadiusPower[MAG_MAX_MODEL_DEGREES+1];   [earth_reference_radius_km  sph. radius ]^n
             Escal cos_mlambda[MAG_MAX_MODEL_DEGREES+1]; cp(m)  - cosine of (mspherical coord. longitude)
             Escal sin_mlambda[MAG_MAX_MODEL_DEGREES+1];  sp(m)  - sine of (mspherical coord. longitude)
     CALLS : none
 */
{
    Escal cos_lambda, sin_lambda;
    int m, n;
    cos_lambda = cos(DEG2RAD(CoordSpherical.lambda));
    sin_lambda = sin(DEG2RAD(CoordSpherical.lambda));
    /* for n = 0 ... model_order, compute (Radius of Earth / Spherical radius r)^(n+2)
    for n  1..nMax-1 (this is much faster than calling pow MAX_N+1 times).      */
    SphVariables->RelativeRadiusPower[0] = (Ellip.re / CoordSpherical.r) * (Ellip.re / CoordSpherical.r);
    for(n = 1; n <= nMax; n++)
    {
        SphVariables->RelativeRadiusPower[n] = SphVariables->RelativeRadiusPower[n - 1] * (Ellip.re / CoordSpherical.r);
    }

    /*
     Compute cos(m*lambda), sin(m*lambda) for m = 0 ... nMax
           cos(a + b) = cos(a)*cos(b) - sin(a)*sin(b)
           sin(a + b) = cos(a)*sin(b) + sin(a)*cos(b)
     */
    SphVariables->cos_mlambda[0] = 1.0;
    SphVariables->sin_mlambda[0] = 0.0;

    SphVariables->cos_mlambda[1] = cos_lambda;
    SphVariables->sin_mlambda[1] = sin_lambda;
    for(m = 2; m <= nMax; m++)
    {
        SphVariables->cos_mlambda[m] = SphVariables->cos_mlambda[m - 1] * cos_lambda - SphVariables->sin_mlambda[m - 1] * sin_lambda;
        SphVariables->sin_mlambda[m] = SphVariables->cos_mlambda[m - 1] * sin_lambda + SphVariables->sin_mlambda[m - 1] * cos_lambda;
    }
    return TRUE;
} /*MAG_ComputeSphericalHarmonicVariables*/


int MAG_GradY(MAGtype_Ellipsoid Ellip, MAGtype_CoordSpherical CoordSpherical, MAGtype_CoordGeodetic CoordGeodetic,
        MAGtype_MagneticModel *TimedMagneticModel, MAGtype_GeoMagneticElements GeoMagneticElements, MAGtype_GeoMagneticElements *GradYElements)
{
    MAGtype_LegendreFunction *LegendreFunction;
    MAGtype_SphericalHarmonicVariables *SphVariables;
    int NumTerms;
    MAGtype_MagneticResults GradYResultsSph, GradYResultsGeo;

    NumTerms = ((TimedMagneticModel->nMax + 1) * (TimedMagneticModel->nMax + 2) / 2);
    LegendreFunction = MAG_AllocateLegendreFunctionMemory(NumTerms); /* For storing the ALF functions */
    SphVariables = MAG_AllocateSphVarMemory(TimedMagneticModel->nMax);
    MAG_ComputeSphericalHarmonicVariables(Ellip, CoordSpherical, TimedMagneticModel->nMax, SphVariables); /* Compute Spherical Harmonic variables  */
    MAG_AssociatedLegendreFunction(CoordSpherical, TimedMagneticModel->nMax, LegendreFunction); /* Compute ALF  */
    MAG_GradYSummation(LegendreFunction, TimedMagneticModel, *SphVariables, CoordSpherical, &GradYResultsSph); /* Accumulate the spherical harmonic coefficients*/
    MAG_RotateMagneticVector(CoordSpherical, CoordGeodetic, GradYResultsSph, &GradYResultsGeo); /* Map the computed Magnetic fields to Geodetic coordinates  */
    MAG_CalculateGradientElements(GradYResultsGeo, GeoMagneticElements, GradYElements); /* Calculate the Geomagnetic elements, Equation 18 , WMM Technical report */

    MAG_FreeLegendreMemory(LegendreFunction);
    MAG_FreeSphVarMemory(SphVariables);

    return TRUE;
}


int MAG_GradYSummation(MAGtype_LegendreFunction *LegendreFunction, MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *GradY)
{
    int m, n, index;
    Escal cos_phi;
    GradY->Bz = 0.0;
    GradY->By = 0.0;
    GradY->Bx = 0.0;
    for(n = 1; n <= MagneticModel->nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);

            GradY->Bz -= SphVariables.RelativeRadiusPower[n] *
                    (-1 * MagneticModel->Main_Field_Coeff_G[index] * SphVariables.sin_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.cos_mlambda[m])
                    * (Escal) (n + 1) * (Escal) (m) * LegendreFunction-> Pcup[index] * (1/CoordSpherical.r);
            GradY->By += SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables.cos_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.sin_mlambda[m])
                    * (Escal) (m * m) * LegendreFunction-> Pcup[index] * (1/CoordSpherical.r);
            GradY->Bx -= SphVariables.RelativeRadiusPower[n] *
                    (-1 * MagneticModel->Main_Field_Coeff_G[index] * SphVariables.sin_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.cos_mlambda[m])
                    * (Escal) (m) * LegendreFunction-> dPcup[index] * (1/CoordSpherical.r);



        }
    }

    cos_phi = cos(DEG2RAD(CoordSpherical.phig));
    if(fabs(cos_phi) > 1.0e-10)
    {
        GradY->By = GradY->By / (cos_phi * cos_phi);
        GradY->Bx = GradY->Bx / (cos_phi);
        GradY->Bz = GradY->Bz / (cos_phi);
    } else
        /* Special calculation for component - By - at Geographic poles.
         * If the user wants to aint using this function,  please make sure that
         * the latitude is not exactly +/-90. An option is to make use the function
         * MAG_CheckGeographicPoles.
         */
    {
       /* MAG_SummationSpecial(MagneticModel, SphVariables, CoordSpherical, GradY); */
    }

    return TRUE;
}


int MAG_PcupHigh(Escal *Pcup, Escal *dPcup, Escal x, int nMax)

/*  This function evaluates all of the Schmidt-semi normalized associated Legendre
        functions up to degree nMax. The functions are initially scaled by
        10^280 sin^m in order to minimize the effects of underflow at large m
        near the poles (see Holmes and Featherstone 2002, J. Geodesy, 76, 279-299).
        Note that this function performs the same operation as MAG_PcupLow.
        However this function also can be used for high degree (large nMax) models.

        Calling Parameters:
                INPUT
                        nMax:    Maximum spherical harmonic degree to compute.
                        x:      cos(colatitude) or sin(latitude).

                OUTPUT
                        Pcup:   A vector of all associated Legendgre polynomials evaluated at
                                        x up to nMax. The lenght must by greater or equal to (nMax+1)*(nMax+2)/2.
                  dPcup:   Derivative of Pcup(x) with respect to latitude

                CALLS : none
        Notes:



  Adopted from the FORTRAN code written by Mark Wieczorek September 25, 2005.

  Manoj Nair, Nov, 2009 Manoj.C.Nair@Noaa.Gov

  Change from the previous version
  The prevous version computes the derivatives as
  dP(n,m)(x)/dx, where x = sin(latitude) (or cos(colatitude) ).
  However, the WMM Geomagnetic routines requires dP(n,m)(x)/dlatitude.
  Hence the derivatives are multiplied by sin(latitude).
  Removed the options for CS phase and normalizations.

  Note: In geomagnetism, the derivatives of ALF are usually found with
  respect to the colatitudes. Here the derivatives are found with respect
  to the latitude. The difference is a sign reversal for the derivative of
  the Associated Legendre Functions.

  The derivatives can't be computed for latitude = |90| degrees.
 */
{
    Escal pm2, pm1, pmm, plm, rescalem, z, scalef;
    Escal *f1, *f2, *PreSqr;
    int k, kstart, m, n, NumTerms;

    NumTerms = ((nMax + 1) * (nMax + 2) / 2);


    if(fabs(x) == 1.0)
    {
        printf("Error in PcupHigh: derivative cannot be calculated at poles\n");
        return FALSE;
    }


    f1 = (Escal *) malloc((NumTerms + 1) * sizeof ( Escal));
    if(f1 == NULL)
    {
        MAG_Error(18);
        return FALSE;
    }


    PreSqr = (Escal *) malloc((NumTerms + 1) * sizeof ( Escal));

    if(PreSqr == NULL)
    {
        MAG_Error(18);
        return FALSE;
    }

    f2 = (Escal *) malloc((NumTerms + 1) * sizeof ( Escal));

    if(f2 == NULL)
    {
        MAG_Error(18);
        return FALSE;
    }

    scalef = 1.0e-280;

    for(n = 0; n <= 2 * nMax + 1; ++n)
    {
        PreSqr[n] = sqrt((Escal) (n));
    }

    k = 2;

    for(n = 2; n <= nMax; n++)
    {
        k = k + 1;
        f1[k] = (Escal) (2 * n - 1) / (Escal) (n);
        f2[k] = (Escal) (n - 1) / (Escal) (n);
        for(m = 1; m <= n - 2; m++)
        {
            k = k + 1;
            f1[k] = (Escal) (2 * n - 1) / PreSqr[n + m] / PreSqr[n - m];
            f2[k] = PreSqr[n - m - 1] * PreSqr[n + m - 1] / PreSqr[n + m] / PreSqr[n - m];
        }
        k = k + 2;
    }

    /*z = sin (geocentric latitude) */
    z = sqrt((1.0 - x)*(1.0 + x));
    pm2 = 1.0;
    Pcup[0] = 1.0;
    dPcup[0] = 0.0;
    if(nMax == 0)
        return FALSE;
    pm1 = x;
    Pcup[1] = pm1;
    dPcup[1] = z;
    k = 1;

    for(n = 2; n <= nMax; n++)
    {
        k = k + n;
        plm = f1[k] * x * pm1 - f2[k] * pm2;
        Pcup[k] = plm;
        dPcup[k] = (Escal) (n) * (pm1 - x * plm) / z;
        pm2 = pm1;
        pm1 = plm;
    }

    pmm = PreSqr[2] * scalef;
    rescalem = 1.0 / scalef;
    kstart = 0;

    for(m = 1; m <= nMax - 1; ++m)
    {
        rescalem = rescalem*z;

        /* Calculate Pcup(m,m)*/
        kstart = kstart + m + 1;
        pmm = pmm * PreSqr[2 * m + 1] / PreSqr[2 * m];
        Pcup[kstart] = pmm * rescalem / PreSqr[2 * m + 1];
        dPcup[kstart] = -((Escal) (m) * x * Pcup[kstart] / z);
        pm2 = pmm / PreSqr[2 * m + 1];
        /* Calculate Pcup(m+1,m)*/
        k = kstart + m + 1;
        pm1 = x * PreSqr[2 * m + 1] * pm2;
        Pcup[k] = pm1*rescalem;
        dPcup[k] = ((pm2 * rescalem) * PreSqr[2 * m + 1] - x * (Escal) (m + 1) * Pcup[k]) / z;
        /* Calculate Pcup(n,m)*/
        for(n = m + 2; n <= nMax; ++n)
        {
            k = k + n;
            plm = x * f1[k] * pm1 - f2[k] * pm2;
            Pcup[k] = plm*rescalem;
            dPcup[k] = (PreSqr[n + m] * PreSqr[n - m] * (pm1 * rescalem) - (Escal) (n) * x * Pcup[k]) / z;
            pm2 = pm1;
            pm1 = plm;
        }
    }

    /* Calculate Pcup(nMax,nMax)*/
    rescalem = rescalem*z;
    kstart = kstart + m + 1;
    pmm = pmm / PreSqr[2 * nMax];
    Pcup[kstart] = pmm * rescalem;
    dPcup[kstart] = -(Escal) (nMax) * x * Pcup[kstart] / z;
    free(f1);
    free(PreSqr);
    free(f2);

    return TRUE;
} /* MAG_PcupHigh */


int MAG_PcupLow(Escal *Pcup, Escal *dPcup, Escal x, int nMax)

/*   This function evaluates all of the Schmidt-semi normalized associated Legendre
        functions up to degree nMax.

        Calling Parameters:
                INPUT
                        nMax:    Maximum spherical harmonic degree to compute.
                        x:      cos(colatitude) or sin(latitude).

                OUTPUT
                        Pcup:   A vector of all associated Legendgre polynomials evaluated at
                                        x up to nMax.
                   dPcup: Derivative of Pcup(x) with respect to latitude

        Notes: Overflow may occur if nMax > 20 , especially for high-latitudes.
        Use MAG_PcupHigh for large nMax.

   Written by Manoj Nair, June, 2009 . Manoj.C.Nair@Noaa.Gov.

  Note: In geomagnetism, the derivatives of ALF are usually found with
  respect to the colatitudes. Here the derivatives are found with respect
  to the latitude. The difference is a sign reversal for the derivative of
  the Associated Legendre Functions.
 */
{
    int n, m, index, index1, index2, NumTerms;
    Escal k, z, *schmidtQuasiNorm;
    Pcup[0] = 1.0;
    dPcup[0] = 0.0;
    /*sin (geocentric latitude) - sin_phi */
    z = sqrt((1.0 - x) * (1.0 + x));

    NumTerms = ((nMax + 1) * (nMax + 2) / 2);
    schmidtQuasiNorm = (Escal *) malloc((NumTerms + 1) * sizeof ( Escal));

    if(schmidtQuasiNorm == NULL)
    {
        MAG_Error(19);
        return FALSE;
    }

    /*   First, Compute the Gauss-normalized associated Legendre  functions*/
    for(n = 1; n <= nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            if(n == m)
            {
                index1 = (n - 1) * n / 2 + m - 1;
                Pcup [index] = z * Pcup[index1];
                dPcup[index] = z * dPcup[index1] + x * Pcup[index1];
            } else if(n == 1 && m == 0)
            {
                index1 = (n - 1) * n / 2 + m;
                Pcup[index] = x * Pcup[index1];
                dPcup[index] = x * dPcup[index1] - z * Pcup[index1];
            } else if(n > 1 && n != m)
            {
                index1 = (n - 2) * (n - 1) / 2 + m;
                index2 = (n - 1) * n / 2 + m;
                if(m > n - 2)
                {
                    Pcup[index] = x * Pcup[index2];
                    dPcup[index] = x * dPcup[index2] - z * Pcup[index2];
                } else
                {
                    k = (Escal) (((n - 1) * (n - 1)) - (m * m)) / (Escal) ((2 * n - 1) * (2 * n - 3));
                    Pcup[index] = x * Pcup[index2] - k * Pcup[index1];
                    dPcup[index] = x * dPcup[index2] - z * Pcup[index2] - k * dPcup[index1];
                }
            }
        }
    }
    /* Compute the ration between the the Schmidt quasi-normalized associated Legendre
     * functions and the Gauss-normalized version. */

    schmidtQuasiNorm[0] = 1.0;
    for(n = 1; n <= nMax; n++)
    {
        index = (n * (n + 1) / 2);
        index1 = (n - 1) * n / 2;
        /* for m = 0 */
        schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * (Escal) (2 * n - 1) / (Escal) n;

        for(m = 1; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            index1 = (n * (n + 1) / 2 + m - 1);
            schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * sqrt((Escal) ((n - m + 1) * (m == 1 ? 2 : 1)) / (Escal) (n + m));
        }

    }

    /* Converts the  Gauss-normalized associated Legendre
              functions to the Schmidt quasi-normalized version using pre-computed
              relation stored in the variable schmidtQuasiNorm */

    for(n = 1; n <= nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            Pcup[index] = Pcup[index] * schmidtQuasiNorm[index];
            dPcup[index] = -dPcup[index] * schmidtQuasiNorm[index];
            /* The sign is changed since the new WMM routines use derivative with respect to latitude
            insted of co-latitude */
        }
    }

    if(schmidtQuasiNorm)
        free(schmidtQuasiNorm);
    return TRUE;
} /*MAG_PcupLow */


int MAG_SecVarSummation(MAGtype_LegendreFunction *LegendreFunction, MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults)
{
    /*This Function sums the secular variation coefficients to get the secular variation of the Magnetic vector.
    INPUT :  LegendreFunction
                    MagneticModel
                    SphVariables
                    CoordSpherical
    OUTPUT : MagneticResults

    CALLS : MAG_SecVarSummationSpecial

     */
    int m, n, index;
    Escal cos_phi;
    MagneticModel->SecularVariationUsed = TRUE;
    MagneticResults->Bz = 0.0;
    MagneticResults->By = 0.0;
    MagneticResults->Bx = 0.0;
    for(n = 1; n <= MagneticModel->nMaxSecVar; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);

            /*          nMax    (n+2)     n     m            m           m
                    Bz =   -SUM (a/r)   (n+1) SUM  [g cos(m p) + h sin(m p)] P (sin(phi))
                                    n=1               m=0   n            n           n  */
            /*  Derivative with respect to radius.*/
            MagneticResults->Bz -= SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Secular_Var_Coeff_G[index] * SphVariables.cos_mlambda[m] +
                    MagneticModel->Secular_Var_Coeff_H[index] * SphVariables.sin_mlambda[m])
                    * (Escal) (n + 1) * LegendreFunction-> Pcup[index];

            /*        1 nMax  (n+2)    n     m            m           m
                    By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
                               n=1             m=0   n            n           n  */
            /* Derivative with respect to longitude, divided by radius. */
            MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Secular_Var_Coeff_G[index] * SphVariables.sin_mlambda[m] -
                    MagneticModel->Secular_Var_Coeff_H[index] * SphVariables.cos_mlambda[m])
                    * (Escal) (m) * LegendreFunction-> Pcup[index];
            /*         nMax  (n+2) n     m            m           m
                    Bx = - SUM (a/r)   SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
                               n=1         m=0   n            n           n  */
            /* Derivative with respect to latitude, divided by radius. */

            MagneticResults->Bx -= SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Secular_Var_Coeff_G[index] * SphVariables.cos_mlambda[m] +
                    MagneticModel->Secular_Var_Coeff_H[index] * SphVariables.sin_mlambda[m])
                    * LegendreFunction-> dPcup[index];
        }
    }
    cos_phi = cos(DEG2RAD(CoordSpherical.phig));
    if(fabs(cos_phi) > 1.0e-10)
    {
        MagneticResults->By = MagneticResults->By / cos_phi;
    } else
        /* Special calculation for component By at Geographic poles */
    {
        MAG_SecVarSummationSpecial(MagneticModel, SphVariables, CoordSpherical, MagneticResults);
    }
    return TRUE;
} /*MAG_SecVarSummation*/


int MAG_SecVarSummationSpecial(MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults)
{
    /*Special calculation for the secular variation summation at the poles.


    INPUT: MagneticModel
               SphVariables
               CoordSpherical
    OUTPUT: MagneticResults
    CALLS : none


     */
    int n, index;
    Escal k, sin_phi, *PcupS, schmidtQuasiNorm1, schmidtQuasiNorm2, schmidtQuasiNorm3;

    PcupS = (Escal *) malloc((MagneticModel->nMaxSecVar + 1) * sizeof (Escal));

    if(PcupS == NULL)
    {
        MAG_Error(15);
        return FALSE;
    }

    PcupS[0] = 1;
    schmidtQuasiNorm1 = 1.0;

    MagneticResults->By = 0.0;
    sin_phi = sin(DEG2RAD(CoordSpherical.phig));

    for(n = 1; n <= MagneticModel->nMaxSecVar; n++)
    {
        index = (n * (n + 1) / 2 + 1);
        schmidtQuasiNorm2 = schmidtQuasiNorm1 * (Escal) (2 * n - 1) / (Escal) n;
        schmidtQuasiNorm3 = schmidtQuasiNorm2 * sqrt((Escal) (n * 2) / (Escal) (n + 1));
        schmidtQuasiNorm1 = schmidtQuasiNorm2;
        if(n == 1)
        {
            PcupS[n] = PcupS[n - 1];
        } else
        {
            k = (Escal) (((n - 1) * (n - 1)) - 1) / (Escal) ((2 * n - 1) * (2 * n - 3));
            PcupS[n] = sin_phi * PcupS[n - 1] - k * PcupS[n - 2];
        }

        /*        1 nMax  (n+2)    n     m            m           m
                By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
                           n=1             m=0   n            n           n  */
        /* Derivative with respect to longitude, divided by radius. */

        MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
                (MagneticModel->Secular_Var_Coeff_G[index] * SphVariables.sin_mlambda[1] -
                MagneticModel->Secular_Var_Coeff_H[index] * SphVariables.cos_mlambda[1])
                * PcupS[n] * schmidtQuasiNorm3;
    }

    if(PcupS)
        free(PcupS);
    return TRUE;
}/*SecVarSummationSpecial*/


int MAG_Summation(MAGtype_LegendreFunction *LegendreFunction, MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults)
{
    /* Computes Geomagnetic Field Elements X, Y and Z in Spherical coordinate system using
    spherical harmonic summation.


    The vector Magnetic field is given by -grad V, where V is Geomagnetic scalar potential
    The gradient in spherical coordinates is given by:

                     dV ^     1 dV ^        1     dV ^
    grad V = -- r  +  - -- t  +  -------- -- p
                     dr       r dt       r sin(t) dp


    INPUT :  LegendreFunction
                    MagneticModel
                    SphVariables
                    CoordSpherical
    OUTPUT : MagneticResults

    CALLS : MAG_SummationSpecial



    Manoj Nair, June, 2009 Manoj.C.Nair@Noaa.Gov
     */
    int m, n, index;
    Escal cos_phi;
    MagneticResults->Bz = 0.0;
    MagneticResults->By = 0.0;
    MagneticResults->Bx = 0.0;
    for(n = 1; n <= MagneticModel->nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);

            /*          nMax    (n+2)     n     m            m           m
                    Bz =   -SUM (a/r)   (n+1) SUM  [g cos(m p) + h sin(m p)] P (sin(phi))
                                    n=1               m=0   n            n           n  */
            /* Equation 12 in the WMM Technical report.  Derivative with respect to radius.*/
            MagneticResults->Bz -= SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables.cos_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.sin_mlambda[m])
                    * (Escal) (n + 1) * LegendreFunction-> Pcup[index];

            /*        1 nMax  (n+2)    n     m            m           m
                    By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
                               n=1             m=0   n            n           n  */
            /* Equation 11 in the WMM Technical report. Derivative with respect to longitude, divided by radius. */
            MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables.sin_mlambda[m] -
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.cos_mlambda[m])
                    * (Escal) (m) * LegendreFunction-> Pcup[index];
            /*         nMax  (n+2) n     m            m           m
                    Bx = - SUM (a/r)   SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
                               n=1         m=0   n            n           n  */
            /* Equation 10  in the WMM Technical report. Derivative with respect to latitude, divided by radius. */

            MagneticResults->Bx -= SphVariables.RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables.cos_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables.sin_mlambda[m])
                    * LegendreFunction-> dPcup[index];



        }
    }

    cos_phi = cos(DEG2RAD(CoordSpherical.phig));
    if(fabs(cos_phi) > 1.0e-10)
    {
        MagneticResults->By = MagneticResults->By / cos_phi;
    } else
        /* Special calculation for component - By - at Geographic poles.
         * If the user wants to aint using this function,  please make sure that
         * the latitude is not exactly +/-90. An option is to make use the function
         * MAG_CheckGeographicPoles.
         */
    {
        MAG_SummationSpecial(MagneticModel, SphVariables, CoordSpherical, MagneticResults);
    }
    return TRUE;
}/*MAG_Summation */


int MAG_SummationSpecial(MAGtype_MagneticModel *MagneticModel, MAGtype_SphericalHarmonicVariables SphVariables, MAGtype_CoordSpherical CoordSpherical, MAGtype_MagneticResults *MagneticResults)
/* Special calculation for the component By at Geographic poles.
Manoj Nair, June, 2009 manoj.c.nair@noaa.gov
INPUT: MagneticModel
           SphVariables
           CoordSpherical
OUTPUT: MagneticResults
CALLS : none
See Section 1.4, "SINGULARITIES AT THE GEOGRAPHIC POLES", WMM Technical report

 */
{
    int n, index;
    Escal k, sin_phi, *PcupS, schmidtQuasiNorm1, schmidtQuasiNorm2, schmidtQuasiNorm3;

    PcupS = (Escal *) malloc((MagneticModel->nMax + 1) * sizeof (Escal));
    if(PcupS == 0)
    {
        MAG_Error(14);
        return FALSE;
    }

    PcupS[0] = 1;
    schmidtQuasiNorm1 = 1.0;

    MagneticResults->By = 0.0;
    sin_phi = sin(DEG2RAD(CoordSpherical.phig));

    for(n = 1; n <= MagneticModel->nMax; n++)
    {

        /*Compute the ration between the Gauss-normalized associated Legendre
  functions and the Schmidt quasi-normalized version. This is equivalent to
  sqrt((m==0?1:2)*(n-m)!/(n+m!))*(2n-1)!!/(n-m)!  */

        index = (n * (n + 1) / 2 + 1);
        schmidtQuasiNorm2 = schmidtQuasiNorm1 * (Escal) (2 * n - 1) / (Escal) n;
        schmidtQuasiNorm3 = schmidtQuasiNorm2 * sqrt((Escal) (n * 2) / (Escal) (n + 1));
        schmidtQuasiNorm1 = schmidtQuasiNorm2;
        if(n == 1)
        {
            PcupS[n] = PcupS[n - 1];
        } else
        {
            k = (Escal) (((n - 1) * (n - 1)) - 1) / (Escal) ((2 * n - 1) * (2 * n - 3));
            PcupS[n] = sin_phi * PcupS[n - 1] - k * PcupS[n - 2];
        }

        /*        1 nMax  (n+2)    n     m            m           m
                By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
                           n=1             m=0   n            n           n  */
        /* Equation 11 in the WMM Technical report. Derivative with respect to longitude, divided by radius. */

        MagneticResults->By += SphVariables.RelativeRadiusPower[n] *
                (MagneticModel->Main_Field_Coeff_G[index] * SphVariables.sin_mlambda[1] -
                MagneticModel->Main_Field_Coeff_H[index] * SphVariables.cos_mlambda[1])
                * PcupS[n] * schmidtQuasiNorm3;
    }

    if(PcupS)
        free(PcupS);
    return TRUE;
}/*MAG_SummationSpecial */


int MAG_TimelyModifyMagneticModel(MAGtype_Date UserDate, MAGtype_MagneticModel *MagneticModel, MAGtype_MagneticModel *TimedMagneticModel)

/* Time change the Model coefficients from the base year of the model using secular variation coefficients.
Store the coefficients of the static model with their values advanced from epoch t0 to epoch t.
Copy the SV coefficients.  If input "t�" is the same as "t0", then this is merely a copy operation.
If the address of "TimedMagneticModel" is the same as the address of "MagneticModel", then this procedure overwrites
the given item "MagneticModel".

INPUT: UserDate
           MagneticModel
OUTPUT:TimedMagneticModel
CALLS : none
 */
{
    int n, m, index, a, b;
    TimedMagneticModel->EditionDate = MagneticModel->EditionDate;
    TimedMagneticModel->epoch = MagneticModel->epoch;
    TimedMagneticModel->nMax = MagneticModel->nMax;
    TimedMagneticModel->nMaxSecVar = MagneticModel->nMaxSecVar;
    a = TimedMagneticModel->nMaxSecVar;
    b = (a * (a + 1) / 2 + a);
    strcpy(TimedMagneticModel->ModelName, MagneticModel->ModelName);
    for(n = 1; n <= MagneticModel->nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            if(index <= b)
            {
                TimedMagneticModel->Main_Field_Coeff_H[index] = MagneticModel->Main_Field_Coeff_H[index] + (UserDate.DecimalYear - MagneticModel->epoch) * MagneticModel->Secular_Var_Coeff_H[index];
                TimedMagneticModel->Main_Field_Coeff_G[index] = MagneticModel->Main_Field_Coeff_G[index] + (UserDate.DecimalYear - MagneticModel->epoch) * MagneticModel->Secular_Var_Coeff_G[index];
                TimedMagneticModel->Secular_Var_Coeff_H[index] = MagneticModel->Secular_Var_Coeff_H[index]; /* We need a copy of the secular var coef to calculate secular change */
                TimedMagneticModel->Secular_Var_Coeff_G[index] = MagneticModel->Secular_Var_Coeff_G[index];
            } else
            {
                TimedMagneticModel->Main_Field_Coeff_H[index] = MagneticModel->Main_Field_Coeff_H[index];
                TimedMagneticModel->Main_Field_Coeff_G[index] = MagneticModel->Main_Field_Coeff_G[index];
            }
        }
    }
    return TRUE;
} /* MAG_TimelyModifyMagneticModel */

/*End of Spherical Harmonic Functions*/


/******************************************************************************
 *************************************Geoid************************************
 * This grouping consists of functions that make calculations to adjust
 * ellipsoid height to height above the geoid (Height above MSL).
 ******************************************************************************
 ******************************************************************************/


int MAG_ConvertGeoidToEllipsoidHeight(MAGtype_CoordGeodetic *CoordGeodetic, MAGtype_Geoid *Geoid)

/*
 * The function Convert_Geoid_To_Ellipsoid_Height converts the specified WGS84
 * Geoid height at the specified geodetic coordinates to the equivalent
 * ellipsoid height, using the EGM96 gravity model.
 *
 *   CoordGeodetic->phi        : Geodetic latitude in degress           (input)
 *    CoordGeodetic->lambda     : Geodetic longitude in degrees          (input)
 *    CoordGeodetic->HeightAboveEllipsoid        : Ellipsoid height, in kilometers         (output)
 *    CoordGeodetic->HeightAboveGeoid: Geoid height, in kilometers           (input)
 *
        CALLS : MAG_GetGeoidHeight (

 */
{
    Escal DeltaHeight;
    int Error_Code;
    Escal lat, lon;

    if(Geoid->UseGeoid == 1)
    { /* Geoid correction required */
      /* To ensure that latitude is less than 90 call MAG_EquivalentLatLon() */
        MAG_EquivalentLatLon(CoordGeodetic->phi, CoordGeodetic->lambda, &lat, &lon);
        Error_Code = MAG_GetGeoidHeight(lat, lon, &DeltaHeight, Geoid);
        CoordGeodetic->HeightAboveEllipsoid = CoordGeodetic->HeightAboveGeoid + DeltaHeight / 1000; /*  Input and output should be kilometers,
            However MAG_GetGeoidHeight returns Geoid height in meters - Hence division by 1000 */
    } else /* Geoid correction not required, copy the MSL height to Ellipsoid height */
    {
        CoordGeodetic->HeightAboveEllipsoid = CoordGeodetic->HeightAboveGeoid;
        Error_Code = TRUE;
    }
    return ( Error_Code);
} /* MAG_ConvertGeoidToEllipsoidHeight*/


int MAG_GetGeoidHeight(Escal Latitude,
        Escal Longitude,
        Escal *DeltaHeight,
        MAGtype_Geoid *Geoid)
/*
 * The  function MAG_GetGeoidHeight returns the height of the
 * EGM96 geiod above or below the WGS84 ellipsoid,
 * at the specified geodetic coordinates,
 * using a grid of height adjustments from the EGM96 gravity model.
 *
 *    Latitude            : Geodetic latitude in radians           (input)
 *    Longitude           : Geodetic longitude in radians          (input)
 *    DeltaHeight         : Height Adjustment, in meters.          (output)
 *    Geoid               : MAGtype_Geoid with Geoid grid          (input)
        CALLS : none
 */
{
    long Index;
    Escal DeltaX, DeltaY;
    Escal ElevationSE, ElevationSW, ElevationNE, ElevationNW;
    Escal OffsetX, OffsetY;
    Escal PostX, PostY;
    Escal UpperY, LowerY;
    int Error_Code = 0;

    if(!Geoid->Geoid_Initialized)
    {
        MAG_Error(5);
        return (FALSE);
    }
    if((Latitude < -90) || (Latitude > 90))
    { /* Latitude out of range */
        Error_Code |= 1;
    }
    if((Longitude < -180) || (Longitude > 360))
    { /* Longitude out of range */
        Error_Code |= 1;
    }

    if(!Error_Code)
    { /* no errors */
        /*  Compute X and Y Offsets into Geoid Height Array:                          */

        if(Longitude < 0.0)
        {
            OffsetX = (Longitude + 360.0) * Geoid->ScaleFactor;
        } else
        {
            OffsetX = Longitude * Geoid->ScaleFactor;
        }
        OffsetY = (90.0 - Latitude) * Geoid->ScaleFactor;

        /*  Find Four Nearest Geoid Height Cells for specified Latitude, Longitude;   */
        /*  Assumes that (0,0) of Geoid Height Array is at Northwest corner:          */

        PostX = floor(OffsetX);
        if((PostX + 1) == Geoid->NumbGeoidCols)
            PostX--;
        PostY = floor(OffsetY);
        if((PostY + 1) == Geoid->NumbGeoidRows)
            PostY--;

        Index = (long) (PostY * Geoid->NumbGeoidCols + PostX);
        ElevationNW = (Escal) Geoid->GeoidHeightBuffer[ Index ];
        ElevationNE = (Escal) Geoid->GeoidHeightBuffer[ Index + 1 ];

        Index = (long) ((PostY + 1) * Geoid->NumbGeoidCols + PostX);
        ElevationSW = (Escal) Geoid->GeoidHeightBuffer[ Index ];
        ElevationSE = (Escal) Geoid->GeoidHeightBuffer[ Index + 1 ];

        /*  Perform Bi-Linear Interpolation to compute Height above Ellipsoid:        */

        DeltaX = OffsetX - PostX;
        DeltaY = OffsetY - PostY;

        UpperY = ElevationNW + DeltaX * (ElevationNE - ElevationNW);
        LowerY = ElevationSW + DeltaX * (ElevationSE - ElevationSW);

        *DeltaHeight = UpperY + DeltaY * (LowerY - UpperY);
    } else
    {
        MAG_Error(17);
        return (FALSE);
    }
    return TRUE;
} /*MAG_GetGeoidHeight*/


int MAG_EquivalentLatLon(Escal lat, Escal lon, Escal *repairedLat, Escal  *repairedLon)
/*This function takes a latitude and longitude that are ordinarily out of range
 and gives in range values that are equivalent on the Earth's surface.  This is
 required to get correct values for the geoid function.*/
{
    Escal colat;
    colat = 90 - lat;
    *repairedLon = lon;
    if (colat < 0)
        colat = -colat;
    while(colat > 360) {
        colat-=360;
    }
    if (colat > 180) {
        colat-=180;
        *repairedLon = *repairedLon+180;
    }
    *repairedLat = 90 - colat;
    if (*repairedLon > 360)
        *repairedLon-=360;
    if (*repairedLon < -180)
        *repairedLon+=360;
    return TRUE;
}


//@}
/// @endcond
//////////////////////////////////////////////////

} // namespace physics
} // namespace stokesim
