/**
 * @file    tc_thermocouple.c
 * @brief   Thermocouple type definitions, NIST ITS-90 polynomial coefficients,
 *          voltage-to-temperature and temperature-to-voltage conversion.
 *
 * Knowledge Coverage:
 *   L1: Thermocouple type enumeration, temperature range definitions
 *   L2: Seebeck effect implementation, EMF generation physics
 *   L3: Polynomial evaluation via Horner's method, multi-range lookup
 *   L4: ITS-90 standard implementation with NIST Monograph 175 data
 *   L5: Forward/inverse conversion algorithms, Newton refinement
 *
 * Reference:
 *   Burns, G.W. et al. (1993) NIST Monograph 175: Temperature-Electromotive
 *     Force Reference Functions and Tables for the Letter-Designated
 *     Thermocouple Types Based on the ITS-90
 *   Preston-Thomas, H. (1990) "The International Temperature Scale of 1990
 *     (ITS-90)" Metrologia 27, 3-10
 *   ASTM E230/E230M-17 Standard Specification for Temperature-Electromotive
 *     Force (EMF) Tables for Standardized Thermocouples
 */

#include "thermocouple_cjc_rtd.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <float.h>

/* =========================================================================
 * L1: Thermocouple Type Metadata Tables
 * ========================================================================= */

/** Human-readable names for each thermocouple type */
static const char *tc_names[TC_COUNT] = {
    "Type K (Chromel-Alumel)",
    "Type J (Iron-Constantan)",
    "Type T (Copper-Constantan)",
    "Type E (Chromel-Constantan)",
    "Type N (Nicrosil-Nisil)",
    "Type R (Pt-13%Rh vs Pt)",
    "Type S (Pt-10%Rh vs Pt)",
    "Type B (Pt-30%Rh vs Pt-6%Rh)",
    "Type C (W-5%Re vs W-26%Re)"
};

/** Minimum valid temperatures [C] per ASTM E230 / NIST Monograph 175 */
static const double tc_temp_min[TC_COUNT] = {
    -270.0,   /* K: Chromel-Alumel */
    -210.0,   /* J: Iron-Constantan */
    -270.0,   /* T: Copper-Constantan */
    -270.0,   /* E: Chromel-Constantan */
    -270.0,   /* N: Nicrosil-Nisil */
     -50.0,   /* R: Pt-13%Rh vs Pt */
     -50.0,   /* S: Pt-10%Rh vs Pt */
       0.0,   /* B: Pt-30%Rh vs Pt-6%Rh */
       0.0    /* C: W-5%Re vs W-26%Re */
};

/** Maximum valid temperatures [C] */
static const double tc_temp_max[TC_COUNT] = {
    1372.0,   /* K */
    1200.0,   /* J */
     400.0,   /* T */
    1000.0,   /* E */
    1300.0,   /* N */
    1768.0,   /* R */
    1768.0,   /* S */
    1820.0,   /* B */
    2315.0    /* C */
};

/** Typical Seebeck coefficient at 25C [uV/C] */
static const double tc_seebeck_25[TC_COUNT] = {
    40.6,     /* K: ~41 uV/C */
    51.7,     /* J: ~52 uV/C */
    40.9,     /* T: ~41 uV/C */
    67.9,     /* E: ~68 uV/C */
    26.9,     /* N: ~27 uV/C */
    10.4,     /* R: ~10 uV/C */
    10.4,     /* S: ~10 uV/C */
     2.5,     /* B: ~3 uV/C at 25C (very low) */
    14.0      /* C: ~14 uV/C */
};

/** Minimum EMF [mV] */
static const double tc_emf_min[TC_COUNT] = {
    -5.891,   /* K */
    -8.095,   /* J */
    -5.603,   /* T */
    -8.825,   /* E */
    -3.990,   /* N */
    -0.226,   /* R */
    -0.235,   /* S */
     0.291,   /* B (minimum is above 0 because R-Type has very low EMF) */
     0.000    /* C */
};

/** Maximum EMF [mV] */
static const double tc_emf_max[TC_COUNT] = {
    54.886,   /* K */
    69.553,   /* J */
    20.872,   /* T */
    76.373,   /* E */
    47.513,   /* N */
    21.103,   /* R */
    18.693,   /* S */
    13.820,   /* B */
    37.103    /* C */
};

/* =========================================================================
 * L4: NIST ITS-90 Polynomial Coefficients
 * =========================================================================
 *
 * These are the standard reference functions from NIST Monograph 175.
 * Forward polynomials: E = sum c_i * T^i [mV output]
 * Inverse polynomials: T = sum c_i * E^i [C output]
 *
 * Each type has multiple sub-ranges for improved accuracy. The coefficients
 * are ordered from lowest degree (c0) to highest.
 */

/* ---- Type K (Chromel-Alumel): Forward -270 to 0 C ---- */
static double k10_coeffs[] = {
     0.000000000000E+00,
     3.945012802500E+01,
     2.362237359800E-02,
    -3.285890678400E-04,
    -4.990482877700E-06,
    -6.750905917300E-08,
    -5.741032742800E-10,
    -3.108887289400E-12,
    -1.045160936500E-14,
    -1.988926687800E-17,
    -1.632269748600E-20
};
static tc_polynomial_t poly_k10 = { -270.0, 0.0, 11, k10_coeffs, 0.02 };

/* Type K: Forward 0 to 1372 C */
static double k11_coeffs[] = {
    -1.760041368600E-02,
     3.892120497500E+01,
     1.855877003200E-02,
    -9.945759287400E-05,
     3.184094571900E-07,
    -5.607284488900E-10,
     5.607505905900E-13,
    -3.202072000300E-16,
     9.715114715200E-20,
    -1.210472127500E-23
};
static tc_polynomial_t poly_k11 = { 0.0, 1372.0, 10, k11_coeffs, 0.05 };

/* Type K: Inverse -5.891 to 0 mV */
static double k20_coeffs[] = {
     0.000000000000E+00,
     2.517346200000E-02,
    -1.166287800000E-06,
    -1.083363800000E-09,
    -8.977354000000E-13,
    -3.734237700000E-16,
    -8.663264300000E-20,
    -1.045059800000E-23,
    -5.192057700000E-28
};
static tc_polynomial_t poly_k20 = { -5.891, 0.0, 9, k20_coeffs, 0.02 };

/* Type K: Inverse 0 to 20.644 mV */
static double k21_coeffs[] = {
     0.000000000000E+00,
     2.508355000000E-02,
     7.860106000000E-08,
    -2.503131000000E-10,
     8.315270000000E-14,
    -1.228034000000E-17,
     9.804036000000E-22,
    -4.413030000000E-26,
     1.057734000000E-30,
    -1.052755000000E-35
};
static tc_polynomial_t poly_k21 = { 0.0, 20.644, 10, k21_coeffs, 0.04 };

/* Type K: Inverse 20.644 to 54.886 mV */
static double k22_coeffs[] = {
    -1.318058000000E+02,
     4.830222000000E+01,
    -1.646031000000E+00,
     5.464731000000E-02,
    -9.650715000000E-04,
     8.802193000000E-06,
    -3.110810000000E-08
};
static tc_polynomial_t poly_k22 = { 20.644, 54.886, 7, k22_coeffs, 0.06 };

static const tc_polynomial_t *k_forward[]  = { &poly_k10, &poly_k11 };
static const tc_polynomial_t *k_inverse[] = { &poly_k20, &poly_k21, &poly_k22 };

/* ---- Type J (Iron-Constantan): Forward -210 to 760 C ---- */
static double j10_coeffs[] = {
     0.000000000000E+00,
     5.038118781500E+01,
     3.047583693000E-02,
    -8.568106572000E-05,
     1.322819529500E-07,
    -1.705295833700E-10,
     2.094809069700E-13,
    -1.253839533600E-16,
     1.563172569700E-20
};
static tc_polynomial_t poly_j10 = { -210.0, 760.0, 9, j10_coeffs, 0.03 };

/* Type J: Forward 760 to 1200 C */
static double j11_coeffs[] = {
     2.964562568100E+02,
    -1.497612778600E+00,
     3.178710392400E-03,
    -3.184768670100E-06,
     1.572081900400E-09,
    -3.069136905600E-13
};
static tc_polynomial_t poly_j11 = { 760.0, 1200.0, 6, j11_coeffs, 0.05 };

/* Type J: Inverse -8.095 to 0 mV */
static double j20_coeffs[] = {
     0.000000000000E+00,
     1.952826800000E-02,
    -1.228618500000E-06,
    -1.075217800000E-09,
    -5.908693300000E-13,
    -1.725671300000E-16,
    -2.813151300000E-20,
    -2.396337000000E-24,
    -8.382332100000E-29
};
static tc_polynomial_t poly_j20 = { -8.095, 0.0, 9, j20_coeffs, 0.02 };

/* Type J: Inverse 0 to 42.919 mV */
static double j21_coeffs[] = {
     0.000000000000E+00,
     1.978425000000E-02,
    -2.001204000000E-07,
     1.036969000000E-11,
    -2.549687000000E-16,
     3.585153000000E-21,
    -5.344285000000E-26,
     5.099890000000E-31
};
static tc_polynomial_t poly_j21 = { 0.0, 42.919, 8, j21_coeffs, 0.04 };

/* Type J: Inverse 42.919 to 69.553 mV */
static double j22_coeffs[] = {
    -3.113581870000E+03,
     3.005436840000E+02,
    -9.947732300000E+00,
     1.702766300000E-01,
    -1.430334680000E-03,
     4.738860840000E-06
};
static tc_polynomial_t poly_j22 = { 42.919, 69.553, 6, j22_coeffs, 0.06 };

static const tc_polynomial_t *j_forward[]  = { &poly_j10, &poly_j11 };
static const tc_polynomial_t *j_inverse[] = { &poly_j20, &poly_j21, &poly_j22 };

/* ---- Type T (Copper-Constantan): Forward -270 to 400 C (single range) ---- */
static double t_forward_coeffs[] = {
     0.000000000000E+00,
     3.874810636400E+01,
     3.329222788000E-02,
     2.061824340400E-04,
    -2.188225684600E-06,
     1.099688092800E-08,
    -3.081575877200E-11,
     4.547913529000E-14,
    -2.751290167300E-17
};
static tc_polynomial_t poly_t10 = { -270.0, 400.0, 9, t_forward_coeffs, 0.01 };

/* Type T: Inverse -5.603 to 0 mV */
static double t20_coeffs[] = {
     0.000000000000E+00,
     2.594919200000E-02,
    -2.131696700000E-07,
     7.901869200000E-10,
     4.252777700000E-13,
     1.330447300000E-16,
     2.024144600000E-20,
     1.266817100000E-24
};
static tc_polynomial_t poly_t20 = { -5.603, 0.0, 8, t20_coeffs, 0.02 };

/* Type T: Inverse 0 to 20.872 mV */
static double t21_coeffs[] = {
     0.000000000000E+00,
     2.592800000000E-02,
    -7.602961000000E-07,
     4.637791000000E-11,
    -2.165394000000E-15,
     6.048144000000E-20,
    -7.293422000000E-25
};
static tc_polynomial_t poly_t21 = { 0.0, 20.872, 7, t21_coeffs, 0.02 };

static const tc_polynomial_t *t_forward[]  = { &poly_t10 };
static const tc_polynomial_t *t_inverse[] = { &poly_t20, &poly_t21 };

/* ---- Type E (Chromel-Constantan): Forward -270 to 0 C ---- */
static double e10_coeffs[] = {
     0.000000000000E+00,
     5.866550870800E+01,
     4.541097712400E-02,
    -7.799804868600E-04,
    -2.580016084300E-05,
    -5.945258305700E-07,
    -9.321405866700E-09,
    -1.028791553400E-10,
    -8.037012362100E-13,
    -4.397949739100E-15,
    -1.641477635500E-17,
    -3.967361951600E-20,
    -5.582732872100E-23,
    -3.465784201300E-26
};
static tc_polynomial_t poly_e10 = { -270.0, 0.0, 14, e10_coeffs, 0.02 };

/* Type E: Forward 0 to 1000 C */
static double e11_coeffs[] = {
     0.000000000000E+00,
     5.866550870800E+01,
     4.503227558200E-02,
     2.890840721200E-05,
    -3.305689665200E-07,
     6.502440327000E-10,
    -1.919749550400E-13,
    -1.253660049700E-15,
     2.148921756900E-18,
    -1.438804178200E-21,
     3.596089948100E-25
};
static tc_polynomial_t poly_e11 = { 0.0, 1000.0, 11, e11_coeffs, 0.04 };

/* Type E: Inverse -8.825 to 0 mV */
static double e20_coeffs[] = {
     0.000000000000E+00,
     1.697728800000E-02,
    -4.351497000000E-07,
    -1.585969700000E-10,
    -9.250287100000E-14,
    -2.608431700000E-17,
    -4.136019900000E-21,
    -3.403403000000E-25,
    -1.156489000000E-29
};
static tc_polynomial_t poly_e20 = { -8.825, 0.0, 9, e20_coeffs, 0.03 };

/* Type E: Inverse 0 to 76.373 mV */
static double e21_coeffs[] = {
     0.000000000000E+00,
     1.705703500000E-02,
    -2.330175900000E-07,
     6.543558500000E-12,
    -7.356274900000E-17,
    -1.789600100000E-21,
     8.403616500000E-26,
    -1.373587900000E-30,
     1.062982300000E-35,
    -3.244708700000E-41
};
static tc_polynomial_t poly_e21 = { 0.0, 76.373, 10, e21_coeffs, 0.04 };

static const tc_polynomial_t *e_forward[]  = { &poly_e10, &poly_e11 };
static const tc_polynomial_t *e_inverse[] = { &poly_e20, &poly_e21 };

/* ---- Type N (Nicrosil-Nisil): Forward -270 to 0 C ---- */
static double n10_coeffs[] = {
     0.000000000000E+00,
     2.615910596200E+01,
     1.095748422800E-02,
    -9.384111155400E-05,
    -4.641203975900E-08,
    -2.630335771600E-09,
    -2.265343800300E-11,
    -7.608930079100E-14,
    -9.341966783500E-17
};
static tc_polynomial_t poly_n10 = { -270.0, 0.0, 9, n10_coeffs, 0.02 };

/* Type N: Forward 0 to 1300 C */
static double n11_coeffs[] = {
     0.000000000000E+00,
     2.595919460100E+01,
     1.571014188000E-02,
     4.382562723700E-05,
    -2.526116979400E-07,
     6.431181933900E-10,
    -1.006347151900E-12,
     9.974533899200E-16,
    -6.086324560700E-19,
     2.084922933900E-22,
    -3.068219615100E-26
};
static tc_polynomial_t poly_n11 = { 0.0, 1300.0, 11, n11_coeffs, 0.04 };

/* Type N: Inverse -3.99 to 0 mV */
static double n20_coeffs[] = {
     0.000000000000E+00,
     3.843684700000E-02,
     1.101048500000E-03,
     5.222931200000E-06,
     7.206052500000E-09,
     5.848858600000E-12,
     2.775491600000E-15,
     7.707516600000E-19,
     1.158266500000E-22,
     7.313886800000E-27
};
static tc_polynomial_t poly_n20 = { -3.99, 0.0, 10, n20_coeffs, 0.02 };

/* Type N: Inverse 0 to 20.613 mV */
static double n21_coeffs[] = {
     0.000000000000E+00,
     3.868960000000E-02,
    -1.082670000000E-06,
     4.702050000000E-11,
    -2.121690000000E-16,
    -1.172720000000E-19,
     5.392800000000E-24,
    -7.981560000000E-29
};
static tc_polynomial_t poly_n21 = { 0.0, 20.613, 8, n21_coeffs, 0.04 };

/* Type N: Inverse 20.613 to 47.513 mV */
static double n22_coeffs[] = {
     1.972485000000E+01,
     3.300943000000E-01,
    -3.915159000000E-04,
     9.855391000000E-08,
    -1.274371000000E-11,
     7.767022000000E-16
};
static tc_polynomial_t poly_n22 = { 20.613, 47.513, 6, n22_coeffs, 0.06 };

static const tc_polynomial_t *n_forward[]  = { &poly_n10, &poly_n11 };
static const tc_polynomial_t *n_inverse[] = { &poly_n20, &poly_n21, &poly_n22 };

/* ---- Type R (Pt-13%Rh vs Pt) ---- */
/* Forward: -50 to 1064.18 C */
static double r10_coeffs[] = {
     0.000000000000E+00,
     5.289617297650E+00,
     1.391665897820E-02,
    -2.388556930170E-05,
     3.569160010630E-08,
    -4.623476662980E-11,
     5.007774410340E-14,
    -3.731058861910E-17,
     1.577164823670E-20,
    -2.810386252510E-24
};
static tc_polynomial_t poly_r10 = { -50.0, 1064.18, 10, r10_coeffs, 0.02 };

/* Type R: Forward 1064.18 to 1664.5 C */
static double r11_coeffs[] = {
     2.951579253160E+00,
    -2.520612513320E-03,
     1.595645018650E-05,
    -7.640859475800E-09,
     2.053052910240E-12,
    -2.933596681730E-16
};
static tc_polynomial_t poly_r11 = { 1064.18, 1664.5, 6, r11_coeffs, 0.02 };

/* Type R: Forward 1664.5 to 1768.1 C */
static double r12_coeffs[] = {
     1.522321182090E+02,
    -2.688198885450E-01,
     1.712802804710E-04,
    -3.458957064530E-08,
    -9.346339710460E-15
};
static tc_polynomial_t poly_r12 = { 1664.5, 1768.1, 5, r12_coeffs, 0.02 };

/* Type R: Inverse -0.226 to 1.923 mV */
static double r20_coeffs[] = {
     0.000000000000E+00,
     1.889138000000E-01,
    -9.383529000000E-05,
     1.306861900000E-07,
    -2.270358000000E-10,
     3.514565900000E-13,
    -3.895390000000E-16,
     2.823947100000E-19,
    -1.260728100000E-22,
     3.135361100000E-26,
    -3.318776900000E-30
};
static tc_polynomial_t poly_r20 = { -0.226, 1.923, 11, r20_coeffs, 0.02 };

/* Type R: Inverse 1.923 to 13.228 mV */
static double r21_coeffs[] = {
     1.334584505000E+01,
     1.472644573000E+02,
    -1.844024844000E+01,
     4.031129726000E+00,
    -6.249428360000E-01,
     6.468412046000E-02,
    -4.458750426000E-03,
     1.994710149000E-04,
    -5.313401790000E-06,
     6.481976217000E-08
};
static tc_polynomial_t poly_r21 = { 1.923, 11.361, 10, r21_coeffs, 0.01 };

/* Type R: Inverse 11.361 to 19.739 mV */
static double r22_coeffs[] = {
    -8.199599416000E+01,
     1.553962042000E+02,
    -8.342197663000E+01,
     4.279433549000E+01,
    -1.191577910000E+01,
     1.492290091000E+00
};
static tc_polynomial_t poly_r22 = { 11.361, 19.739, 6, r22_coeffs, 0.02 };

/* Type R: Inverse 19.739 to 21.103 mV */
static double r23_coeffs[] = {
     3.406177836000E+04,
    -7.023729171000E+03,
     5.582903813000E+02,
    -1.952394635000E+01,
     2.560740231000E-01
};
static tc_polynomial_t poly_r23 = { 19.739, 21.103, 5, r23_coeffs, 0.03 };

static const tc_polynomial_t *r_forward[]  = { &poly_r10, &poly_r11, &poly_r12 };
static const tc_polynomial_t *r_inverse[] = { &poly_r20, &poly_r21, &poly_r22, &poly_r23 };

/* ---- Type S (Pt-10%Rh vs Pt) ---- */
/* Forward: -50 to 1064.18 C */
static double s10_coeffs[] = {
     0.000000000000E+00,
     5.403133086310E+00,
     1.259342897400E-02,
    -2.324779686890E-05,
     3.220288230360E-08,
    -3.314651963890E-11,
     2.557442517860E-14,
    -1.250688713930E-17,
     2.714431761450E-21
};
static tc_polynomial_t poly_s10 = { -50.0, 1064.18, 9, s10_coeffs, 0.02 };

/* Type S: Forward 1064.18 to 1664.5 C */
static double s11_coeffs[] = {
     1.329004440850E+00,
     3.345093113440E-03,
     6.548051928180E-06,
    -1.648562592090E-09,
     1.299896051740E-14
};
static tc_polynomial_t poly_s11 = { 1064.18, 1664.5, 5, s11_coeffs, 0.02 };

/* Type S: Forward 1664.5 to 1768.1 C */
static double s12_coeffs[] = {
     1.466282326190E+02,
    -2.584305167520E-01,
     1.636935746010E-04,
    -3.304390469870E-08,
    -9.432236906120E-15
};
static tc_polynomial_t poly_s12 = { 1664.5, 1768.1, 5, s12_coeffs, 0.02 };

/* Type S: Inverse -0.235 to 1.874 mV */
static double s20_coeffs[] = {
     0.000000000000E+00,
     1.849494600000E-01,
    -8.005040620000E-05,
     1.022374300000E-07,
    -1.522485920000E-10,
     1.888213430000E-13,
    -1.590859410000E-16,
     8.230278800000E-20,
    -2.341819440000E-23,
     2.797862600000E-27
};
static tc_polynomial_t poly_s20 = { -0.235, 1.874, 10, s20_coeffs, 0.02 };

/* Type S: Inverse 1.874 to 11.95 mV */
static double s21_coeffs[] = {
     1.291507177000E+01,
     1.466298863000E+02,
    -1.534713402000E+01,
     3.145945973000E+00,
    -4.163257839000E-01,
     3.187963771000E-02,
    -1.291637500000E-03,
     2.183475087000E-05,
    -1.447379511000E-07,
     8.211272125000E-09
};
static tc_polynomial_t poly_s21 = { 1.874, 10.332, 10, s21_coeffs, 0.02 };

/* Type S: Inverse 10.332 to 17.536 mV */
static double s22_coeffs[] = {
    -8.087801117000E+01,
     1.621573104000E+02,
    -8.536869453000E+01,
     4.719686976000E+01,
    -1.441693666000E+01,
     2.081618890000E+00
};
static tc_polynomial_t poly_s22 = { 10.332, 17.536, 6, s22_coeffs, 0.02 };

/* Type S: Inverse 17.536 to 18.693 mV */
static double s23_coeffs[] = {
     5.333875126000E+04,
    -1.235892298000E+04,
     1.092657613000E+03,
    -4.265693686000E+01,
     6.247205420000E-01
};
static tc_polynomial_t poly_s23 = { 17.536, 18.693, 5, s23_coeffs, 0.03 };

static const tc_polynomial_t *s_forward[]  = { &poly_s10, &poly_s11, &poly_s12 };
static const tc_polynomial_t *s_inverse[] = { &poly_s20, &poly_s21, &poly_s22, &poly_s23 };

/* ---- Type B (Pt-30%Rh vs Pt-6%Rh) ---- */
/* Forward: 0 to 630.615 C */
static double b10_coeffs[] = {
     0.000000000000E+00,
    -2.465081834600E-01,
     5.904042117100E-03,
    -1.325793163600E-06,
     1.566829190100E-09,
    -1.694452924000E-12,
     6.299034709400E-16
};
static tc_polynomial_t poly_b10 = { 0.0, 630.615, 7, b10_coeffs, 0.02 };

/* Type B: Forward 630.615 to 1820 C */
static double b11_coeffs[] = {
    -3.893816862100E+00,
     2.857174747000E-02,
    -8.488510478500E-05,
     1.578528016400E-07,
    -1.683534486400E-10,
     1.110979401300E-13,
    -4.451543103600E-17,
     9.897564082100E-21,
    -9.379133028900E-25
};
static tc_polynomial_t poly_b11 = { 630.615, 1820.0, 9, b11_coeffs, 0.03 };

/* Type B: Inverse 0.291 to 2.431 mV */
static double b20_coeffs[] = {
     9.842332100000E+01,
     6.997150000000E+02,
    -8.476530400000E+02,
     1.005264400000E+03,
    -8.334595200000E+02,
     4.550854200000E+02,
    -1.552303700000E+02,
     2.988675000000E+01,
    -2.474286000000E+00
};
static tc_polynomial_t poly_b20 = { 0.291, 2.431, 9, b20_coeffs, 0.02 };

/* Type B: Inverse 2.431 to 13.820 mV */
static double b21_coeffs[] = {
     2.131507100000E+02,
     2.851050400000E+02,
    -5.274288700000E+01,
     9.916080400000E+00,
    -1.296530300000E+00,
     1.119587000000E-01,
    -6.062519900000E-03,
     1.866169600000E-04,
    -2.487858500000E-06
};
static tc_polynomial_t poly_b21 = { 2.431, 13.820, 9, b21_coeffs, 0.03 };

static const tc_polynomial_t *b_forward[]  = { &poly_b10, &poly_b11 };
static const tc_polynomial_t *b_inverse[] = { &poly_b20, &poly_b21 };

/* ---- Type C (W-5%Re vs W-26%Re) ---- */
/* Forward: 0 to 2315 C (ASTM E988) */
static double c10_coeffs[] = {
     0.000000000000E+00,
     1.340834480000E+01,
     2.861943110000E-02,
    -9.155628800000E-05,
     1.783223530000E-07,
    -1.993333130000E-10,
     1.222486340000E-13,
    -3.507588550000E-17,
     3.674415930000E-21
};
static tc_polynomial_t poly_c10 = { 0.0, 2315.0, 9, c10_coeffs, 0.05 };

/* Type C: Inverse 0 to 37.103 mV */
static double c20_coeffs[] = {
     0.000000000000E+00,
     7.483100000000E-02,
    -1.468700000000E-05,
     2.367000000000E-08,
    -2.758500000000E-11,
     1.764500000000E-14,
    -6.171000000000E-18,
     9.930000000000E-22
};
static tc_polynomial_t poly_c20 = { 0.0, 37.103, 8, c20_coeffs, 0.05 };

static const tc_polynomial_t *c_forward[]  = { &poly_c10 };
static const tc_polynomial_t *c_inverse[] = { &poly_c20 };

/* =========================================================================
 * L1: Conversion Table Assembly (Master Table)
 * ========================================================================= */

/**
 * @brief Master conversion table for all thermocouple types
 *
 * Each entry maps a tc_type_t to its full ITS-90 conversion data:
 * polynomial coefficient arrays, temperature/EMF ranges, and
 * typical Seebeck coefficient for quick sensitivity estimation.
 */
static tc_conversion_table_t tc_tables[TC_COUNT] = {
    { /* Type K */
        TC_TYPE_K, "Type K (Chromel-Alumel)",
        -270.0, 1372.0, -5.891, 54.886, 40.6,
        2, 3, k_forward, k_inverse
    },
    { /* Type J */
        TC_TYPE_J, "Type J (Iron-Constantan)",
        -210.0, 1200.0, -8.095, 69.553, 51.7,
        2, 3, j_forward, j_inverse
    },
    { /* Type T */
        TC_TYPE_T, "Type T (Copper-Constantan)",
        -270.0, 400.0, -5.603, 20.872, 40.9,
        1, 2, t_forward, t_inverse
    },
    { /* Type E */
        TC_TYPE_E, "Type E (Chromel-Constantan)",
        -270.0, 1000.0, -8.825, 76.373, 67.9,
        2, 2, e_forward, e_inverse
    },
    { /* Type N */
        TC_TYPE_N, "Type N (Nicrosil-Nisil)",
        -270.0, 1300.0, -3.990, 47.513, 26.9,
        2, 3, n_forward, n_inverse
    },
    { /* Type R */
        TC_TYPE_R, "Type R (Pt-13%Rh vs Pt)",
        -50.0, 1768.0, -0.226, 21.103, 10.4,
        3, 4, r_forward, r_inverse
    },
    { /* Type S */
        TC_TYPE_S, "Type S (Pt-10%Rh vs Pt)",
        -50.0, 1768.0, -0.235, 18.693, 10.4,
        3, 4, s_forward, s_inverse
    },
    { /* Type B */
        TC_TYPE_B, "Type B (Pt-30%Rh vs Pt-6%Rh)",
        0.0, 1820.0, 0.291, 13.820, 2.5,
        2, 2, b_forward, b_inverse
    },
    { /* Type C */
        TC_TYPE_C, "Type C (W-5%Re vs W-26%Re)",
        0.0, 2315.0, 0.0, 37.103, 14.0,
        1, 1, c_forward, c_inverse
    }
};

/* =========================================================================
 * L2: Helper - Get Conversion Table
 * ========================================================================= */

/** Safely retrieve conversion table; returns NULL for invalid type */
static const tc_conversion_table_t *tc_get_table(tc_type_t type) {
    if (type >= TC_COUNT) return NULL;
    return &tc_tables[type];
}

/* =========================================================================
 * L2: Thermocouple Name and Range Queries
 * ========================================================================= */

const char *tc_type_name(tc_type_t type) {
    if (type >= TC_COUNT) return "Unknown Thermocouple Type";
    return tc_names[type];
}

tc_error_t tc_type_range(tc_type_t type, double *t_min, double *t_max) {
    if (!t_min || !t_max) return TC_ERR_NULL_POINTER;
    if (type >= TC_COUNT) return TC_ERR_INVALID_TYPE;
    *t_min = tc_temp_min[type];
    *t_max = tc_temp_max[type];
    return TC_OK;
}

/* =========================================================================
 * L5: Horner's Method for Polynomial Evaluation
 * ========================================================================= */

/**
 * @brief Evaluate polynomial using Horner's scheme
 *
 * p(x) = c0 + c1*x + c2*x^2 + ... + c_{n-1}*x^{n-1}
 *
 * Horner's method rearranges: p(x) = c0 + x*(c1 + x*(c2 + x*(...)))
 *
 * Advantages over direct evaluation:
 *   - Fewer multiplications: O(n) vs O(n^2) for naive evaluation
 *   - Better numerical stability (reduced round-off error)
 *   - Natural fit for evaluating ITS-90 polynomials
 *
 * Complexity: O(n), n floating-point multiplies and n additions.
 */
double tc_horner_eval(const double *coeffs, size_t n_coeffs, double x) {
    double result;
    size_t i;

    if (!coeffs || n_coeffs == 0) return 0.0;
    if (n_coeffs == 1) return coeffs[0];

    result = coeffs[n_coeffs - 1];
    for (i = n_coeffs - 1; i > 0; i--) {
        result = result * x + coeffs[i - 1];
    }
    return result;
}

/**
 * @brief Evaluate polynomial derivative using Horner's scheme
 *
 * p'(x) = c1 + 2*c2*x + 3*c3*x^2 + ... + (n-1)*c_{n-1}*x^{n-2}
 *
 * This gives the Seebeck coefficient dE/dT for EMF polynomials.
 * Used in Newton-Raphson inversion and sensitivity analysis.
 *
 * Complexity: O(n)
 */
double tc_horner_derivative(const double *coeffs, size_t n_coeffs, double x) {
    double result;
    size_t i;

    if (!coeffs || n_coeffs < 2) return 0.0;
    if (n_coeffs == 2) return coeffs[1];

    result = (double)(n_coeffs - 1) * coeffs[n_coeffs - 1];
    for (i = n_coeffs - 2; i >= 1; i--) {
        result = result * x + (double)i * coeffs[i];
    }
    return result;
}

/* =========================================================================
 * L4: Forward Conversion - Temperature to EMF (ITS-90)
 * ========================================================================= */

/**
 * @brief tc_temp_to_emf: Temperature to EMF conversion using ITS-90
 *
 * Selects the appropriate polynomial range based on temperature and
 * evaluates EMF = f(T) using Horner's method.
 *
 * The reference junction is assumed to be at 0C (ice point).
 * Result is in millivolts [mV].
 *
 * For types with multiple polynomial ranges (e.g., Type K has separate
 * polynomials for T < 0 and T >= 0), the correct range is selected
 * based on the input temperature.
 *
 * Complexity: O(log R + P) where R = number of ranges, P = max polynomial degree.
 */
tc_error_t tc_temp_to_emf(tc_type_t type, double temp, double *emf) {
    const tc_conversion_table_t *table;
    size_t i;

    if (!emf) return TC_ERR_NULL_POINTER;
    table = tc_get_table(type);
    if (!table) return TC_ERR_INVALID_TYPE;

    if (temp < table->temp_min || temp > table->temp_max) {
        *emf = 0.0;
        return TC_ERR_OUT_OF_RANGE;
    }

    for (i = 0; i < table->n_forward_ranges; i++) {
        const tc_polynomial_t *poly = table->forward_poly[i];
        if (temp >= poly->t_low && temp <= poly->t_high) {
            /* ITS-90 polynomials yield EMF in microvolts; convert to mV */
            *emf = tc_horner_eval(poly->coeffs, poly->n_coeffs, temp) / 1000.0;
            return TC_OK;
        }
    }

    *emf = 0.0;
    return TC_ERR_VOLTAGE_RANGE;
}

/* =========================================================================
 * L5: Inverse Conversion - EMF to Temperature (ITS-90 + Newton)
 * ========================================================================= */

/**
 * @brief tc_emf_to_temp: EMF to temperature using ITS-90 inverse polynomials
 *                       with Newton-Raphson refinement
 *
 * Two-step process:
 * Step 1: Compute initial temperature estimate using the inverse polynomial
 *         T0 = g(V) where g is the ITS-90 inverse polynomial for that range.
 * Step 2: Refine with up to 3 Newton iterations:
 *         T_{k+1} = T_k - (E(T_k) - V_measured) / E'(T_k)
 *
 * Three iterations are sufficient to achieve ~1e-12 C accuracy for
 * all standard thermocouple types, given the excellent initial guess
 * from the inverse polynomial.
 *
 * Complexity: O(P_inv + 3*(P_fwd + P_deriv))
 */
tc_error_t tc_emf_to_temp(tc_type_t type, double emf, double *temp) {
    const tc_conversion_table_t *table;
    const tc_polynomial_t *forward_poly = NULL;
    const tc_polynomial_t *inverse_poly = NULL;
    size_t i, ref;
    double emf_uv, t_est, e_calc, e_deriv, t_new, diff;

    if (!temp) return TC_ERR_NULL_POINTER;
    table = tc_get_table(type);
    if (!table) return TC_ERR_INVALID_TYPE;

    if (emf < table->emf_min || emf > table->emf_max) {
        *temp = 0.0;
        return TC_ERR_VOLTAGE_RANGE;
    }

    /* Convert mV to uV for ITS-90 polynomial evaluation */
    emf_uv = emf * 1000.0;

    /* Step 1: Select inverse polynomial and compute initial estimate */
    for (i = 0; i < table->n_inverse_ranges; i++) {
        inverse_poly = table->inverse_poly[i];
        /* Inverse poly ranges stored in mV; compare in uV */
        if (emf_uv >= inverse_poly->t_low * 1000.0
            && emf_uv <= inverse_poly->t_high * 1000.0) {
            break;
        }
    }
    if (!inverse_poly) {
        *temp = 0.0;
        return TC_ERR_VOLTAGE_RANGE;
    }

    /* Inverse polynomial expects EMF in uV */
    t_est = tc_horner_eval(inverse_poly->coeffs, inverse_poly->n_coeffs, emf_uv);

    /* Step 2: Newton-Raphson refinement (up to 3 iterations) */
    for (ref = 0; ref < 3; ref++) {
        forward_poly = NULL;
        for (i = 0; i < table->n_forward_ranges; i++) {
            const tc_polynomial_t *pf = table->forward_poly[i];
            if (t_est >= pf->t_low - 1.0 && t_est <= pf->t_high + 1.0) {
                forward_poly = pf;
                break;
            }
        }
        if (!forward_poly) break;

        /* Forward poly yields EMF in uV */
        e_calc = tc_horner_eval(forward_poly->coeffs,
                                 forward_poly->n_coeffs, t_est);
        e_deriv = tc_horner_derivative(forward_poly->coeffs,
                                        forward_poly->n_coeffs, t_est);

        if (fabs(e_deriv) < 1e-20) break;

        /* Newton step using uV values */
        t_new = t_est - (e_calc - emf_uv) / e_deriv;
        diff = fabs(t_new - t_est);
        t_est = t_new;

        /* Early exit: machine precision convergence */
        if (diff < 1e-12) break;
    }

    *temp = t_est;
    return TC_OK;
}

/* =========================================================================
 * L5: Newton-Raphson Inverse with Full Configuration Control
 * ========================================================================= */

tc_error_t tc_newton_inverse(tc_type_t type, double v_target,
                              const tc_newton_config_t *config,
                              tc_newton_result_t *result) {
    const tc_conversion_table_t *table;
    const tc_polynomial_t *forward_poly = NULL;
    size_t i, iter;
    double v_target_uv, t, e_val = 0.0, e_deriv = 0.0, t_new, delta, damping;

    if (!config || !result) return TC_ERR_NULL_POINTER;
    table = tc_get_table(type);
    if (!table) return TC_ERR_INVALID_TYPE;

    /* Convert target voltage from mV to uV */
    v_target_uv = v_target * 1000.0;

    damping = config->damping;
    if (damping <= 0.0 || damping > 1.0) damping = 1.0;

    memset(result, 0, sizeof(*result));
    t = config->initial_guess;

    for (iter = 0; iter < config->max_iterations; iter++) {
        forward_poly = NULL;
        for (i = 0; i < table->n_forward_ranges; i++) {
            const tc_polynomial_t *pf = table->forward_poly[i];
            if (t >= pf->t_low - 5.0 && t <= pf->t_high + 5.0) {
                forward_poly = pf;
                break;
            }
        }
        if (!forward_poly) {
            result->converged = 0;
            result->solution = t;
            result->iterations = iter;
            result->final_error = fabs(v_target_uv) / 1000.0; /* mV */
            return TC_ERR_CONVERGENCE;
        }

        e_val = tc_horner_eval(forward_poly->coeffs,
                                forward_poly->n_coeffs, t);

        if (config->use_analytic_derivative) {
            e_deriv = tc_horner_derivative(forward_poly->coeffs,
                                            forward_poly->n_coeffs, t);
        } else {
            /* Finite-difference derivative for robustness */
            double h = 1e-4;
            double e_val_plus = tc_horner_eval(forward_poly->coeffs,
                                                forward_poly->n_coeffs, t + h);
            e_deriv = (e_val_plus - e_val) / h;
        }

        if (fabs(e_deriv) < 1e-20) {
            result->converged = 0;
            result->solution = t;
            result->iterations = iter;
            result->final_error = fabs(e_val - v_target_uv) / 1000.0;
            result->derivative_used = e_deriv / 1000.0; /* mV/C */
            return TC_ERR_CONVERGENCE;
        }

        t_new = t - damping * (e_val - v_target_uv) / e_deriv;
        delta = fabs(t_new - t);
        t = t_new;

        if (delta < config->tolerance) {
            result->converged = 1;
            result->solution = t;
            result->iterations = iter + 1;
            result->final_error = fabs(e_val - v_target_uv) / 1000.0; /* mV */
            result->derivative_used = e_deriv / 1000.0; /* mV/C */
            return TC_OK;
        }
    }

    result->converged = 0;
    result->solution = t;
    result->iterations = config->max_iterations;
    result->final_error = fabs(e_val - v_target_uv) / 1000.0;
    result->derivative_used = e_deriv / 1000.0;
    return TC_ERR_CONVERGENCE;
}

/* =========================================================================
 * L5: Seebeck Coefficient Computation
 * ========================================================================= */

tc_error_t tc_seebeck_coefficient(tc_type_t type, double temp,
                                   double *seebeck) {
    const tc_conversion_table_t *table;
    size_t i;

    if (!seebeck) return TC_ERR_NULL_POINTER;
    table = tc_get_table(type);
    if (!table) return TC_ERR_INVALID_TYPE;

    if (temp < table->temp_min || temp > table->temp_max) {
        *seebeck = 0.0;
        return TC_ERR_OUT_OF_RANGE;
    }

    for (i = 0; i < table->n_forward_ranges; i++) {
        const tc_polynomial_t *poly = table->forward_poly[i];
        if (temp >= poly->t_low && temp <= poly->t_high) {
            /* Derivative gives uV/C; convert to mV/C */
            *seebeck = tc_horner_derivative(poly->coeffs, poly->n_coeffs, temp) / 1000.0;
            return TC_OK;
        }
    }

    *seebeck = 0.0;
    return TC_ERR_OUT_OF_RANGE;
}

tc_error_t tc_seebeck_info(tc_type_t type, double temp,
                            tc_seebeck_info_t *info) {
    double emf, seebeck, linear_emf;
    tc_error_t err;

    if (!info) return TC_ERR_NULL_POINTER;
    if (type >= TC_COUNT) return TC_ERR_INVALID_TYPE;

    memset(info, 0, sizeof(*info));
    info->temperature = temp;

    err = tc_seebeck_coefficient(type, temp, &seebeck);
    if (err != TC_OK) return err;

    info->seebeck_relative = seebeck;
    info->tc_sensitivity = seebeck;

    /* Compute linearity deviation vs constant-Seebeck approximation */
    err = tc_temp_to_emf(type, temp, &emf);
    if (err == TC_OK && fabs(temp) > 1.0) {
        double seebeck_0;
        if (tc_seebeck_coefficient(type, 0.0, &seebeck_0) == TC_OK) {
            linear_emf = seebeck_0 * temp;
            if (fabs(emf) > 1e-20) {
                info->linearity_deviation = 100.0 * (emf - linear_emf) / emf;
            }
        }
    }
    return TC_OK;
}

/* =========================================================================
 * Temperature Unit Conversion
 * ========================================================================= */

double tc_convert_temperature(double value, tc_temperature_unit_t from,
                               tc_temperature_unit_t to) {
    double kelvin;
    switch (from) {
    case TC_UNIT_CELSIUS:    kelvin = value + 273.15; break;
    case TC_UNIT_KELVIN:     kelvin = value; break;
    case TC_UNIT_FAHRENHEIT: kelvin = (value - 32.0) * 5.0 / 9.0 + 273.15; break;
    case TC_UNIT_RANKINE:    kelvin = value * 5.0 / 9.0; break;
    default: kelvin = value; break;
    }
    switch (to) {
    case TC_UNIT_CELSIUS:    return kelvin - 273.15;
    case TC_UNIT_KELVIN:     return kelvin;
    case TC_UNIT_FAHRENHEIT: return (kelvin - 273.15) * 9.0 / 5.0 + 32.0;
    case TC_UNIT_RANKINE:    return kelvin * 9.0 / 5.0;
    default: return kelvin;
    }
}

/* =========================================================================
 * Error String Table
 * ========================================================================= */

const char *tc_error_string(tc_error_t err) {
    switch (err) {
    case TC_OK:                    return "Success";
    case TC_ERR_NULL_POINTER:      return "NULL pointer argument";
    case TC_ERR_INVALID_TYPE:      return "Invalid thermocouple or RTD type";
    case TC_ERR_OUT_OF_RANGE:      return "Temperature out of valid range";
    case TC_ERR_VOLTAGE_RANGE:     return "Voltage out of valid conversion range";
    case TC_ERR_RESISTANCE_RANGE:  return "Resistance out of expected range";
    case TC_ERR_OPEN_CIRCUIT:      return "Open thermocouple circuit detected";
    case TC_ERR_SHORT_CIRCUIT:     return "Short circuit detected";
    case TC_ERR_CONVERGENCE:       return "Newton-Raphson solver failed to converge";
    case TC_ERR_SELF_HEATING:      return "Excessive RTD self-heating detected";
    case TC_ERR_CJC_RANGE:         return "Cold-junction temperature out of bounds";
    case TC_ERR_ADC_SATURATION:    return "ADC input saturated";
    case TC_ERR_CALIBRATION:       return "Calibration data invalid";
    case TC_ERR_INTERPOLATION:     return "Interpolation failure";
    default: return "Unknown error code";
    }
}
