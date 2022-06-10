//-----------------------------------------------------------//
// File: AeroC.h
// 
// This class computes the aerodynamic forces and moments
// exerted on the missile during flight using basic aero.
//                                                           
// Developer: Dennis Strickland                              
//-----------------------------------------------------------//

#ifndef Aero_H
#define Aero_H
#include "main.h"
#include <sstream>

class Output;
class System;

class Aero : public Block{

public:
  int kt;
  double t1000, xd, tick, t;

  //FILE *fp0;

  int firstFlag;
  ofstream outAero;

  void angle_of_attack( Vec v_b,
  double &alphp, double &alphy, double &alpha);

  Aero(string infile, Output *outp, System *sysp);

  double sdt;
  double defl1, defl2, defl3, defl4;
  double delp, dely, delr;

  double q, mach, totalAngleOfAttackDegrees, phiPrimeDegrees;
  
  double CA_PON_C, CA_POFF_C;
  double CA_C, CN_C, CY_C;
  double CLL_C, CMcg_C, CLNcg_C;
  
  double caon_stb, caoff_stb, cn_stb, cm_stb, cy_stb, cln_stb, cll_stb;
  double caon_d, caoff_d, cn_d, cm_d, cy_d, cln_d, cll_d;
  
  double xmrp, vtol;
  double phat, qhat;

  Table2 *cllp_table;
  Table2 *cmqC_table;
  //
  Table3 *caon_table;
  Table3 *caoff_table;
  Table3 *cll_table;
  Table3 *cln_table;
  Table3 *cm_table;
  Table3 *cn_table;
  Table3 *cy_table;

  Table4 *dp_inc_ca_table;
  Table4 *dp_inc_cll_table;
  Table4 *dp_inc_cln_table;
  Table4 *dp_inc_cm_table;
  Table4 *dp_inc_cn_table;
  Table4 *dp_inc_cy_table;

  Table4 *dr_inc_ca_table;
  Table4 *dr_inc_cll_table;
  Table4 *dr_inc_cln_table;
  Table4 *dr_inc_cm_table;
  Table4 *dr_inc_cn_table;
  Table4 *dr_inc_cy_table;

  Table4 *dy_inc_ca_table;
  Table4 *dy_inc_cll_table;
  Table4 *dy_inc_cln_table;
  Table4 *dy_inc_cm_table;
  Table4 *dy_inc_cn_table;
  Table4 *dy_inc_cy_table;

  Table5 *dpdr_inc_ca_table;
  Table5 *dpdr_inc_cll_table;
  Table5 *dpdr_inc_cln_table;
  Table5 *dpdr_inc_cm_table;
  Table5 *dpdr_inc_cn_table;
  Table5 *dpdr_inc_cy_table;

  Table5 *dpdy_inc_ca_table;
  Table5 *dpdy_inc_cll_table;
  Table5 *dpdy_inc_cln_table;
  Table5 *dpdy_inc_cm_table;
  Table5 *dpdy_inc_cn_table;
  Table5 *dpdy_inc_cy_table;

  Table5 *dydr_inc_ca_table;
  Table5 *dydr_inc_cll_table;
  Table5 *dydr_inc_cln_table;
  Table5 *dydr_inc_cm_table;
  Table5 *dydr_inc_cn_table;
  Table5 *dydr_inc_cy_table;

  double STABILITY_CAF;
  double STABILITY_CAU;
  double STABILITY_CLL;
  double STABILITY_CLN;
  double STABILITY_CM;
  double STABILITY_CN;
  double STABILITY_CY;
  double DAMP_CLLP, DAMP_CMQnr;
  double dP_INC_CA, dP_INC_CLL, dP_INC_CLN, dP_INC_CM, dP_INC_CN, dP_INC_CY;
  double dR_INC_CA, dR_INC_CLL, dR_INC_CLN, dR_INC_CM, dR_INC_CN, dR_INC_CY;
  double dY_INC_CA, dY_INC_CLL, dY_INC_CLN, dY_INC_CM, dY_INC_CN, dY_INC_CY;
  double dPdR_INC_CA, dPdR_INC_CLL, dPdR_INC_CLN, dPdR_INC_CM, dPdR_INC_CN, dPdR_INC_CY;
  double dPdY_INC_CA, dPdY_INC_CLL, dPdY_INC_CLN, dPdY_INC_CM, dPdY_INC_CN, dPdY_INC_CY;
  double dYdR_INC_CA, dYdR_INC_CLL, dYdR_INC_CLN, dYdR_INC_CM, dYdR_INC_CN, dYdR_INC_CY;

  void writeAero();
  void getAeroCoeffs();

  int aero_flag;
  int    errAero_Flag;
  double ca_sig, cy_sig, cn_sig, cln_sig;
  double cm_sig, cmq_sig, cll_sig, clp_sig, xcp_sig;
  //
  double ca_Err, cy_Err, cn_Err, cln_Err;
  double cm_Err, cmq_Err, cll_Err, clp_Err, xcp_Err;
  double xcal;
 
  Vec force, moment;

  void init();
  void update(
    double finDeflOne,
    double finDeflTwo,
    double finDeflThree,
    double finDeflFour,
    double centerOfGravity,
    double thrust,
    double refDiam,
    double refArea,
    double dynamicPressure,
    double inputMach,
    double totalAngleOfAttack,
    double phiPrime,
    double speed,
    double nonRolledRollRate,
    double nonRolledPitchRate
  );
  void rpt();

private:
  Output       *out;
  System       *sys;

protected:


};

#endif
