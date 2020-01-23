/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__tone2pyrDNE
#define _nrn_initial _nrn_initial__tone2pyrDNE
#define nrn_cur _nrn_cur__tone2pyrDNE
#define _nrn_current _nrn_current__tone2pyrDNE
#define nrn_jacob _nrn_jacob__tone2pyrDNE
#define nrn_state _nrn_state__tone2pyrDNE
#define _net_receive _net_receive__tone2pyrDNE 
#define release release__tone2pyrDNE 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define Cdur_nmda _p[0]
#define AlphaTmax_nmda _p[1]
#define Beta_nmda _p[2]
#define Erev_nmda _p[3]
#define gbar_nmda _p[4]
#define Cdur_ampa _p[5]
#define AlphaTmax_ampa _p[6]
#define Beta_ampa _p[7]
#define Erev_ampa _p[8]
#define gbar_ampa _p[9]
#define Cainf _p[10]
#define pooldiam _p[11]
#define z _p[12]
#define tauCa _p[13]
#define P0 _p[14]
#define fCa _p[15]
#define initW _p[16]
#define lambda1 _p[17]
#define lambda2 _p[18]
#define threshold1 _p[19]
#define threshold2 _p[20]
#define fmax _p[21]
#define fmin _p[22]
#define inmda _p[23]
#define g_nmda _p[24]
#define on_nmda _p[25]
#define W_nmda _p[26]
#define iampa _p[27]
#define g_ampa _p[28]
#define on_ampa _p[29]
#define W _p[30]
#define ICa _p[31]
#define iCatotal _p[32]
#define Wmax _p[33]
#define Wmin _p[34]
#define maxChange _p[35]
#define normW _p[36]
#define scaleW _p[37]
#define pregid _p[38]
#define postgid _p[39]
#define r_nmda _p[40]
#define r_ampa _p[41]
#define capoolcon _p[42]
#define eca _p[43]
#define t0 _p[44]
#define Afactor _p[45]
#define dW_ampa _p[46]
#define Dr_nmda _p[47]
#define Dr_ampa _p[48]
#define Dcapoolcon _p[49]
#define v _p[50]
#define _g _p[51]
#define _tsav _p[52]
#define _nd_area  *_ppvar[0]._pval
#define _ion_eca	*_ppvar[2]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_NE2();
 static double _hoc_NEn();
 static double _hoc_eta();
 static double _hoc_omega();
 static double _hoc_sfunc();
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "NE2", _hoc_NE2,
 "NEn", _hoc_NEn,
 "eta", _hoc_eta,
 "omega", _hoc_omega,
 "sfunc", _hoc_sfunc,
 0, 0
};
#define NE2 NE2_tone2pyrDNE
#define NEn NEn_tone2pyrDNE
#define eta eta_tone2pyrDNE
#define omega omega_tone2pyrDNE
#define sfunc sfunc_tone2pyrDNE
 extern double NE2( _threadargsprotocomma_ double , double );
 extern double NEn( _threadargsprotocomma_ double , double );
 extern double eta( _threadargsprotocomma_ double );
 extern double omega( _threadargsprotocomma_ double , double , double );
 extern double sfunc( _threadargsprotocomma_ double );
 /* declare global and static user variables */
#define Beta2 Beta2_tone2pyrDNE
 double Beta2 = 0.0001;
#define Beta1 Beta1_tone2pyrDNE
 double Beta1 = 0.001;
#define NE_S NE_S_tone2pyrDNE
 double NE_S = 1.3;
#define NE_t2 NE_t2_tone2pyrDNE
 double NE_t2 = 0.9;
#define NE_t1 NE_t1_tone2pyrDNE
 double NE_t1 = 1;
#define NEstop2 NEstop2_tone2pyrDNE
 double NEstop2 = 36000;
#define NEstart2 NEstart2_tone2pyrDNE
 double NEstart2 = 35900;
#define NEstop1 NEstop1_tone2pyrDNE
 double NEstop1 = 40000;
#define NEstart1 NEstart1_tone2pyrDNE
 double NEstart1 = 39500;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "Beta1_tone2pyrDNE", "/ms",
 "Beta2_tone2pyrDNE", "/ms",
 "Cdur_nmda", "ms",
 "AlphaTmax_nmda", "/ms",
 "Beta_nmda", "/ms",
 "Erev_nmda", "mV",
 "gbar_nmda", "uS",
 "Cdur_ampa", "ms",
 "AlphaTmax_ampa", "/ms",
 "Beta_ampa", "/ms",
 "Erev_ampa", "mV",
 "gbar_ampa", "uS",
 "Cainf", "mM",
 "pooldiam", "micrometer",
 "tauCa", "ms",
 "inmda", "nA",
 "g_nmda", "uS",
 "iampa", "nA",
 "g_ampa", "uS",
 "ICa", "mA",
 "iCatotal", "mA",
 0,0
};
 static double capoolcon0 = 0;
 static double delta_t = 0.01;
 static double r_ampa0 = 0;
 static double r_nmda0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "NEstart1_tone2pyrDNE", &NEstart1_tone2pyrDNE,
 "NEstop1_tone2pyrDNE", &NEstop1_tone2pyrDNE,
 "NEstart2_tone2pyrDNE", &NEstart2_tone2pyrDNE,
 "NEstop2_tone2pyrDNE", &NEstop2_tone2pyrDNE,
 "NE_t1_tone2pyrDNE", &NE_t1_tone2pyrDNE,
 "NE_t2_tone2pyrDNE", &NE_t2_tone2pyrDNE,
 "NE_S_tone2pyrDNE", &NE_S_tone2pyrDNE,
 "Beta1_tone2pyrDNE", &Beta1_tone2pyrDNE,
 "Beta2_tone2pyrDNE", &Beta2_tone2pyrDNE,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[3]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"tone2pyrDNE",
 "Cdur_nmda",
 "AlphaTmax_nmda",
 "Beta_nmda",
 "Erev_nmda",
 "gbar_nmda",
 "Cdur_ampa",
 "AlphaTmax_ampa",
 "Beta_ampa",
 "Erev_ampa",
 "gbar_ampa",
 "Cainf",
 "pooldiam",
 "z",
 "tauCa",
 "P0",
 "fCa",
 "initW",
 "lambda1",
 "lambda2",
 "threshold1",
 "threshold2",
 "fmax",
 "fmin",
 0,
 "inmda",
 "g_nmda",
 "on_nmda",
 "W_nmda",
 "iampa",
 "g_ampa",
 "on_ampa",
 "W",
 "ICa",
 "iCatotal",
 "Wmax",
 "Wmin",
 "maxChange",
 "normW",
 "scaleW",
 "pregid",
 "postgid",
 0,
 "r_nmda",
 "r_ampa",
 "capoolcon",
 0,
 0};
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 53, _prop);
 	/*initialize range parameters*/
 	Cdur_nmda = 16.765;
 	AlphaTmax_nmda = 0.2659;
 	Beta_nmda = 0.008;
 	Erev_nmda = 0;
 	gbar_nmda = 0.0005;
 	Cdur_ampa = 1.421;
 	AlphaTmax_ampa = 3.8142;
 	Beta_ampa = 0.1429;
 	Erev_ampa = 0;
 	gbar_ampa = 0.001;
 	Cainf = 5e-05;
 	pooldiam = 1.8172;
 	z = 2;
 	tauCa = 50;
 	P0 = 0.015;
 	fCa = 0.024;
 	initW = 5.5;
 	lambda1 = 80;
 	lambda2 = 0.04;
 	threshold1 = 0.4;
 	threshold2 = 0.53;
 	fmax = 3.6;
 	fmin = 0.8;
  }
 	_prop->param = _p;
 	_prop->param_size = 53;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[2]._pval = &prop_ion->param[0]; /* eca */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _net_receive(Point_process*, double*, double);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _tone2pyrDNE_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("ca", -10000.);
 	_ca_sym = hoc_lookup("ca_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 53, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 tone2pyrDNE /home/nest/Desktop/Project_codes/KimEtAl2013/x86_64/tone2pyrDNE.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96485.0;
 static double pi = 3.141592;
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[3], _dlist1[3];
 static int release(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   if ( t0 > 0.0 ) {
     if ( t - t0 < Cdur_nmda ) {
       on_nmda = 1.0 ;
       }
     else {
       on_nmda = 0.0 ;
       }
     if ( t - t0 < Cdur_ampa ) {
       on_ampa = 1.0 ;
       }
     else {
       on_ampa = 0.0 ;
       }
     }
   Dr_nmda = AlphaTmax_nmda * on_nmda * ( 1.0 - r_nmda ) - Beta_nmda * r_nmda ;
   Dr_ampa = AlphaTmax_ampa * on_ampa * ( 1.0 - r_ampa ) - Beta_ampa * r_ampa ;
   dW_ampa = eta ( _threadargscomma_ capoolcon ) * ( lambda1 * omega ( _threadargscomma_ capoolcon , threshold1 , threshold2 ) - lambda2 * W ) * dt ;
   if ( fabs ( dW_ampa ) > maxChange ) {
     if ( dW_ampa < 0.0 ) {
       dW_ampa = - 1.0 * maxChange ;
       }
     else {
       dW_ampa = maxChange ;
       }
     }
   normW = ( W - Wmin ) / ( Wmax - Wmin ) ;
   if ( dW_ampa < 0.0 ) {
     scaleW = sqrt ( fabs ( normW ) ) ;
     }
   else {
     scaleW = sqrt ( fabs ( 1.0 - normW ) ) ;
     }
   W = W + dW_ampa * scaleW ;
   if ( W > Wmax ) {
     W = Wmax ;
     }
   else if ( W < Wmin ) {
     W = Wmin ;
     }
   g_nmda = gbar_nmda * r_nmda * NEn ( _threadargscomma_ NEstart1 , NEstop1 ) * NE2 ( _threadargscomma_ NEstart2 , NEstop2 ) ;
   inmda = W_nmda * g_nmda * ( v - Erev_nmda ) * sfunc ( _threadargscomma_ v ) ;
   g_ampa = gbar_ampa * r_ampa ;
   iampa = W * g_ampa * ( v - Erev_ampa ) ;
   ICa = P0 * g_nmda * ( v - eca ) * sfunc ( _threadargscomma_ v ) ;
   Dcapoolcon = - fCa * Afactor * ICa + ( Cainf - capoolcon ) / tauCa ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 if ( t0 > 0.0 ) {
   if ( t - t0 < Cdur_nmda ) {
     on_nmda = 1.0 ;
     }
   else {
     on_nmda = 0.0 ;
     }
   if ( t - t0 < Cdur_ampa ) {
     on_ampa = 1.0 ;
     }
   else {
     on_ampa = 0.0 ;
     }
   }
 Dr_nmda = Dr_nmda  / (1. - dt*( ( AlphaTmax_nmda * on_nmda )*( ( ( - 1.0 ) ) ) - ( Beta_nmda )*( 1.0 ) )) ;
 Dr_ampa = Dr_ampa  / (1. - dt*( ( AlphaTmax_ampa * on_ampa )*( ( ( - 1.0 ) ) ) - ( Beta_ampa )*( 1.0 ) )) ;
 dW_ampa = eta ( _threadargscomma_ capoolcon ) * ( lambda1 * omega ( _threadargscomma_ capoolcon , threshold1 , threshold2 ) - lambda2 * W ) * dt ;
 if ( fabs ( dW_ampa ) > maxChange ) {
   if ( dW_ampa < 0.0 ) {
     dW_ampa = - 1.0 * maxChange ;
     }
   else {
     dW_ampa = maxChange ;
     }
   }
 normW = ( W - Wmin ) / ( Wmax - Wmin ) ;
 if ( dW_ampa < 0.0 ) {
   scaleW = sqrt ( fabs ( normW ) ) ;
   }
 else {
   scaleW = sqrt ( fabs ( 1.0 - normW ) ) ;
   }
 W = W + dW_ampa * scaleW ;
 if ( W > Wmax ) {
   W = Wmax ;
   }
 else if ( W < Wmin ) {
   W = Wmin ;
   }
 g_nmda = gbar_nmda * r_nmda * NEn ( _threadargscomma_ NEstart1 , NEstop1 ) * NE2 ( _threadargscomma_ NEstart2 , NEstop2 ) ;
 inmda = W_nmda * g_nmda * ( v - Erev_nmda ) * sfunc ( _threadargscomma_ v ) ;
 g_ampa = gbar_ampa * r_ampa ;
 iampa = W * g_ampa * ( v - Erev_ampa ) ;
 ICa = P0 * g_nmda * ( v - eca ) * sfunc ( _threadargscomma_ v ) ;
 Dcapoolcon = Dcapoolcon  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tauCa )) ;
  return 0;
}
 /*END CVODE*/
 static int release (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   if ( t0 > 0.0 ) {
     if ( t - t0 < Cdur_nmda ) {
       on_nmda = 1.0 ;
       }
     else {
       on_nmda = 0.0 ;
       }
     if ( t - t0 < Cdur_ampa ) {
       on_ampa = 1.0 ;
       }
     else {
       on_ampa = 0.0 ;
       }
     }
    r_nmda = r_nmda + (1. - exp(dt*(( AlphaTmax_nmda * on_nmda )*( ( ( - 1.0 ) ) ) - ( Beta_nmda )*( 1.0 ))))*(- ( ( ( AlphaTmax_nmda )*( on_nmda ) )*( ( 1.0 ) ) ) / ( ( ( AlphaTmax_nmda )*( on_nmda ) )*( ( ( - 1.0 ) ) ) - ( Beta_nmda )*( 1.0 ) ) - r_nmda) ;
    r_ampa = r_ampa + (1. - exp(dt*(( AlphaTmax_ampa * on_ampa )*( ( ( - 1.0 ) ) ) - ( Beta_ampa )*( 1.0 ))))*(- ( ( ( AlphaTmax_ampa )*( on_ampa ) )*( ( 1.0 ) ) ) / ( ( ( AlphaTmax_ampa )*( on_ampa ) )*( ( ( - 1.0 ) ) ) - ( Beta_ampa )*( 1.0 ) ) - r_ampa) ;
   dW_ampa = eta ( _threadargscomma_ capoolcon ) * ( lambda1 * omega ( _threadargscomma_ capoolcon , threshold1 , threshold2 ) - lambda2 * W ) * dt ;
   if ( fabs ( dW_ampa ) > maxChange ) {
     if ( dW_ampa < 0.0 ) {
       dW_ampa = - 1.0 * maxChange ;
       }
     else {
       dW_ampa = maxChange ;
       }
     }
   normW = ( W - Wmin ) / ( Wmax - Wmin ) ;
   if ( dW_ampa < 0.0 ) {
     scaleW = sqrt ( fabs ( normW ) ) ;
     }
   else {
     scaleW = sqrt ( fabs ( 1.0 - normW ) ) ;
     }
   W = W + dW_ampa * scaleW ;
   if ( W > Wmax ) {
     W = Wmax ;
     }
   else if ( W < Wmin ) {
     W = Wmin ;
     }
   g_nmda = gbar_nmda * r_nmda * NEn ( _threadargscomma_ NEstart1 , NEstop1 ) * NE2 ( _threadargscomma_ NEstart2 , NEstop2 ) ;
   inmda = W_nmda * g_nmda * ( v - Erev_nmda ) * sfunc ( _threadargscomma_ v ) ;
   g_ampa = gbar_ampa * r_ampa ;
   iampa = W * g_ampa * ( v - Erev_ampa ) ;
   ICa = P0 * g_nmda * ( v - eca ) * sfunc ( _threadargscomma_ v ) ;
    capoolcon = capoolcon + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tauCa)))*(- ( ( ( - fCa )*( Afactor ) )*( ICa ) + ( ( Cainf ) ) / tauCa ) / ( ( ( ( - 1.0 ) ) ) / tauCa ) - capoolcon) ;
   }
  return 0;
}
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{  double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _thread = (Datum*)0; _nt = (_NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
   t0 = t ;
   } }
 
double sfunc ( _threadargsprotocomma_ double _lv ) {
   double _lsfunc;
  _lsfunc = 1.0 / ( 1.0 + 0.33 * exp ( - 0.06 * _lv ) ) ;
    
return _lsfunc;
 }
 
static double _hoc_sfunc(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  sfunc ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
double eta ( _threadargsprotocomma_ double _lCani ) {
   double _leta;
 double _ltaulearn , _lP1 , _lP2 , _lP4 , _lCacon ;
 _lP1 = 0.1 ;
   _lP2 = _lP1 * 1e-4 ;
   _lP4 = 1.0 ;
   _lCacon = _lCani * 1e3 ;
   _ltaulearn = _lP1 / ( _lP2 + _lCacon * _lCacon * _lCacon ) + _lP4 ;
   _leta = 1.0 / _ltaulearn * 0.001 ;
   
return _leta;
 }
 
static double _hoc_eta(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  eta ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
double omega ( _threadargsprotocomma_ double _lCani , double _lthreshold1 , double _lthreshold2 ) {
   double _lomega;
 double _lr , _lmid , _lCacon ;
 _lCacon = _lCani * 1e3 ;
   _lr = ( _lthreshold2 - _lthreshold1 ) / 2.0 ;
   _lmid = ( _lthreshold1 + _lthreshold2 ) / 2.0 ;
   if ( _lCacon <= _lthreshold1 ) {
     _lomega = 0.0 ;
     }
   else if ( _lCacon >= _lthreshold2 ) {
     _lomega = 1.0 / ( 1.0 + 50.0 * exp ( - 50.0 * ( _lCacon - _lthreshold2 ) ) ) ;
     }
   else {
     _lomega = - sqrt ( _lr * _lr - ( _lCacon - _lmid ) * ( _lCacon - _lmid ) ) ;
     }
   
return _lomega;
 }
 
static double _hoc_omega(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  omega ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
double NEn ( _threadargsprotocomma_ double _lNEstart1 , double _lNEstop1 ) {
   double _lNEn;
 double _lNEtemp1 , _lNEtemp2 , _lNEtemp3 , _lNEtemp4 , _lNEtemp5 , _lNEtemp6 , _lNEtemp7 , _lNEtemp8 , _lNEtemp9 , _lNEtemp10 , _lNEtemp11 , _lNEtemp12 , _lNEtemp13 , _lNEtemp14 , _lNEtemp15 , _lNEtemp16 , _lNEtemp17 , _lNEtemp18 , _ls ;
 _lNEtemp1 = 64000.0 ;
   _lNEtemp2 = 70000.0 ;
   _lNEtemp3 = 76000.0 ;
   _lNEtemp4 = 82000.0 ;
   _lNEtemp5 = 88000.0 ;
   _lNEtemp6 = 94000.0 ;
   _lNEtemp7 = 100000.0 ;
   _lNEtemp8 = 106000.0 ;
   _lNEtemp9 = 112000.0 ;
   _lNEtemp10 = 118000.0 ;
   _lNEtemp11 = 124000.0 ;
   _lNEtemp12 = 130000.0 ;
   _lNEtemp13 = 136000.0 ;
   _lNEtemp14 = 142000.0 ;
   _lNEtemp15 = 248000.0 ;
   _lNEtemp16 = 254000.0 ;
   _lNEtemp17 = 260000.0 ;
   _lNEtemp18 = 266000.0 ;
   if ( t <= _lNEstart1 ) {
     _lNEn = 1.0 ;
     }
   else if ( t >= _lNEstart1  && t <= _lNEstop1 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEstop1  && t < _lNEtemp1 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - _lNEstop1 ) ) ;
     }
   else if ( t >= _lNEtemp1  && t <= _lNEtemp1 + 2000.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp1 + 2000.0  && t < _lNEtemp2 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - _lNEstop1 ) ) ;
     }
   else if ( t >= _lNEtemp2  && t <= _lNEtemp2 + 2000.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp2 + 2000.0  && t < _lNEtemp3 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lNEtemp2 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp3  && t <= _lNEtemp3 + 2000.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp3 + 2000.0  && t < _lNEtemp4 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lNEtemp3 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp4  && t <= _lNEtemp4 + 2000.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp4 + 2000.0  && t < _lNEtemp5 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lNEtemp4 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp5  && t <= _lNEtemp5 + 2000.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp5 + 2000.0  && t < _lNEtemp6 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lNEtemp5 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp6  && t <= _lNEtemp6 + 2000.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp6 + 2000.0  && t < _lNEtemp7 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lNEtemp6 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp7  && t <= _lNEtemp7 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp7 + 2000.0  && t < _lNEtemp8 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp7 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp8  && t <= _lNEtemp8 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp8 + 2000.0  && t < _lNEtemp9 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp8 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp9  && t <= _lNEtemp9 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp9 + 2000.0  && t < _lNEtemp10 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp9 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp10  && t <= _lNEtemp10 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp10 + 2000.0  && t < _lNEtemp11 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp10 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp11  && t <= _lNEtemp11 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp11 + 2000.0  && t < _lNEtemp12 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp11 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp12  && t <= _lNEtemp12 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp12 + 2000.0  && t < _lNEtemp13 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp12 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp13  && t <= _lNEtemp13 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp13 + 2000.0  && t < _lNEtemp14 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp13 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp14  && t <= _lNEtemp14 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp14 + 2000.0  && t < _lNEtemp15 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp14 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp15  && t <= _lNEtemp15 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp15 + 2000.0  && t < _lNEtemp16 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp15 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp16  && t <= _lNEtemp16 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp16 + 2000.0  && t < _lNEtemp17 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp16 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp17  && t <= _lNEtemp17 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp17 + 2000.0  && t < _lNEtemp18 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp17 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp18  && t <= _lNEtemp18 + 2000.0 ) {
     _lNEn = NE_t2 ;
     }
   else {
     _lNEn = 1.0 ;
     }
   
return _lNEn;
 }
 
static double _hoc_NEn(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  NEn ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) );
 return(_r);
}
 
double NE2 ( _threadargsprotocomma_ double _lNEstart2 , double _lNEstop2 ) {
   double _lNE2;
 double _lNE2temp1 , _lNE2temp2 , _lNE2temp3 , _lNE2temp4 , _lNE2temp5 , _lNE2temp6 , _lNE2temp7 , _lNE2temp8 , _lNE2temp9 , _lNE2temp10 , _lNE2temp11 , _lNE2temp12 , _lNE2temp13 , _lNE2temp14 , _lNE2temp15 , _lNE2temp16 , _ls ;
 _lNE2temp1 = _lNEstart2 + 6000.0 ;
   _lNE2temp2 = _lNE2temp1 + 6000.0 ;
   _lNE2temp3 = _lNE2temp2 + 6000.0 ;
   _lNE2temp4 = _lNE2temp3 + 6000.0 ;
   _lNE2temp5 = _lNE2temp4 + 6000.0 ;
   _lNE2temp6 = _lNE2temp5 + 6000.0 ;
   _lNE2temp7 = _lNE2temp6 + 6000.0 ;
   _lNE2temp8 = _lNE2temp7 + 6000.0 ;
   _lNE2temp9 = _lNE2temp8 + 6000.0 ;
   _lNE2temp10 = _lNE2temp9 + 6000.0 ;
   _lNE2temp11 = _lNE2temp10 + 6000.0 ;
   _lNE2temp12 = _lNE2temp11 + 6000.0 ;
   _lNE2temp13 = _lNE2temp12 + 6000.0 ;
   _lNE2temp14 = _lNE2temp13 + 6000.0 ;
   _lNE2temp15 = _lNE2temp14 + 6000.0 ;
   if ( t <= _lNEstart2 ) {
     _lNE2 = 1.0 ;
     }
   else if ( t >= _lNEstart2  && t <= _lNEstop2 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNEstop2  && t < _lNE2temp1 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEstop2 + 2000.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp1  && t <= _lNE2temp1 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp1 + 200.0  && t < _lNE2temp2 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp1 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp2  && t <= _lNE2temp2 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp2 + 200.0  && t < _lNE2temp3 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp2 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp3  && t <= _lNE2temp3 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp3 + 200.0  && t < _lNE2temp4 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp3 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp4  && t <= _lNE2temp4 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp4 + 200.0  && t < _lNE2temp5 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp4 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp5  && t <= _lNE2temp5 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp5 + 200.0  && t < _lNE2temp6 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp5 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp6  && t <= _lNE2temp6 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp6 + 200.0  && t < _lNE2temp7 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp6 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp7  && t <= _lNE2temp7 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp7 + 200.0  && t < _lNE2temp8 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp7 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp8  && t <= _lNE2temp8 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp8 + 200.0  && t < _lNE2temp9 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp8 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp9  && t <= _lNE2temp9 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp9 + 200.0  && t < _lNE2temp10 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp9 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp10  && t <= _lNE2temp10 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp10 + 200.0  && t < _lNE2temp11 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp10 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp11  && t <= _lNE2temp11 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp11 + 200.0  && t < _lNE2temp12 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp11 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp12  && t <= _lNE2temp12 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp12 + 200.0  && t < _lNE2temp13 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp12 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp13  && t <= _lNE2temp13 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp13 + 200.0  && t < _lNE2temp14 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp13 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp14  && t <= _lNE2temp14 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp14 + 200.0  && t < _lNE2temp15 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp14 + 200.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp15  && t <= _lNE2temp15 + 200.0 ) {
     _lNE2 = NE_S ;
     }
   else {
     _lNE2 = 1.0 ;
     }
   
return _lNE2;
 }
 
static double _hoc_NE2(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  NE2 ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) );
 return(_r);
}
 
static int _ode_count(int _type){ return 3;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  eca = _ion_eca;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 3; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  eca = _ion_eca;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 2, 0);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  capoolcon = capoolcon0;
  r_ampa = r_ampa0;
  r_nmda = r_nmda0;
 {
   on_nmda = 0.0 ;
   r_nmda = 0.0 ;
   W_nmda = initW ;
   on_ampa = 0.0 ;
   r_ampa = 0.0 ;
   W = initW ;
   t0 = - 1.0 ;
   Wmax = fmax * initW ;
   Wmin = fmin * initW ;
   maxChange = ( Wmax - Wmin ) / 10.0 ;
   dW_ampa = 0.0 ;
   capoolcon = Cainf ;
   Afactor = 1.0 / ( z * FARADAY * 4.0 / 3.0 * pi * pow( ( pooldiam / 2.0 ) , 3.0 ) ) * ( 1e6 ) ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  eca = _ion_eca;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   }
 _current += inmda;
 _current += iampa;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
  eca = _ion_eca;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
  eca = _ion_eca;
 {   release(_p, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(r_nmda) - _p;  _dlist1[0] = &(Dr_nmda) - _p;
 _slist1[1] = &(r_ampa) - _p;  _dlist1[1] = &(Dr_ampa) - _p;
 _slist1[2] = &(capoolcon) - _p;  _dlist1[2] = &(Dcapoolcon) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/home/nest/Desktop/Project_codes/KimEtAl2013/mods/tone2pyrDNE.mod";
static const char* nmodl_file_text = 
  ":Tone to Pyramidal Cells AMPA+NMDA with local Ca2+ pool\n"
  " \n"
  "NEURON {\n"
  "	POINT_PROCESS tone2pyrDNE\n"
  "	USEION ca READ eca	\n"
  "	NONSPECIFIC_CURRENT inmda, iampa\n"
  "	RANGE initW\n"
  "	RANGE Cdur_nmda, AlphaTmax_nmda, Beta_nmda, Erev_nmda, gbar_nmda, W_nmda, on_nmda, g_nmda\n"
  "	RANGE Cdur_ampa, AlphaTmax_ampa, Beta_ampa, Erev_ampa, gbar_ampa, W, on_ampa, g_ampa\n"
  "	RANGE eca, ICa, P0, fCa, tauCa, iCatotal\n"
  "	RANGE Cainf, pooldiam, z\n"
  "	RANGE lambda1, lambda2, threshold1, threshold2\n"
  "	RANGE fmax, fmin, Wmax, Wmin, maxChange, normW, scaleW\n"
  "	RANGE pregid,postgid\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(mV) = (millivolt)\n"
  "        (nA) = (nanoamp)\n"
  "	(uS) = (microsiemens)\n"
  "	FARADAY = 96485 (coul)\n"
  "	pi = 3.141592 (1)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	Cdur_nmda = 16.7650 (ms)\n"
  "	AlphaTmax_nmda = .2659 (/ms)\n"
  "	Beta_nmda = 0.008 (/ms)\n"
  "	Erev_nmda = 0 (mV)\n"
  "	gbar_nmda = .5e-3 (uS)\n"
  "\n"
  "	Cdur_ampa = 1.4210 (ms)\n"
  "	AlphaTmax_ampa = 3.8142 (/ms)\n"
  "	Beta_ampa = 0.1429 (/ms)\n"
  "	Erev_ampa = 0 (mV)\n"
  "	gbar_ampa = 1e-3 (uS)\n"
  "\n"
  "	eca = 120\n"
  "\n"
  "	Cainf = 50e-6 (mM)\n"
  "	pooldiam =  1.8172 (micrometer)\n"
  "	z = 2\n"
  "\n"
  "	tauCa = 50 (ms)\n"
  "	P0 = .015\n"
  "	fCa = .024\n"
  "\n"
  "	NEstart1 = 39500\n"
  "	NEstop1 = 40000	\n"
  "	NEstart2 = 35900\n"
  "	NEstop2 = 36000		\n"
  "\n"
  "	NE_t1 = 1 : 1\n"
  "	NE_t2 = 0.9 : 1\n"
  "	NE_S = 1.3	\n"
  "	Beta1 = 0.001  (/ms) : 1/decay time for neuromodulators\n"
  "	Beta2 = 0.0001  (/ms)	\n"
  "	\n"
  "	initW = 5.5 : 5 : 4 : 3 : 4 : 8\n"
  "\n"
  "	lambda1 = 80 : 60 : 70: 20 : 15\n"
  "	lambda2 = .04 : .0 : 0.03\n"
  "	threshold1 = 0.40 : 0.35 : 0.45 : 0.5 : 0.3 (uM)\n"
  "	threshold2 = 0.53 : 0.55 : 0.60 : 0.55 : 0.55 : 0.6 : 0.4 : 0.45 (uM)\n"
  "\n"
  "	fmax = 3.6 : 4 : 3\n"
  "	fmin = .8\n"
  "\n"
  "\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v (mV)\n"
  "\n"
  "	inmda (nA)\n"
  "	g_nmda (uS)\n"
  "	on_nmda\n"
  "	W_nmda\n"
  "\n"
  "	iampa (nA)\n"
  "	g_ampa (uS)\n"
  "	on_ampa\n"
  "	W\n"
  "\n"
  "	t0 (ms)\n"
  "\n"
  "	ICa (mA)\n"
  "	Afactor	(mM/ms/nA)\n"
  "	iCatotal (mA)\n"
  "\n"
  "	dW_ampa\n"
  "	Wmax\n"
  "	Wmin\n"
  "	maxChange\n"
  "	normW\n"
  "	scaleW\n"
  "	\n"
  "	pregid\n"
  "	postgid\n"
  "}\n"
  "\n"
  "STATE { r_nmda r_ampa capoolcon }\n"
  "\n"
  "INITIAL {\n"
  "	on_nmda = 0\n"
  "	r_nmda = 0\n"
  "	W_nmda = initW\n"
  "\n"
  "	on_ampa = 0\n"
  "	r_ampa = 0\n"
  "	W = initW\n"
  "\n"
  "	t0 = -1\n"
  "\n"
  "	Wmax = fmax*initW\n"
  "	Wmin = fmin*initW\n"
  "	maxChange = (Wmax-Wmin)/10\n"
  "	dW_ampa = 0\n"
  "\n"
  "	capoolcon = Cainf\n"
  "	Afactor	= 1/(z*FARADAY*4/3*pi*(pooldiam/2)^3)*(1e6)\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE release METHOD cnexp\n"
  "}\n"
  "\n"
  "DERIVATIVE release {\n"
  "	if (t0>0) {\n"
  "		if (t-t0 < Cdur_nmda) {\n"
  "			on_nmda = 1\n"
  "		} else {\n"
  "			on_nmda = 0\n"
  "		}\n"
  "		if (t-t0 < Cdur_ampa) {\n"
  "			on_ampa = 1\n"
  "		} else {\n"
  "			on_ampa = 0\n"
  "		}\n"
  "	}\n"
  "	r_nmda' = AlphaTmax_nmda*on_nmda*(1-r_nmda)-Beta_nmda*r_nmda\n"
  "	r_ampa' = AlphaTmax_ampa*on_ampa*(1-r_ampa)-Beta_ampa*r_ampa\n"
  "\n"
  "	dW_ampa = eta(capoolcon)*(lambda1*omega(capoolcon, threshold1, threshold2)-lambda2*W)*dt\n"
  "\n"
  "	: Limit for extreme large weight changes\n"
  "	if (fabs(dW_ampa) > maxChange) {\n"
  "		if (dW_ampa < 0) {\n"
  "			dW_ampa = -1*maxChange\n"
  "		} else {\n"
  "			dW_ampa = maxChange\n"
  "		}\n"
  "	}\n"
  "\n"
  "	:Normalize the weight change\n"
  "	normW = (W-Wmin)/(Wmax-Wmin)\n"
  "	if (dW_ampa < 0) {\n"
  "		scaleW = sqrt(fabs(normW))\n"
  "	} else {\n"
  "		scaleW = sqrt(fabs(1.0-normW))\n"
  "	}\n"
  "\n"
  "	W = W + dW_ampa*scaleW\n"
  "	\n"
  "	:Weight value limits\n"
  "	if (W > Wmax) { \n"
  "		W = Wmax\n"
  "	} else if (W < Wmin) {\n"
  " 		W = Wmin\n"
  "	}\n"
  "\n"
  "	g_nmda = gbar_nmda*r_nmda*NEn(NEstart1,NEstop1)*NE2(NEstart2,NEstop2)\n"
  "	inmda = W_nmda*g_nmda*(v - Erev_nmda)*sfunc(v)\n"
  "\n"
  "	g_ampa = gbar_ampa*r_ampa\n"
  "	iampa = W*g_ampa*(v - Erev_ampa)\n"
  "\n"
  "	ICa = P0*g_nmda*(v - eca)*sfunc(v)\n"
  "	capoolcon'= -fCa*Afactor*ICa + (Cainf-capoolcon)/tauCa\n"
  "}\n"
  "\n"
  "NET_RECEIVE(dummy_weight) {\n"
  "	t0 = t\n"
  "}\n"
  "\n"
  ":::::::::::: FUNCTIONs and PROCEDUREs ::::::::::::\n"
  "\n"
  "FUNCTION sfunc (v (mV)) {\n"
  "	UNITSOFF\n"
  "	sfunc = 1/(1+0.33*exp(-0.06*v))\n"
  "	UNITSON\n"
  "}\n"
  "\n"
  "FUNCTION eta(Cani (mM)) {\n"
  "	LOCAL taulearn, P1, P2, P4, Cacon\n"
  "	P1 = 0.1\n"
  "	P2 = P1*1e-4\n"
  "	P4 = 1\n"
  "	Cacon = Cani*1e3\n"
  "	taulearn = P1/(P2+Cacon*Cacon*Cacon)+P4\n"
  "	eta = 1/taulearn*0.001\n"
  "}\n"
  "\n"
  "FUNCTION omega(Cani (mM), threshold1 (uM), threshold2 (uM)) {\n"
  "	LOCAL r, mid, Cacon\n"
  "	Cacon = Cani*1e3\n"
  "	r = (threshold2-threshold1)/2\n"
  "	mid = (threshold1+threshold2)/2\n"
  "	if (Cacon <= threshold1) { omega = 0}\n"
  "	else if (Cacon >= threshold2) {	omega = 1/(1+50*exp(-50*(Cacon-threshold2)))}\n"
  "	else {omega = -sqrt(r*r-(Cacon-mid)*(Cacon-mid))}\n"
  "}\n"
  "\n"
  "FUNCTION NEn(NEstart1 (ms), NEstop1 (ms)) {\n"
  "	LOCAL NEtemp1, NEtemp2, NEtemp3, NEtemp4, NEtemp5, NEtemp6, NEtemp7, NEtemp8, NEtemp9, NEtemp10, NEtemp11, NEtemp12, NEtemp13, NEtemp14, NEtemp15, NEtemp16, NEtemp17, NEtemp18,s\n"
  "	NEtemp1 = 64000\n"
  "	NEtemp2 = 70000\n"
  "	NEtemp3 = 76000\n"
  "	NEtemp4 = 82000\n"
  "	NEtemp5 = 88000\n"
  "	NEtemp6 = 94000\n"
  "	NEtemp7 = 100000\n"
  "	NEtemp8 = 106000\n"
  "	NEtemp9 = 112000\n"
  "	NEtemp10 = 118000\n"
  "	NEtemp11 = 124000\n"
  "	NEtemp12 = 130000\n"
  "	NEtemp13 = 136000\n"
  "	NEtemp14 = 142000\n"
  "	NEtemp15 = 248000 : NEtemp14 + 6000 + 100000     : 100sec Gap\n"
  "	NEtemp16 = 254000 : NEtemp15 + 6000 \n"
  "	NEtemp17 = 260000 : NEtemp16 + 6000\n"
  "	NEtemp18 = 266000 : NEtemp17 + 6000\n"
  "\n"
  "	if (t <= NEstart1) { NEn = 1.0}\n"
  "	else if (t >= NEstart1 && t <= NEstop1) {NEn = NE_t1}					: 2nd tone in early conditioning (EC)\n"
  "		else if (t > NEstop1 && t < NEtemp1) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-NEstop1))}  		: Basal level\n"
  "	else if (t >= NEtemp1 && t <= NEtemp1+2000) {NEn = NE_t1}					: 3rd tone EC\n"
  "		else if (t > NEtemp1+2000 && t < NEtemp2) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-NEstop1))}  	: Basal level\n"
  "	else if (t >= NEtemp2 && t <= NEtemp2+2000) {NEn = NE_t1}					: 4th tone EC\n"
  "		else if (t > NEtemp2+2000 && t < NEtemp3) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-(NEtemp2+2000)))}  	: Basal level	\n"
  "	else if (t >= NEtemp3 && t <= NEtemp3+2000) {NEn = NE_t1}					: 5th tone EC\n"
  "		else if (t > NEtemp3+2000 && t < NEtemp4) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-(NEtemp3+2000)))}  	: Basal level\n"
  "	else if (t >= NEtemp4 && t <= NEtemp4+2000) {NEn = NE_t1}					: 6th tone EC\n"
  "		else if (t > NEtemp4+2000 && t < NEtemp5) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-(NEtemp4+2000)))}  		: Basal level\n"
  "	else if (t >= NEtemp5 && t <= NEtemp5+2000) {NEn = NE_t1}					: 7th tone EC\n"
  "		else if (t > NEtemp5+2000 && t < NEtemp6) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-(NEtemp5+2000)))}  	: Basal level\n"
  "	else if (t >= NEtemp6 && t <= NEtemp6+2000) {NEn = NE_t1}					: 8th tone EC\n"
  "		else if (t > NEtemp6+2000 && t < NEtemp7) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-(NEtemp6+2000)))}  	: Basal level\n"
  "	\n"
  "	else if (t >= NEtemp7 && t <= NEtemp7+2000) {NEn = NE_t2}					: 9th tone	- Second Step late cond (LC)\n"
  "		else if (t > NEtemp7+2000 && t < NEtemp8) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp7+2000)))}  		: Basal level\n"
  "	else if (t >= NEtemp8 && t <= NEtemp8+2000) {NEn = NE_t2}					: 10th tone  LC\n"
  "		else if (t > NEtemp8+2000 && t < NEtemp9) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp8+2000)))}		: Basal level	\n"
  "	else if (t >= NEtemp9 && t <= NEtemp9+2000) {NEn = NE_t2}					: 11th tone  LC \n"
  "		else if (t > NEtemp9+2000 && t < NEtemp10) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp9+2000)))}  	: Basal level	\n"
  "	else if (t >= NEtemp10 && t <= NEtemp10+2000) {NEn = NE_t2}					: 12th tone  LC\n"
  "		else if (t > NEtemp10+2000 && t < NEtemp11) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp10+2000)))}  	: Basal level\n"
  "	else if (t >= NEtemp11 && t <= NEtemp11+2000) {NEn = NE_t2}					: 13th tone  LC\n"
  "		else if (t > NEtemp11+2000 && t < NEtemp12) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp11+2000)))}  	: Basal level\n"
  "	else if (t >= NEtemp12 && t <= NEtemp12+2000) {NEn = NE_t2}					: 14th tone  LC\n"
  "		else if (t > NEtemp12+2000 && t < NEtemp13) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp12+2000)))}  	: Basal level\n"
  "	else if (t >= NEtemp13 && t <= NEtemp13+2000) {NEn = NE_t2}					: 15th tone  LC\n"
  "		else if (t > NEtemp13+2000 && t < NEtemp14) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp13+2000)))}  	: Basal level\n"
  "	else if (t >= NEtemp14 && t <= NEtemp14+2000) {NEn = NE_t2}					: 16th tone  LC\n"
  "		else if (t > NEtemp14+2000 && t < NEtemp15) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp14+2000)))}  	: Basal level\n"
  "	\n"
  "	else if (t >= NEtemp15 && t <= NEtemp15+2000) {NEn = NE_t2}					: 1st tone in Early Extinction EE\n"
  "		else if (t > NEtemp15+2000 && t < NEtemp16) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp15+2000)))} 		: Basal level\n"
  "	else if (t >= NEtemp16 && t <= NEtemp16+2000) {NEn = NE_t2}					: 2nd tone EE\n"
  "		else if (t > NEtemp16+2000 && t < NEtemp17) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp16+2000)))}  	: Basal level\n"
  "	else if (t >= NEtemp17 && t <= NEtemp17+2000) {NEn = NE_t2}					: 3rd tone EE\n"
  "		else if (t > NEtemp17+2000 && t < NEtemp18) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp17+2000)))}  	: Basal level	\n"
  "	else if (t >= NEtemp18 && t <= NEtemp18+2000) {NEn = NE_t2}					: 4th tone EE	\n"
  "		else  {	NEn = 1.0}\n"
  "}\n"
  "FUNCTION NE2(NEstart2 (ms), NEstop2 (ms)) {\n"
  "	LOCAL NE2temp1, NE2temp2, NE2temp3, NE2temp4, NE2temp5, NE2temp6, NE2temp7, NE2temp8, NE2temp9, NE2temp10, NE2temp11, NE2temp12, NE2temp13, NE2temp14, NE2temp15, NE2temp16,s\n"
  "	NE2temp1 = NEstart2 + 6000\n"
  "	NE2temp2 = NE2temp1 + 6000\n"
  "	NE2temp3 = NE2temp2 + 6000\n"
  "	NE2temp4 = NE2temp3 + 6000\n"
  "	NE2temp5 = NE2temp4 + 6000\n"
  "	NE2temp6 = NE2temp5 + 6000\n"
  "	NE2temp7 = NE2temp6 + 6000\n"
  "	NE2temp8 = NE2temp7 + 6000\n"
  "	NE2temp9 = NE2temp8 + 6000\n"
  "	NE2temp10 = NE2temp9 + 6000\n"
  "	NE2temp11 = NE2temp10 + 6000\n"
  "	NE2temp12 = NE2temp11 + 6000 \n"
  "	NE2temp13 = NE2temp12 + 6000\n"
  "	NE2temp14 = NE2temp13 + 6000\n"
  "	NE2temp15 = NE2temp14 + 6000\n"
  "	\n"
  "	if (t <= NEstart2) { NE2 = 1.0}\n"
  "	else if (t >= NEstart2 && t <= NEstop2) {NE2 = NE_S }					: 1st shock\n"
  "		else if (t > NEstop2 && t < NE2temp1) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NEstop2+2000)))} \n"
  "	else if (t >= NE2temp1 && t <= NE2temp1+200) {NE2=NE_S}					: 2nd shock\n"
  "		else if (t > NE2temp1+200 && t < NE2temp2) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp1+200)))}   				 \n"
  "	else if (t >= NE2temp2 && t <= NE2temp2+200) {NE2=NE_S}					: 3rd shock\n"
  "		else if (t > NE2temp2+200 && t < NE2temp3) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp2+200)))}  				 \n"
  "	else if (t >= NE2temp3 && t <= NE2temp3+200) {NE2=NE_S}					: 4th shock\n"
  "		else if (t > NE2temp3+200 && t < NE2temp4) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp3+200)))}  				 \n"
  "	else if (t >= NE2temp4 && t <= NE2temp4+200) {NE2=NE_S}					: 5th shock\n"
  "		else if (t > NE2temp4+200 && t < NE2temp5) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp4+200)))}  				 \n"
  "	else if (t >= NE2temp5 && t <= NE2temp5+200) {NE2=NE_S}					: 6th shock\n"
  "		else if (t > NE2temp5+200 && t < NE2temp6) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp5+200)))} 				 \n"
  "	else if (t >= NE2temp6 && t <= NE2temp6+200) {NE2=NE_S}					: 7th shock\n"
  "		else if (t > NE2temp6+200 && t < NE2temp7) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp6+200)))}  				 \n"
  "	else if (t >= NE2temp7 && t <= NE2temp7+200) {NE2=NE_S}					: 8th shock\n"
  "		else if (t > NE2temp7+200 && t < NE2temp8) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp7+200)))}  				    \n"
  "	else if (t >= NE2temp8 && t <= NE2temp8+200) {NE2=NE_S }					: 9th shock\n"
  "		else if (t > NE2temp8+200 && t < NE2temp9) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp8+200)))}  				    \n"
  "	else if (t >= NE2temp9 && t <= NE2temp9+200) {NE2=NE_S }					: 10th shock\n"
  "		else if (t > NE2temp9+200 && t < NE2temp10) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp9+200)))}  				    \n"
  "	else if (t >= NE2temp10 && t <= NE2temp10+200) {NE2=NE_S}					: 11th shock\n"
  "		else if (t > NE2temp10+200 && t < NE2temp11) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp10+200)))}  				 \n"
  "	else if (t >= NE2temp11 && t <= NE2temp11+200) {NE2=NE_S }					: 12th shock\n"
  "		else if (t > NE2temp11+200 && t < NE2temp12) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp11+200)))}  				 \n"
  "	else if (t >= NE2temp12 && t <= NE2temp12+200) {NE2=NE_S}					: 13th shock\n"
  "		else if (t > NE2temp12+200 && t < NE2temp13) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp12+200)))} 				 \n"
  "	else if (t >= NE2temp13 && t <= NE2temp13+200) {NE2=NE_S }					: 14th shock\n"
  "		else if (t > NE2temp13+200 && t < NE2temp14) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp13+200)))}   				 \n"
  "	else if (t >= NE2temp14 && t <= NE2temp14+200) {NE2=NE_S}					: 15th shock\n"
  "		else if (t > NE2temp14+200 && t < NE2temp15) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp14+200)))}  				 \n"
  "	else if (t >= NE2temp15 && t <= NE2temp15+200) {NE2=NE_S}					: 16th shock\n"
  "		else  {	NE2 = 1.0}\n"
  "}\n"
  "\n"
  ;
#endif
