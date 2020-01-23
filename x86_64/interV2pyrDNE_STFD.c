/* Created by Language version: 7.7.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
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
 
#define nrn_init _nrn_init__interV2pyrDNE_STFD
#define _nrn_initial _nrn_initial__interV2pyrDNE_STFD
#define nrn_cur _nrn_cur__interV2pyrDNE_STFD
#define _nrn_current _nrn_current__interV2pyrDNE_STFD
#define nrn_jacob _nrn_jacob__interV2pyrDNE_STFD
#define nrn_state _nrn_state__interV2pyrDNE_STFD
#define _net_receive _net_receive__interV2pyrDNE_STFD 
#define release release__interV2pyrDNE_STFD 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define srcid _p[0]
#define destid _p[1]
#define Cdur_gaba _p[2]
#define AlphaTmax_gaba _p[3]
#define Beta_gaba _p[4]
#define Erev_gaba _p[5]
#define gbar_gaba _p[6]
#define Cainf _p[7]
#define pooldiam _p[8]
#define z _p[9]
#define tauCa _p[10]
#define P0g _p[11]
#define fCag _p[12]
#define lambda1 _p[13]
#define lambda2 _p[14]
#define threshold1 _p[15]
#define threshold2 _p[16]
#define initW _p[17]
#define fmax _p[18]
#define fmin _p[19]
#define thr_rp _p[20]
#define facfactor _p[21]
#define f _p[22]
#define tauF _p[23]
#define d1 _p[24]
#define tauD1 _p[25]
#define d2 _p[26]
#define tauD2 _p[27]
#define igaba _p[28]
#define g_gaba _p[29]
#define on_gaba _p[30]
#define W _p[31]
#define ICag _p[32]
#define Icatotal _p[33]
#define Wmax _p[34]
#define Wmin _p[35]
#define maxChange _p[36]
#define normW _p[37]
#define scaleW _p[38]
#define pregid _p[39]
#define postgid _p[40]
#define F _p[41]
#define D1 _p[42]
#define D2 _p[43]
#define r_nmda _p[44]
#define r_gaba _p[45]
#define capoolcon _p[46]
#define eca _p[47]
#define ica _p[48]
#define t0 _p[49]
#define ICan _p[50]
#define Afactor _p[51]
#define dW_gaba _p[52]
#define rp _p[53]
#define tsyn _p[54]
#define fa _p[55]
#define Dr_nmda _p[56]
#define Dr_gaba _p[57]
#define Dcapoolcon _p[58]
#define _g _p[59]
#define _tsav _p[60]
#define _nd_area  *_ppvar[0]._pval
#define _ion_eca	*_ppvar[2]._pval
#define _ion_ica	*_ppvar[3]._pval
 
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
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_GAP1();
 static double _hoc_NE2();
 static double _hoc_NEn();
 static double _hoc_eta();
 static double _hoc_omega();
 static double _hoc_unirand();
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
 _p = _prop->param; _ppvar = _prop->dparam;
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
 "GAP1", _hoc_GAP1,
 "NE2", _hoc_NE2,
 "NEn", _hoc_NEn,
 "eta", _hoc_eta,
 "omega", _hoc_omega,
 "unirand", _hoc_unirand,
 0, 0
};
#define GAP1 GAP1_interV2pyrDNE_STFD
#define NE2 NE2_interV2pyrDNE_STFD
#define NEn NEn_interV2pyrDNE_STFD
#define eta eta_interV2pyrDNE_STFD
#define omega omega_interV2pyrDNE_STFD
#define unirand unirand_interV2pyrDNE_STFD
 extern double GAP1( double , double );
 extern double NE2( double , double );
 extern double NEn( double , double );
 extern double eta( double );
 extern double omega( double , double , double );
 extern double unirand( );
 /* declare global and static user variables */
#define Beta2 Beta2_interV2pyrDNE_STFD
 double Beta2 = 0.0001;
#define Beta1 Beta1_interV2pyrDNE_STFD
 double Beta1 = 0.001;
#define GAPstop1 GAPstop1_interV2pyrDNE_STFD
 double GAPstop1 = 196000;
#define GAPstart1 GAPstart1_interV2pyrDNE_STFD
 double GAPstart1 = 96000;
#define NE_S NE_S_interV2pyrDNE_STFD
 double NE_S = 1;
#define NE_t2 NE_t2_interV2pyrDNE_STFD
 double NE_t2 = 1;
#define NE_t1 NE_t1_interV2pyrDNE_STFD
 double NE_t1 = 1;
#define NEstop2 NEstop2_interV2pyrDNE_STFD
 double NEstop2 = 36000;
#define NEstart2 NEstart2_interV2pyrDNE_STFD
 double NEstart2 = 35900;
#define NEstop1 NEstop1_interV2pyrDNE_STFD
 double NEstop1 = 40000;
#define NEstart1 NEstart1_interV2pyrDNE_STFD
 double NEstart1 = 39500;
#define k k_interV2pyrDNE_STFD
 double k = 0.01;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "d2", 0, 1,
 "d1", 0, 1,
 "f", 0, 1e+09,
 "tauD2", 1e-09, 1e+09,
 "tauD1", 1e-09, 1e+09,
 "tauF", 1e-09, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "Beta1_interV2pyrDNE_STFD", "/ms",
 "Beta2_interV2pyrDNE_STFD", "/ms",
 "srcid", "1",
 "destid", "1",
 "Cdur_gaba", "ms",
 "AlphaTmax_gaba", "/ms",
 "Beta_gaba", "/ms",
 "Erev_gaba", "mV",
 "gbar_gaba", "uS",
 "Cainf", "mM",
 "pooldiam", "micrometer",
 "tauCa", "ms",
 "f", "1",
 "tauF", "ms",
 "d1", "1",
 "tauD1", "ms",
 "d2", "1",
 "tauD2", "ms",
 "igaba", "nA",
 "g_gaba", "uS",
 "ICag", "mA",
 "Icatotal", "mA",
 0,0
};
 static double capoolcon0 = 0;
 static double delta_t = 0.01;
 static double r_gaba0 = 0;
 static double r_nmda0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "k_interV2pyrDNE_STFD", &k_interV2pyrDNE_STFD,
 "GAPstart1_interV2pyrDNE_STFD", &GAPstart1_interV2pyrDNE_STFD,
 "GAPstop1_interV2pyrDNE_STFD", &GAPstop1_interV2pyrDNE_STFD,
 "NEstart1_interV2pyrDNE_STFD", &NEstart1_interV2pyrDNE_STFD,
 "NEstop1_interV2pyrDNE_STFD", &NEstop1_interV2pyrDNE_STFD,
 "NEstart2_interV2pyrDNE_STFD", &NEstart2_interV2pyrDNE_STFD,
 "NEstop2_interV2pyrDNE_STFD", &NEstop2_interV2pyrDNE_STFD,
 "NE_t1_interV2pyrDNE_STFD", &NE_t1_interV2pyrDNE_STFD,
 "NE_t2_interV2pyrDNE_STFD", &NE_t2_interV2pyrDNE_STFD,
 "NE_S_interV2pyrDNE_STFD", &NE_S_interV2pyrDNE_STFD,
 "Beta1_interV2pyrDNE_STFD", &Beta1_interV2pyrDNE_STFD,
 "Beta2_interV2pyrDNE_STFD", &Beta2_interV2pyrDNE_STFD,
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
 
#define _cvode_ieq _ppvar[4]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"interV2pyrDNE_STFD",
 "srcid",
 "destid",
 "Cdur_gaba",
 "AlphaTmax_gaba",
 "Beta_gaba",
 "Erev_gaba",
 "gbar_gaba",
 "Cainf",
 "pooldiam",
 "z",
 "tauCa",
 "P0g",
 "fCag",
 "lambda1",
 "lambda2",
 "threshold1",
 "threshold2",
 "initW",
 "fmax",
 "fmin",
 "thr_rp",
 "facfactor",
 "f",
 "tauF",
 "d1",
 "tauD1",
 "d2",
 "tauD2",
 0,
 "igaba",
 "g_gaba",
 "on_gaba",
 "W",
 "ICag",
 "Icatotal",
 "Wmax",
 "Wmin",
 "maxChange",
 "normW",
 "scaleW",
 "pregid",
 "postgid",
 "F",
 "D1",
 "D2",
 0,
 "r_nmda",
 "r_gaba",
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
 	_p = nrn_prop_data_alloc(_mechtype, 61, _prop);
 	/*initialize range parameters*/
 	srcid = -1;
 	destid = -1;
 	Cdur_gaba = 0.7254;
 	AlphaTmax_gaba = 7.2609;
 	Beta_gaba = 0.2667;
 	Erev_gaba = -75;
 	gbar_gaba = 0.0006;
 	Cainf = 5e-05;
 	pooldiam = 1.8172;
 	z = 2;
 	tauCa = 50;
 	P0g = 0.01;
 	fCag = 0.024;
 	lambda1 = 4;
 	lambda2 = 0.01;
 	threshold1 = 0.47;
 	threshold2 = 0.52;
 	initW = 4.5;
 	fmax = 4.2;
 	fmin = 0.8;
 	thr_rp = 1;
 	facfactor = 1;
 	f = 0;
 	tauF = 20;
 	d1 = 0.95;
 	tauD1 = 40;
 	d2 = 0.9;
 	tauD2 = 70;
  }
 	_prop->param = _p;
 	_prop->param_size = 61;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[2]._pval = &prop_ion->param[0]; /* eca */
 	_ppvar[3]._pval = &prop_ion->param[3]; /* ica */
 
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

 void _interV2pyrDNE_STFD_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("ca", -10000.);
 	_ca_sym = hoc_lookup("ca_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 61, 5);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 interV2pyrDNE_STFD /home/nest/lascon_project/x86_64/interV2pyrDNE_STFD.mod\n");
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
 static int _slist1[2], _dlist1[2];
 static int release(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   if ( t0 > 0.0 ) {
     if ( rp < thr_rp ) {
       if ( t - t0 < Cdur_gaba ) {
         on_gaba = 1.0 ;
         }
       else {
         on_gaba = 0.0 ;
         }
       }
     else {
       on_gaba = 0.0 ;
       }
     }
   if ( t0 > 0.0 ) {
     if ( rp < thr_rp ) {
       if ( t - t0 < Cdur_gaba ) {
         on_gaba = 1.0 ;
         }
       else {
         on_gaba = 0.0 ;
         }
       }
     else {
       on_gaba = 0.0 ;
       }
     }
   Dr_gaba = AlphaTmax_gaba * on_gaba * ( 1.0 - r_gaba ) - Beta_gaba * r_gaba ;
   dW_gaba = eta ( _threadargscomma_ capoolcon ) * ( lambda1 * omega ( _threadargscomma_ capoolcon , threshold1 , threshold2 ) - lambda2 * GAP1 ( _threadargscomma_ GAPstart1 , GAPstop1 ) * W ) * dt ;
   if ( fabs ( dW_gaba ) > maxChange ) {
     if ( dW_gaba < 0.0 ) {
       dW_gaba = - 1.0 * maxChange ;
       }
     else {
       dW_gaba = maxChange ;
       }
     }
   normW = ( W - Wmin ) / ( Wmax - Wmin ) ;
   if ( dW_gaba < 0.0 ) {
     scaleW = sqrt ( fabs ( normW ) ) ;
     }
   else {
     scaleW = sqrt ( fabs ( 1.0 - normW ) ) ;
     }
   W = W + dW_gaba * scaleW ;
   if ( W > Wmax ) {
     W = Wmax ;
     }
   else if ( W < Wmin ) {
     W = Wmin ;
     }
   g_gaba = gbar_gaba * r_gaba * facfactor * NEn ( _threadargscomma_ NEstart1 , NEstop1 ) * NE2 ( _threadargscomma_ NEstart2 , NEstop2 ) ;
   igaba = W * g_gaba * ( v - Erev_gaba ) ;
   ICag = P0g * g_gaba * ( v - eca ) ;
   Icatotal = ICag + k * ica * 4.0 * pi * ( pow( ( 15.0 / 2.0 ) , 2.0 ) ) * ( 0.01 ) ;
   Dcapoolcon = - fCag * Afactor * Icatotal + ( Cainf - capoolcon ) / tauCa ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 if ( t0 > 0.0 ) {
   if ( rp < thr_rp ) {
     if ( t - t0 < Cdur_gaba ) {
       on_gaba = 1.0 ;
       }
     else {
       on_gaba = 0.0 ;
       }
     }
   else {
     on_gaba = 0.0 ;
     }
   }
 if ( t0 > 0.0 ) {
   if ( rp < thr_rp ) {
     if ( t - t0 < Cdur_gaba ) {
       on_gaba = 1.0 ;
       }
     else {
       on_gaba = 0.0 ;
       }
     }
   else {
     on_gaba = 0.0 ;
     }
   }
 Dr_gaba = Dr_gaba  / (1. - dt*( ( AlphaTmax_gaba * on_gaba )*( ( ( - 1.0 ) ) ) - ( Beta_gaba )*( 1.0 ) )) ;
 dW_gaba = eta ( _threadargscomma_ capoolcon ) * ( lambda1 * omega ( _threadargscomma_ capoolcon , threshold1 , threshold2 ) - lambda2 * GAP1 ( _threadargscomma_ GAPstart1 , GAPstop1 ) * W ) * dt ;
 if ( fabs ( dW_gaba ) > maxChange ) {
   if ( dW_gaba < 0.0 ) {
     dW_gaba = - 1.0 * maxChange ;
     }
   else {
     dW_gaba = maxChange ;
     }
   }
 normW = ( W - Wmin ) / ( Wmax - Wmin ) ;
 if ( dW_gaba < 0.0 ) {
   scaleW = sqrt ( fabs ( normW ) ) ;
   }
 else {
   scaleW = sqrt ( fabs ( 1.0 - normW ) ) ;
   }
 W = W + dW_gaba * scaleW ;
 if ( W > Wmax ) {
   W = Wmax ;
   }
 else if ( W < Wmin ) {
   W = Wmin ;
   }
 g_gaba = gbar_gaba * r_gaba * facfactor * NEn ( _threadargscomma_ NEstart1 , NEstop1 ) * NE2 ( _threadargscomma_ NEstart2 , NEstop2 ) ;
 igaba = W * g_gaba * ( v - Erev_gaba ) ;
 ICag = P0g * g_gaba * ( v - eca ) ;
 Icatotal = ICag + k * ica * 4.0 * pi * ( pow( ( 15.0 / 2.0 ) , 2.0 ) ) * ( 0.01 ) ;
 Dcapoolcon = Dcapoolcon  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tauCa )) ;
  return 0;
}
 /*END CVODE*/
 static int release () {_reset=0;
 {
   if ( t0 > 0.0 ) {
     if ( rp < thr_rp ) {
       if ( t - t0 < Cdur_gaba ) {
         on_gaba = 1.0 ;
         }
       else {
         on_gaba = 0.0 ;
         }
       }
     else {
       on_gaba = 0.0 ;
       }
     }
   if ( t0 > 0.0 ) {
     if ( rp < thr_rp ) {
       if ( t - t0 < Cdur_gaba ) {
         on_gaba = 1.0 ;
         }
       else {
         on_gaba = 0.0 ;
         }
       }
     else {
       on_gaba = 0.0 ;
       }
     }
    r_gaba = r_gaba + (1. - exp(dt*(( AlphaTmax_gaba * on_gaba )*( ( ( - 1.0 ) ) ) - ( Beta_gaba )*( 1.0 ))))*(- ( ( ( AlphaTmax_gaba )*( on_gaba ) )*( ( 1.0 ) ) ) / ( ( ( AlphaTmax_gaba )*( on_gaba ) )*( ( ( - 1.0 ) ) ) - ( Beta_gaba )*( 1.0 ) ) - r_gaba) ;
   dW_gaba = eta ( _threadargscomma_ capoolcon ) * ( lambda1 * omega ( _threadargscomma_ capoolcon , threshold1 , threshold2 ) - lambda2 * GAP1 ( _threadargscomma_ GAPstart1 , GAPstop1 ) * W ) * dt ;
   if ( fabs ( dW_gaba ) > maxChange ) {
     if ( dW_gaba < 0.0 ) {
       dW_gaba = - 1.0 * maxChange ;
       }
     else {
       dW_gaba = maxChange ;
       }
     }
   normW = ( W - Wmin ) / ( Wmax - Wmin ) ;
   if ( dW_gaba < 0.0 ) {
     scaleW = sqrt ( fabs ( normW ) ) ;
     }
   else {
     scaleW = sqrt ( fabs ( 1.0 - normW ) ) ;
     }
   W = W + dW_gaba * scaleW ;
   if ( W > Wmax ) {
     W = Wmax ;
     }
   else if ( W < Wmin ) {
     W = Wmin ;
     }
   g_gaba = gbar_gaba * r_gaba * facfactor * NEn ( _threadargscomma_ NEstart1 , NEstop1 ) * NE2 ( _threadargscomma_ NEstart2 , NEstop2 ) ;
   igaba = W * g_gaba * ( v - Erev_gaba ) ;
   ICag = P0g * g_gaba * ( v - eca ) ;
   Icatotal = ICag + k * ica * 4.0 * pi * ( pow( ( 15.0 / 2.0 ) , 2.0 ) ) * ( 0.01 ) ;
    capoolcon = capoolcon + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tauCa)))*(- ( ( ( - fCag )*( Afactor ) )*( Icatotal ) + ( ( Cainf ) ) / tauCa ) / ( ( ( ( - 1.0 ) ) ) / tauCa ) - capoolcon) ;
   }
  return 0;
}
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{    _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
   t0 = t ;
   rp = unirand ( _threadargs_ ) ;
   D1 = 1.0 - ( 1.0 - D1 ) * exp ( - ( t - tsyn ) / tauD1 ) ;
   D2 = 1.0 - ( 1.0 - D2 ) * exp ( - ( t - tsyn ) / tauD2 ) ;
   tsyn = t ;
   facfactor = F * D1 * D2 ;
   if ( F > 3.0 ) {
     F = 3.0 ;
     }
   if ( facfactor < 0.7 ) {
     facfactor = 0.7 ;
     }
   D1 = D1 * d1 ;
   D2 = D2 * d2 ;
   } }
 
double eta (  double _lCani ) {
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
    _hoc_setdata(_vptr);
 _r =  eta (  *getarg(1) );
 return(_r);
}
 
double omega (  double _lCani , double _lthreshold1 , double _lthreshold2 ) {
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
    _hoc_setdata(_vptr);
 _r =  omega (  *getarg(1) , *getarg(2) , *getarg(3) );
 return(_r);
}
 
double NEn (  double _lNEstart1 , double _lNEstop1 ) {
   double _lNEn;
 double _lNEtemp1 , _lNEtemp2 , _lNEtemp3 , _lNEtemp4 , _lNEtemp5 , _lNEtemp6 , _lNEtemp7 , _lNEtemp8 , _lNEtemp9 , _lNEtemp10 , _lNEtemp11 , _lNEtemp12 , _lNEtemp13 , _lNEtemp14 , _lNEtemp15 , _lNEtemp16 , _lNEtemp17 , _lNEtemp18 , _ls ;
 _lNEtemp1 = _lNEstart1 + 4000.0 ;
   _lNEtemp2 = _lNEtemp1 + 4000.0 ;
   _lNEtemp3 = _lNEtemp2 + 4000.0 ;
   _lNEtemp4 = _lNEtemp3 + 4000.0 ;
   _lNEtemp5 = _lNEtemp4 + 4000.0 ;
   _lNEtemp6 = _lNEtemp5 + 4000.0 ;
   _lNEtemp7 = _lNEtemp6 + 4000.0 ;
   _lNEtemp8 = _lNEtemp7 + 4000.0 ;
   _lNEtemp9 = _lNEtemp8 + 4000.0 ;
   _lNEtemp10 = _lNEtemp9 + 4000.0 ;
   _lNEtemp11 = _lNEtemp10 + 4000.0 ;
   _lNEtemp12 = _lNEtemp11 + 4000.0 ;
   _lNEtemp13 = _lNEtemp12 + 4000.0 ;
   _lNEtemp14 = _lNEtemp13 + 4000.0 ;
   _lNEtemp15 = _lNEtemp14 + 4000.0 + 100000.0 ;
   _lNEtemp16 = _lNEtemp15 + 4000.0 ;
   _lNEtemp17 = _lNEtemp16 + 4000.0 ;
   _lNEtemp18 = _lNEtemp17 + 4000.0 ;
   if ( t <= _lNEstart1 ) {
     _lNEn = 1.0 ;
     }
   else if ( t >= _lNEstart1  && t <= _lNEstop1 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEstop1  && t < _lNEtemp1 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - _lNEstop1 ) ) ;
     }
   else if ( t >= _lNEtemp1  && t <= _lNEtemp1 + 500.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp1 + 500.0  && t < _lNEtemp2 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - _lNEstop1 ) ) ;
     }
   else if ( t >= _lNEtemp2  && t <= _lNEtemp2 + 500.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp2 + 500.0  && t < _lNEtemp3 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lNEtemp2 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp3  && t <= _lNEtemp3 + 500.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp3 + 500.0  && t < _lNEtemp4 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lNEtemp3 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp4  && t <= _lNEtemp4 + 500.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp4 + 500.0  && t < _lNEtemp5 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lNEtemp4 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp5  && t <= _lNEtemp5 + 500.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp5 + 500.0  && t < _lNEtemp6 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lNEtemp5 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp6  && t <= _lNEtemp6 + 500.0 ) {
     _lNEn = NE_t1 ;
     }
   else if ( t > _lNEtemp6 + 500.0  && t < _lNEtemp7 ) {
     _lNEn = 1.0 + ( NE_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lNEtemp6 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp7  && t <= _lNEtemp7 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp7 + 500.0  && t < _lNEtemp8 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp7 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp8  && t <= _lNEtemp8 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp8 + 500.0  && t < _lNEtemp9 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp8 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp9  && t <= _lNEtemp9 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp9 + 500.0  && t < _lNEtemp10 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp9 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp10  && t <= _lNEtemp10 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp10 + 500.0  && t < _lNEtemp11 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp10 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp11  && t <= _lNEtemp11 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp11 + 500.0  && t < _lNEtemp12 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp11 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp12  && t <= _lNEtemp12 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp12 + 500.0  && t < _lNEtemp13 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp12 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp13  && t <= _lNEtemp13 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp13 + 500.0  && t < _lNEtemp14 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp13 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp14  && t <= _lNEtemp14 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp14 + 500.0  && t < _lNEtemp15 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp14 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp15  && t <= _lNEtemp15 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp15 + 500.0  && t < _lNEtemp16 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp15 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp16  && t <= _lNEtemp16 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp16 + 500.0  && t < _lNEtemp17 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp16 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp17  && t <= _lNEtemp17 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else if ( t > _lNEtemp17 + 500.0  && t < _lNEtemp18 ) {
     _lNEn = 1.0 + ( NE_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEtemp17 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNEtemp18  && t <= _lNEtemp18 + 500.0 ) {
     _lNEn = NE_t2 ;
     }
   else {
     _lNEn = 1.0 ;
     }
   
return _lNEn;
 }
 
static double _hoc_NEn(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  NEn (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double NE2 (  double _lNEstart2 , double _lNEstop2 ) {
   double _lNE2;
 double _lNE2temp1 , _lNE2temp2 , _lNE2temp3 , _lNE2temp4 , _lNE2temp5 , _lNE2temp6 , _lNE2temp7 , _lNE2temp8 , _lNE2temp9 , _lNE2temp10 , _lNE2temp11 , _lNE2temp12 , _lNE2temp13 , _lNE2temp14 , _lNE2temp15 , _lNE2temp16 , _ls ;
 _lNE2temp1 = _lNEstart2 + 4000.0 ;
   _lNE2temp2 = _lNE2temp1 + 4000.0 ;
   _lNE2temp3 = _lNE2temp2 + 4000.0 ;
   _lNE2temp4 = _lNE2temp3 + 4000.0 ;
   _lNE2temp5 = _lNE2temp4 + 4000.0 ;
   _lNE2temp6 = _lNE2temp5 + 4000.0 ;
   _lNE2temp7 = _lNE2temp6 + 4000.0 ;
   _lNE2temp8 = _lNE2temp7 + 4000.0 ;
   _lNE2temp9 = _lNE2temp8 + 4000.0 ;
   _lNE2temp10 = _lNE2temp9 + 4000.0 ;
   _lNE2temp11 = _lNE2temp10 + 4000.0 ;
   _lNE2temp12 = _lNE2temp11 + 4000.0 ;
   _lNE2temp13 = _lNE2temp12 + 4000.0 ;
   _lNE2temp14 = _lNE2temp13 + 4000.0 ;
   _lNE2temp15 = _lNE2temp14 + 4000.0 ;
   if ( t <= _lNEstart2 ) {
     _lNE2 = 1.0 ;
     }
   else if ( t >= _lNEstart2  && t <= _lNEstop2 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNEstop2  && t < _lNE2temp1 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNEstop2 + 500.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp1  && t <= _lNE2temp1 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp1 + 100.0  && t < _lNE2temp2 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp1 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp2  && t <= _lNE2temp2 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp2 + 100.0  && t < _lNE2temp3 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp2 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp3  && t <= _lNE2temp3 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp3 + 100.0  && t < _lNE2temp4 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp3 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp4  && t <= _lNE2temp4 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp4 + 100.0  && t < _lNE2temp5 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp4 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp5  && t <= _lNE2temp5 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp5 + 100.0  && t < _lNE2temp6 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp5 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp6  && t <= _lNE2temp6 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp6 + 100.0  && t < _lNE2temp7 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp6 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp7  && t <= _lNE2temp7 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp7 + 100.0  && t < _lNE2temp8 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp7 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp8  && t <= _lNE2temp8 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp8 + 100.0  && t < _lNE2temp9 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp8 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp9  && t <= _lNE2temp9 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp9 + 100.0  && t < _lNE2temp10 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp9 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp10  && t <= _lNE2temp10 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp10 + 100.0  && t < _lNE2temp11 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp10 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp11  && t <= _lNE2temp11 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp11 + 100.0  && t < _lNE2temp12 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp11 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp12  && t <= _lNE2temp12 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp12 + 100.0  && t < _lNE2temp13 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp12 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp13  && t <= _lNE2temp13 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp13 + 100.0  && t < _lNE2temp14 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp13 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp14  && t <= _lNE2temp14 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else if ( t > _lNE2temp14 + 100.0  && t < _lNE2temp15 ) {
     _lNE2 = 1.0 + ( NE_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lNE2temp14 + 100.0 ) ) ) ;
     }
   else if ( t >= _lNE2temp15  && t <= _lNE2temp15 + 100.0 ) {
     _lNE2 = NE_S ;
     }
   else {
     _lNE2 = 1.0 ;
     }
   
return _lNE2;
 }
 
static double _hoc_NE2(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  NE2 (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double GAP1 (  double _lGAPstart1 , double _lGAPstop1 ) {
   double _lGAP1;
 double _ls ;
 if ( t <= _lGAPstart1 ) {
     _lGAP1 = 1.0 ;
     }
   else if ( t >= _lGAPstart1  && t <= _lGAPstop1 ) {
     _lGAP1 = 1.0 ;
     }
   else {
     _lGAP1 = 1.0 ;
     }
   
return _lGAP1;
 }
 
static double _hoc_GAP1(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  GAP1 (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double unirand (  ) {
   double _lunirand;
 _lunirand = scop_random ( ) ;
   
return _lunirand;
 }
 
static double _hoc_unirand(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  unirand (  );
 return(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  eca = _ion_eca;
  ica = _ion_ica;
     _ode_spec1 ();
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 2; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  eca = _ion_eca;
  ica = _ion_ica;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 2, 0);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 3, 3);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  capoolcon = capoolcon0;
  r_gaba = r_gaba0;
  r_nmda = r_nmda0;
 {
   on_gaba = 0.0 ;
   r_gaba = 0.0 ;
   W = initW ;
   t0 = - 1.0 ;
   Wmax = fmax * initW ;
   Wmin = fmin * initW ;
   maxChange = ( Wmax - Wmin ) / 10.0 ;
   dW_gaba = 0.0 ;
   capoolcon = Cainf ;
   Afactor = 1.0 / ( z * FARADAY * 4.0 / 3.0 * pi * pow( ( pooldiam / 2.0 ) , 3.0 ) ) * ( 1e6 ) ;
   fa = 0.0 ;
   F = 1.0 ;
   D1 = 1.0 ;
   D2 = 1.0 ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  ica = _ion_ica;
 initmodel();
}}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   }
 _current += igaba;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  ica = _ion_ica;
 _g = _nrn_current(_v + .001);
 	{ _rhs = _nrn_current(_v);
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
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  ica = _ion_ica;
 { error =  release();
 if(error){fprintf(stderr,"at line 146 in file interV2pyrDNE_STFD.mod:\n	SOLVE release METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 }}}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(r_gaba) - _p;  _dlist1[0] = &(Dr_gaba) - _p;
 _slist1[1] = &(capoolcon) - _p;  _dlist1[1] = &(Dcapoolcon) - _p;
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "/home/nest/lascon_project/KimEtAl2013/interV2pyrDNE_STFD.mod";
static const char* nmodl_file_text = 
  ":Interneuron Cells to Pyramidal Cells GABA with local Ca2+ pool and read public soma Ca2+ pool\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS interV2pyrDNE_STFD\n"
  "	USEION ca READ eca,ica\n"
  "	NONSPECIFIC_CURRENT igaba\n"
  "	RANGE initW\n"
  "	RANGE Cdur_gaba, AlphaTmax_gaba, Beta_gaba, Erev_gaba, gbar_gaba, W, on_gaba, g_gaba\n"
  "	RANGE eca, tauCa, Icatotal\n"
  "	RANGE ICag, P0g, fCag\n"
  "	RANGE Cainf, pooldiam, z\n"
  "	RANGE lambda1, lambda2, threshold1, threshold2\n"
  "	RANGE fmax, fmin, Wmax, Wmin, maxChange, normW, scaleW, srcid, destid\n"
  "	RANGE pregid,postgid, thr_rp\n"
  "	RANGE F, f, tauF, D1, d1, tauD1, D2, d2, tauD2\n"
  "	RANGE facfactor\n"
  "}\n"
  " \n"
  "UNITS {\n"
  "	(mV) = (millivolt)\n"
  "        (nA) = (nanoamp)\n"
  "	(uS) = (microsiemens)\n"
  "	FARADAY = 96485 (coul)\n"
  "	pi = 3.141592 (1)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "\n"
  "	srcid = -1 (1)\n"
  "	destid = -1 (1)\n"
  "	\n"
  "	Cdur_gaba = 0.7254 (ms)\n"
  "	AlphaTmax_gaba = 7.2609 (/ms)\n"
  "	Beta_gaba = 0.2667 (/ms)\n"
  "	Erev_gaba = -75 (mV)\n"
  "	gbar_gaba = 0.6e-3 (uS)\n"
  "\n"
  "	Cainf = 50e-6 (mM)\n"
  "	pooldiam =  1.8172 (micrometer)\n"
  "	z = 2\n"
  "\n"
  "	k = 0.01	\n"
  "	\n"
  "	tauCa = 50 (ms)\n"
  "	\n"
  "	P0g = .01\n"
  "	fCag = .024\n"
  "	\n"
  "	lambda1 = 4 : 3 : 2.0 : 2.0\n"
  "	lambda2 = .01\n"
  "	threshold1 = 0.47 :  0.48 : 0.45 : 0.4 : 0.95 : 1.35 :0.75 :0.55 (uM)\n"
  "	threshold2 = 0.52 :  0.53 : 0.5 : 0.45 : 1.0 : 1.4 : 0.8 : 0.65 :0.70 (uM)\n"
  "\n"
  "	:GABA Weight\n"
  "	initW = 4.5 :  :  3 :  2.5 : 3 : 5 : 6.25 : 5\n"
  "	fmax = 4.2\n"
  "	fmin = .8\n"
  "	\n"
  "	GAPstart1 = 96000\n"
  "	GAPstop1 = 196000\n"
  "	\n"
  "	thr_rp = 1 : .7\n"
  "	\n"
  "	facfactor = 1\n"
  "	: the (1) is needed for the range limits to be effective\n"
  "        f = 0 (1) < 0, 1e9 > : 1.3 (1) < 0, 1e9 >    : facilitation\n"
  "        tauF = 20 (ms) < 1e-9, 1e9 >\n"
  "        d1 = 0.95 (1) < 0, 1 >     : fast depression\n"
  "        tauD1 = 40 (ms) < 1e-9, 1e9 >\n"
  "        d2 = 0.9 (1) < 0, 1 >     : slow depression\n"
  "        tauD2 = 70 (ms) < 1e-9, 1e9 >	\n"
  "	\n"
  "	NEstart1 = 39500\n"
  "	NEstop1 = 40000	\n"
  "	NEstart2 = 35900\n"
  "	NEstop2 = 36000	\n"
  "\n"
  "	NE_t1 = 1 : 0.95\n"
  "	NE_t2 = 1 :0.7 : 0.8\n"
  "	NE_S = 1 : 0.4	\n"
  "	Beta1 = 0.001  (/ms) : 1/decay time for neuromodulators\n"
  "	Beta2 = 0.0001  (/ms)			\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v (mV)\n"
  "	eca (mV)\n"
  "	ica (nA)\n"
  "	\n"
  "	igaba (nA)\n"
  "	g_gaba (uS)\n"
  "	on_gaba\n"
  "	W\n"
  "\n"
  "	t0 (ms)\n"
  "\n"
  "	ICan (mA)\n"
  "	ICag (mA)\n"
  "	Afactor	(mM/ms/nA)\n"
  "	Icatotal (mA)\n"
  "\n"
  "	dW_gaba\n"
  "	Wmax\n"
  "	Wmin\n"
  "	maxChange\n"
  "	normW\n"
  "	scaleW\n"
  "	\n"
  "	pregid\n"
  "	postgid\n"
  "\n"
  "	rp\n"
  "	tsyn\n"
  "	\n"
  "	fa\n"
  "	F\n"
  "	D1\n"
  "	D2	\n"
  "}\n"
  "\n"
  "STATE { r_nmda r_gaba capoolcon }\n"
  "\n"
  "INITIAL {\n"
  "\n"
  "	on_gaba = 0\n"
  "	r_gaba = 0\n"
  "	W = initW\n"
  "\n"
  "	t0 = -1\n"
  "\n"
  "	Wmax = fmax*initW\n"
  "	Wmin = fmin*initW\n"
  "	maxChange = (Wmax-Wmin)/10\n"
  "	dW_gaba = 0\n"
  "\n"
  "	capoolcon = Cainf\n"
  "	Afactor	= 1/(z*FARADAY*4/3*pi*(pooldiam/2)^3)*(1e6)\n"
  "\n"
  "	fa =0\n"
  "	F = 1\n"
  "	D1 = 1\n"
  "	D2 = 1	\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE release METHOD cnexp\n"
  "}\n"
  "\n"
  "DERIVATIVE release {\n"
  "	if (t0>0) {\n"
  "		if (rp < thr_rp) {\n"
  "			if (t-t0 < Cdur_gaba) {\n"
  "				on_gaba = 1\n"
  "			} else {\n"
  "				on_gaba = 0\n"
  "			}\n"
  "		} else {\n"
  "			on_gaba = 0\n"
  "		}\n"
  "	}\n"
  "	if (t0>0) {\n"
  "		if (rp < thr_rp) {\n"
  "			if (t-t0 < Cdur_gaba) {\n"
  "				on_gaba = 1\n"
  "			} else {\n"
  "				on_gaba = 0\n"
  "			}\n"
  "		} else {\n"
  "			on_gaba = 0\n"
  "		}\n"
  "	}\n"
  "\n"
  "	r_gaba' = AlphaTmax_gaba*on_gaba*(1-r_gaba)-Beta_gaba*r_gaba\n"
  "\n"
  "	dW_gaba = eta(capoolcon)*(lambda1*omega(capoolcon, threshold1, threshold2)-lambda2*GAP1(GAPstart1, GAPstop1)*W)*dt\n"
  "\n"
  "	: Limit for extreme large weight changes\n"
  "	if (fabs(dW_gaba) > maxChange) {\n"
  "		if (dW_gaba < 0) {\n"
  "			dW_gaba = -1*maxChange\n"
  "		} else {\n"
  "			dW_gaba = maxChange\n"
  "		}\n"
  "	}\n"
  "\n"
  "	:Normalize the weight change\n"
  "	normW = (W-Wmin)/(Wmax-Wmin)\n"
  "	if (dW_gaba < 0) {\n"
  "		scaleW = sqrt(fabs(normW))\n"
  "	} else {\n"
  "		scaleW = sqrt(fabs(1.0-normW))\n"
  "	}\n"
  "\n"
  "	W = W + dW_gaba*scaleW\n"
  "	\n"
  "	:Weight value limits\n"
  "	if (W > Wmax) { \n"
  "		W = Wmax\n"
  "	} else if (W < Wmin) {\n"
  " 		W = Wmin\n"
  "	}\n"
  "\n"
  "\n"
  "	g_gaba = gbar_gaba*r_gaba*facfactor*NEn(NEstart1,NEstop1)*NE2(NEstart2,NEstop2)   : Norepinephrine effect on GABA		    \n"
  "	igaba = W*g_gaba*(v - Erev_gaba)\n"
  "\n"
  "	ICag = P0g*g_gaba*(v - eca)	\n"
  "	Icatotal = ICag + k*ica*4*pi*((15/2)^2)*(0.01)    :  icag+k*ica*Area of soma*unit change\n"
  "	capoolcon'= -fCag*Afactor*Icatotal + (Cainf-capoolcon)/tauCa\n"
  "}\n"
  "\n"
  "NET_RECEIVE(dummy_weight) {\n"
  "	t0 = t\n"
  "	rp = unirand()	\n"
  "	\n"
  "	:F  = 1 + (F-1)* exp(-(t - tsyn)/tauF)\n"
  "	D1 = 1 - (1-D1)*exp(-(t - tsyn)/tauD1)\n"
  "	D2 = 1 - (1-D2)*exp(-(t - tsyn)/tauD2)\n"
  " :printf(\"%g\\t%g\\t%g\\t%g\\t%g\\t%g\\n\", t, t-tsyn, F, D1, D2, facfactor)\n"
  "	:printf(\"%g\\t%g\\t%g\\t%g\\n\", F, D1, D2, facfactor)\n"
  "	tsyn = t\n"
  "	\n"
  "	facfactor = F * D1 * D2\n"
  "\n"
  "	::F = F+f  :F * f\n"
  "	\n"
  "	if (F > 3) { \n"
  "	F=3	}	\n"
  "	if (facfactor < 0.7) { \n"
  "	facfactor=0.7\n"
  "	}\n"
  "	D1 = D1 * d1\n"
  "	D2 = D2 * d2\n"
  ":printf(\"\\t%g\\t%g\\t%g\\n\", F, D1, D2)\n"
  "}\n"
  "\n"
  ":::::::::::: FUNCTIONs and PROCEDUREs ::::::::::::\n"
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
  "FUNCTION NEn(NEstart1 (ms), NEstop1 (ms)) {\n"
  "	LOCAL NEtemp1, NEtemp2, NEtemp3, NEtemp4, NEtemp5, NEtemp6, NEtemp7, NEtemp8, NEtemp9, NEtemp10, NEtemp11, NEtemp12, NEtemp13, NEtemp14, NEtemp15, NEtemp16, NEtemp17, NEtemp18,s\n"
  "	NEtemp1 = NEstart1+4000\n"
  "	NEtemp2 = NEtemp1+4000\n"
  "	NEtemp3 = NEtemp2+4000\n"
  "	NEtemp4 = NEtemp3+4000\n"
  "	NEtemp5 = NEtemp4+4000\n"
  "	NEtemp6 = NEtemp5+4000\n"
  "	NEtemp7 = NEtemp6+4000\n"
  "	NEtemp8 = NEtemp7+4000\n"
  "	NEtemp9 = NEtemp8+4000\n"
  "	NEtemp10 = NEtemp9+4000\n"
  "	NEtemp11 = NEtemp10+4000\n"
  "	NEtemp12 = NEtemp11+4000\n"
  "	NEtemp13 = NEtemp12+4000\n"
  "	NEtemp14 = NEtemp13+4000\n"
  "	NEtemp15 = NEtemp14 + 4000 + 100000     : 100sec Gap\n"
  "	NEtemp16 = NEtemp15 + 4000 \n"
  "	NEtemp17 = NEtemp16 + 4000\n"
  "	NEtemp18 = NEtemp17 + 4000\n"
  "\n"
  "	if (t <= NEstart1) { NEn = 1.0}\n"
  "	else if (t >= NEstart1 && t <= NEstop1) {NEn = NE_t1}					: 2nd tone in early conditioning (EC)\n"
  "		else if (t > NEstop1 && t < NEtemp1) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-NEstop1))}  		: Basal level\n"
  "	else if (t >= NEtemp1 && t <= NEtemp1+500) {NEn = NE_t1}					: 3rd tone EC\n"
  "		else if (t > NEtemp1+500 && t < NEtemp2) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-NEstop1))}  	: Basal level\n"
  "	else if (t >= NEtemp2 && t <= NEtemp2+500) {NEn = NE_t1}					: 4th tone EC\n"
  "		else if (t > NEtemp2+500 && t < NEtemp3) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-(NEtemp2+500)))}  	: Basal level	\n"
  "	else if (t >= NEtemp3 && t <= NEtemp3+500) {NEn = NE_t1}					: 5th tone EC\n"
  "		else if (t > NEtemp3+500 && t < NEtemp4) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-(NEtemp3+500)))}  	: Basal level\n"
  "	else if (t >= NEtemp4 && t <= NEtemp4+500) {NEn = NE_t1}					: 6th tone EC\n"
  "		else if (t > NEtemp4+500 && t < NEtemp5) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-(NEtemp4+500)))}  		: Basal level\n"
  "	else if (t >= NEtemp5 && t <= NEtemp5+500) {NEn = NE_t1}					: 7th tone EC\n"
  "		else if (t > NEtemp5+500 && t < NEtemp6) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-(NEtemp5+500)))}  	: Basal level\n"
  "	else if (t >= NEtemp6 && t <= NEtemp6+500) {NEn = NE_t1}					: 8th tone EC\n"
  "		else if (t > NEtemp6+500 && t < NEtemp7) {NEn = 1.0 + (NE_t1-1)*exp(-Beta1*(t-(NEtemp6+500)))}  	: Basal level\n"
  "	\n"
  "	else if (t >= NEtemp7 && t <= NEtemp7+500) {NEn = NE_t2}					: 9th tone	- Second Step late cond (LC)\n"
  "		else if (t > NEtemp7+500 && t < NEtemp8) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp7+500)))}  		: Basal level\n"
  "	else if (t >= NEtemp8 && t <= NEtemp8+500) {NEn = NE_t2}					: 10th tone  LC\n"
  "		else if (t > NEtemp8+500 && t < NEtemp9) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp8+500)))}		: Basal level	\n"
  "	else if (t >= NEtemp9 && t <= NEtemp9+500) {NEn = NE_t2}					: 11th tone  LC \n"
  "		else if (t > NEtemp9+500 && t < NEtemp10) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp9+500)))}  	: Basal level	\n"
  "	else if (t >= NEtemp10 && t <= NEtemp10+500) {NEn = NE_t2}					: 12th tone  LC\n"
  "		else if (t > NEtemp10+500 && t < NEtemp11) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp10+500)))}  	: Basal level\n"
  "	else if (t >= NEtemp11 && t <= NEtemp11+500) {NEn = NE_t2}					: 13th tone  LC\n"
  "		else if (t > NEtemp11+500 && t < NEtemp12) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp11+500)))}  	: Basal level\n"
  "	else if (t >= NEtemp12 && t <= NEtemp12+500) {NEn = NE_t2}					: 14th tone  LC\n"
  "		else if (t > NEtemp12+500 && t < NEtemp13) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp12+500)))}  	: Basal level\n"
  "	else if (t >= NEtemp13 && t <= NEtemp13+500) {NEn = NE_t2}					: 15th tone  LC\n"
  "		else if (t > NEtemp13+500 && t < NEtemp14) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp13+500)))}  	: Basal level\n"
  "	else if (t >= NEtemp14 && t <= NEtemp14+500) {NEn = NE_t2}					: 16th tone  LC\n"
  "		else if (t > NEtemp14+500 && t < NEtemp15) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp14+500)))}  	: Basal level\n"
  "	\n"
  "	else if (t >= NEtemp15 && t <= NEtemp15+500) {NEn = NE_t2}					: 1st tone in Early Extinction EE\n"
  "		else if (t > NEtemp15+500 && t < NEtemp16) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp15+500)))} 		: Basal level\n"
  "	else if (t >= NEtemp16 && t <= NEtemp16+500) {NEn = NE_t2}					: 2nd tone EE\n"
  "		else if (t > NEtemp16+500 && t < NEtemp17) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp16+500)))}  	: Basal level\n"
  "	else if (t >= NEtemp17 && t <= NEtemp17+500) {NEn = NE_t2}					: 3rd tone EE\n"
  "		else if (t > NEtemp17+500 && t < NEtemp18) {NEn = 1.0 + (NE_t2-1)*exp(-Beta2*(t-(NEtemp17+500)))}  	: Basal level	\n"
  "	else if (t >= NEtemp18 && t <= NEtemp18+500) {NEn = NE_t2}					: 4th tone EE	\n"
  "		else  {	NEn = 1.0}\n"
  "}\n"
  "FUNCTION NE2(NEstart2 (ms), NEstop2 (ms)) {\n"
  "	LOCAL NE2temp1, NE2temp2, NE2temp3, NE2temp4, NE2temp5, NE2temp6, NE2temp7, NE2temp8, NE2temp9, NE2temp10, NE2temp11, NE2temp12, NE2temp13, NE2temp14, NE2temp15, NE2temp16,s\n"
  "	NE2temp1 = NEstart2 + 4000\n"
  "	NE2temp2 = NE2temp1 + 4000\n"
  "	NE2temp3 = NE2temp2 + 4000\n"
  "	NE2temp4 = NE2temp3 + 4000\n"
  "	NE2temp5 = NE2temp4 + 4000\n"
  "	NE2temp6 = NE2temp5 + 4000\n"
  "	NE2temp7 = NE2temp6 + 4000\n"
  "	NE2temp8 = NE2temp7 + 4000\n"
  "	NE2temp9 = NE2temp8 + 4000\n"
  "	NE2temp10 = NE2temp9 + 4000\n"
  "	NE2temp11 = NE2temp10 + 4000\n"
  "	NE2temp12 = NE2temp11 + 4000 \n"
  "	NE2temp13 = NE2temp12 + 4000\n"
  "	NE2temp14 = NE2temp13 + 4000\n"
  "	NE2temp15 = NE2temp14 + 4000\n"
  "	\n"
  "	if (t <= NEstart2) { NE2 = 1.0}\n"
  "	else if (t >= NEstart2 && t <= NEstop2) {NE2 = NE_S }					: 1st shock\n"
  "		else if (t > NEstop2 && t < NE2temp1) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NEstop2+500)))} \n"
  "	else if (t >= NE2temp1 && t <= NE2temp1+100) {NE2=NE_S}					: 2nd shock\n"
  "		else if (t > NE2temp1+100 && t < NE2temp2) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp1+100)))}   				 \n"
  "	else if (t >= NE2temp2 && t <= NE2temp2+100) {NE2=NE_S}					: 3rd shock\n"
  "		else if (t > NE2temp2+100 && t < NE2temp3) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp2+100)))}  				 \n"
  "	else if (t >= NE2temp3 && t <= NE2temp3+100) {NE2=NE_S}					: 4th shock\n"
  "		else if (t > NE2temp3+100 && t < NE2temp4) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp3+100)))}  				 \n"
  "	else if (t >= NE2temp4 && t <= NE2temp4+100) {NE2=NE_S}					: 5th shock\n"
  "		else if (t > NE2temp4+100 && t < NE2temp5) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp4+100)))}  				 \n"
  "	else if (t >= NE2temp5 && t <= NE2temp5+100) {NE2=NE_S}					: 6th shock\n"
  "		else if (t > NE2temp5+100 && t < NE2temp6) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp5+100)))} 				 \n"
  "	else if (t >= NE2temp6 && t <= NE2temp6+100) {NE2=NE_S}					: 7th shock\n"
  "		else if (t > NE2temp6+100 && t < NE2temp7) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp6+100)))}  				 \n"
  "	else if (t >= NE2temp7 && t <= NE2temp7+100) {NE2=NE_S}					: 8th shock\n"
  "		else if (t > NE2temp7+100 && t < NE2temp8) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp7+100)))}  				    \n"
  "	else if (t >= NE2temp8 && t <= NE2temp8+100) {NE2=NE_S }					: 9th shock\n"
  "		else if (t > NE2temp8+100 && t < NE2temp9) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp8+100)))}  				    \n"
  "	else if (t >= NE2temp9 && t <= NE2temp9+100) {NE2=NE_S }					: 10th shock\n"
  "		else if (t > NE2temp9+100 && t < NE2temp10) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp9+100)))}  				    \n"
  "	else if (t >= NE2temp10 && t <= NE2temp10+100) {NE2=NE_S}					: 11th shock\n"
  "		else if (t > NE2temp10+100 && t < NE2temp11) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp10+100)))}  				 \n"
  "	else if (t >= NE2temp11 && t <= NE2temp11+100) {NE2=NE_S }					: 12th shock\n"
  "		else if (t > NE2temp11+100 && t < NE2temp12) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp11+100)))}  				 \n"
  "	else if (t >= NE2temp12 && t <= NE2temp12+100) {NE2=NE_S}					: 13th shock\n"
  "		else if (t > NE2temp12+100 && t < NE2temp13) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp12+100)))} 				 \n"
  "	else if (t >= NE2temp13 && t <= NE2temp13+100) {NE2=NE_S }					: 14th shock\n"
  "		else if (t > NE2temp13+100 && t < NE2temp14) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp13+100)))}   				 \n"
  "	else if (t >= NE2temp14 && t <= NE2temp14+100) {NE2=NE_S}					: 15th shock\n"
  "		else if (t > NE2temp14+100 && t < NE2temp15) {NE2 = 1.0 + (NE_S-1)*exp(-Beta2*(t-(NE2temp14+100)))}  				 \n"
  "	else if (t >= NE2temp15 && t <= NE2temp15+100) {NE2=NE_S}					: 16th shock\n"
  "		else  {	NE2 = 1.0}\n"
  "}\n"
  "\n"
  "FUNCTION GAP1(GAPstart1 (ms), GAPstop1 (ms)) {\n"
  "	LOCAL s\n"
  "	if (t <= GAPstart1) { GAP1 = 1}\n"
  "	else if (t >= GAPstart1 && t <= GAPstop1) {GAP1 = 1}					: During the Gap, apply lamda2*2\n"
  "	else  {	GAP1 = 1}\n"
  "}\n"
  "FUNCTION unirand() {    : uniform random numbers between 0 and 1\n"
  "        unirand = scop_random()\n"
  "}\n"
  ;
#endif
