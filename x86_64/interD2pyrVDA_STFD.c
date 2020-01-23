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
 
#define nrn_init _nrn_init__interD2pyrVDA_STFD
#define _nrn_initial _nrn_initial__interD2pyrVDA_STFD
#define nrn_cur _nrn_cur__interD2pyrVDA_STFD
#define _nrn_current _nrn_current__interD2pyrVDA_STFD
#define nrn_jacob _nrn_jacob__interD2pyrVDA_STFD
#define nrn_state _nrn_state__interD2pyrVDA_STFD
#define _net_receive _net_receive__interD2pyrVDA_STFD 
#define release release__interD2pyrVDA_STFD 
 
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
 static double _hoc_DA2();
 static double _hoc_DA1();
 static double _hoc_GAP1();
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
 "DA2", _hoc_DA2,
 "DA1", _hoc_DA1,
 "GAP1", _hoc_GAP1,
 "eta", _hoc_eta,
 "omega", _hoc_omega,
 "unirand", _hoc_unirand,
 0, 0
};
#define DA2 DA2_interD2pyrVDA_STFD
#define DA1 DA1_interD2pyrVDA_STFD
#define GAP1 GAP1_interD2pyrVDA_STFD
#define eta eta_interD2pyrVDA_STFD
#define omega omega_interD2pyrVDA_STFD
#define unirand unirand_interD2pyrVDA_STFD
 extern double DA2( double , double );
 extern double DA1( double , double );
 extern double GAP1( double , double );
 extern double eta( double );
 extern double omega( double , double , double );
 extern double unirand( );
 /* declare global and static user variables */
#define Beta2 Beta2_interD2pyrVDA_STFD
 double Beta2 = 0.0001;
#define Beta1 Beta1_interD2pyrVDA_STFD
 double Beta1 = 0.001;
#define DA_S DA_S_interD2pyrVDA_STFD
 double DA_S = 1.5;
#define DA_t2 DA_t2_interD2pyrVDA_STFD
 double DA_t2 = 1.3;
#define DA_t1 DA_t1_interD2pyrVDA_STFD
 double DA_t1 = 0.8;
#define DAstop2 DAstop2_interD2pyrVDA_STFD
 double DAstop2 = 36000;
#define DAstart2 DAstart2_interD2pyrVDA_STFD
 double DAstart2 = 35900;
#define DAstop1 DAstop1_interD2pyrVDA_STFD
 double DAstop1 = 40000;
#define DAstart1 DAstart1_interD2pyrVDA_STFD
 double DAstart1 = 39500;
#define GAPstop1 GAPstop1_interD2pyrVDA_STFD
 double GAPstop1 = 196000;
#define GAPstart1 GAPstart1_interD2pyrVDA_STFD
 double GAPstart1 = 96000;
#define k k_interD2pyrVDA_STFD
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
 "Beta1_interD2pyrVDA_STFD", "/ms",
 "Beta2_interD2pyrVDA_STFD", "/ms",
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
 "k_interD2pyrVDA_STFD", &k_interD2pyrVDA_STFD,
 "GAPstart1_interD2pyrVDA_STFD", &GAPstart1_interD2pyrVDA_STFD,
 "GAPstop1_interD2pyrVDA_STFD", &GAPstop1_interD2pyrVDA_STFD,
 "DAstart1_interD2pyrVDA_STFD", &DAstart1_interD2pyrVDA_STFD,
 "DAstop1_interD2pyrVDA_STFD", &DAstop1_interD2pyrVDA_STFD,
 "DAstart2_interD2pyrVDA_STFD", &DAstart2_interD2pyrVDA_STFD,
 "DAstop2_interD2pyrVDA_STFD", &DAstop2_interD2pyrVDA_STFD,
 "DA_t1_interD2pyrVDA_STFD", &DA_t1_interD2pyrVDA_STFD,
 "DA_t2_interD2pyrVDA_STFD", &DA_t2_interD2pyrVDA_STFD,
 "DA_S_interD2pyrVDA_STFD", &DA_S_interD2pyrVDA_STFD,
 "Beta1_interD2pyrVDA_STFD", &Beta1_interD2pyrVDA_STFD,
 "Beta2_interD2pyrVDA_STFD", &Beta2_interD2pyrVDA_STFD,
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
"interD2pyrVDA_STFD",
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
 	lambda1 = 2;
 	lambda2 = 0.01;
 	threshold1 = 0.47;
 	threshold2 = 0.52;
 	initW = 4.5;
 	fmax = 4;
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

 void _interD2pyrVDA_STFD_reg() {
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
 	ivoc_help("help ?1 interD2pyrVDA_STFD /home/nest/lascon_project/x86_64/interD2pyrVDA_STFD.mod\n");
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
   g_gaba = gbar_gaba * r_gaba * facfactor * DA1 ( _threadargscomma_ DAstart1 , DAstop1 ) * DA2 ( _threadargscomma_ DAstart2 , DAstop2 ) ;
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
 g_gaba = gbar_gaba * r_gaba * facfactor * DA1 ( _threadargscomma_ DAstart1 , DAstop1 ) * DA2 ( _threadargscomma_ DAstart2 , DAstop2 ) ;
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
   g_gaba = gbar_gaba * r_gaba * facfactor * DA1 ( _threadargscomma_ DAstart1 , DAstop1 ) * DA2 ( _threadargscomma_ DAstart2 , DAstop2 ) ;
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
 
double DA1 (  double _lDAstart1 , double _lDAstop1 ) {
   double _lDA1;
 double _lDAtemp1 , _lDAtemp2 , _lDAtemp3 , _lDAtemp4 , _lDAtemp5 , _lDAtemp6 , _lDAtemp7 , _lDAtemp8 , _lDAtemp9 , _lDAtemp10 , _lDAtemp11 , _lDAtemp12 , _lDAtemp13 , _lDAtemp14 , _lDAtemp15 , _lDAtemp16 , _lDAtemp17 , _lDAtemp18 , _ls ;
 _lDAtemp1 = _lDAstart1 + 4000.0 ;
   _lDAtemp2 = _lDAtemp1 + 4000.0 ;
   _lDAtemp3 = _lDAtemp2 + 4000.0 ;
   _lDAtemp4 = _lDAtemp3 + 4000.0 ;
   _lDAtemp5 = _lDAtemp4 + 4000.0 ;
   _lDAtemp6 = _lDAtemp5 + 4000.0 ;
   _lDAtemp7 = _lDAtemp6 + 4000.0 ;
   _lDAtemp8 = _lDAtemp7 + 4000.0 ;
   _lDAtemp9 = _lDAtemp8 + 4000.0 ;
   _lDAtemp10 = _lDAtemp9 + 4000.0 ;
   _lDAtemp11 = _lDAtemp10 + 4000.0 ;
   _lDAtemp12 = _lDAtemp11 + 4000.0 ;
   _lDAtemp13 = _lDAtemp12 + 4000.0 ;
   _lDAtemp14 = _lDAtemp13 + 4000.0 ;
   _lDAtemp15 = _lDAtemp14 + 4000.0 + 100000.0 ;
   _lDAtemp16 = _lDAtemp15 + 4000.0 ;
   _lDAtemp17 = _lDAtemp16 + 4000.0 ;
   _lDAtemp18 = _lDAtemp17 + 4000.0 ;
   if ( t <= _lDAstart1 ) {
     _lDA1 = 1.0 ;
     }
   else if ( t >= _lDAstart1  && t <= _lDAstop1 ) {
     _lDA1 = DA_t1 ;
     }
   else if ( t > _lDAstop1  && t < _lDAtemp1 ) {
     _lDA1 = 1.0 + ( DA_t1 - 1.0 ) * exp ( - Beta1 * ( t - _lDAstop1 ) ) ;
     }
   else if ( t >= _lDAtemp1  && t <= _lDAtemp1 + 500.0 ) {
     _lDA1 = DA_t1 ;
     }
   else if ( t > _lDAtemp1 + 500.0  && t < _lDAtemp2 ) {
     _lDA1 = 1.0 + ( DA_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lDAtemp1 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp2  && t <= _lDAtemp2 + 500.0 ) {
     _lDA1 = DA_t1 ;
     }
   else if ( t > _lDAtemp2 + 500.0  && t < _lDAtemp3 ) {
     _lDA1 = 1.0 + ( DA_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lDAtemp2 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp3  && t <= _lDAtemp3 + 500.0 ) {
     _lDA1 = DA_t1 ;
     }
   else if ( t > _lDAtemp3 + 500.0  && t < _lDAtemp4 ) {
     _lDA1 = 1.0 + ( DA_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lDAtemp3 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp4  && t <= _lDAtemp4 + 500.0 ) {
     _lDA1 = DA_t1 ;
     }
   else if ( t > _lDAtemp4 + 500.0  && t < _lDAtemp5 ) {
     _lDA1 = 1.0 + ( DA_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lDAtemp4 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp5  && t <= _lDAtemp5 + 500.0 ) {
     _lDA1 = DA_t1 ;
     }
   else if ( t > _lDAtemp5 + 500.0  && t < _lDAtemp6 ) {
     _lDA1 = 1.0 + ( DA_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lDAtemp5 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp6  && t <= _lDAtemp6 + 500.0 ) {
     _lDA1 = DA_t1 ;
     }
   else if ( t > _lDAtemp6 + 500.0  && t < _lDAtemp7 ) {
     _lDA1 = 1.0 + ( DA_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lDAtemp6 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp7  && t <= _lDAtemp7 + 500.0 ) {
     _lDA1 = DA_t1 ;
     }
   else if ( t > _lDAtemp7 + 500.0  && t < _lDAtemp8 ) {
     _lDA1 = 1.0 + ( DA_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lDAtemp7 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp8  && t <= _lDAtemp8 + 500.0 ) {
     _lDA1 = DA_t1 ;
     }
   else if ( t > _lDAtemp8 + 500.0  && t < _lDAtemp9 ) {
     _lDA1 = 1.0 + ( DA_t1 - 1.0 ) * exp ( - Beta1 * ( t - ( _lDAtemp8 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp9  && t <= _lDAtemp9 + 500.0 ) {
     _lDA1 = DA_t2 ;
     }
   else if ( t > _lDAtemp9 + 500.0  && t < _lDAtemp10 ) {
     _lDA1 = 1.0 + ( DA_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lDAtemp9 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp10  && t <= _lDAtemp10 + 500.0 ) {
     _lDA1 = DA_t2 ;
     }
   else if ( t > _lDAtemp10 + 500.0  && t < _lDAtemp11 ) {
     _lDA1 = 1.0 + ( DA_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lDAtemp10 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp11  && t <= _lDAtemp11 + 500.0 ) {
     _lDA1 = DA_t2 ;
     }
   else if ( t > _lDAtemp11 + 500.0  && t < _lDAtemp12 ) {
     _lDA1 = 1.0 + ( DA_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lDAtemp11 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp12  && t <= _lDAtemp12 + 500.0 ) {
     _lDA1 = DA_t2 ;
     }
   else if ( t > _lDAtemp12 + 500.0  && t < _lDAtemp13 ) {
     _lDA1 = 1.0 + ( DA_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lDAtemp12 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp13  && t <= _lDAtemp13 + 500.0 ) {
     _lDA1 = DA_t2 ;
     }
   else if ( t > _lDAtemp13 + 500.0  && t < _lDAtemp14 ) {
     _lDA1 = 1.0 + ( DA_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lDAtemp13 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp14  && t <= _lDAtemp14 + 500.0 ) {
     _lDA1 = DA_t2 ;
     }
   else if ( t > _lDAtemp14 + 500.0  && t < _lDAtemp15 ) {
     _lDA1 = 1.0 + ( DA_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lDAtemp14 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp15  && t <= _lDAtemp15 + 500.0 ) {
     _lDA1 = DA_t2 ;
     }
   else if ( t > _lDAtemp15 + 500.0  && t < _lDAtemp16 ) {
     _lDA1 = 1.0 + ( DA_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lDAtemp15 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp16  && t <= _lDAtemp16 + 500.0 ) {
     _lDA1 = DA_t2 ;
     }
   else if ( t > _lDAtemp16 + 500.0  && t < _lDAtemp17 ) {
     _lDA1 = 1.0 + ( DA_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lDAtemp16 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp17  && t <= _lDAtemp17 + 500.0 ) {
     _lDA1 = DA_t2 ;
     }
   else if ( t > _lDAtemp17 + 500.0  && t < _lDAtemp18 ) {
     _lDA1 = 1.0 + ( DA_t2 - 1.0 ) * exp ( - Beta2 * ( t - ( _lDAtemp17 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDAtemp18  && t <= _lDAtemp18 + 500.0 ) {
     _lDA1 = 1.0 ;
     }
   else {
     _lDA1 = 1.0 ;
     }
   
return _lDA1;
 }
 
static double _hoc_DA1(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  DA1 (  *getarg(1) , *getarg(2) );
 return(_r);
}
 
double DA2 (  double _lDAstart2 , double _lDAstop2 ) {
   double _lDA2;
 double _lDA2temp1 , _lDA2temp2 , _lDA2temp3 , _lDA2temp4 , _lDA2temp5 , _lDA2temp6 , _lDA2temp7 , _lDA2temp8 , _lDA2temp9 , _lDA2temp10 , _lDA2temp11 , _lDA2temp12 , _lDA2temp13 , _lDA2temp14 , _lDA2temp15 , _lDA2temp16 , _ls ;
 _lDA2temp1 = _lDAstart2 + 4000.0 ;
   _lDA2temp2 = _lDA2temp1 + 4000.0 ;
   _lDA2temp3 = _lDA2temp2 + 4000.0 ;
   _lDA2temp4 = _lDA2temp3 + 4000.0 ;
   _lDA2temp5 = _lDA2temp4 + 4000.0 ;
   _lDA2temp6 = _lDA2temp5 + 4000.0 ;
   _lDA2temp7 = _lDA2temp6 + 4000.0 ;
   _lDA2temp8 = _lDA2temp7 + 4000.0 ;
   _lDA2temp9 = _lDA2temp8 + 4000.0 ;
   _lDA2temp10 = _lDA2temp9 + 4000.0 ;
   _lDA2temp11 = _lDA2temp10 + 4000.0 ;
   _lDA2temp12 = _lDA2temp11 + 4000.0 ;
   _lDA2temp13 = _lDA2temp12 + 4000.0 ;
   _lDA2temp14 = _lDA2temp13 + 4000.0 ;
   _lDA2temp15 = _lDA2temp14 + 4000.0 ;
   if ( t <= _lDAstart2 ) {
     _lDA2 = 1.0 ;
     }
   else if ( t >= _lDAstart2  && t <= _lDAstop2 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDAstop2  && t < _lDA2temp1 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDAstop2 + 500.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp1  && t <= _lDA2temp1 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp1 + 100.0  && t < _lDA2temp2 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp1 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp2  && t <= _lDA2temp2 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp2 + 100.0  && t < _lDA2temp3 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp2 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp3  && t <= _lDA2temp3 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp3 + 100.0  && t < _lDA2temp4 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp3 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp4  && t <= _lDA2temp4 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp4 + 100.0  && t < _lDA2temp5 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp4 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp5  && t <= _lDA2temp5 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp5 + 100.0  && t < _lDA2temp6 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp5 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp6  && t <= _lDA2temp6 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp6 + 100.0  && t < _lDA2temp7 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp6 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp7  && t <= _lDA2temp7 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp7 + 100.0  && t < _lDA2temp8 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp7 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp8  && t <= _lDA2temp8 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp8 + 100.0  && t < _lDA2temp9 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp8 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp9  && t <= _lDA2temp9 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp9 + 100.0  && t < _lDA2temp10 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp9 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp10  && t <= _lDA2temp10 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp10 + 100.0  && t < _lDA2temp11 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp10 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp11  && t <= _lDA2temp11 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp11 + 100.0  && t < _lDA2temp12 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp11 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp12  && t <= _lDA2temp12 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp12 + 100.0  && t < _lDA2temp13 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp12 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp13  && t <= _lDA2temp13 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp13 + 100.0  && t < _lDA2temp14 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp13 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp14  && t <= _lDA2temp14 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else if ( t > _lDA2temp14 + 100.0  && t < _lDA2temp15 ) {
     _lDA2 = 1.0 + ( DA_S - 1.0 ) * exp ( - Beta2 * ( t - ( _lDA2temp14 + 100.0 ) ) ) ;
     }
   else if ( t >= _lDA2temp15  && t <= _lDA2temp15 + 100.0 ) {
     _lDA2 = DA_S ;
     }
   else {
     _lDA2 = 1.0 ;
     }
   
return _lDA2;
 }
 
static double _hoc_DA2(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  DA2 (  *getarg(1) , *getarg(2) );
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
 if(error){fprintf(stderr,"at line 147 in file interD2pyrVDA_STFD.mod:\n	SOLVE release METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
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
static const char* nmodl_filename = "/home/nest/lascon_project/KimEtAl2013/interD2pyrVDA_STFD.mod";
static const char* nmodl_file_text = 
  ":Interneuron Cells to Pyramidal Cells GABA with local Ca2+ pool and read public soma Ca2+ pool\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS interD2pyrVDA_STFD\n"
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
  "	AlphaTmax_gaba = 7.2609(/ms)\n"
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
  "	lambda1 = 2 : 1 : 1.5 : 2.0\n"
  "	lambda2 = 0.01\n"
  "	threshold1 = 0.47 :  0.45 : 0.4 : 0.43 : 0.8 : 1.0 : 1.1 : 0.55 (uM)\n"
  "	threshold2 = 0.52 :  0.5 : 0.45 : 0.48 : 1.2 : 1.3 : 1.2 : 0.70 (uM)\n"
  "\n"
  "\n"
  "	:GABA Weight\n"
  "	initW = 4.5 :  :  3 :  2.5 : 1 : 5 : 6.25 : 5\n"
  "	fmax = 4 : 3.5 : 3 :4\n"
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
  "        tauD2 = 70 (ms) < 1e-9, 1e9 >		\n"
  "	\n"
  "	DAstart1 = 39500\n"
  "	DAstop1 = 40000	\n"
  "	DAstart2 = 35900\n"
  "	DAstop2 = 36000	\n"
  "\n"
  "	DA_t1 = 0.8\n"
  "	DA_t2 = 1.3 : 1.2\n"
  "	DA_S = 1.5 : 1.8	\n"
  "	Beta1 = 0.001  (/ms) : 1/decay time for neuromodulators\n"
  "	Beta2 = 0.0001  (/ms)		\n"
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
  "	g_gaba = gbar_gaba*r_gaba*facfactor*DA1(DAstart1,DAstop1)*DA2(DAstart2,DAstop2)		    : Dopamine effect on GABA\n"
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
  "    :printf(\"%g\\t%g\\t%g\\t%g\\t%g\\t%g\\n\", t, t-tsyn, F, D1, D2, facfactor)\n"
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
  "\n"
  "FUNCTION DA1(DAstart1 (ms), DAstop1 (ms)) {\n"
  "	LOCAL DAtemp1, DAtemp2, DAtemp3, DAtemp4, DAtemp5, DAtemp6, DAtemp7, DAtemp8, DAtemp9, DAtemp10, DAtemp11, DAtemp12, DAtemp13, DAtemp14, DAtemp15, DAtemp16, DAtemp17, DAtemp18,s\n"
  "	DAtemp1 = DAstart1+4000\n"
  "	DAtemp2 = DAtemp1+4000\n"
  "	DAtemp3 = DAtemp2+4000\n"
  "	DAtemp4 = DAtemp3+4000\n"
  "	DAtemp5 = DAtemp4+4000\n"
  "	DAtemp6 = DAtemp5+4000\n"
  "	DAtemp7 = DAtemp6+4000\n"
  "	DAtemp8 = DAtemp7+4000\n"
  "	DAtemp9 = DAtemp8+4000\n"
  "	DAtemp10 = DAtemp9+4000\n"
  "	DAtemp11 = DAtemp10+4000\n"
  "	DAtemp12 = DAtemp11+4000\n"
  "	DAtemp13 = DAtemp12+4000\n"
  "	DAtemp14 = DAtemp13+4000\n"
  "	DAtemp15 = DAtemp14 + 4000 + 100000     : 100sec Gap\n"
  "	DAtemp16 = DAtemp15 + 4000 \n"
  "	DAtemp17 = DAtemp16 + 4000\n"
  "	DAtemp18 = DAtemp17 + 4000\n"
  "\n"
  "	if (t <= DAstart1) { DA1 = 1.0}\n"
  "	else if (t >= DAstart1 && t <= DAstop1) {DA1 = DA_t1}					: 2nd tone in conditioning\n"
  "		else if (t > DAstop1 && t < DAtemp1) {DA1 = 1.0 + (DA_t1-1)*exp(-Beta1*(t-DAstop1))}  			: Basal level\n"
  "	else if (t >= DAtemp1 && t <= DAtemp1+500) {DA1=DA_t1}					: 3rd tone\n"
  "		else if (t > DAtemp1+500 && t < DAtemp2) {DA1 = 1.0 + (DA_t1-1)*exp(-Beta1*(t-(DAtemp1+500)))} 		: Basal level\n"
  "	else if (t >= DAtemp2 && t <= DAtemp2+500) {DA1=DA_t1}					: 4th tone\n"
  "		else if (t > DAtemp2+500 && t < DAtemp3) {DA1 = 1.0 + (DA_t1-1)*exp(-Beta1*(t-(DAtemp2+500)))} 		: Basal level	\n"
  "	else if (t >= DAtemp3 && t <= DAtemp3+500) {DA1=DA_t1}					: 5th tone\n"
  "		else if (t > DAtemp3+500 && t < DAtemp4) {DA1 = 1.0 + (DA_t1-1)*exp(-Beta1*(t-(DAtemp3+500)))} 		: Basal level\n"
  "	else if (t >= DAtemp4 && t <= DAtemp4+500) {DA1=DA_t1}					: 6th tone\n"
  "		else if (t > DAtemp4+500 && t < DAtemp5) {DA1 = 1.0 + (DA_t1-1)*exp(-Beta1*(t-(DAtemp4+500)))} 		: Basal level\n"
  "	else if (t >= DAtemp5 && t <= DAtemp5+500) {DA1=DA_t1}					: 7th tone\n"
  "		else if (t > DAtemp5+500 && t < DAtemp6) {DA1 = 1.0 + (DA_t1-1)*exp(-Beta1*(t-(DAtemp5+500)))} 		: Basal level\n"
  "	else if (t >= DAtemp6 && t <= DAtemp6+500) {DA1=DA_t1}					: 8th tone\n"
  "		else if (t > DAtemp6+500 && t < DAtemp7) {DA1 = 1.0 + (DA_t1-1)*exp(-Beta1*(t-(DAtemp6+500)))} 		: Basal level\n"
  "	else if (t >= DAtemp7 && t <= DAtemp7+500) {DA1=DA_t1}					: 9th tone\n"
  "		else if (t > DAtemp7+500 && t < DAtemp8) {DA1 = 1.0 + (DA_t1-1)*exp(-Beta1*(t-(DAtemp7+500)))} 		: Basal level\n"
  "	else if (t >= DAtemp8 && t <= DAtemp8+500) {DA1=DA_t1}					: 10th tone  \n"
  "		else if (t > DAtemp8+500 && t < DAtemp9) {DA1 = 1.0 + (DA_t1-1)*exp(-Beta1*(t-(DAtemp8+500)))} 		: Basal level\n"
  "	\n"
  "	else if (t >= DAtemp9 && t <= DAtemp9+500) {DA1=DA_t2}					: 11th tone   - Second Step\n"
  "		else if (t > DAtemp9+500 && t < DAtemp10) {DA1 = 1.0 + (DA_t2-1)*exp(-Beta2*(t-(DAtemp9+500)))}		: Basal level	\n"
  "	else if (t >= DAtemp10 && t <= DAtemp10+500) {DA1=DA_t2}					: 12th tone\n"
  "		else if (t > DAtemp10+500 && t < DAtemp11) {DA1 = 1.0 + (DA_t2-1)*exp(-Beta2*(t-(DAtemp10+500)))}	: Basal level\n"
  "	else if (t >= DAtemp11 && t <= DAtemp11+500) {DA1=DA_t2}					: 13th tone\n"
  "		else if (t > DAtemp11+500 && t < DAtemp12) {DA1 = 1.0 + (DA_t2-1)*exp(-Beta2*(t-(DAtemp11+500)))}	: Basal level\n"
  "	else if (t >= DAtemp12 && t <= DAtemp12+500) {DA1=DA_t2}					: 14th tone \n"
  "		else if (t > DAtemp12+500 && t < DAtemp13) {DA1 = 1.0 + (DA_t2-1)*exp(-Beta2*(t-(DAtemp12+500)))}	: Basal level\n"
  "	else if (t >= DAtemp13 && t <= DAtemp13+500) {DA1=DA_t2}					: 15th tone\n"
  "		else if (t > DAtemp13+500 && t < DAtemp14) {DA1 = 1.0 + (DA_t2-1)*exp(-Beta2*(t-(DAtemp13+500)))}	: Basal level\n"
  "	else if (t >= DAtemp14 && t <= DAtemp14+500) {DA1=DA_t2}					: 16th tone\n"
  "		else if (t > DAtemp14+500 && t < DAtemp15) {DA1 = 1.0 + (DA_t2-1)*exp(-Beta2*(t-(DAtemp14+500)))} 	: Basal level\n"
  "	\n"
  "	else if (t >= DAtemp15 && t <= DAtemp15+500) {DA1=DA_t2}					: 1st tone in Extinction\n"
  "		else if (t > DAtemp15+500 && t < DAtemp16) {DA1 = 1.0 + (DA_t2-1)*exp(-Beta2*(t-(DAtemp15+500)))}	: Basal level\n"
  "	else if (t >= DAtemp16 && t <= DAtemp16+500) {DA1=DA_t2}					: 2nd tone\n"
  "		else if (t > DAtemp16+500 && t < DAtemp17) {DA1 = 1.0 + (DA_t2-1)*exp(-Beta2*(t-(DAtemp16+500)))}	: Basal level\n"
  "	else if (t >= DAtemp17 && t <= DAtemp17+500) {DA1=DA_t2}					: 3rd tone\n"
  "		else if (t > DAtemp17+500 && t < DAtemp18) {DA1 = 1.0 + (DA_t2-1)*exp(-Beta2*(t-(DAtemp17+500)))}	: Basal level	\n"
  "	else if (t >= DAtemp18 && t <= DAtemp18+500) {DA1=1.0}					: 4th tone	\n"
  "		else  {	DA1 = 1.0}\n"
  "}\n"
  "FUNCTION DA2(DAstart2 (ms), DAstop2 (ms)) {\n"
  "	LOCAL DA2temp1, DA2temp2, DA2temp3, DA2temp4, DA2temp5, DA2temp6, DA2temp7, DA2temp8, DA2temp9, DA2temp10, DA2temp11, DA2temp12, DA2temp13, DA2temp14, DA2temp15, DA2temp16,s\n"
  "	DA2temp1 = DAstart2 + 4000\n"
  "	DA2temp2 = DA2temp1 + 4000\n"
  "	DA2temp3 = DA2temp2 + 4000\n"
  "	DA2temp4 = DA2temp3 + 4000\n"
  "	DA2temp5 = DA2temp4 + 4000\n"
  "	DA2temp6 = DA2temp5 + 4000\n"
  "	DA2temp7 = DA2temp6 + 4000\n"
  "	DA2temp8 = DA2temp7 + 4000\n"
  "	DA2temp9 = DA2temp8 + 4000\n"
  "	DA2temp10 = DA2temp9 + 4000\n"
  "	DA2temp11 = DA2temp10 + 4000\n"
  "	DA2temp12 = DA2temp11 + 4000 \n"
  "	DA2temp13 = DA2temp12 + 4000\n"
  "	DA2temp14 = DA2temp13 + 4000\n"
  "	DA2temp15 = DA2temp14 + 4000\n"
  "	\n"
  "	if (t <= DAstart2) { DA2 = 1.0}\n"
  "	else if (t >= DAstart2 && t <= DAstop2) {DA2 = DA_S }					: 1st shock\n"
  "		else if (t > DAstop2 && t < DA2temp1) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DAstop2+500)))}  					 \n"
  "	else if (t >= DA2temp1 && t <= DA2temp1+100) {DA2=DA_S}					: 2nd shock\n"
  "		else if (t > DA2temp1+100 && t < DA2temp2) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp1+100)))}    				 \n"
  "	else if (t >= DA2temp2 && t <= DA2temp2+100) {DA2=DA_S}					: 3rd shock\n"
  "		else if (t > DA2temp2+100 && t < DA2temp3) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp2+100)))}   				 \n"
  "	else if (t >= DA2temp3 && t <= DA2temp3+100) {DA2=DA_S}					: 4th shock\n"
  "		else if (t > DA2temp3+100 && t < DA2temp4) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp3+100)))}   				 \n"
  "	else if (t >= DA2temp4 && t <= DA2temp4+100) {DA2=DA_S}					: 5th shock\n"
  "		else if (t > DA2temp4+100 && t < DA2temp5) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp4+100)))}   				 \n"
  "	else if (t >= DA2temp5 && t <= DA2temp5+100) {DA2=DA_S}					: 6th shock\n"
  "		else if (t > DA2temp5+100 && t < DA2temp6) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp5+100)))}    				 \n"
  "	else if (t >= DA2temp6 && t <= DA2temp6+100) {DA2=DA_S}					: 7th shock\n"
  "		else if (t > DA2temp6+100 && t < DA2temp7) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp6+100)))}   				 \n"
  "	else if (t >= DA2temp7 && t <= DA2temp7+100) {DA2=DA_S}					: 8th shock\n"
  "		else if (t > DA2temp7+100 && t < DA2temp8) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp7+100)))}   				    \n"
  "	else if (t >= DA2temp8 && t <= DA2temp8+100) {DA2=DA_S }					: 9th shock\n"
  "		else if (t > DA2temp8+100 && t < DA2temp9) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp8+100)))}   				    \n"
  "	else if (t >= DA2temp9 && t <= DA2temp9+100) {DA2=DA_S }					: 10th shock\n"
  "		else if (t > DA2temp9+100 && t < DA2temp10) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp9+100)))}   				    \n"
  "	else if (t >= DA2temp10 && t <= DA2temp10+100) {DA2=DA_S}					: 11th shock\n"
  "		else if (t > DA2temp10+100 && t < DA2temp11) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp10+100)))}   				 \n"
  "	else if (t >= DA2temp11 && t <= DA2temp11+100) {DA2=DA_S }					: 12th shock\n"
  "		else if (t > DA2temp11+100 && t < DA2temp12) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp11+100)))}   				 \n"
  "	else if (t >= DA2temp12 && t <= DA2temp12+100) {DA2=DA_S}					: 13th shock\n"
  "		else if (t > DA2temp12+100 && t < DA2temp13) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp12+100)))}   				 \n"
  "	else if (t >= DA2temp13 && t <= DA2temp13+100) {DA2=DA_S }					: 14th shock\n"
  "		else if (t > DA2temp13+100 && t < DA2temp14) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp13+100)))}   				 \n"
  "	else if (t >= DA2temp14 && t <= DA2temp14+100) {DA2=DA_S}					: 15th shock\n"
  "		else if (t > DA2temp14+100 && t < DA2temp15) {DA2 = 1.0 + (DA_S-1)*exp(-Beta2*(t-(DA2temp14+100)))}   				 \n"
  "	else if (t >= DA2temp15 && t <= DA2temp15+100) {DA2=DA_S}					: 16th shock\n"
  "		else  {	DA2 = 1.0}\n"
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
