#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;

extern void _bg2inter_reg(void);
extern void _bg2pyr_reg(void);
extern void _cadyn_reg(void);
extern void _cal2_reg(void);
extern void _ca_reg(void);
extern void _capool_reg(void);
extern void _function_TMonitor_reg(void);
extern void _h_reg(void);
extern void _im_reg(void);
extern void _interD2pyrDDANE_STFD_reg(void);
extern void _interD2pyrDDA_STFD_reg(void);
extern void _interD2pyrDNE_STFD_reg(void);
extern void _interD2pyrD_STFD_reg(void);
extern void _interD2pyrVDA_STFD_reg(void);
extern void _interD2pyrV_STFD_reg(void);
extern void _interV2pyrDDANE_STFD_reg(void);
extern void _interV2pyrDDA_STFD_reg(void);
extern void _interV2pyrDNE_STFD_reg(void);
extern void _interV2pyrD_STFD_reg(void);
extern void _interV2pyrVDA_STFD_reg(void);
extern void _interV2pyrV_STFD_reg(void);
extern void _kadist_reg(void);
extern void _kaprox_reg(void);
extern void _kdrca1DA_reg(void);
extern void _kdrca1_reg(void);
extern void _kdrinter_reg(void);
extern void _leakDA_reg(void);
extern void _leakinter_reg(void);
extern void _leak_reg(void);
extern void _na3DA_reg(void);
extern void _na3_reg(void);
extern void _nainter_reg(void);
extern void _pyrD2interD_STFD_reg(void);
extern void _pyrD2interV_STFD_reg(void);
extern void _pyrD2pyrDDA_STFD_reg(void);
extern void _pyrD2pyrD_STFD_reg(void);
extern void _pyrD2pyrVDA_STFD_reg(void);
extern void _pyrD2pyrV_STFD_reg(void);
extern void _pyrV2interD_STFD_reg(void);
extern void _pyrV2interV_STFD_reg(void);
extern void _pyrV2pyrDDA_STFD_reg(void);
extern void _pyrV2pyrD_STFD_reg(void);
extern void _pyrV2pyrVDA_STFD_reg(void);
extern void _pyrV2pyrV_STFD_reg(void);
extern void _sahp_reg(void);
extern void _sahpNE_reg(void);
extern void _shock2interD_reg(void);
extern void _shock2interV_reg(void);
extern void _shock2pyrD_reg(void);
extern void _shock2pyrV_reg(void);
extern void _tone2interD_reg(void);
extern void _tone2interDNE_reg(void);
extern void _tone2interV_reg(void);
extern void _tone2interVNE_reg(void);
extern void _tone2pyrD_LAdv_reg(void);
extern void _tone2pyrD_reg(void);
extern void _tone2pyrDNE_LAdv_reg(void);
extern void _tone2pyrDNE_reg(void);
extern void _tone2pyrV_LAdd_reg(void);
extern void _tone2pyrV_reg(void);
extern void _tone2pyrVNE_LAdd_reg(void);
extern void _tone2pyrVNE_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," ./KimEtAl2013/bg2inter.mod");
    fprintf(stderr," ./KimEtAl2013/bg2pyr.mod");
    fprintf(stderr," ./KimEtAl2013/cadyn.mod");
    fprintf(stderr," ./KimEtAl2013/cal2.mod");
    fprintf(stderr," ./KimEtAl2013/ca.mod");
    fprintf(stderr," ./KimEtAl2013/capool.mod");
    fprintf(stderr," ./KimEtAl2013/function_TMonitor.mod");
    fprintf(stderr," ./KimEtAl2013/h.mod");
    fprintf(stderr," ./KimEtAl2013/im.mod");
    fprintf(stderr," ./KimEtAl2013/interD2pyrDDANE_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interD2pyrDDA_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interD2pyrDNE_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interD2pyrD_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interD2pyrVDA_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interD2pyrV_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interV2pyrDDANE_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interV2pyrDDA_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interV2pyrDNE_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interV2pyrD_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interV2pyrVDA_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/interV2pyrV_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/kadist.mod");
    fprintf(stderr," ./KimEtAl2013/kaprox.mod");
    fprintf(stderr," ./KimEtAl2013/kdrca1DA.mod");
    fprintf(stderr," ./KimEtAl2013/kdrca1.mod");
    fprintf(stderr," ./KimEtAl2013/kdrinter.mod");
    fprintf(stderr," ./KimEtAl2013/leakDA.mod");
    fprintf(stderr," ./KimEtAl2013/leakinter.mod");
    fprintf(stderr," ./KimEtAl2013/leak.mod");
    fprintf(stderr," ./KimEtAl2013/na3DA.mod");
    fprintf(stderr," ./KimEtAl2013/na3.mod");
    fprintf(stderr," ./KimEtAl2013/nainter.mod");
    fprintf(stderr," ./KimEtAl2013/pyrD2interD_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrD2interV_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrD2pyrDDA_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrD2pyrD_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrD2pyrVDA_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrD2pyrV_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrV2interD_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrV2interV_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrV2pyrDDA_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrV2pyrD_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrV2pyrVDA_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/pyrV2pyrV_STFD.mod");
    fprintf(stderr," ./KimEtAl2013/sahp.mod");
    fprintf(stderr," ./KimEtAl2013/sahpNE.mod");
    fprintf(stderr," ./KimEtAl2013/shock2interD.mod");
    fprintf(stderr," ./KimEtAl2013/shock2interV.mod");
    fprintf(stderr," ./KimEtAl2013/shock2pyrD.mod");
    fprintf(stderr," ./KimEtAl2013/shock2pyrV.mod");
    fprintf(stderr," ./KimEtAl2013/tone2interD.mod");
    fprintf(stderr," ./KimEtAl2013/tone2interDNE.mod");
    fprintf(stderr," ./KimEtAl2013/tone2interV.mod");
    fprintf(stderr," ./KimEtAl2013/tone2interVNE.mod");
    fprintf(stderr," ./KimEtAl2013/tone2pyrD_LAdv.mod");
    fprintf(stderr," ./KimEtAl2013/tone2pyrD.mod");
    fprintf(stderr," ./KimEtAl2013/tone2pyrDNE_LAdv.mod");
    fprintf(stderr," ./KimEtAl2013/tone2pyrDNE.mod");
    fprintf(stderr," ./KimEtAl2013/tone2pyrV_LAdd.mod");
    fprintf(stderr," ./KimEtAl2013/tone2pyrV.mod");
    fprintf(stderr," ./KimEtAl2013/tone2pyrVNE_LAdd.mod");
    fprintf(stderr," ./KimEtAl2013/tone2pyrVNE.mod");
    fprintf(stderr, "\n");
  }
  _bg2inter_reg();
  _bg2pyr_reg();
  _cadyn_reg();
  _cal2_reg();
  _ca_reg();
  _capool_reg();
  _function_TMonitor_reg();
  _h_reg();
  _im_reg();
  _interD2pyrDDANE_STFD_reg();
  _interD2pyrDDA_STFD_reg();
  _interD2pyrDNE_STFD_reg();
  _interD2pyrD_STFD_reg();
  _interD2pyrVDA_STFD_reg();
  _interD2pyrV_STFD_reg();
  _interV2pyrDDANE_STFD_reg();
  _interV2pyrDDA_STFD_reg();
  _interV2pyrDNE_STFD_reg();
  _interV2pyrD_STFD_reg();
  _interV2pyrVDA_STFD_reg();
  _interV2pyrV_STFD_reg();
  _kadist_reg();
  _kaprox_reg();
  _kdrca1DA_reg();
  _kdrca1_reg();
  _kdrinter_reg();
  _leakDA_reg();
  _leakinter_reg();
  _leak_reg();
  _na3DA_reg();
  _na3_reg();
  _nainter_reg();
  _pyrD2interD_STFD_reg();
  _pyrD2interV_STFD_reg();
  _pyrD2pyrDDA_STFD_reg();
  _pyrD2pyrD_STFD_reg();
  _pyrD2pyrVDA_STFD_reg();
  _pyrD2pyrV_STFD_reg();
  _pyrV2interD_STFD_reg();
  _pyrV2interV_STFD_reg();
  _pyrV2pyrDDA_STFD_reg();
  _pyrV2pyrD_STFD_reg();
  _pyrV2pyrVDA_STFD_reg();
  _pyrV2pyrV_STFD_reg();
  _sahp_reg();
  _sahpNE_reg();
  _shock2interD_reg();
  _shock2interV_reg();
  _shock2pyrD_reg();
  _shock2pyrV_reg();
  _tone2interD_reg();
  _tone2interDNE_reg();
  _tone2interV_reg();
  _tone2interVNE_reg();
  _tone2pyrD_LAdv_reg();
  _tone2pyrD_reg();
  _tone2pyrDNE_LAdv_reg();
  _tone2pyrDNE_reg();
  _tone2pyrV_LAdd_reg();
  _tone2pyrV_reg();
  _tone2pyrVNE_LAdd_reg();
  _tone2pyrVNE_reg();
}