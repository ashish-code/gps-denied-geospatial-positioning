#!/bin/bash

DO_GEN=1
DO_CONVERTGPS=1
DO_FIT=1
DO_FILTER=1
DO_EXTRACT=1
DO_COMPERR=1
DO_DISPLAY=1
DO_DISPLAYFRAME=1
DO_DISPLAYCROP=1
DO_DISPLAYCROPTRACK=1
DO_MKMOVIE=1
DO_PLOTSTATS=1
DO_PLOTERRS=1
DO_TABLES=1

#MPIEXEC="mpiexec -machinefile mpd-1.hosts"
#MPI_NUMCPUS=2
MPIEXEC="mpiexec"
MPI_NUMCPUS=16
MPICMD="$MPIEXEC -n $MPI_NUMCPUS"

#PLOT_PARAMS="--img_format=png"
PLOT_PARAMS="--img_format=pdf"

if [ $DO_GEN -ne 0 ]; then
    for i in 00 01 02 03 04 05 06 07 08 09 10; do
        ./map_localization.py --mode=synthesize_odometry ../../data/odometry/gps/${i}-gps.obs configs/${i}-stereo.dcfg 0 0 &

        CFGFILE=configs/${i}-gps.dcfg
        cat configs/${i}-stereo.dcfg | sed -e "s/data_source = Stereo/data_source = GPS/" \
               | sed -e "s/libviso2_stereo\/${i}.txt/gps\/${i}-gps.obs/" \
               | sed -e "s/dataname=${i}stereo/dataname = ${i}gps/" > $CFGFILE
        echo "snr = 0" >> $CFGFILE

        CFGFILE=configs/${i}-gps-1kind.dcfg
        cat configs/${i}-stereo-1kind.dcfg | sed -e "s/data_source = Stereo/data_source = GPS/" \
               | sed -e "s/libviso2_stereo\/${i}.txt/gps\/${i}-gps.obs/" \
               | sed -e "s/dataname=${i}stereo/dataname = ${i}gps/" > $CFGFILE
        echo "snr = 0" >> $CFGFILE
    done
    wait
fi

#CFGFILES=configs/*.dcfg
#CFGFILES="configs/0[0-9]-stereo.dcfg configs/10-stereo.dcfg"
#CFGFILES=configs/*o.dcfg
#CFGFILES=configs/noise/*1kind.dcfg
#DISP_CFGFILES="configs/0*-stereo.dcfg configs/10-stereo.dcfg configs/0*-mono.dcfg configs/10-mono.dcfg"
DISP_CFGFILES=""
#DISP_CFGFILES+=" configs/0*-stereo.dcfg configs/10-stereo.dcfg"
#DISP_CFGFILES+=" configs/0*-mono.dcfg configs/10-mono.dcfg"
DISP_CFGFILES+=" configs/1[1-9]-stereo.dcfg configs/2*-stereo.dcfg"
#DISP_CFGFILES+=" configs/1[1-9]-mono.dcfg configs/2*-mono.dcfg"
CFGFILES=""
CFGFILES+=" configs/*-stereo.dcfg"
#CFGFILES+=" configs/*-mono.dcfg"
#CFGFILES+=" configs/*-gps.dcfg"
#CFGFILES+=" configs/*gps-1kind.dcfg"

if [ $DO_CONVERTGPS -ne 0 ]; then
  CVTCFGFILES="configs/*-gps.dcfg"
  $MPICMD ./map_localization.py --mode=convert_gps_data --out_dir=results $CVTCFGFILES || exit 1
fi

if [ $DO_FIT -ne 0 ]; then
  for i in mono stereo; do
    TRAINCFGFILES="configs/0*${i}.dcfg configs/10*${i}.dcfg"
    ./map_localization.py --mode=fit_motionmodel --out_dir=results params/${i}_dynamics_params.p $TRAINCFGFILES || exit 1

    TRAINCFGFILES="configs/0*${i}-1kind.dcfg configs/10*${i}-1kind.dcfg"
    ./map_localization.py --mode=fit_motionmodel --out_dir=results --dynamics=state2rigid_1kind params/${i}_1kind_dynamics_params.p $TRAINCFGFILES || exit 1 
  done
  TRAINCFGFILES="configs/noise/0*${i}-1kind.dcfg configs/noise/10*${i}-1kind.dcfg"
  ./map_localization.py --mode=fit_motionmodel --out_dir=results --dynamics=state2rigid_1kind params/gps_noise0.01_1kind_dynamics_params.p $TRAINCFGFILES || exit 1 
fi

if [ $DO_FILTER -ne 0 ]; then
  for i in $CFGFILES; do
    $MPICMD ./map_localization.py --mode=filter $i --out_dir=results || exit 1 
  done
fi

if [ $DO_EXTRACT -ne 0 ]; then
  $MPICMD ./map_localization.py --mode=extract_modes --out_dir=results $CFGFILES || exit 1
fi

if [ $DO_COMPERR -ne 0 ]; then
  $MPICMD ./map_localization.py --mode=compute_errors --out_dir=results $CFGFILES
fi

if [ $DO_DISPLAYCROPTRACK -ne 0 ]; then
  $MPICMD ./map_localization.py --mode=display_croptrack --out_dir=results $DISP_CFGFILES || exit 1
fi
if [ $DO_DISPLAYCROP -ne 0 ]; then
  $MPICMD ./map_localization.py --mode=display_crop --out_dir=results $DISP_CFGFILES || exit 1
fi

if [ $DO_DISPLAYFRAME -ne 0 ]; then
  $MPICMD ./map_localization.py --mode=display_frame --out_dir=results $DISP_CFGFILES || exit 1
fi
if [ $DO_DISPLAY -ne 0 ]; then
  $MPICMD ./map_localization.py --mode=display --out_dir=results $DISP_CFGFILES || exit 1
fi
if [ $DO_MKMOVIE -ne 0 ]; then
  $MPICMD ./map_localization.py --mode=make_movie --out_dir=results $DISP_CFGFILES || exit 1
fi


if [ $DO_PLOTSTATS -ne 0 ]; then
  $MPICMD ./map_localization.py --mode=plot_stats $PLOT_PARAMS --out_dir=results $CFGFILES || exit 1
fi
if [ $DO_PLOTERRS -ne 0 ]; then
  for i in mono stereo mono-1kind stereo-1kind; do
    GRPCFGFILES="configs/0*${i}.dcfg configs/10*${i}.dcfg"
    $MPICMD ./map_localization.py --mode=plot_errors $PLOT_PARAMS --out_dir=results ${i} $GRPCFGFILES || exit 1
  done
fi
if [ $DO_TABLES -ne 0 ]; then
  TBLCFGFILES=""
  TBLCFGFILES+=" configs/0*-stereo.dcfg "
  TBLCFGFILES+=" configs/0*-mono.dcfg "
  TBLCFGFILES+=" configs/0*-gps.dcfg "
  TBLCFGFILES+=" configs/10-stereo.dcfg "
  TBLCFGFILES+=" configs/10-mono.dcfg "
  TBLCFGFILES+=" configs/10-gps.dcfg "
  echo $TBLCFGFILES
  $MPICMD ./map_localization.py --mode=latex_tables --out_dir=results results/tab_errssummary.tex $TBLCFGFILES || exit 1
fi


