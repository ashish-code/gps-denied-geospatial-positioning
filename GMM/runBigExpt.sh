#!/bin/bash

#MPIEXEC="mpiexec -machinefile mpd-1.hosts"
#MPI_NUMCPUS=2
#MPIEXEC="mpiexec -machinefile mpd-4.hosts"
#MPI_NUMCPUS=8
MPIEXEC="mpiexec"
MPI_NUMCPUS=16

MPICMD="$MPIEXEC -n $MPI_NUMCPUS"
#MPICMD=""

MAP_PARAMS="-m kitti.p"
PLOT_PARAMS="--img_format=pdf"

SEQS=""
SEQS+=" 00 01 02 03 05 07 08 09 10"
SEQS+=" 11 12 15 16 17 18 19 20 21"

DO_CONVERTGPS=1
DO_FILTER=1
DO_EXTRACT=1
DO_COMPERR=1
DO_PLOTSTATS=1
DO_PLOTERRS=1
DO_DISPLAY=1
DO_DISPLAYFRAME=1
DO_DISPLAYCROP=1
DO_DISPLAYCROPTRACK=1
DO_MKMOVIE=1
DO_TABLES=0

CFGFILES=""
for i in $SEQS; do
    CFGFILES+=" configs/${i}-stereo.dcfg "
done

if [ $DO_CONVERTGPS -ne 0 ]; then
  $MPICMD ./map_localization.py $MAP_PARAMS --mode=convert_gps_data --out_dir=results $CFGFILES || exit 1
fi

if [ $DO_FILTER -ne 0 ]; then
  for i in $CFGFILES; do
    $MPICMD ./map_localization.py $MAP_PARAMS --mode=filter $i --out_dir=results || exit 1 
  done
fi

if [ $DO_EXTRACT -ne 0 ]; then
  $MPICMD ./map_localization.py $MAP_PARAMS --mode=extract_modes --out_dir=results $CFGFILES || exit 1
fi

if [ $DO_COMPERR -ne 0 ]; then
  $MPICMD ./map_localization.py $MAP_PARAMS --mode=compute_errors --out_dir=results $CFGFILES || exit 1
fi

if [ $DO_PLOTERRS -ne 0 ]; then
  $MPICMD ./map_localization.py $MAP_PARAMS --mode=plot_errors $PLOT_PARAMS --out_dir=results $CFGFILES || exit 1
fi

if [ $DO_PLOTSTATS -ne 0 ]; then
  $MPICMD ./map_localization.py $MAP_PARAMS --mode=plot_stats $PLOT_PARAMS --out_dir=results $CFGFILES || exit 1
fi

if [ $DO_DISPLAYFRAME -ne 0 ]; then
  $MPICMD ./map_localization.py $MAP_PARAMS --mode=display_frame --out_dir=results $CFGFILES || exit 1
fi
if [ $DO_MKMOVIE -ne 0 ]; then
  $MPICMD ./map_localization.py $MAP_PARAMS --mode=make_movie --out_dir=results $CFGFILES || exit 1
fi
if [ $DO_DISPLAYCROPTRACK -ne 0 ]; then
  $MPICMD ./map_localization.py $MAP_PARAMS --mode=display_croptrack --out_dir=results $CFGFILES || exit 1
fi

if [ $DO_TABLES -ne 0 ]; then
#  $MPICMD ./map_localization.py $MAP_PARAMS --mode=latex_tables --out_dir=results results/tab_bigerrssummary.tex $CFGFILES || exit 1
  ./map_localization.py $MAP_PARAMS --mode=latex_tables --out_dir=results results/tab_bigerrssummary.tex $CFGFILES || exit 1
fi


