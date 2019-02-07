#!/bin/bash

DO_FILTERCROP=1
DO_EXTRACTCROP=1
DO_COMPCROPERR=1
DO_PLOTCROPERRS=1

#MPICMD="mpiexec -n 16"
MPICMD=""

SCALES="0.125 0.25 0.5 1.0 2.0"

#CROP_CFGFILES="configs/0*o.dcfg configs/10*o.dcfg"
CROP_CFGFILES="configs/0[0235789]*o.dcfg"

#PLOT_PARAMS="--img_format=png"
PLOT_PARAMS="--img_format=pdf"


if [ $DO_FILTERCROP -ne 0 ]; then
  for cfg in $CROP_CFGFILES; do
    for scl in $SCALES; do
      $MPICMD ./map_localization.py --mode=filter --crop_size=${scl} --out_dir=results/ ${cfg} || exit 1
    done
  done
fi

if [ $DO_EXTRACTCROP -ne 0 ]; then
  for cfg in $CROP_CFGFILES; do
    for scl in $SCALES; do
      $MPICMD ./map_localization.py --mode=extract_modes --crop_size=${scl} --out_dir=results/ ${cfg} || exit 1
    done
  done
fi

if [ $DO_COMPCROPERR -ne 0 ]; then
  for scl in $SCALES; do
    $MPICMD ./map_localization.py --mode=compute_errors --crop_size=${scl} --out_dir=results/ $CROP_CFGFILES || exit 1
  done
fi

if [ $DO_PLOTCROPERRS -ne 0 ]; then
  $MPICMD ./map_localization.py --mode=plot_crop_errors $PLOT_PARAMS --out_dir=results/ "$SCALES" $CROP_CFGFILES || exit 1
fi

