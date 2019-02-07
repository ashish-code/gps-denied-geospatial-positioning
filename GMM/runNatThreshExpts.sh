#!/bin/bash

NATVALS="0.00001 0.0001 0.001 0.01 0.1"

DO_FILTER=1
DO_EXTRACT=1
DO_COMPERRS=1
DO_PLOTERRS=1

#PLOT_PARAMS="--img_format=png"
PLOT_PARAMS="--img_format=pdf"

CFGFILES=""
for i in 00 01 02 03 04 05 06 07 08 09 10; do
    CFGFILES+=" configs/${i}-stereo.dcfg "
    CFGFILES+=" configs/${i}-mono.dcfg "
done


if [ $DO_FILTER -ne 0 ]; then
    for CFGFILE in $CFGFILES; do
        for natThresh in $NATVALS; do
            mpiexec -n 16 ./map_localization.py --mode=filter --out_dir=results --simplify_thresh_name=1 --simplify_threshold=$natThresh ${CFGFILE} || exit 1
        done
    done
fi

if [ $DO_EXTRACT -ne 0 ]; then
    for natThresh in $NATVALS; do
        mpiexec -n 16 ./map_localization.py --mode=extract_modes --out_dir=results --simplify_thresh_name=1 --simplify_threshold=$natThresh ${CFGFILES} || exit 1
    done
fi

if [ $DO_COMPERRS -ne 0 ]; then
    for CFGFILE in $CFGFILES; do
        for natThresh in $NATVALS; do
            mpiexec -n 16 ./map_localization.py --mode=compute_errors --out_dir=results --simplify_thresh_name=1 --simplify_threshold=$natThresh ${CFGFILE} || exit 1
        done
    done
fi

if [ $DO_PLOTERRS -ne 0 ]; then
    mpiexec -n 16 ./map_localization.py --mode=plot_natthresh_errors $PLOT_PARAMS --out_dir=results --simplify_thresh_name=1 "$NATVALS" ${CFGFILES} || exit 1
fi

