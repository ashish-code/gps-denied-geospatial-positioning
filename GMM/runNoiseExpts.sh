#!/bin/bash

STEPS="0.001 0.01 0.05 0.1 0.25 0.5 0.75 1.0 1.5 2.0 5.0 10.0"
#STEPS="0.001 5.0 10.0"
SAMPLES="0 1 2 3 4"
#SAMPLES="0 1"

#MPICMD="mpiexec -n 16"
MPICMD="mpiexec -machinefile mpd-2.hosts -n 4"
#MPICMD=""

DO_GEN=1
DO_FIT=1
DO_CONVERTGPS=1 
DO_FILTER=1
DO_EXTRACT=1
DO_COMPERRS=1
DO_PLOTERRS=1

#PLOT_PARAMS="--img_format=png"
PLOT_PARAMS="--img_format=pdf"

CFGFILES=""
#for i in 00 01 02 03 04 05 06 07 08 09 10; do
for i in 00 01 02 03 05 07 08 09 10; do
    for n in $STEPS; do
        #CFGFILES+=" configs/noise/${i}-gps_noise${n}.dcfg "
        for s in $SAMPLES; do
            CFGFILES+=" configs/noise/${i}-gps_noise${n}_sample${s}.dcfg "
        done
    done
done


if [ $DO_GEN -ne 0 ]; then
    for i in 00 01 02 03 04 05 06 07 08 09 10; do
        echo "seq = ${i}"
        for n in $STEPS; do
            for s in $SAMPLES; do
                ./map_localization.py --mode=synthesize_odometry ../../data/odometry/gps/${i}-gps_noise${n}_sample${s}.obs configs/${i}-stereo.dcfg $n $n &
                #./map_localization.py --mode=synthesize_odometry ../../data/odometry/gps/${i}-gps_noise${n}_sample${s}.obs configs/${i}-stereo.dcfg $n $n || exit 1 
                DYNPARAMFILE=gps_noise${n}_dynamics_params.p
                CFGFILE=configs/noise/${i}-gps_noise${n}_sample${s}.dcfg
                cat configs/${i}-stereo.dcfg | sed -e "s/data_source = Stereo/data_source = GPS/" \
                       | sed -e "s/dynamics_params = params\/stereo_dynamics_params.p/dynamics_params = params\/${DYNPARAMFILE}/" \
                       | sed -e "s/libviso2_stereo\/${i}.txt/gps\/${i}-gps_noise${n}_sample${s}.obs/" \
                       | sed -e "s/dataname=${i}stereo/dataname = ${i}gps_noise${n}_${s}/" > $CFGFILE
                echo "snr = ${n}" >> $CFGFILE
            done
            echo "  snr = ${n}"
        done
        wait
    done
fi

if [ $DO_FIT -ne 0 ]; then
    for n in $STEPS; do
        DYNPARAMFILE=params/gps_noise${n}_dynamics_params.p
#        TRAINCFGFILES="configs/noise/0*-gps_noise${n}.dcfg configs/noise/10-gps_noise${n}.dcfg"
        TRAINCFGFILES="configs/noise/0*-gps_noise${n}_sample*.dcfg configs/noise/10-gps_noise${n}_sample*.dcfg"
        #./map_localization.py --mode=fit_motionmodel --out_dir=results ${DYNPARAMFILE} $TRAINCFGFILES || exit 1
        ./map_localization.py --mode=fit_motionmodel --out_dir=results ${DYNPARAMFILE} $TRAINCFGFILES &
    done
    wait
fi

if [ $DO_FILTER -ne 0 ]; then
    for CFGFILE in $CFGFILES; do
        $MPICMD ./map_localization.py --mode=filter --out_dir=results ${CFGFILE} || exit 1
    done
fi

if [ $DO_CONVERTGPS -ne 0 ]; then
  $MPICMD ./map_localization.py --mode=convert_gps_data --out_dir=results $CFGFILES || exit 1
fi

if [ $DO_EXTRACT -ne 0 ]; then
    echo "Extracting modes..."
    $MPICMD ./map_localization.py --mode=extract_modes --out_dir=results $CFGFILES || exit 1
fi

if [ $DO_COMPERRS -ne 0 ]; then
    echo "Computing errors..."
    for CFGFILE in $CFGFILES; do
        $MPICMD ./map_localization.py --mode=compute_errors --out_dir=results ${CFGFILE} || exit 1
    done
fi

if [ $DO_PLOTERRS -ne 0 ]; then
    echo "Plotting errors..."
    $MPICMD ./map_localization.py --mode=plot_snr_errors $PLOT_PARAMS --out_dir=results $CFGFILES || exit 1
fi

