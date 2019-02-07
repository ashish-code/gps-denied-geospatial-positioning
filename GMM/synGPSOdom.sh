#!/bin/bash

STEPS="0.01 0.1 0.25 0.5 0.75 1.0 1.5 2.0"

if false; then
    for i in 00 01 02 03 04 05 06 07 08 09 10; do
        ./map_localization.py --mode=synthesize_odometry ${i}-gps.obs configs/${i}-stereo.dcfg 0 0
    done

    ./map_localization.py --mode=print_odometry_stats *-gps.obs --dt=0.1

    for i in 00 01 02 03 04 05 06 07 08 09 10; do
        for n in $STEPS; do
            ./map_localization.py --mode=synthesize_odometry ../../data/odomoetry/gps/${i}-gps_noise${n}.obs configs/${i}-stereo.dcfg $n $n
        done
    done

    for i in 00 01 02 03 04 05 06 07 08 09 10; do
        for n in $STEPS; do
            DYNPARAMFILE=gps_noise${n}_dynamics_params.p
            CFGFILE=configs/noise/${i}-gps_noise${n}.dcfg
            cat configs/${i}-stereo.dcfg | sed -e "s/data_source = Stereo/data_source = GPS/" \
                   | sed -e "s/dynamics_params = params\/stereo_dynamics_params.p/dynamics_params = params\/${DYNPARAMFILE}/" \
                   | sed -e "s/libviso2_stereo\/${i}.txt/gps\/${i}-gps_noise${n}.obs/" > $CFGFILE
            echo "snr = ${n}" >> $CFGFILE
        done
    done

    for n in $STEPS; do
        DYNPARAMFILE=params/gps_noise${n}_dynamics_params.p
        TRAINCFGFILES="configs/noise/0*-gps_noise${n}.dcfg configs/noise/10-gps_noise${n}.dcfg"
        ./map_localization.py --mode=fit_motionmodel --out_dir=results ${DYNPARAMFILE} $TRAINCFGFILES || exit 1
    done
fi

