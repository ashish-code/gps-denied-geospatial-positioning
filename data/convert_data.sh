#!/bin/bash

for i in *.osm; do
	./osm_convert -i $i -o ${i/osm/bin} --split-streets --prune-buildings --prune-non-drivables --prune-service-roads --prune-tracks --street-segment-min-dist 3
done
