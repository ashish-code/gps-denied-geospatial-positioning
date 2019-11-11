# Ubiquitous Geo-spatial Positioning

## Patent
[Systems, methods and devices for geo-localization, US Patent US 10101466 B2](https://patents.google.com/patent/WO2017143144A1/en?oq=Systems%2c+methods%2c+and+devices+for+geo-localization)

## Publications
[GPS-denied Geo-Localisation using Visual Odometry](https://www.researchgate.net/profile/Ashish_Gupta7/publication/296196417_GPS-denied_Geo-Localisation_using_Visual_Odometry/links/56d3543b08ae85c8234c6eae.pdf)

[Ubiquitous Real-Time Geo-Spatial Localization](https://www.academia.edu/30242567/Ubiquitous_Real-Time_Geo-Spatial_Localization)

[Indoor Positioning using Visual and Inertial Sensors](https://www.academia.edu/30242481/Indoor_Positioning_using_Visual_and_Inertial_Sensors)

## Abstract
Rapidly growing technologies like autonomous navigation require accurate geo-localization in both outdoor and indoor environments. GNSS based outdoor localization has limitation of accuracy, which deteriorates in urban canyons, forested region and is unavailable indoors. Technologies like RFID, UWB, WiFi are used for indoor localization. These suffer limitations of high infrastructure costs, and signal transmission issues like multi-path, and frequent replacement of transciever batteries. We propose an alternative to localize an individual or a vehcile that is moving inside or outside a building. Instead of mobile RF transceivers, we utilize a sensor suite that includes a video camera and an inertial measurement unit. We estimate a motion trajectory of this sensor suite using Visual Odometery. Instead of pre-installed transceivers, we use GIS map for outdoors, or a BIM model for indoors. The transport layer in GIS map or navigable paths in BIM are abstracted as a graph structure. The geo-location of the mobile platform is inferred by first localizing its trajectory. We introduce an adaptive probabilistic inference approach to search for this trajectory in the entire map with no initialization information. Using an effective graph traversal spawn-and-prune strategy, we can localize the mobile platform in real-time. In comparison to other technologies, our approach requires economical sensors and the required map data is typically available in the public domain. Additionally, unlike other technologies which function exclusively indoors or outdoors, our approach functions in both  environments. We demonstrate our approach on real world examples of both indoor and outdoor locations.

![Ubiquitous localization using visual odometry and GIS/BIM. We focus on localizing the trajectory of the sensors in the outdoor/indoor environment and thereby infer the geospatial location of the sensor in real-time.](https://www.dropbox.com/s/hyvr7fst90hq9t0/systemBlockDiaVert.png?raw=1)


## Trajectory of mobile platform
We use Structure from Motion (SfM) based approach to generate a trajectory of the mobile platform using visual and inertial sensors.

### Block diagram of SfM trajectory generation
<img src="https://www.dropbox.com/s/jt762bgfbpzs1fu/trajectory.png?raw=1" width="350">

### Sample trajectory generation (excerpt from Karlshure dataset)
<img src="https://media.giphy.com/media/Y3YZgLtmrneC8le4Q6/giphy.gif" width="640">

## GIS -> Graph search

### Block diagram of path network graph search
<img src="https://www.dropbox.com/s/7b6yr3whx3nsmgp/algoBlockDiaVertical.png?raw=1" width="350">

### Encode trajectory of mobile platform
<img src='https://www.dropbox.com/s/2z8vzxkm7c7dhqa/graphPathEncode.png?raw=1' width='350'>



## Demo

### Positioning in city (Washington DC)
<img src="https://media.giphy.com/media/Pkotxl7AxT50DgKCG7/giphy.gif" width="640">

### Positioning in large geographic region (Montpellier area in south France)
<img src="https://media.giphy.com/media/jt8nUHJ32PY9wKtaz0/giphy.gif" width="640">

### Positioning in indoor environment (OSU Physics Building 2nd floor)
<img src="https://media.giphy.com/media/eJjs9dr3ftUze7wSyR/giphy.gif" width="640">

