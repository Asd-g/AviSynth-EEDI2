## Description

EEDI2 resizes an image by 2x in the vertical direction by copying the existing image to 2\*y(n) and interpolating the missing field. It is intended for edge-directed interpolation for deinterlacing (i.e. not really made for resizing a normal image, but can do that as well).

This is [a port of the VapourSynth plugin EEDI2](https://github.com/HomeOfVapourSynthEvolution/VapourSynth-EEDI2).

### Requirements:

- AviSynth 2.60 / AviSynth+ 3.4 or later

- Microsoft VisualC++ Redistributable Package 2022 (can be downloaded from [here](https://github.com/abbodi1406/vcredist/releases))

### Usage:

```
EEDI2 (clip, int "mthresh", int "lthresh", int "vthresh", int "estr", int "dstr", int "maxd", int "field", int "map", int "nt", int "pp")
```

### Parameters:

- clip\
    A clip to process. It must be in 8..16-bit planar format with/without alpha.
    
- mthresh\
    It is the edge magnitude threshold. Lower values will detect weaker edges.\
    Must be greater than 0.\
    Default: 10.
    
- lthresh\
    It is the laplacian threshold. Lower values will detect weaker lines.\
    Must be greater than 0.\
    Default: 20.
    
- vthresh\
    It is the variance threshold. Lower values will detect weaker edges.\
    Must be greater than 0.\
    Default: 20.
    
- estr\
    Erosion of the edge map.\
    It sets the required number of edge pixels (<=) in a 3x3 area, in which the center pixel has been detected as an edge pixel, for the center pixel to be removed from the edge map..\
    Must be greater than 0.\
    Default: 2.
    
- dstr\
    Dilation and erosion of the edge map.\
    It sets the required number of edge pixels (>=) in a 3x3 area, in which the center pixel has not been detected as an edge pixel, for the center pixel to be added to the edge map.\
    Must be greater than 0.\
    Default: 4.

- maxd\
    It sets the maximum pixel search distance for determining the interpolation direction. Larger values will be able to connect edges and lines of smaller slope but can lead to artifacts. Sometimes using a smaller maxd will give better results than a larger setting.\
    Must be between 0 and 29.\
    Default: 24.
    
- field\
    It controls which field in the resized image the original image will be copied to.\
    -2: Alternates each frame, uses AviSynth's internal parity value to start.\
    -1: It uses AviSynth's internal parity value.\
    0: Bottom field.\
    1: Top field.\
    2: It alternates each frame, starts with bottom.\
    3: It alternates each frame, starts with top.\
    Default: -1.
    
- map\
    0: No map.\
    1: Edge map (edge pixels will be set to 255 and non-edge pixels will be set to 0).\
    2: Original scale direction map.\
    3: 2x scale direction map.\
    Default: 0.
    
- nt\
    It defines a noise threshold between pixels in the sliding vectors, this is used to set initial starting values. Lower values should reduce artifacts but sacrifice edge reconstruction... while higher values should improve edge recontruction but lead to more artifacts.\
    Must be between greater than 0.\
    Default: 50.
    
- pp\
    It enables two optional post-processing modes aimed at reducing artifacts by identifying problem areas and then using plain vertical linear interpolation in those parts. Using the pp modes will slow down processing and can cause some loss of edge directedness.\
    0: No post-processing.\
    1: Check for spatial consistency of final interpolation directions.\
    2: Check for junctions and corners.\
    3: Do both 1 and 2.\
    Default: 1.
    