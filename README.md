# acperfo
MATLAB utilities to compute aircraft performance  
Adrien Crovato 2020

[![Apache License Version 2.0](https://img.shields.io/badge/license-Apache_2.0-green.svg)](LICENSE)

## Overview
acperfo is a collection of MATLAB functions that compute aircraft performance such as during climb, turn and take-off and landing.  
acperfo uses simple formula obtained from empirical correlations or global physical laws. The code currently handles aircraft flying in the subsonic regime only.

## How-to
1) Create a file containing the parameters defining the aircraft, e.g. `myaircraft.m`
2) Run using `acperfo('path/to/myaircraft')`

*Notes*:
- example files can be found under [tests](https://github.com/acrovato/acperfo/tree/master/tests)
- the folder containing `acperfo.m` must be accessible by MATLAB

## References
- Daniel P. Raymer, *Aircraft Design: A Conceptual Approach*
- Snorri Gudmundsson, *General Aviation Aircraft Design*
- Egbert Torenbeek, *Synthesis of Subsonic Airplane Design*