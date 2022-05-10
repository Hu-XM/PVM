# Overview

This repository is associated with the manuscript ***Effects of the direction and speed strategies on pedestrian dynamics (Under Review)***. Here, we provide the source code and simulation data supporting the analysis.

## Table of Contents

- **Code**
- **Data - costs of movement**
- **Data - fundamental diagrams**
- **Data - crowd stability**
- **Animation**

## Code

The source code of the proposed model (implemented in Python)

## Data

The simulation was carried out in a straight corridor of 10m long and 6m wide. Data required to reproduce our results are available in the Data folder, which is subdivided into three folders for the costs of movement, the fundamental diagrams and the crowd stability. 

### costs of movement
How the costs of movements (the travel time T, the route distance D, the total energy cost E, and the energy cost per unit distance Eud) change with p1 and p2. We did 20 simulations to eliminate the initialization effect.

### fundamental diagrams

1.The simulated fundamental diagrams over the (rho, epsilon) space. Density level was ranged from 0.17 ped/m^2 to 5 ped/m^2. The threshold parameter epsilon was ranged from -0.9 to 1.0, at a step of 0.05.

2.The simulated fundamental diagrams with two rho-density quadratic fitting functions to compare with the empirical data.

### crowd stability

The density and velocity distribution in crowded conditions (5 ped/m^2) with four typical parameter combinations. The corridor is discretized into 0.1m×0.1m grids. 

(a) p1=0.5, p2=0.2, epsilon=-0.2;

(b) p1=0.8, p2=0.8, epsilon=-0.2; 

(c) p1=0.5, p2=0.2, epsilon=-0.8;

(d) p1=0.8, p2=0.8, epsilon=-0.8. 

## Animation

Two simulation animations at the diluted (10 peds) and crowded  (270 peds) extremes.
