# Cassini - Sajgonki
8th CASSINI Hackathon Poland\
23-24.11.2024


# Concept for Drone Pathfinding Algorithm for Fire Detection

## Overview
This document outlines the **concept** for a drone pathfinding algorithm designed to inspect high-risk fire zones. The described approach is not yet implemented but serves as a blueprint for future development.

The proposed algorithm calculates optimal drone flight paths based on input data, such as cloud-covered areas, urban zones, and locations of fire watch towers. It aims to maximize the inspection of high-risk areas while efficiently covering the region using graph-based pathfinding techniques (e.g., A*).

## Key Steps

1. **Input Data:**
   - **Cloud-covered areas:** Coordinates of regions with cloud coverage.
   - **Urban areas:** Locations of cities and settlements.
   - **Fire watch towers:** Locations of observation towers for fire monitoring.

2. **Graph Representation:**
   - The entire area is divided into a grid, creating a graph for the drone to navigate. 
   - The drone must inspect cloud-covered areas, avoid populated zones (as there are enught people to react on fire fastly), and focus on high-risk fire zones.

3. **Pathfinding Algorithm:**
   - The A* algorithm is used to find the optimal path that ensures the drone flies over cloud-covered regions while avoiding urban areas.
   - The drone's path is optimized to cover high-risk fire zones, ensuring maximum efficiency.

4. **Drone's Altitude and Scanning Angle:**
   - The drone’s flight height determines its scanning field of view.
   - The algorithm adjusts the path to ensure the drone flies at an altitude that maximizes coverage of the target areas.

5. **Optimized Flight Path:**
   - The drone travels to the target area, covering cloud zones and high-risk areas efficiently.
   - The return path avoids previously scanned areas, ensuring that missed regions are covered.

## Conclusion
This pathfinding algorithm enables a drone to inspect high-risk fire zones efficiently by maximizing coverage, minimizing overlap, and returning to the starting point. The drone’s altitude and scanning capabilities are taken into account to ensure it covers the largest area possible, ensuring quick fire detection during critical times.
