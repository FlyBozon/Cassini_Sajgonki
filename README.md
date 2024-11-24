# Cassini - Sajgonki
8th CASSINI Hackathon Poland\
23-24.11.2024

# Drone Pathfinding Algorithm for Fire Detection

## Overview
This algorithm receives information about areas covered by clouds, urban areas, and locations of fire watch towers in forests. Based on this data, the algorithm calculates the optimal path for the drone to inspect high-risk fire zones.

The algorithm models the area as a graph, where each region is divided into squares. Using graph traversal algorithms (e.g., A*), the drone optimizes its flight path to cover the highest possible area of concern. The drone's altitude affects its scanning capabilities, which are taken into account when determining the optimal route to cover as much area as possible.

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
