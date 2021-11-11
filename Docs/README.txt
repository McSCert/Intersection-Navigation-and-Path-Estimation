* Version 1.0
* September 2020

# Introduction 
This tool is an extension of to the autonomous driving toolbox for: directed graph creation, path estimation, and vehicle to vehicle communication.   The tool provides a method for researchers to perform analysis on intersection related problems. These essential capabilities can help researchers create scenarios, develop path planning algorithms and view them through Mathworks’s  Bird Eye Camera. A strong simulation platform helps to provide developers with the tools necessary to develop complex algorithms. Autonomous navigation through intersections is included as a case study since current methods of lane marker following is not a feasible methods of navigation.  

# Using the Tool

To begin using the tool, please perform the following steps:
1.	Add the project to your Matlab path by right clicking the folder “IntersectionSimulation” and selecting add folder and sub folders to path
2.	run init.m
3.	Open SimulationModel.slx and run the simulation

# Scenario Modification
## Simulation Parameter Modification
Parameters of the simulation are held in /IntersectionSimulation/Init/
One can change the simulation parameters by modifying one of these files and changing the reference in init.m
## Map Modification
To use your own map rather then the one supplied, please:
1.	Open SimulationModel.slx
2.	Modify “SimulationModel/Plant/Scenario Reader” to use your road network
3.	 G = Map(‘Your Road Network’, 10e-4)
4.	Save the object to a .mat file
5.	Open “VehicleLogicSimulator.m” and replace 'MarcCityGraphObj.mat' with your new file

# Testcases
Testcases for the project can be found in /IntersectionSimulation/Testcases

# Tool Status and Limitations
For an  understanding about the limitations and scope of the tool, please look at:
https://macsphere.mcmaster.ca/handle/11375/25878
