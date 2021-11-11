%% Initalize the simulation
% WRAPPER_CALLINITSIM
% calls and sets up the system and assigns important 
% variables to the base workspace
function Wrapper_CallInitSim()
    VehicleLogicSim = VehicleLogicSimulator();
    VehicleLogicSim = VehicleLogicSim.InitSim();
    assignin('base', 'VehicleLogicSim', VehicleLogicSim);
end

