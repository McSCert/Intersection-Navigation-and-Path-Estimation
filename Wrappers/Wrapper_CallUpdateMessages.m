%% Update the messages of the vehicles
% WRAPPER_CALLUPDATEMESSAGES
% wrapper function to update the vehicles messages
function Wrapper_CallUpdateMessages()
    VehicleLogicSim = evalin('base', 'VehicleLogicSim');
    VehicleLogicSim.UpdateMessages();
    assignin('base', 'VehicleLogicSim', VehicleLogicSim);
end

