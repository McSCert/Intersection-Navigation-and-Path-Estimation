%% Update the positions of the vehicles
% WRAPPER_CALLUPDATEPOSITIONS
% wrapper function to update the vehicles positions
function output =  Wrapper_CallUpdatePositions(indx, front_pos, rear_pos, velocities, yaws)
    VehicleLogicSim = evalin('base', 'VehicleLogicSim');
    assignin('base', 'vehicle_positions', front_pos)
    [target_speed, accel, streering_angle] = ...
        VehicleLogicSim.UpdatePositions(indx, front_pos, rear_pos, velocities,yaws);
    output(1) = target_speed;
    output(2) = accel;
    output(3) = streering_angle;
    assignin('base', 'VehicleLogicSim', VehicleLogicSim);    
end

