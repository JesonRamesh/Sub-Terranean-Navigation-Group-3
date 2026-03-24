classdef iks02a1_error < matlab.System
    methods (Access = protected)
        function stepImpl(obj, trigger)
            if trigger == true
                % Stop the simulation
                error('NucleoError: Unplug and Replug the Nucleo!');
            end
        end
    end
end
