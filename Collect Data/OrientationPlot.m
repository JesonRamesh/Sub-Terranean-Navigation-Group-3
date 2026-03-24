classdef OrientationPlot < matlab.System
    % Visualize orientation using roll, pitch, and yaw (in radians)

    %   Copyright 2022 The MathWorks, Inc.

    %#codegen
    properties (Access = protected)
        visObj
    end

    methods (Access = protected)
        function setupImpl(obj)
            if coder.target('MATLAB')
                obj.visObj = poseplot;
                xlabel("x (m)");
                ylabel("y (m)");
                zlabel("z (m)");
                title("Orientation Plot");
            end
        end

        function stepImpl(obj, pitch, yaw, roll)
            if coder.target('MATLAB')
                if isvalid(obj.visObj)
                    % Compute the rotation matrix from roll, pitch, and yaw
                    orient = eulToRotm([-pitch+pi, -yaw, -roll]); % Custom function
                  
                    % Set the orientation
                    obj.visObj.Orientation = orient;
                    drawnow limitrate;
                end
            end
        end

        function releaseImpl(obj)
            obj.visObj = [];
        end

        %% Validate input and properties
        function validateInputsImpl(~, roll, pitch, yaw)
            validateattributes(roll,  {'double', 'single'}, {'real', 'scalar'}, '', 'Roll');
            validateattributes(pitch, {'double', 'single'}, {'real', 'scalar'}, '', 'Pitch');
            validateattributes(yaw,   {'double', 'single'}, {'real', 'scalar'}, '', 'Yaw');
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = "Interpreted execution";
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end

function R = eulToRotm(eul)
    % Converts Euler angles [pitch, yaw, roll] (XYZ convention) to a rotation matrix
    %
    % Inputs:
    %   eul - 1x3 vector of Euler angles [pitch, yaw, roll] in radians
    % Outputs:
    %   R   - 3x3 rotation matrix

    roll = eul(1);   % Rotation around Z-axis
    pitch = eul(2); % Rotation around Y-axis
    yaw = eul(3);  % Rotation around X-axis

    % Rotation matrix for yaw (Z-axis)
    Rz = [
        cos(yaw), -sin(yaw), 0;
        sin(yaw),  cos(yaw), 0;
        0,         0,        1
    ];

    % Rotation matrix for pitch (Y-axis)
    Ry = [
        cos(pitch),  0, sin(pitch);
        0,           1, 0;
        -sin(pitch), 0, cos(pitch)
    ];

    % Rotation matrix for roll (X-axis)
    Rx = [
        1, 0,          0;
        0, cos(roll), -sin(roll);
        0, sin(roll),  cos(roll)
    ];

    % Combined rotation matrix (ZYX convention)
    R = Rz * Ry * Rx;
end
