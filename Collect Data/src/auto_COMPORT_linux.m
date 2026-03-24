function STLINK_COMPORT = auto_COMPORT_linux()
    persistent savedCOMPort;
    
    % Check if the persistent variable is empty
    if isempty(savedCOMPort)
        % Get the list of available /dev/ttyACM devices
        serialPorts = dir('/dev/ttyACM*');
        serialPorts = {serialPorts.name};
        
        % Check if any /dev/ttyACM devices are found
        if isempty(serialPorts)
            error('No STM32 Nucleo found. If Nucleo is connected, you are missing drivers.');
        end
        
        % If only one device is found, select it directly
        if numel(serialPorts) == 1
            STLINK_COMPORT = ['/dev/' serialPorts{1}];
            disp(['Found STM32 device: ', STLINK_COMPORT]);
            savedCOMPort = STLINK_COMPORT;
            return;
        end
        
        % For multiple devices, query their information using udevadm
        matchingDevices = {};
        for i = 1:numel(serialPorts)
            devicePath = ['/dev/' serialPorts{i}];
            [status, cmdout] = system(['udevadm info --query=all --name=' devicePath ' | grep ID_USB_SERIAL=']);
            if status == 0 && contains(cmdout, 'STMicroelectronics_STM32_STLink')
                matchingDevices{end+1} = devicePath; % Collect matching devices
            end
        end
        
        % Check if any matching devices were found
        if numel(matchingDevices) == 1
            STLINK_COMPORT = matchingDevices{1};
            disp(['Found STM32 device: ', STLINK_COMPORT]);
            savedCOMPort = STLINK_COMPORT;
            return;
        elseif numel(matchingDevices) > 1
            % If there are multiple matching devices, ask the user to select one
            [selection, ok] = listdlg(...
                'PromptString', 'Multiple STM32 devices found. Select one:', ...
                'SelectionMode', 'single', ...
                'ListString', matchingDevices, ...
                'Name', 'Select STM32 COM Port');
            if ok
                STLINK_COMPORT = matchingDevices{selection};
                disp(['Selected STM32 device: ', STLINK_COMPORT]);
                savedCOMPort = STLINK_COMPORT;
                return;
            else
                error('No STM32 device selected. Aborting.');
            end
        end
        
        % If no matching device was found, prompt the user to select a port
        [selection, ok] = listdlg(...
            'PromptString', 'Select the ST-Link COM Port:', ...
            'SelectionMode', 'single', ...
            'ListString', serialPorts, ...
            'Name', 'Select COM Port');
        
        % Handle user response
        if ok
            STLINK_COMPORT = ['/dev/' serialPorts{selection}];
            disp(['Selected ST-Link COM Port: ', STLINK_COMPORT]);
            savedCOMPort = STLINK_COMPORT;
        else
            error('No COM port selected. Aborting.');
        end
    else
        % Use the saved persistent COM port
        STLINK_COMPORT = savedCOMPort;
        disp(['ST-Link COM Port already set to: ', STLINK_COMPORT]);
    end
end
