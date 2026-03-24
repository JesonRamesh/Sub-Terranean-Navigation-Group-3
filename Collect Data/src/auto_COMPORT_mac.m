function STLINK_COMPORT = auto_COMPORT_mac()
    % auto_COMPORT_mac - Automatically detects and selects a USB modem COM port on macOS.
    % 
    % If one USB modem is found, it is selected automatically.
    % If multiple USB modems are found, the user is prompted to select one.
    % If none are found, an error is raised.
    %
    % Output:
    %   selectedPort - The selected USB modem COM port (e.g., '/dev/tty.usbmodem12345').
    
    % Use system command to list all tty.usbmodem devices
    [status, cmdout] = system('ls /dev/tty.usbmodem* 2>/dev/null');
    
    % Check if the command was successful and there are results
    if status ~= 0 || isempty(cmdout)
        error('No USB modem devices found.');
    end
    
    % Parse the output into individual lines
    matchingDevices = strsplit(strtrim(cmdout), '\n');
    
    % Check the number of matching devices found
    if numel(matchingDevices) == 1
        % If exactly one device is found, select it automatically
        STLINK_COMPORT = matchingDevices{1};
        disp(['Found USB modem: ', STLINK_COMPORT]);
    elseif numel(matchingDevices) > 1
        % If multiple devices are found, prompt the user to select one
        [selection, ok] = listdlg(...
            'PromptString', 'Multiple USB modems found. Select one:', ...
            'SelectionMode', 'single', ...
            'ListString', matchingDevices, ...
            'Name', 'Select USB Modem');
        
        % Handle user response
        if ok
            STLINK_COMPORT = matchingDevices{selection};
            disp(['Selected USB modem: ', STLINK_COMPORT]);
        else
            error('No USB modem selected. Aborting.');
        end
    else
        error('Unexpected error while detecting USB modems.');
    end
end
