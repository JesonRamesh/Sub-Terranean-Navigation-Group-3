function STLINK_COMPORT = auto_COMPORT()
    persistent savedCOMPort;
    dialogPrms = get_param(gcs+"/Communicating with STM32 Nucleo", 'DialogParameters');
    dialogPrmNames = fieldnames(dialogPrms);
    automate = get_param(gcs+"/Communicating with STM32 Nucleo", dialogPrmNames{1});
    automate = eval(automate);

    if automate
        if ispc
            STLINK_COMPORT = auto_COMPORT_win();
        end
        if isunix
            STLINK_COMPORT = auto_COMPORT_linux();
        end
        if ismac
            STLINK_COMPORT = auto_COMPORT_mac();
        end
    elseif isempty(savedCOMPort)
        serialPorts = serialportlist;

% Create a figure for the dropdown menu
        fig = uifigure('Name', 'Select COM Port', 'Position', [500, 500, 300, 150]);
        
        % Use HTML-style formatting for line breaks
        uilabel(fig, ...
            'Text', 'Select the ST-Link COM Port:', ...
            'Position', [20, 100, 260, 50], ...
            'HorizontalAlignment', 'center', ...
            'Interpreter', 'html'); % Enable HTML interpreter for line breaks

        % Create the dropdown menu
        dropdown = uidropdown(fig, ...
            'Items', serialPorts, ...
            'Position', [20, 60, 260, 22], ...
            'Value', serialPorts{1});

        % Create an OK button
        btn = uibutton(fig, ...
            'Text', 'OK', ...
            'Position', [100, 20, 100, 30], ...
            'ButtonPushedFcn', @(btn, event) uiresume(fig));

        % Wait for user to select
        uiwait(fig);

        % Retrieve the selected COM port
        STLINK_COMPORT = dropdown.Value;
        disp(['Selected ST-Link COM Port: ', STLINK_COMPORT]);

        % Save the selected COM port in the persistent variable
        savedCOMPort = STLINK_COMPORT;

        % Close the figure
        delete(fig);

    % If STLINK_COMPORT already exists in the base workspace and is not empty, return it
    else
        STLINK_COMPORT = savedCOMPort;
        disp(['ST-Link COM Port already set to: ', STLINK_COMPORT, ', if this is incorrect, delete the STLINK_COMPORT Workspace variable']);
        return;
    end
end