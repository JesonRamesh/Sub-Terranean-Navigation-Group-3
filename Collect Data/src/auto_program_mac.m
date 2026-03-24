function auto_program_mac(filename)
    % Get connected drives and their details
    drives = get_mac_drives();

    % Iterate through the drives
    for i = 1:length(drives)
        drive_name = drives{i}.name;

        % Extract the first part of the drive name
        split_name = strsplit(drive_name);
        cleaned_drive_name = split_name{1};

        % Check if the drive name contains 'F411RE'
        if contains(cleaned_drive_name, 'F411RE')
            disp("Found STM32 F411RE Drive: " + cleaned_drive_name);

            % Copy the binary file to the drive
            drive_path = "/Volumes/"+drives{i}.name
            file_path = "binaries/" +filename +".bin"
            copyfile(file_path, drive_path);

            pause(3); % Pause to ensure the file is copied
            disp("STM32 Programmed");
            return;
        end
        if contains(cleaned_drive_name, 'F303ZE')
            disp("Found STM32 F303ZE Drive: " + cleaned_drive_name);

            % Copy the binary file to the drive
            drive_path = "/Volumes/"+drives{i}.name
            file_path = "binaries/" +filename +"_F303ZE.bin"
            copyfile(file_path, drive_path);

            pause(3); % Pause to ensure the file is copied
            disp("STM32 Programmed");
            return;
        end
    end

    % If no STM32 drive is found, throw an error
    error("No STM32 Nucleo F411RE Found");
end

function drives = get_mac_drives()
    % Executes macOS command to list all drives and extract relevant details
    [status, cmdout] = system('diskutil list');

    if status ~= 0
        error("Failed to fetch drive information");
    end

    % Parse the command output
    drives = {};
    lines = strsplit(cmdout, '\n');
    currentDrive = struct('name', '', 'identifier', '');

    for i = 1:length(lines)
        line = strtrim(lines{i});
        if isempty(line)
            continue;
        end

        % Look for entries like '/dev/diskX'
        if startsWith(line, '/dev/disk')
            % If a previous drive is in progress, save it
            if ~isempty(currentDrive.name) && ~isempty(currentDrive.identifier)
                drives{end+1} = currentDrive;
                currentDrive = struct('name', '', 'identifier', '');
            end
            % Extract identifier from '/dev/diskX'
            parts = strsplit(line);
            currentDrive.identifier = parts{1};
        elseif contains(line, 'NOD_F411RE')
            % Extract the name if it matches 'NOD_F411RE'
            parts = strsplit(line);
            currentDrive.name = strjoin(parts(2)); % Combine name parts
        end
    end

    % Add the last drive, if valid
    if ~isempty(currentDrive.name) && ~isempty(currentDrive.identifier)
        drives{end+1} = currentDrive;
    end
end
