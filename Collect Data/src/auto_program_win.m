function auto_program_win(filename)
    drives = getdrives('-nofloppy');
    num_drives = size(drives,2);
    for i = linspace(1,num_drives,num_drives)
        drive_letter = drives{i};
        drive_name = DriveName(drive_letter(1));
        if length(drive_name)<5 % if the driver name is too short
            continue
        end
        if drive_name(end-5:end) == 'F411RE' % found the STM Drag and Drop Programming
            STLINK_DRIVE = upper(drive_letter);
            disp("Found STM32 F411RE Drive")
            copyfile('binaries/'+filename+'.bin' , upper(drive_letter));
            pause(3);
            disp("STM32 Programmed")
            STM32_ODR = 200;
            return 
        end 
        if drive_name(end-5:end) == 'F303ZE' % found the STM Drag and Drop Programming
            STLINK_DRIVE = upper(drive_letter);
            disp("Found STM32 F303ZE Drive")
            copyfile('binaries/'+filename+'_F303ZE.bin' , upper(drive_letter));
            pause(3);
            disp("STM32 Programmed")
            STM32_ODR = 200;
            return 
        end 
        
    end
    error("No STM32 Nucleo F411RE Found")
    %clearvars -except STLINK_COMPORT STLINK_DRIVE STM32_ODR
end


function drive_name = DriveName( drive_letter ) 
    cmd_str = sprintf( 'vol %s:', drive_letter );
    [~,msg] = system( cmd_str );
    cac = strsplit( msg, '\n' );
    drive_name = regexp( cac{1}, '(?<= is ).+$', 'match', 'once' );
end


function ret = getdrives(varargin)
%GETDRIVES  Get the drive letters of all mounted filesystems.
%   F = GETDRIVES returns the roots of all mounted filesystems as a cell
%   array of char arrays.
%   
%   On Windows systems, this is a list of the names of all single-letter 
%   mounted drives.
%
%   On Unix systems (including Macintosh) this is the single root
%   directory, /.
% 
%   F = GETDRIVES('-nofloppy') does not scan for the a: or b: drives on
%   Windows, which usually results in annoying grinding sounds coming from
%   the floppy drive.
%   
%   F = GETDRIVES('-twoletter') scans for both one- AND two-letter drive
%   names on Windows.  While two-letter drive names are technically supported,
%   their presence is in fact rare, and slows the search considerably.
%
%   Note that only the names of MOUNTED volumes are returned.  In 
%   particular, removable media drives that do not have the media inserted 
%   (such as an empty CD-ROM drive) are not returned.
%
%   See also EXIST, COMPUTER, UIGETFILE, UIPUTFILE.

%   Copyright 2001-2009 The MathWorks, Inc.

% Short-circut on non-Windows machines.
if ~ispc
    ret = {'/'};
    return;
end

twoletter = false;
nofloppy = false;

% Interpret optional arguments
for i = 1:nargin
    if strcmp('-nofloppy', varargin{i}), nofloppy = true; end
    if strcmp('-twoletter', varargin{i}), twoletter = true; end
end

% Initialize return cell array
ret = {};

% Handle -nofloppy flag, or lack thereof.
startletter = 'a';
if nofloppy
    startletter = 'c';
end

% Look for single-letter drives, starting at a: or c: as appropriate
for i = double(startletter):double('z')
    if exist(['' i ':\'], 'dir') == 7
        ret{end+1} = [i ':\']; %#ok<AGROW>
    end
end

end