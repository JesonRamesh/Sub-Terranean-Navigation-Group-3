function STLINK_COMPORT = auto_COMPORT()
    Skey = 'HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM';
    % Find connected serial devices and clean up the output
    [~, list] = dos(['REG QUERY ' Skey]);
    list = strread(list,'%s','delimiter',' ');
    coms = 0;
    for i = 1:numel(list)
      if strcmp(list{i}(1:3),'COM')
          if ~iscell(coms)
              coms = list(i);
          else
              coms{end+1} = list{i};
          end
      end
    end
    key = 'HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB\';
    % Find all installed USB devices entries and clean up the output
    [~, vals] = dos(['REG QUERY ' key ' /s /f "FriendlyName" /t "REG_SZ"']);
    vals = textscan(vals,'%s','delimiter','\t');
    vals = cat(1,vals{:});
    out = 0;
    % Find all friendly name property entries
    for i = 1:numel(vals)
      if strcmp(vals{i}(1:min(12,end)),'FriendlyName')
          if ~iscell(out)
              out = vals(i);
          else
              out{end+1} = vals{i};
          end
      end
    end
    % Compare friendly name entries with connected ports and generate output
    for i = 1:numel(coms)
      match = strfind(out,[coms{i},')']);
      ind = 0;
      for j = 1:numel(match)
          if ~isempty(match{j})
              ind = j;
          end
      end
      if ind ~= 0
          com = str2double(coms{i}(4:end));
    % Trim the trailing ' (COM##)' from the friendly name - works on ports from 1 to 99
          if com > 9
              length = 8;
          else
              length = 7;
          end
          devs{i,1} = out{ind}(27:end-length);
          devs{i,2} = "COM"+string(com);
      end
    end

    STLINK_COMPORT = "NULL";
    if size(devs,1) == 1 % only nucleo plugged in
        STLINK_COMPORT = devs{1,2};
    else
        for i = size(devs,1) % checks for nucleo specific comport (needs drivers)
            if devs{i,1} == 'STMicroelectronics STLink Virtual COM Port';
                STLINK_COMPORT = devs{i,2};
            end
        end
        if STLINK_COMPORT == "NULL";
            for i = size(devs,1) % checks for generic comport (assumes others are specifc)
                if devs{i,1} == 'USB Serial Device';
                    STLINK_COMPORT = devs{i,2};
                end
            end
        end
    end
end