function auto_program(filename)
    dialogPrms = get_param(gcs+"/Communicating with STM32 Nucleo", 'DialogParameters');
    dialogPrmNames = fieldnames(dialogPrms);
    automate = get_param(gcs+"/Communicating with STM32 Nucleo", dialogPrmNames{1});
    automate = eval(automate);

    if automate
        if ispc
            auto_program_win(filename)
        end
        if isunix
            auto_program_linux(filename)
        end
        if ismac
            auto_program_mac(filename)
        end
    else
        % Check and prompt to copy the file to the "F411" directory
        if exist('filename', 'var') && ~isempty(filename)
            msg = sprintf(['Please ensure the file "binaries/%s.bin" is copied to the "F411RE" USB Drive before proceeding.\n', ...
               'Also, make sure you pressed the reset button (black) if you are just re-running this model!'], filename);
            uiwait(msgbox(msg, 'Copy File Reminder', 'warn'));
        else
            error('The variable "filename" is not defined or is empty. Aborting.');
        end
    end
end