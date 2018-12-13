function [ ] = rescaleGainsForFirmwareV10( familyName )
%RESCALEGAINSFORFIRMWAREV10 - Updates the gains on all the modules with a
%certain family name to be scaled appropriately after updating them to 
%version 10 of the firmware.
%
% rescaleGainsForFirmwareV10( familyName )
%
% The function takes in a string 'familyName', that is used to make a group
% using the HebiLookup.newGroupFromFamily().  If you want set the gains on
% all the modules currently on the network, you can set the family name to
% the wild-card character '*'.
%
% This scales the Ki gains up by 1000X and scales the Kd gains down by 
% 1000X, to handle the conversion from milliseconds to seconds.  There is
% no need to change any other gains when updating to firmware v10.
% 
%
% There are some checks to see if the firmware is the right version, or if 
% gains have possibly been set already.
%
% Dave Rollinson
% Sep 2017

    % Clear the module list in case there are any stale or disconnected
    % modules in the lookup table.  Pause a little afterwards to let the
    % table rebuild.
    HebiLookup.clearModuleList();
    pause(0.25);
    
    fprintf('\nMaking group...');

    % Make a new group based on the family name
    group = HebiLookup.newGroupFromFamily( familyName );
    
    % Gains structure used for just changing the control strategy
    gains = GainStruct();
    
    % Get the gains that are currently active.  We will use this to set the
    % control strategy back to its original state when we're done.
    originalGains = group.getGains();
    
    numModules = group.getNumModules;
    oldFirmwareDetected = false(1,numModules);
    
    fprintf(['Found ' num2str(numModules) ' module(s).\n']);
    
    % Get info on the group.  Make sure all the modules in the group are
    % running version 10 of the firmware.
    groupInfo = group.getInfo;
    firmwareRev = groupInfo.firmwareRevision;
    moduleName = groupInfo.name;

    for i=1:numModules
        if ~strcmp(firmwareRev{i}(1:3),'10.')
            disp(['WARNING: Module "' moduleName{i} '" is not running latest firmware.']);
            oldFirmwareDetected(i) = true;
        end
    end
    
    % Only proceed if all the modules are running v10
    if any(oldFirmwareDetected)
        disp(' ');
        disp('MODULES HAVE NOT BEEN UPDATED.');
        disp('At least one module in the group was not running firmware version 10.');
        disp('Please update all modules to the latest firmware version using Scope.');
        disp(' ');
        disp('You can download Scope at:');
        disp('http://docs.hebi.us/#downloads/');
        return;
    end
    
    % Iterate thru for control strategies 2, 3, and 4.
    for controlStrategy = 2:4
        
        fprintf( [ '\nRe-scaling Ki and Kd gains for Strategy ' ...
                        num2str(controlStrategy) '...' ] );
        
        % Set the control strategy, so we can get the gains.
        gains.controlStrategy = controlStrategy * ones(1,numModules);
        group.send('gains',gains);
        
        % Pause long enough for the gain info to update.  
        % (Updates based on lookupFrequency, default of 5 Hz)
        pause(0.25);
        
        % Get the gains for the group
        groupGains = group.getGains();
        
        % Check to see if there are any large Kd gains
        % (Indicates that gains DO need to be changed)
        checkKd = groupGains.positionKd > .002 | ...
                  groupGains.velocityKd > .002 | ...
                  groupGains.effortKd > .002;
        
        % Check to see if there are any large Ki gains
        % (Indicates that gains DO NOT need to be changed)
        checkKi = groupGains.positionKi > .02 | ...
                  groupGains.velocityKi > .02 | ...
                  groupGains.effortKi > .02;
        
        if any(checkKi)
            fprintf('Skipping this strategy.\n');
            disp('  Ki gains on some modules seem to have already been re-scaled.');
        elseif any(checkKd)
            % Rescale the Ki and Kd terms for each PID controller
            groupGains.positionKi = 1000 * groupGains.positionKi;
            groupGains.velocityKi = 1000 * groupGains.velocityKi;
            groupGains.effortKi = 1000 * groupGains.effortKi;

            groupGains.positionKd = .001 * groupGains.positionKd;
            groupGains.velocityKd = .001 * groupGains.velocityKd;
            groupGains.effortKd = .001 * groupGains.effortKd;

            % Send the gains
            group.send('gains',groupGains,'led','w','persist',true);
            fprintf('DONE!');
        else
            fprintf('Skipping this strategy.\n');
            disp('  Kd gains all modules seem to have already been re-scaled, or are all zero.');
        end
    end
    
    % Change the control strategy back to what they were originally
    gains.controlStrategy = originalGains.controlStrategy;
    
    % Persist the gains so that they remain after restart
    fprintf('\nPersisting the gains...');
    group.send( 'gains', gains, 'led', [], 'persist', true );
    fprintf('DONE!\n\n');
end
