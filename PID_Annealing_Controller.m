function PID_Annealing_Controller()
% =====================================================================
%    PID Annealing Controller (v1.0 - First Published Version)
% =====================================================================
% This version fixes all major issues:
% - Non-blocking UI with continuous monitoring
% - Proper state management and error handling
% - Efficient plotting with data management
% - Robust shutdown and resource management
% - No race conditions or timer conflicts

% Use 'close all force' to prevent errors from leftover figure windows
clear; clc; close all force;

%% ======================= Configuration ===============================
PSU_COM_PORT = 'COM5';
PSU_BAUD_RATE = 115200;
PSU_CURRENT_LIMIT = 6.0;
PSU_MAX_VOLTAGE = 1.3; % Do not exceed 1.3V or DNA monomers will disassemble and crystals will not form
DAQ_DEVICE_ID = 'Dev4';
DAQ_CHANNEL = 'ai0';
DAQ_VOLTS_TO_TEMP_FACTOR = 10;
CONTROL_KP = 0.01;
CONTROL_KI = 0;
CONTROL_KD = 0.125;
TEMP_VOLTAGE_GUESS_MAP = [30.0, 55.0; 0.50, 1.25];
STEADY_STATE_DURATION_SEC = 30;
STEADY_STATE_TEMP_STDEV = 0.06;
STEADY_STATE_TEMP_TOLERANCE = 0.4;
ANNEAL_VOLTAGE_STEP = -0.005;
ANNEAL_TIME_PER_STEP_MIN = 40;
LOOP_PAUSE_DURATION = 0.5;
MAX_DATA_POINTS = 10000; % Prevent unlimited memory growth

%% ======================= Initialization ==============================
disp('--- PID Annealing Controller v1.0 ---');

% Initialize shared variables
psu = []; daqSession = []; tmr = [];
runExperiment = true;
timeData = []; tempData = []; voltageData = []; targetTempHistory = [];
figureHandle = []; ax = []; liveTempPlot = []; targetTempLine = [];
integral_error = 0;
last_measured_temp = NaN;
currentVoltage = 0;
annealStepTimer = [];
experimentStartTime = [];
currentState = 'WAITING_FOR_INITIAL_TARGET'; % New initial state
stabilityStartTime = [];
stabilityData = [];
annealMode = '';
annealEndValue = NaN;
annealStepSize = NaN;
annealStepTime = NaN;
annealModeDropdown = [];
annealEndValueInput = [];
annealStepSizeInput = [];
annealStepTimeInput = [];

% UI elements for non-blocking input
uiFigure = []; targetInput = []; actionButtons = []; statusText = [];
targetTemp = []; % Will be set via UI

% --- Continuous logging setup ---
tempLogFile = 'temp_experiment_log.csv';
logFID = fopen(tempLogFile, 'w');
if logFID == -1
    error('Could not create temporary log file.');
end
fprintf(logFID, 'Time_min,Temperature_C,Voltage_V,Target_Temp_C\n');

try
    cleanupObj = onCleanup(@shutdown);

    %% ======================= Hardware & Plot Setup =====================
    % Create main figure in fullscreen
    figureHandle = figure('Name', 'Live Temperature Monitor', 'NumberTitle', 'off', ...
                         'CloseRequestFcn', @onFigureClose, 'WindowState', 'maximized');
    
    % Create a more square graph area by using custom axes positioning
    ax = axes('Parent', figureHandle, 'Position', [0.08, 0.22, 0.84, 0.73]); hold(ax, 'on'); grid(ax, 'on');
    xlabel(ax, 'Time (minutes)'); ylabel(ax, 'Temperature (°C)');
    liveTempPlot = plot(ax, NaN, NaN, '-b', 'LineWidth', 1.5);
    targetTempLine = plot(ax, NaN, NaN, '--r', 'LineWidth', 1.5);
    legend(ax, 'Measured Temperature', 'Target Temperature', 'Location', 'southeast', 'AutoUpdate', 'off');
    title(ax, 'Waiting for initial target temperature...');
    
    % UI controls directly on the figure (not on a subplot/axes)
    % Use normalized units for responsive layout
    x0 = 0.08; y0 = 0.05; h = 0.06; pad = 0.01;
    statusWidth = 0.32; labelWidth = 0.12; inputWidth = 0.07; btnWidth = 0.10; btnHeight = h;
    
    % Status text
    statusText = uicontrol('Parent', figureHandle, 'Style', 'text', 'String', 'Please enter initial target temperature', ...
        'Units', 'normalized', 'Position', [x0, y0, statusWidth, h], 'FontSize', 11, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94, 0.94, 0.94]);
    
    % Target temperature label
    x1 = x0 + statusWidth + pad;
    uicontrol('Parent', figureHandle, 'Style', 'text', 'String', 'Target Temp (°C):', ...
        'Units', 'normalized', 'Position', [x1, y0, labelWidth, h], 'HorizontalAlignment', 'left', 'FontSize', 11, 'BackgroundColor', [0.94, 0.94, 0.94]);
    
    % Target temperature input
    x2 = x1 + labelWidth + pad;
    targetInput = uicontrol('Parent', figureHandle, 'Style', 'edit', 'String', '', ...
        'Units', 'normalized', 'Position', [x2, y0+0.01, inputWidth, h-0.02], 'Callback', @onTargetInput, 'FontSize', 11);
    
    % Set Target button
    x3 = x2 + inputWidth + pad;
    actionButtons(1) = uicontrol('Parent', figureHandle, 'Style', 'pushbutton', 'String', 'Set Target', ...
        'Units', 'normalized', 'Position', [x3, y0, btnWidth, btnHeight], 'Callback', @onSetTarget, 'FontSize', 11);
    
    % Start Annealing button
    x4 = x3 + btnWidth + pad;
    actionButtons(2) = uicontrol('Parent', figureHandle, 'Style', 'pushbutton', 'String', 'Start Annealing', ...
        'Units', 'normalized', 'Position', [x4, y0, btnWidth+0.02, btnHeight], 'Callback', @onStartAnnealing, 'FontSize', 11);
    
    % Exit button
    x5 = x4 + btnWidth + 0.02 + pad;
    actionButtons(3) = uicontrol('Parent', figureHandle, 'Style', 'pushbutton', 'String', 'Exit', ...
        'Units', 'normalized', 'Position', [x5, y0, btnWidth, btnHeight], 'Callback', @onExit, 'FontSize', 11);
    
    % Initially only enable Set Target and Exit buttons
    set(actionButtons([1,3]), 'Enable', 'on'); % Set Target and Exit always available
    set(actionButtons(2), 'Enable', 'off');    % Annealing only when stable

    % Initialize hardware
    fprintf('Connecting to Power Supply on %s...', PSU_COM_PORT);
    psu = serialport(PSU_COM_PORT, PSU_BAUD_RATE);
    configureTerminator(psu, "LF"); 
    disp('OK.');
    
    fprintf('Creating DAQ session on %s...', DAQ_DEVICE_ID);
    daqSession = daq("ni");
    addinput(daqSession, DAQ_DEVICE_ID, DAQ_CHANNEL, 'Voltage');
    daqSession.Rate = 20; 
    disp('OK.');
    
    % Initialize power supply
    sendSCPI(psu, sprintf('CURR %.2f', PSU_CURRENT_LIMIT));
    sendSCPI(psu, 'OUTP ON');

    % Get initial temperature (don't set voltage until target is provided)
    last_measured_temp = readAveragedTemperature(daqSession, DAQ_VOLTS_TO_TEMP_FACTOR);
    fprintf('Initial temperature reading: %.2f°C\n', last_measured_temp);
    
    %% ======================= Create Non-blocking UI ====================
    createControlUI();
    drawnow;
    set(figureHandle, 'Position', get(figureHandle, 'Position'));
    
    %% ======================= Timer and Main Loop ======================
    tmr = timer('ExecutionMode', 'fixedRate', 'Period', LOOP_PAUSE_DURATION, ...
                'TimerFcn', @mainLoopTick, 'ErrorFcn', @onTimerError, 'StartDelay', 1);
    
    experimentStartTime = tic;
    start(tmr);
    disp(' '); disp('INFO: Control loop timer started. System is now active.');
    
    % Main loop - much simpler now
    while runExperiment
        pause(0.1); % Short pause to allow UI updates
    end
    
    disp('Main loop exited. Initiating shutdown...');

catch ME
    if strcmp(ME.identifier, 'PIDCONTROL:WindowClosed')
        disp(ME.message);
    else
        fprintf('\n!!! An unexpected error occurred. Shutting down. !!!\n');
        rethrow(ME);
    end
finally
    shutdown();
end

%% ======================= Nested Functions ============================
    function createControlUI()
        % Delete any existing controls to prevent overlap/duplication
        delete(findall(figureHandle, 'Type', 'uicontrol'));
        
        % Two-row layout for control panel
        y0 = 0.09; h = 0.06; pad = 0.01;
        statusWidth = 0.22; labelWidth = 0.09; inputWidth = 0.06; btnWidth = 0.09; btnHeight = h;
        row2y = y0 - h - 0.01;
        
        % Row 1: Status, Target Temp, Set Target, Exit
        x0 = 0.02;
        statusText = uicontrol('Parent', figureHandle, 'Style', 'text', 'String', 'Please enter initial target temperature', ...
            'Units', 'normalized', 'Position', [x0, y0, statusWidth, h], 'FontSize', 11, 'HorizontalAlignment', 'left', 'BackgroundColor', [0.94, 0.94, 0.94]);
        x1 = x0 + statusWidth + pad;
        uicontrol('Parent', figureHandle, 'Style', 'text', 'String', 'Target Temp (°C):', ...
            'Units', 'normalized', 'Position', [x1, y0, labelWidth, h], 'HorizontalAlignment', 'left', 'FontSize', 11, 'BackgroundColor', [0.94, 0.94, 0.94]);
        x2 = x1 + labelWidth + pad;
        targetInput = uicontrol('Parent', figureHandle, 'Style', 'edit', 'String', '', ...
            'Units', 'normalized', 'Position', [x2, y0+0.01, inputWidth, h-0.02], 'Callback', @onTargetInput, 'FontSize', 11);
        x3 = x2 + inputWidth + pad;
        actionButtons(1) = uicontrol('Parent', figureHandle, 'Style', 'pushbutton', 'String', 'Set Target', ...
            'Units', 'normalized', 'Position', [x3, y0, btnWidth, btnHeight], 'Callback', @onSetTarget, 'FontSize', 11);
        x4 = x3 + btnWidth + pad;
        actionButtons(3) = uicontrol('Parent', figureHandle, 'Style', 'pushbutton', 'String', 'Exit', ...
            'Units', 'normalized', 'Position', [x4, y0, btnWidth, btnHeight], 'Callback', @onExit, 'FontSize', 11);
        
        % Row 2: Annealing settings
        xA = 0.02;
        uicontrol('Parent', figureHandle, 'Style', 'text', 'String', 'End Mode:', ...
            'Units', 'normalized', 'Position', [xA, row2y, labelWidth, h], 'HorizontalAlignment', 'left', 'FontSize', 11, 'BackgroundColor', [0.94, 0.94, 0.94]);
        xB = xA + labelWidth + pad;
        annealModeDropdown = uicontrol('Parent', figureHandle, 'Style', 'popupmenu', 'String', {'Temperature','Voltage'}, ...
            'Units', 'normalized', 'Position', [xB, row2y+0.01, inputWidth, h-0.02], 'FontSize', 11);
        xC = xB + inputWidth + pad;
        uicontrol('Parent', figureHandle, 'Style', 'text', 'String', 'End Value:', ...
            'Units', 'normalized', 'Position', [xC, row2y, labelWidth, h], 'HorizontalAlignment', 'left', 'FontSize', 11, 'BackgroundColor', [0.94, 0.94, 0.94]);
        xD = xC + labelWidth + pad;
        annealEndValueInput = uicontrol('Parent', figureHandle, 'Style', 'edit', 'String', '', ...
            'Units', 'normalized', 'Position', [xD, row2y+0.01, inputWidth, h-0.02], 'FontSize', 11);
        xE = xD + inputWidth + pad;
        uicontrol('Parent', figureHandle, 'Style', 'text', 'String', 'Step Size (V):', ...
            'Units', 'normalized', 'Position', [xE, row2y, labelWidth, h], 'HorizontalAlignment', 'left', 'FontSize', 11, 'BackgroundColor', [0.94, 0.94, 0.94]);
        xF = xE + labelWidth + pad;
        annealStepSizeInput = uicontrol('Parent', figureHandle, 'Style', 'edit', 'String', '-0.005', ...
            'Units', 'normalized', 'Position', [xF, row2y+0.01, inputWidth, h-0.02], 'FontSize', 11, ...
            'TooltipString', 'Positive = increase voltage, Negative = decrease voltage');
        xG = xF + inputWidth + pad;
        uicontrol('Parent', figureHandle, 'Style', 'text', 'String', 'Step Time (min):', ...
            'Units', 'normalized', 'Position', [xG, row2y, labelWidth, h], 'HorizontalAlignment', 'left', 'FontSize', 11, 'BackgroundColor', [0.94, 0.94, 0.94]);
        xH = xG + labelWidth + pad;
        annealStepTimeInput = uicontrol('Parent', figureHandle, 'Style', 'edit', 'String', '40', ...
            'Units', 'normalized', 'Position', [xH, row2y+0.01, inputWidth, h-0.02], 'FontSize', 11);
        xI = xH + inputWidth + pad;
        actionButtons(2) = uicontrol('Parent', figureHandle, 'Style', 'pushbutton', 'String', 'Start Annealing', ...
            'Units', 'normalized', 'Position', [xI, row2y, btnWidth+0.02, btnHeight], 'Callback', @onStartAnnealing, 'FontSize', 11);
        
        % Initially only enable Set Target and Exit buttons
        set(actionButtons([1,3]), 'Enable', 'on'); % Set Target and Exit always available
        set(actionButtons(2), 'Enable', 'off');    % Annealing only when stable
    end

    function mainLoopTick(~, ~)
        try
            if ~runExperiment || ~ishandle(figureHandle), return; end
            
            % Check if we have a valid target temperature
            if isempty(targetTemp) || isnan(targetTemp)
                set(statusText, 'String', 'Please enter a valid target temperature');
                return;
            end
            
            % Read temperature and update data
            currentTemp = readAveragedTemperature(daqSession, DAQ_VOLTS_TO_TEMP_FACTOR);
            if isnan(currentTemp)
                set(statusText, 'String', 'Error: Cannot read temperature');
                return;
            end
            
            elapsedTimeMin = toc(experimentStartTime) / 60;
            
            % Manage data arrays to prevent unlimited growth
            if length(timeData) >= MAX_DATA_POINTS
                % Keep only the most recent data points
                keepPoints = MAX_DATA_POINTS / 2;
                timeData = timeData(end-keepPoints+1:end);
                tempData = tempData(end-keepPoints+1:end);
                voltageData = voltageData(end-keepPoints+1:end);
                targetTempHistory = targetTempHistory(end-keepPoints+1:end);
            end
            
            % Add new data points - ensure arrays are initialized properly
            if isempty(timeData)
                timeData = elapsedTimeMin;
                tempData = currentTemp;
                targetTempHistory = targetTemp;
                voltageData = currentVoltage;
            else
                timeData(end+1) = elapsedTimeMin; 
                tempData(end+1) = currentTemp;
                targetTempHistory(end+1) = targetTemp; 
                voltageData(end+1) = currentVoltage;
            end
            
            % --- Append to temp log file ---
            if exist('logFID', 'var') && logFID > 0
                fprintf(logFID, '%.4f,%.4f,%.4f,%.4f\n', elapsedTimeMin, currentTemp, currentVoltage, targetTemp);
            end
            
            % Update plot efficiently
            updatePlot(figureHandle, liveTempPlot, targetTempLine, timeData, tempData, targetTempHistory);
            
            % PID control calculations
            dt = tmr.Period;
            error = targetTemp - currentTemp;

            if any(strcmp(currentState, {'FINDING_TEMPERATURE', 'CHECKING_STABILITY', 'STABLE'}))
                % Only update integral if not saturated
                [adjustment, temp_integral] = calculatePIDAdjustment(error, currentTemp, last_measured_temp, integral_error, dt, CONTROL_KP, CONTROL_KI, CONTROL_KD);
                newVoltage = currentVoltage + adjustment;
                
                % Anti-windup: Only update integral if not saturated
                if newVoltage > 0 && newVoltage < PSU_MAX_VOLTAGE
                    integral_error = temp_integral;
                end
                
                % Clamp voltage
                if newVoltage >= PSU_MAX_VOLTAGE
                    newVoltage = PSU_MAX_VOLTAGE;
                end
                if newVoltage <= 0
                    newVoltage = 0;
                end
                sendSCPI(psu, sprintf('VOLT %.4f', newVoltage));
                currentVoltage = newVoltage;
            end
            
            % State machine logic
            switch currentState
                case 'WAITING_FOR_INITIAL_TARGET'
                    title(ax, sprintf('Waiting for target temperature... (Current: %.2f°C)', currentTemp));
                    set(statusText, 'String', sprintf('Current temperature: %.2f°C - Please set target', currentTemp));
                    % Don't do any control until target is set
                    
                case 'FINDING_TEMPERATURE'
                    title(ax, sprintf('PID Control to Target: %.2f°C (Current: %.2f°C)', targetTemp, currentTemp));
                    set(statusText, 'String', sprintf('Seeking target: %.2f°C (Current: %.2f°C)', targetTemp, currentTemp));
                    
                    if abs(error) < STEADY_STATE_TEMP_TOLERANCE
                        currentState = 'CHECKING_STABILITY';
                        stabilityStartTime = tic;
                        stabilityData = [];
                    end

                case 'CHECKING_STABILITY'
                    title(ax, sprintf('Checking Stability at %.2f°C...', targetTemp));
                    set(statusText, 'String', sprintf('Checking stability: %.2f°C', currentTemp));
                    
                    % Collect stability data
                    stabilityData(end+1) = currentTemp;
                    
                    if toc(stabilityStartTime) >= STEADY_STATE_DURATION_SEC
                        isSteady = checkStability(stabilityData, STEADY_STATE_TEMP_STDEV);
                    if isSteady
                            currentState = 'STABLE';
                            set(statusText, 'String', sprintf('STABLE at %.2f°C - Ready for commands', targetTemp));
                            set(actionButtons(2), 'Enable', 'on'); % Enable annealing button
                        else
                            currentState = 'FINDING_TEMPERATURE';
                        end
                    elseif abs(error) >= STEADY_STATE_TEMP_TOLERANCE
                        currentState = 'FINDING_TEMPERATURE';
                    end
                    
                case 'STABLE'
                    title(ax, sprintf('Stable at %.2f°C (Current: %.2f°C)', targetTemp, currentTemp));
                    % Keep monitoring but don't change state unless temperature drifts
                    if abs(error) >= STEADY_STATE_TEMP_TOLERANCE
                        currentState = 'FINDING_TEMPERATURE';
                        set(actionButtons(2), 'Enable', 'off'); % Disable annealing button
                    end
                    
                case 'ANNEALING'
                    % Round voltages to four decimal places for robust comparison
                    currentVoltage = round(currentVoltage, 4);
                    annealEndValue_rounded = round(annealEndValue, 4);
                    title(ax, sprintf('Annealing to %s %.2f... (Current: %.2f°C, %.4fV)', ...
                        upper(annealMode), annealEndValue, currentTemp, currentVoltage));
                    set(statusText, 'String', sprintf('Annealing: %.2f°C, %.4fV -> %s %.2f', ...
                        currentTemp, currentVoltage, upper(annealMode), annealEndValue));
                    tempTolerance = 0.1;
                    % Check end condition FIRST and return if met
                    if strcmp(annealMode, 'temp')
                        if (annealStepSize < 0 && currentTemp < (annealEndValue - tempTolerance)) || ...
                           (annealStepSize > 0 && currentTemp > (annealEndValue + tempTolerance))
                            sendSCPI(psu, 'VOLT 0');
                            currentVoltage = 0;
                            currentState = 'IDLE_AFTER_ANNEAL';
                            set(statusText, 'String', 'Annealing complete. Voltage set to 0. Waiting for new target.');
                            set(actionButtons(2), 'Enable', 'off');
                            return;
                        end
                    elseif strcmp(annealMode, 'volt')
                        if (annealStepSize < 0 && currentVoltage < annealEndValue_rounded) || ...
                           (annealStepSize > 0 && currentVoltage > annealEndValue_rounded)
                            sendSCPI(psu, 'VOLT 0');
                            currentVoltage = 0;
                            currentState = 'IDLE_AFTER_ANNEAL';
                            set(statusText, 'String', 'Annealing complete. Voltage set to 0. Waiting for new target.');
                            set(actionButtons(2), 'Enable', 'off');
                            return;
                        end
                    end
                    % Only step if not at end
                    if toc(annealStepTimer) >= (annealStepTime * 60)
                        newVoltage = round(currentVoltage + annealStepSize, 4);
                        sendSCPI(psu, sprintf('VOLT %.4f', newVoltage));
                        currentVoltage = newVoltage; 
                        annealStepTimer = tic;
                    end
                case 'IDLE_AFTER_ANNEAL'
                    title(ax, 'Idle: Annealing complete. Voltage = 0.');
                    set(statusText, 'String', 'Annealing complete. Voltage set to 0. Set a new target to resume.');
                    % Do nothing else in this state
            end
            last_measured_temp = currentTemp;
            
        catch ME
            fprintf('\nERROR IN TIMER CALLBACK: %s\n', ME.message);
            runExperiment = false;
        end
    end

    % UI Callback functions
    function onTargetInput(~, ~)
        % Validate input as user types
        inputStr = get(targetInput, 'String');
        if ~isempty(inputStr)
            temp = str2double(inputStr);
            if isnan(temp) || temp < 0
                set(targetInput, 'String', '');
            end
        end
    end

    function onSetTarget(~, ~)
        inputStr = get(targetInput, 'String');
        if ~isempty(inputStr)
            newTarget = str2double(inputStr);
            if ~isnan(newTarget) && newTarget >= 0
                targetTemp = newTarget;
                integral_error = 0; % Reset integral term
                
                % If this is the first target, initialize voltage and start control
                if strcmp(currentState, 'WAITING_FOR_INITIAL_TARGET') || strcmp(currentState, 'IDLE_AFTER_ANNEAL')
                    % Set initial voltage based on target
                    vGuess = interp1(TEMP_VOLTAGE_GUESS_MAP(1,:), TEMP_VOLTAGE_GUESS_MAP(2,:), targetTemp, 'linear', 'extrap');
                    currentVoltage = max(0, min(vGuess, PSU_MAX_VOLTAGE));
                    sendSCPI(psu, sprintf('VOLT %.4f', currentVoltage));
                    
                    currentState = 'FINDING_TEMPERATURE';
                    set(statusText, 'String', sprintf('Initial target set: %.2f°C - Starting control', targetTemp));
                    fprintf('Initial target temperature set to: %.2f°C\n', targetTemp);
                else
                currentState = 'FINDING_TEMPERATURE';
                    set(statusText, 'String', sprintf('New target set: %.2f°C', targetTemp));
                    fprintf('New target temperature set to: %.2f°C\n', targetTemp);
                end
                
                set(actionButtons(2), 'Enable', 'off'); % Disable annealing button
                set(targetInput, 'String', '');
            else
                set(targetInput, 'String', '');
                set(statusText, 'String', 'Invalid temperature input');
            end
                    end
                end

    function onStartAnnealing(~, ~)
        % Read annealing settings from the control panel
        modeIdx = get(annealModeDropdown, 'Value');
        if modeIdx == 1
            modeStr = 'temp';
        else
            modeStr = 'volt';
        end
        endValue = str2double(get(annealEndValueInput, 'String'));
        stepSize = str2double(get(annealStepSizeInput, 'String'));
        stepTime = str2double(get(annealStepTimeInput, 'String'));
        currentTemp = tempData(end);
        currentVolt = currentVoltage;
        
        if isnan(endValue) || isnan(stepSize) || isnan(stepTime) || stepTime <= 0
            set(statusText, 'String', 'Invalid annealing settings.');
            return;
        end
        
        annealMode = modeStr;
        annealEndValue = endValue;
        annealStepSize = stepSize;
        annealStepTime = stepTime;
        
        annealStepTimer = tic;
                currentState = 'ANNEALING';
        set(actionButtons(2), 'Enable', 'off');
        set(statusText, 'String', sprintf('Annealing to %s %.3f (%+.4f V/%.1f min)', ...
            upper(annealMode), annealEndValue, annealStepSize, annealStepTime));
        fprintf('Annealing started: mode=%s, end=%.3f, step=%+.4f V, time=%.1f min\n', ...
            annealMode, annealEndValue, annealStepSize, annealStepTime);
    end

    function onExit(~, ~)
        % Ask user if they want to save data
        choice = questdlg('Do you want to save the data before exiting?', 'Save Data', 'Yes', 'No', 'Yes');
        if strcmp(choice, 'Yes')
            % Ask for file location (CSV)
            [csvFile, csvPath] = uiputfile({'*.csv','CSV Files (*.csv)'}, 'Save Data As');
            if ischar(csvFile)
                % Close the temp log file first
                if exist('logFID', 'var') && logFID > 0
                    fclose(logFID);
                    logFID = -1;
                end
                % Move/rename the temp file to the chosen location
                movefile(tempLogFile, fullfile(csvPath, csvFile));
                % Save graph as PNG
                [~, baseName, ~] = fileparts(csvFile);
                pngFile = fullfile(csvPath, [baseName, '.png']);
                exportgraphics(ax, pngFile, 'Resolution', 300);
            end
        else
            % User chose not to save: close and delete temp log file
            if exist('logFID', 'var') && logFID > 0
                fclose(logFID);
                logFID = -1;
            end
            if exist(tempLogFile, 'file')
                delete(tempLogFile);
            end
        end
        runExperiment = false;
        % Force immediate shutdown
        if ~isempty(tmr) && isvalid(tmr)
            stop(tmr);
            delete(tmr);
            tmr = [];
        end
        if ~isempty(psu) && isvalid(psu)
            try
                sendSCPI(psu, 'OUTP OFF');
                clear psu;
            catch
                % Ignore errors during shutdown
            end
        end
        if ~isempty(daqSession) && isvalid(daqSession)
            clear daqSession;
        end
        if ishandle(figureHandle)
            delete(figureHandle);
        end
        disp('Exit button pressed. Shutting down...');
    end

    function onFigureClose(~, ~)
        error('PIDCONTROL:WindowClosed', 'Plot window closed by user. Forcing shutdown.');
    end
    
    function onTimerError(~, event)
        fprintf('\n!!! An error occurred within the timer: %s. Shutting down. !!!\n', event.Data.message);
        runExperiment = false;
    end
    
    function shutdown()
        persistent alreadyShuttingDown;
        if ~isempty(alreadyShuttingDown) && alreadyShuttingDown
            return;
        end
        alreadyShuttingDown = true;
        disp('--- Initiating Shutdown Sequence ---');
        
        % Stop timer first
        if exist('tmr','var') && ~isempty(tmr) && isvalid(tmr)
            stop(tmr); 
            delete(tmr); 
            tmr = []; 
            disp('Timer stopped and deleted.');
        end
        
        % Turn off power supply
        if exist('psu','var') && ~isempty(psu) && isvalid(psu)
            disp('Turning off PSU...'); 
            try
                sendSCPI(psu, 'OUTP OFF');
                clear psu;
            catch
                disp('Warning: Could not turn off PSU cleanly');
            end
        end
        
        % Stop DAQ
        if exist('daqSession','var') && ~isempty(daqSession) && isvalid(daqSession)
            disp('Stopping DAQ...'); 
            clear daqSession;
        end
        
        % Save data
        if exist('timeData','var') && ~isempty(timeData) && length(timeData) > 1
            disp('Saving data...');
            min_len = min([length(timeData), length(tempData), length(voltageData), length(targetTempHistory)]);
            t_s = timeData(1:min_len); 
            temp_s = tempData(1:min_len); 
            v_s = voltageData(1:min_len); 
            target_s = targetTempHistory(1:min_len);
            
            filename = sprintf('ExperimentData_PID_%s.mat', datestr(now, 'yyyy-mm-dd_HHMMSS'));
            dataTable = table(t_s', temp_s', v_s', target_s', ...
                             'VariableNames', {'Time_min', 'Temperature_C', 'Voltage_V', 'Target_Temp_C'});
            save(filename, 'dataTable');
            disp(['Data saved to: ' filename]);
        end
        
        % Close figure
        if exist('figureHandle','var') && ishandle(figureHandle)
            delete(figureHandle);
        end
        
        % --- Close and delete temp log file if still open ---
        if exist('logFID', 'var') && logFID > 0
            fclose(logFID);
            logFID = -1;
        end
        if exist('tempLogFile', 'var') && exist(tempLogFile, 'file')
            delete(tempLogFile);
        end
        
        disp('--- Shutdown Complete ---');
        if exist('runExperiment','var')
            runExperiment = false;
        end
    end
end

%% ======================= Helper Functions ============================
function sendSCPI(psu, command)
    if ~isempty(psu) && isvalid(psu)
        try
            writeline(psu, command); 
            pause(0.05); % Reduced delay
        catch
            fprintf('Warning: SCPI command failed: %s\n', command);
        end
    end
end

function avgTemp = readAveragedTemperature(daqSession, conversionFactor)
    try
        flush(daqSession); 
        data = read(daqSession, 20); 
        avgTemp = mean(data.Variables) * conversionFactor;
    catch
        fprintf('Warning: Temperature reading failed\n');
        avgTemp = NaN;
    end
end

function isStable = checkStability(tempData, threshold)
    if length(tempData) < 10
    isStable = false;
        return;
    end
    stdev_val = std(tempData);
    isStable = stdev_val < threshold;
end

function updatePlot(fig, tempPlot, targetLine, t, temp, target)
    if ~ishandle(fig), return; end
    try
    set(tempPlot, 'XData', t, 'YData', temp);
    set(targetLine, 'XData', t, 'YData', target);
        
        % Auto-resize the plot to show all data
        if ~isempty(t) && length(t) > 1
            ax = get(tempPlot, 'Parent');
            xlim(ax, [min(t), max(t)]);
            
            % Add some padding to y-axis
            allTemps = [temp, target];
            if ~isempty(allTemps)
                tempRange = max(allTemps) - min(allTemps);
                if tempRange > 0
                    ylim(ax, [min(allTemps) - tempRange*0.1, max(allTemps) + tempRange*0.1]);
                end
            end
        end
        
    drawnow('limitrate');
    catch
        % Silently handle plot update errors
    end
end

function [adjustment, new_integral] = calculatePIDAdjustment(error, current_temp, last_temp, integral, dt, Kp, Ki, Kd)
    if isnan(last_temp), last_temp = current_temp; end
    
    % Proportional term
    p_term = Kp * error;
    
    % Integral term with anti-windup
    new_integral = integral + (error * dt);
    i_term = Ki * new_integral;
    
    % Derivative term with filtering
    derivative = (current_temp - last_temp) / dt;
    d_term = -Kd * derivative;
    
    % Combine terms
    adjustment = p_term + i_term + d_term;
end