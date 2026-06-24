function OpenGRF_GUI()

% Single-window GUI for OpenGRF input settings.
%
% Put this file and run_OpenGRF_from_config.m in the same folder as the
% original OpenGRF helper functions, then run:
%
% The GUI collects the OpenGRF inputs in one window.
% Author: Andrea Di Pietro, 2026
app = struct();
app.ModelPath = "";
app.IKPath = "";
app.BodyNames = strings(0,1);
app.Motion = [];
app.HeadMotion = [];
app.Fs = NaN;
advancedVisible = false;
contentHeightCollapsed = 400;
contentHeightExpanded = 700;

fig = uifigure('Name','OpenGRF Input GUI', ...
    'Position',[100 100 800 600], ...
    'CloseRequestFcn',@(~,~) delete(fig));

scrollPanel = uipanel(fig, ...
    'BorderType','none', ...
    'Scrollable','on', ...
    'Position',[1 1 fig.Position(3) fig.Position(4)]);

contentPanel = uipanel(scrollPanel, ...
    'BorderType','none', ...
    'Position',[1 1 fig.Position(3)-2 contentHeightCollapsed]);

root = uigridlayout(contentPanel,[6 1]);
root.RowHeight = {145,150,36,0,'1x',45};
root.Padding = [12 12 12 12];
root.RowSpacing = 8;
fig.SizeChangedFcn = @resizeScrollableContent;

%% 1) Input files
filePanel = uipanel(root,'Title','1) Input files');
fileGrid = uigridlayout(filePanel,[4 4]);
fileGrid.ColumnWidth = {155,'1x',105,150};
fileGrid.RowHeight = {30,30,30,30};
fileGrid.Padding = [10 8 10 8];
fileGrid.RowSpacing = 6;

uilabel(fileGrid,'Text','OpenSim model (.osim)');
modelEdit = uieditfield(fileGrid,'text','Editable','off');
modelEdit.Layout.Column = 2;
uibutton(fileGrid,'Text','Browse...','ButtonPushedFcn',@browseModel);
uilabel(fileGrid,'Text','');

uilabel(fileGrid,'Text','IK result (.mot)');
ikEdit = uieditfield(fileGrid,'text','Editable','off');
ikEdit.Layout.Column = 2;
uibutton(fileGrid,'Text','Browse...','ButtonPushedFcn',@browseIK);
fsLabel = uilabel(fileGrid,'Text','Fs: -- Hz');

uilabel(fileGrid,'Text','External loads setup file');
extForceEdit = uieditfield(fileGrid,'text','Editable','on', ...
    'Placeholder','Optional path to external loads setup file');
extForceEdit.Layout.Column = 2;
uibutton(fileGrid,'Text','Browse...','ButtonPushedFcn',@browseExternalLoads);
uilabel(fileGrid,'Text','Optional');

uilabel(fileGrid,'Text','MOT time range');
motBoundsLabel = uilabel(fileGrid,'Text','--');
motBoundsLabel.Layout.Column = [2 4];

%% 2) Run inputs
runPanel = uipanel(root,'Title','2) Run inputs');
runGrid = uigridlayout(runPanel,[3 6]);
runGrid.ColumnWidth = {145,115,125,115,160,'1x'};
runGrid.RowHeight = {32,32,32};
runGrid.Padding = [10 8 10 8];
runGrid.RowSpacing = 6;

uilabel(runGrid,'Text','Start time [s]');
startField = uieditfield(runGrid,'numeric','Value',0);
uilabel(runGrid,'Text','End time [s]');
endField = uieditfield(runGrid,'numeric','Value',0);
uilabel(runGrid,'Text','Penetration [mm]');
penField = uieditfield(runGrid,'numeric','Value',14,'Limits',[0 Inf]);

autoFreqCheck = uicheckbox(runGrid, ...
    'Text','Automatic frequency', ...
    'Value',true, ...
    'ValueChangedFcn',@autoFreqChanged);
autoFreqCheck.Layout.Column = [1 2];
uilabel(runGrid,'Text','Manual cut-off [Hz]');
freqField = uieditfield(runGrid,'numeric','Value',6,'Limits',[0 Inf], ...
    'Enable','off');
freqHelp = uilabel(runGrid,'Text','Used only when Automatic frequency is unchecked');
freqHelp.Layout.Column = [5 6];

uilabel(runGrid,'Text','Current frequency mode');
freqModeLabel = uilabel(runGrid,'Text','Automatic: DetectFcut(Motion, Fs)');
freqModeLabel.Layout.Column = [2 6];

%% Advanced toggle
advToggleGrid = uigridlayout(root,[1 2]);
advToggleGrid.ColumnWidth = {220,'1x'};
advToggleGrid.Padding = [0 0 0 0];
advButton = uibutton(advToggleGrid,'Text','Show advanced settings ▼', ...
    'ButtonPushedFcn',@toggleAdvanced);
uilabel(advToggleGrid,'Text','');

%% Advanced settings
advPanel = uipanel(root,'Title','Advanced settings','Visible','off');
advPanelGrid = uigridlayout(advPanel,[1 1]);
advPanelGrid.Padding = [8 8 8 8];
advPanelGrid.RowHeight = {'1x'};
advPanelGrid.ColumnWidth = {'1x'};

advTabs = uitabgroup(advPanelGrid);

bodyTab = uitab(advTabs,'Title','Contact body names');
bodyGrid = uigridlayout(bodyTab,[1 1]);
bodyGrid.Padding = [8 8 8 8];
bodyTable = uitable(bodyGrid);
bodyTable.ColumnName = {'Body name','Found in selected model'};
bodyTable.ColumnEditable = [true false];
bodyTable.RowName = {'Right calcaneus','Left calcaneus','Right toes','Left toes','Right hand','Left hand'};
bodyTable.Data = defaultBodyTableData();
bodyTable.CellEditCallback = @updateBodyFoundStatus;

planeTab = uitab(advTabs,'Title','Contact planes');
planeGrid = uigridlayout(planeTab,[1 1]);
planeGrid.Padding = [8 8 8 8];
contactTable = uitable(planeGrid);
contactTable.ColumnName = {'X','Y','Z'};
contactTable.RowName = {'Right foot A','Right foot B','Left foot A','Left foot B','Right hand','Left hand'};
contactTable.ColumnEditable = [true true true];
contactTable.Data = zeros(6,3);

orientTab = uitab(advTabs,'Title','Plane orientation');
orientGrid = uigridlayout(orientTab,[3 4]);
orientGrid.ColumnWidth = {150,130,150,'1x'};
orientGrid.RowHeight = {34,34,'1x'};
orientGrid.Padding = [12 12 12 12];
orientGrid.RowSpacing = 8;
orientGrid.ColumnSpacing = 10;
uilabel(orientGrid,'Text','x_angle [rad]');
xAngleField = uieditfield(orientGrid,'numeric','Value',0);
uilabel(orientGrid,'Text','Rotation around X');
uilabel(orientGrid,'Text','');
uilabel(orientGrid,'Text','z_angle [rad]');
zAngleField = uieditfield(orientGrid,'numeric','Value',-pi/2);
uilabel(orientGrid,'Text','Rotation around Z');
uilabel(orientGrid,'Text','Original default: x = 0, z = -pi/2');

%% Status area
statusArea = uitextarea(root, ...
    'Editable','off', ...
    'Value',{'Select the OpenSim model and IK result file, then run OpenGRF.'});

%% Bottom buttons
buttonGrid = uigridlayout(root,[1 4]);
buttonGrid.ColumnWidth = {'1x',155,140,120};
buttonGrid.Padding = [0 0 0 0];
uilabel(buttonGrid,'Text','');
uibutton(buttonGrid,'Text','Save cfg to workspace', ...
    'ButtonPushedFcn',@saveCfgOnly);
uibutton(buttonGrid,'Text','Run OpenGRF', ...
    'ButtonPushedFcn',@runOpenGRF);
uibutton(buttonGrid,'Text','Close', ...
    'ButtonPushedFcn',@(~,~) delete(fig));

autoFreqChanged();
resizeScrollableContent();

%% Nested callbacks
    function browseModel(~,~)
        [f,p] = uigetfile('*.osim','Choose OpenSim MSK model');
        if isequal(f,0)
            return
        end
        app.ModelPath = string(fullfile(p,f));
        modelEdit.Value = char(app.ModelPath);
        pushStatus("Model selected: " + app.ModelPath);
        readModelBodyNames();
    end

    function browseIK(~,~)
        [f,p] = uigetfile('*.mot','Choose IK result file');
        if isequal(f,0)
            return
        end
        app.IKPath = string(fullfile(p,f));
        ikEdit.Value = char(app.IKPath);
        pushStatus("IK file selected: " + app.IKPath);

        try
            [motion, headMotion] = load_mot(char(app.IKPath));
            app.Motion = motion;
            app.HeadMotion = headMotion;
            if size(motion,1) < 2
                error('The selected MOT file contains fewer than two frames.');
            end
            app.Fs = 1/(motion(2,1)-motion(1,1));
            startField.Value = motion(1,1);
            endField.Value = motion(end,1);
            fsLabel.Text = sprintf('Fs: %.3f Hz', app.Fs);
            motBoundsLabel.Text = sprintf('%.6g s  ->  %.6g s', motion(1,1), motion(end,1));
            pushStatus(sprintf('MOT loaded. Fs = %.3f Hz; range = [%.6g, %.6g] s.', ...
                app.Fs, motion(1,1), motion(end,1)));
        catch ME
            pushStatus("load_mot failed; trying to read only the time column. Details: " + ME.message);
            try
                times = readMotTimes(char(app.IKPath));
                if numel(times) < 2
                    error('Could not find at least two time samples in the MOT file.');
                end
                app.Motion = [times(:), zeros(numel(times),1)];
                app.HeadMotion = {};
                app.Fs = 1/(times(2)-times(1));
                startField.Value = times(1);
                endField.Value = times(end);
                fsLabel.Text = sprintf('Fs: %.3f Hz', app.Fs);
                motBoundsLabel.Text = sprintf('%.6g s  ->  %.6g s', times(1), times(end));
                pushStatus(sprintf('Time column loaded. Fs = %.3f Hz; range = [%.6g, %.6g] s.', ...
                    app.Fs, times(1), times(end)));
            catch ME2
                uialert(fig, ME2.message, 'MOT reading error');
                pushStatus("MOT reading error: " + ME2.message);
            end
        end
    end

    function browseExternalLoads(~,~)
        [f,p] = uigetfile({'*.xml;*.sto;*.mot;*.txt;*.*','External loads setup or related files'}, ...
            'Choose external loads setup file');
        if isequal(f,0)
            return
        end
        extForceEdit.Value = char(string(fullfile(p,f)));
        pushStatus("External loads setup path selected: " + string(extForceEdit.Value));
    end

    function autoFreqChanged(~,~)
        if autoFreqCheck.Value
            freqField.Enable = 'off';
            freqModeLabel.Text = 'Automatic: DetectFcut(Motion, Fs)';
        else
            freqField.Enable = 'on';
            freqModeLabel.Text = 'Manual: use the cut-off frequency entered below';
        end
    end

    function toggleAdvanced(~,~)
        advancedVisible = ~advancedVisible;
        heights = root.RowHeight;
        if advancedVisible
            advPanel.Visible = 'on';
            heights{4} = 250;
            advButton.Text = 'Hide advanced settings ▲';
        else
            advPanel.Visible = 'off';
            heights{4} = 0;
            advButton.Text = 'Show advanced settings ▼';
        end
        root.RowHeight = heights;
        resizeScrollableContent();
    end

    function resizeScrollableContent(~,~)
        figPos = fig.Position;
        scrollPanel.Position = [1 1 figPos(3) figPos(4)];

        if advancedVisible
            targetHeight = contentHeightExpanded;
        else
            targetHeight = contentHeightCollapsed;
        end

        contentHeight = max(figPos(4), targetHeight);
        contentWidth = max(figPos(3)-2, 760);
        contentPanel.Position = [1 1 contentWidth contentHeight];
    end

    function saveCfgOnly(~,~)
        try
            cfg = collectCfg();
            assignin('base','OpenGRF_GUI_config',cfg);
            pushStatus('Configuration saved to the workspace as OpenGRF_GUI_config.');
        catch ME
            uialert(fig, ME.message, 'Invalid input');
        end
    end

    function runOpenGRF(~,~)
        try
            cfg = collectCfg();
            assignin('base','OpenGRF_GUI_config',cfg);
            pushStatus('Configuration validated. Starting OpenGRF; check the MATLAB Command Window for progress.');
            drawnow;

            if exist('run_OpenGRF_from_config','file') ~= 2
                error(['run_OpenGRF_from_config.m is not on the MATLAB path. ' ...
                    'Put New_GUI.m and run_OpenGRF_from_config.m in the OpenGRF folder or add that folder to the path.']);
            end

            out = run_OpenGRF_from_config(cfg);
            assignin('base','OpenGRF_GUI_output',out);
            pushStatus('Analysis completed. Output saved to the workspace as OpenGRF_GUI_output.');
            if isfield(out,'PredictedGRFFile')
                pushStatus("Predicted GRF file: " + string(out.PredictedGRFFile));
            end
        catch ME
            pushStatus("ERROR: " + ME.message);
            uialert(fig, ME.message, 'OpenGRF error');
        end
    end

    function cfg = collectCfg()
        if strlength(app.ModelPath) == 0 || ~isfile(char(app.ModelPath))
            error('Select a valid .osim model file.');
        end
        if strlength(app.IKPath) == 0 || ~isfile(char(app.IKPath))
            error('Select a valid IK .mot file.');
        end

        extPath = strtrim(string(extForceEdit.Value));
        if strlength(extPath) > 0 && ~isfile(char(extPath))
            error('External loads setup file not found: %s', char(extPath));
        end

        if startField.Value >= endField.Value
            error('Start time must be lower than End time.');
        end
        if ~isnan(app.Fs) && ~autoFreqCheck.Value && freqField.Value >= app.Fs/2
            error('Manual cut-off must be lower than the Nyquist frequency: %.6g Hz.', app.Fs/2);
        end

        T = contactTable.Data;
        if ~isnumeric(T) || ~isequal(size(T),[6 3]) || any(~isfinite(T(:)))
            error('The contact plane table must contain finite numeric values in all cells.');
        end

        bodyData = bodyTable.Data;
        names = strings(6,1);
        for rr = 1:6
            names(rr) = strtrim(string(bodyData{rr,1}));
            if strlength(names(rr)) == 0
                error('Contact body name in row %d is empty.', rr);
            end
        end

        cfg = struct();
        cfg.ModelPath = char(app.ModelPath);
        cfg.IKPath = char(app.IKPath);
        cfg.ExternalForceSetupPath = char(extPath);
        cfg.TimeStart = startField.Value;
        cfg.TimeEnd = endField.Value;
        cfg.PenetrationMM = penField.Value;

        cfg.ContactBodies = struct();
        cfg.ContactBodies.CalcnRight = char(names(1));
        cfg.ContactBodies.CalcnLeft  = char(names(2));
        cfg.ContactBodies.ToesRight  = char(names(3));
        cfg.ContactBodies.ToesLeft   = char(names(4));
        cfg.ContactBodies.HandRight  = char(names(5));
        cfg.ContactBodies.HandLeft   = char(names(6));

        cfg.Ground_R_Foot_T_A = T(1,:);
        cfg.Ground_R_Foot_T_B = T(2,:);
        cfg.Ground_L_Foot_T_A = T(3,:);
        cfg.Ground_L_Foot_T_B = T(4,:);
        cfg.Ground_R_Hand_T = T(5,:);
        cfg.Ground_L_Hand_T = T(6,:);

        cfg.x_angle = xAngleField.Value;
        cfg.z_angle = zAngleField.Value;

        cfg.AutoFreq = logical(autoFreqCheck.Value);
        cfg.Freq = freqField.Value;
    end

    function readModelBodyNames()
        app.BodyNames = strings(0,1);
        try
            model = org.opensim.modeling.Model(char(app.ModelPath));
            nBodies = model.getBodySet.getSize;
            names = strings(nBodies,1);
            for ii = 0:nBodies-1
                names(ii+1) = convertCharsToStrings(model.getBodySet.get(ii).toString.toCharArray);
            end
            app.BodyNames = names;
            updateBodyFoundStatus();
            pushStatus(sprintf('Model body names read: %d bodies found.', nBodies));
        catch ME
            updateBodyFoundStatus();
            pushStatus("Could not inspect model body names automatically. You can still edit them in Advanced settings. Details: " + ME.message);
        end
    end

    function updateBodyFoundStatus(~,~)
        data = bodyTable.Data;
        if isempty(data)
            data = defaultBodyTableData();
        end
        if size(data,2) < 2
            data(:,2) = {''};
        end
        for rr = 1:size(data,1)
            candidate = strtrim(string(data{rr,1}));
            if strlength(candidate) == 0
                data{rr,2} = 'empty';
            elseif isempty(app.BodyNames)
                data{rr,2} = 'not checked';
            elseif any(strcmpi(app.BodyNames,candidate))
                data{rr,2} = 'yes';
            else
                data{rr,2} = 'no';
            end
        end
        bodyTable.Data = data;
    end

    function data = defaultBodyTableData()
        data = {'calcn_r','not checked'; ...
                'calcn_l','not checked'; ...
                'toes_r','not checked'; ...
                'toes_l','not checked'; ...
                'hand_r','not checked'; ...
                'hand_l','not checked'};
    end

    function pushStatus(msg)
        old = statusArea.Value;
        if ischar(old); old = {old}; end
        stamp = datestr(now,'HH:MM:SS');
        statusArea.Value = [{sprintf('[%s] %s', stamp, char(msg))}; old(:)];
        drawnow limitrate
    end
end

function times = readMotTimes(filename)
fid = fopen(filename,'r');
if fid < 0
    error('Could not open file: %s', filename);
end
cleaner = onCleanup(@() fclose(fid)); 

times = [];
afterHeader = false;

while true
    line = fgetl(fid);
    if ~ischar(line)
        break
    end

    s = strtrim(line);
    if isempty(s)
        continue
    end

    if ~afterHeader
        if strcmpi(s,'endheader')
            afterHeader = true;
        end
        continue
    end

    nums = sscanf(s,'%f');
    if isempty(nums)
        continue
    end

    times(end+1,1) = nums(1); 
end

if isempty(times)
    frewind(fid);
    while true
        line = fgetl(fid);
        if ~ischar(line)
            break
        end
        nums = sscanf(strtrim(line),'%f');
        if ~isempty(nums)
            times(end+1,1) = nums(1); 
        end
    end
end
end
