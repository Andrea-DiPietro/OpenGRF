function BK = BKanalysis(model, modelPath, IKFolder, IKResult, Freq, timeStart, timeEnd, path_BK, path_Solution, FullBody, n)
%% BKanalysis
% Body Kinematics analysis for OpenGRF.
%
% Inputs:
%   model         OpenSim Model object
%   modelPath     Full path to .osim model
%   IKFolder      Folder of IK .mot file
%   IKResult      IK .mot filename
%   Freq          Low-pass cut-off frequency
%   timeStart     Analysis start time
%   timeEnd       Analysis end time
%   path_BK       Output folder for BodyKinematics results
%   path_Solution Output folder for setup XML
%   FullBody      Boolean flag
%   n             Projection vector
%
% Output:
%   BK            Structure containing BodyKinematics results
%
% Author: Andrea Di Pietro, 2026

      if ~exist(path_BK, 'dir')
        mkdir(path_BK);
    end

    if ~exist(path_Solution, 'dir')
        mkdir(path_Solution);
    end

    %% ------------------------------------------------------------
    % 1. OpenSim Body Kinematics Analysis
    %% ------------------------------------------------------------

    state = model.initSystem();

    AnTool2 = org.opensim.modeling.AnalyzeTool();
    AnTool2.setModel(model);
    AnTool2.setModelFilename(modelPath);
    AnTool2.setCoordinatesFileName(fullfile(IKFolder, IKResult));
    AnTool2.setLowpassCutoffFrequency(Freq);
    AnTool2.setSolveForEquilibrium(1);
    AnTool2.setStartTime(timeStart);
    AnTool2.setFinalTime(timeEnd);
    AnTool2.setResultsDir(path_BK);

    BKTool = org.opensim.modeling.BodyKinematics();
    BKTool.setStartTime(timeStart);
    BKTool.setEndTime(timeEnd);

    AnTool2.getAnalysisSet().cloneAndAppend(BKTool);

    setupBKFile = fullfile(path_Solution, 'Setup_BK.xml');
    AnTool2.print(setupBKFile);

    BK_execute = org.opensim.modeling.AnalyzeTool(setupBKFile);
    BK_execute.run();

    %% ------------------------------------------------------------
    % 2. Load Body Kinematics outputs
    %% ------------------------------------------------------------

    [BodyPos, Head_BK] = load_sto(fullfile(path_BK, '_BodyKinematics_pos_global.sto'));
    [BodyVel, ~]       = load_sto(fullfile(path_BK, '_BodyKinematics_vel_global.sto'));
    [BodyAcc, ~]       = load_sto(fullfile(path_BK, '_BodyKinematics_acc_global.sto'));

    timeBK = round(BodyPos(:,1), 8);

    %% ------------------------------------------------------------
    % 3. Optional hand variables
    %% ------------------------------------------------------------

    if FullBody
        X_Hand_l = BodyPos(:, strcmp(Head_BK, 'hand_l_X'));
        Y_Hand_l = BodyPos(:, strcmp(Head_BK, 'hand_l_Y'));
        Z_Hand_l = BodyPos(:, strcmp(Head_BK, 'hand_l_Z'));

        X_Hand_r = BodyPos(:, strcmp(Head_BK, 'hand_r_X'));
        Y_Hand_r = BodyPos(:, strcmp(Head_BK, 'hand_r_Y'));
        Z_Hand_r = BodyPos(:, strcmp(Head_BK, 'hand_r_Z'));
    else
        X_Hand_l = [];
        Y_Hand_l = [];
        Z_Hand_l = [];

        X_Hand_r = [];
        Y_Hand_r = [];
        Z_Hand_r = [];
    end

    %% ------------------------------------------------------------
    % 4. Centre of mass
    %% ------------------------------------------------------------

    CoM_x = BodyPos(:, strcmp(Head_BK, 'center_of_mass_X'));
    CoM_y = BodyPos(:, strcmp(Head_BK, 'center_of_mass_Y'));
    CoM_z = BodyPos(:, strcmp(Head_BK, 'center_of_mass_Z'));

    %% ------------------------------------------------------------
    % 5. Toes positions, velocities, accelerations
    %% ------------------------------------------------------------

    X_Toes_l = BodyPos(:, strcmp(Head_BK, 'toes_l_X'));
    Y_Toes_l = BodyPos(:, strcmp(Head_BK, 'toes_l_Y'));
    Z_Toes_l = BodyPos(:, strcmp(Head_BK, 'toes_l_Z'));

    X_Toes_r = BodyPos(:, strcmp(Head_BK, 'toes_r_X'));
    Y_Toes_r = BodyPos(:, strcmp(Head_BK, 'toes_r_Y'));
    Z_Toes_r = BodyPos(:, strcmp(Head_BK, 'toes_r_Z'));

    v_toes_l_x = BodyVel(:, strcmp(Head_BK, 'toes_l_X'));
    v_toes_l_y = BodyVel(:, strcmp(Head_BK, 'toes_l_Y'));
    v_toes_l_z = BodyVel(:, strcmp(Head_BK, 'toes_l_Z'));
    v_toes_l = sqrt(v_toes_l_x.^2 + v_toes_l_y.^2 + v_toes_l_z.^2);

    a_toes_l_x = BodyAcc(:, strcmp(Head_BK, 'toes_l_X'));
    a_toes_l_y = BodyAcc(:, strcmp(Head_BK, 'toes_l_Y'));
    a_toes_l_z = BodyAcc(:, strcmp(Head_BK, 'toes_l_Z'));
    a_toes_l = sqrt(a_toes_l_x.^2 + a_toes_l_y.^2 + a_toes_l_z.^2);

    v_toes_r_x = BodyVel(:, strcmp(Head_BK, 'toes_r_X'));
    v_toes_r_y = BodyVel(:, strcmp(Head_BK, 'toes_r_Y'));
    v_toes_r_z = BodyVel(:, strcmp(Head_BK, 'toes_r_Z'));
    v_toes_r = sqrt(v_toes_r_x.^2 + v_toes_r_y.^2 + v_toes_r_z.^2);

    a_toes_r_x = BodyAcc(:, strcmp(Head_BK, 'toes_r_X'));
    a_toes_r_y = BodyAcc(:, strcmp(Head_BK, 'toes_r_Y'));
    a_toes_r_z = BodyAcc(:, strcmp(Head_BK, 'toes_r_Z'));
    a_toes_r = sqrt(a_toes_r_x.^2 + a_toes_r_y.^2 + a_toes_r_z.^2);

    v_toes_r_n = v_toes_r / max(v_toes_r);
    v_toes_l_n = v_toes_l / max(v_toes_l);

    %% ------------------------------------------------------------
    % 6. Calcaneus positions, velocities, accelerations
    %% ------------------------------------------------------------

    X_Calcn_l = BodyPos(:, strcmp(Head_BK, 'calcn_l_X'));
    Y_Calcn_l = BodyPos(:, strcmp(Head_BK, 'calcn_l_Y'));
    Z_Calcn_l = BodyPos(:, strcmp(Head_BK, 'calcn_l_Z'));

    X_Calcn_r = BodyPos(:, strcmp(Head_BK, 'calcn_r_X'));
    Y_Calcn_r = BodyPos(:, strcmp(Head_BK, 'calcn_r_Y'));
    Z_Calcn_r = BodyPos(:, strcmp(Head_BK, 'calcn_r_Z'));

    v_calcn_l_x = BodyVel(:, strcmp(Head_BK, 'calcn_l_X'));
    v_calcn_l_y = BodyVel(:, strcmp(Head_BK, 'calcn_l_Y'));
    v_calcn_l_z = BodyVel(:, strcmp(Head_BK, 'calcn_l_Z'));
    v_calcn_l = sqrt(v_calcn_l_x.^2 + v_calcn_l_y.^2 + v_calcn_l_z.^2);

    a_calcn_l_x = BodyAcc(:, strcmp(Head_BK, 'calcn_l_X'));
    a_calcn_l_y = BodyAcc(:, strcmp(Head_BK, 'calcn_l_Y'));
    a_calcn_l_z = BodyAcc(:, strcmp(Head_BK, 'calcn_l_Z'));
    a_calcn_l = sqrt(a_calcn_l_x.^2 + a_calcn_l_y.^2 + a_calcn_l_z.^2);

    v_calcn_r_x = BodyVel(:, strcmp(Head_BK, 'calcn_r_X'));
    v_calcn_r_y = BodyVel(:, strcmp(Head_BK, 'calcn_r_Y'));
    v_calcn_r_z = BodyVel(:, strcmp(Head_BK, 'calcn_r_Z'));
    v_calcn_r = sqrt(v_calcn_r_x.^2 + v_calcn_r_y.^2 + v_calcn_r_z.^2);

    a_calcn_r_x = BodyAcc(:, strcmp(Head_BK, 'calcn_r_X'));
    a_calcn_r_y = BodyAcc(:, strcmp(Head_BK, 'calcn_r_Y'));
    a_calcn_r_z = BodyAcc(:, strcmp(Head_BK, 'calcn_r_Z'));
    a_calcn_r = sqrt(a_calcn_r_x.^2 + a_calcn_r_y.^2 + a_calcn_r_z.^2);

    v_calcn_r_n = v_calcn_r / max(v_calcn_r);
    v_calcn_l_n = v_calcn_l / max(v_calcn_l);

    %% ------------------------------------------------------------
    % 7. Projection and minimum-position detection
    %% ------------------------------------------------------------

    proj_r = zeros(length(X_Calcn_r), 1);
    proj_l = zeros(length(X_Calcn_l), 1);

    for i = 1:length(X_Calcn_r)
        proj_r(i) = dot([X_Calcn_r(i), Y_Calcn_r(i), Z_Calcn_r(i)], n);
        proj_l(i) = dot([X_Calcn_l(i), Y_Calcn_l(i), Z_Calcn_l(i)], n);
    end

    range_pos_r = proj_r < min(proj_r) + sign(min(proj_r)) * min(proj_r) * 1/1000;
    range_pos_l = proj_l < min(proj_l) + sign(min(proj_l)) * min(proj_l) * 1/1000;

    min_p_calcn_r_y = find(range_pos_r);
    min_p_calcn_l_y = find(range_pos_l);

    [val_pos_r, range_pos_r] = min(abs(a_calcn_r_y(min_p_calcn_r_y)));
    [val_pos_l, range_pos_l] = min(abs(a_calcn_l_y(min_p_calcn_l_y)));

    position_r = min_p_calcn_r_y(range_pos_r);
    position_l = min_p_calcn_l_y(range_pos_l);

    %% ------------------------------------------------------------
    % 8. Output structure
    %% ------------------------------------------------------------

    BK = struct();

    BK.state = state;

    BK.BodyPos = BodyPos;
    BK.BodyVel = BodyVel;
    BK.BodyAcc = BodyAcc;
    BK.Head_BK = Head_BK;
    BK.timeBK = timeBK;

    BK.CoM_x = CoM_x;
    BK.CoM_y = CoM_y;
    BK.CoM_z = CoM_z;

    BK.X_Hand_l = X_Hand_l;
    BK.Y_Hand_l = Y_Hand_l;
    BK.Z_Hand_l = Z_Hand_l;

    BK.X_Hand_r = X_Hand_r;
    BK.Y_Hand_r = Y_Hand_r;
    BK.Z_Hand_r = Z_Hand_r;

    BK.X_Toes_l = X_Toes_l;
    BK.Y_Toes_l = Y_Toes_l;
    BK.Z_Toes_l = Z_Toes_l;

    BK.X_Toes_r = X_Toes_r;
    BK.Y_Toes_r = Y_Toes_r;
    BK.Z_Toes_r = Z_Toes_r;

    BK.v_toes_l_x = v_toes_l_x;
    BK.v_toes_l_y = v_toes_l_y;
    BK.v_toes_l_z = v_toes_l_z;
    BK.v_toes_l = v_toes_l;

    BK.a_toes_l_x = a_toes_l_x;
    BK.a_toes_l_y = a_toes_l_y;
    BK.a_toes_l_z = a_toes_l_z;
    BK.a_toes_l = a_toes_l;

    BK.v_toes_r_x = v_toes_r_x;
    BK.v_toes_r_y = v_toes_r_y;
    BK.v_toes_r_z = v_toes_r_z;
    BK.v_toes_r = v_toes_r;

    BK.a_toes_r_x = a_toes_r_x;
    BK.a_toes_r_y = a_toes_r_y;
    BK.a_toes_r_z = a_toes_r_z;
    BK.a_toes_r = a_toes_r;

    BK.v_toes_r_n = v_toes_r_n;
    BK.v_toes_l_n = v_toes_l_n;

    BK.X_Calcn_l = X_Calcn_l;
    BK.Y_Calcn_l = Y_Calcn_l;
    BK.Z_Calcn_l = Z_Calcn_l;

    BK.X_Calcn_r = X_Calcn_r;
    BK.Y_Calcn_r = Y_Calcn_r;
    BK.Z_Calcn_r = Z_Calcn_r;

    BK.v_calcn_l_x = v_calcn_l_x;
    BK.v_calcn_l_y = v_calcn_l_y;
    BK.v_calcn_l_z = v_calcn_l_z;
    BK.v_calcn_l = v_calcn_l;

    BK.a_calcn_l_x = a_calcn_l_x;
    BK.a_calcn_l_y = a_calcn_l_y;
    BK.a_calcn_l_z = a_calcn_l_z;
    BK.a_calcn_l = a_calcn_l;

    BK.v_calcn_r_x = v_calcn_r_x;
    BK.v_calcn_r_y = v_calcn_r_y;
    BK.v_calcn_r_z = v_calcn_r_z;
    BK.v_calcn_r = v_calcn_r;

    BK.a_calcn_r_x = a_calcn_r_x;
    BK.a_calcn_r_y = a_calcn_r_y;
    BK.a_calcn_r_z = a_calcn_r_z;
    BK.a_calcn_r = a_calcn_r;

    BK.v_calcn_r_n = v_calcn_r_n;
    BK.v_calcn_l_n = v_calcn_l_n;

    BK.proj_r = proj_r;
    BK.proj_l = proj_l;

    BK.range_pos_r = range_pos_r;
    BK.range_pos_l = range_pos_l;

    BK.min_p_calcn_r_y = min_p_calcn_r_y;
    BK.min_p_calcn_l_y = min_p_calcn_l_y;

    BK.val_pos_r = val_pos_r;
    BK.val_pos_l = val_pos_l;

    BK.position_r = position_r;
    BK.position_l = position_l;

    BK.setupBKFile = setupBKFile;
    BK.path_BK = path_BK;

end