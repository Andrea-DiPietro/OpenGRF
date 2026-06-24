function output = run_OpenGRF_from_config(cfg)
% Runs OpenGRF using inputs collected by OpenGRF_GUI.
%
% Required fields in cfg:
%   ModelPath, IKPath, TimeStart, TimeEnd, PenetrationMM
%   ExternalForceSetupPath, ContactBodies
%   Ground_R_Foot_T_A, Ground_R_Foot_T_B, Ground_L_Foot_T_A, Ground_L_Foot_T_B
%   Ground_R_Hand_T, Ground_L_Hand_T, x_angle, z_angle, AutoFreq, Freq.
% Authors: Andrea Di Pietro, Luca Modenese, 2026

if nargin < 1 || isempty(cfg)
    if evalin('base', 'exist(''OpenGRF_GUI_config'', ''var'')')
        cfg = evalin('base', 'OpenGRF_GUI_config');
    else
        error('Missing configuration. Run OpenGRF_GUI or pass a cfg struct.');
    end
end

requiredFields = {'ModelPath','IKPath','TimeStart','TimeEnd','PenetrationMM', ...
    'Ground_R_Foot_T_A','Ground_R_Foot_T_B','Ground_L_Foot_T_A','Ground_L_Foot_T_B', ...
    'Ground_R_Hand_T','Ground_L_Hand_T','x_angle','z_angle','AutoFreq','Freq'};

for rf = 1:numel(requiredFields)
    if ~isfield(cfg, requiredFields{rf})
        error('Missing cfg.%s', requiredFields{rf});
    end
end

modelPath = char(cfg.ModelPath);
motPath   = char(cfg.IKPath);
FextPath = char(cfg.ExternalForceSetupPath);

if ~isfile(modelPath)
    error('Model file not found: %s', modelPath);
end
if ~isfile(motPath)
    error('IK result file not found: %s', motPath);
end

%% Optional external force setup from GUI
ExternalForceSetupPath = '';
if isfield(cfg, 'ExternalForceSetupPath') && ~isempty(cfg.ExternalForceSetupPath)
    ExternalForceSetupPath = char(cfg.ExternalForceSetupPath);
end

[ModelFolder, modelName, modelExt] = fileparts(modelPath);
[IKFolder, IKName, IKExt] = fileparts(motPath);
ModelFile = [modelName modelExt];
IKResult  = [IKName IKExt];

fprintf('Loading model:\n%s\n', modelPath);
model = org.opensim.modeling.Model(modelPath);

fprintf('Loading IK result:\n%s\n', motPath);
[Motion, HeadMotion] = load_mot(motPath); 

if size(Motion, 1) < 2
    error('The selected IK motion file contains less than two frames.');
end

tic;

Fs = 1 / (Motion(2,1) - Motion(1,1)); % Kinematics sampling frequency
fprintf('Sampling frequency: %.3f Hz\n', Fs);

timeStart = double(cfg.TimeStart);
timeEnd   = double(cfg.TimeEnd);
pen_mm    = double(cfg.PenetrationMM);
if ~(isfinite(timeStart) && isfinite(timeEnd) && timeStart < timeEnd)
    error('Invalid time interval. TimeStart must be lower than TimeEnd.');
end
if timeStart < Motion(1,1) || timeEnd > Motion(end,1)
    warning('Selected time interval [%.6g %.6g] is outside IK time bounds [%.6g %.6g].', ...
        timeStart, timeEnd, Motion(1,1), Motion(end,1));
end

Ground_R_Foot_T_A = double(cfg.Ground_R_Foot_T_A(:))';
Ground_R_Foot_T_B = double(cfg.Ground_R_Foot_T_B(:))';
Ground_L_Foot_T_A = double(cfg.Ground_L_Foot_T_A(:))';
Ground_L_Foot_T_B = double(cfg.Ground_L_Foot_T_B(:))';

Ground_R_Hand_T = double(cfg.Ground_R_Hand_T(:))';
Ground_L_Hand_T = double(cfg.Ground_L_Hand_T(:))';

validateattributes(Ground_R_Foot_T_A, {'numeric'}, {'numel',3});
validateattributes(Ground_R_Foot_T_B, {'numeric'}, {'numel',3});
validateattributes(Ground_L_Foot_T_A, {'numeric'}, {'numel',3});
validateattributes(Ground_L_Foot_T_B, {'numeric'}, {'numel',3});
validateattributes(Ground_R_Hand_T, {'numeric'}, {'numel',3});
validateattributes(Ground_L_Hand_T, {'numeric'}, {'numel',3});

x_angle = double(cfg.x_angle);
z_angle = double(cfg.z_angle);

Rot_x=[1 0 0;
    0 cos(x_angle) -sin(x_angle);
    0 sin(x_angle) cos(x_angle)];

Rot_z=[-sin(z_angle) cos(z_angle) 0;
     cos(z_angle) -sin(z_angle) 0;
     0 0 1];

Rot=Rot_x*Rot_z; % 2 successive rotations, first around X and then around z
n=Rot(:,2); % the versor normal to the plane expressed in ground coordinates (in this version all planes have same orientation n)
Ground_Rot=[x_angle 0 z_angle];

%% Creation of storage directory for iterations 
path_Solution=fullfile(IKFolder,"Solution");
path_BK=fullfile(path_Solution,"BK");
path_PK=fullfile(path_Solution,"PK");
path_FR=fullfile(path_Solution,"FR");
path_iter=fullfile(path_Solution,"Iterations");
mkdir(path_Solution);
mkdir(path_BK);
mkdir(path_PK);
mkdir(path_FR);
mkdir(path_iter);
%% check for typology of model (lower body or full body)
FullBody = 0;
for i=0:model.getBodySet.getSize-1
    if contains(lower(string(model.getBodySet.get(i))),"hand")
        FullBody = 1;
        break
    end
end

%% Frequency input settings
if logical(cfg.AutoFreq)
    AutoFreq=1;
    f_cutoff=DetectFcut(Motion,Fs);
    Freq=max(f_cutoff);
else
    AutoFreq=0;
    Freq=double(cfg.Freq);
    if ~(isfinite(Freq) && Freq > 0 && Freq < Fs/2)
        error('Manual cut-off frequency must be finite and between 0 and Nyquist frequency (%.6g Hz).', Fs/2);
    end
end

%% Solver input settings: Do not modify the following parameters
% Internal actuators
CoordForce=1000;
CoordValue=1;
% External actuators
Force=10000;
Moment=100;
HighestValue=1;
% Residial actuators
PelvisForce=1; 
PelvisHighestValue=1E6;
%
SolveForEquilibrium=1;
% Probes setup
Sp_radius=0.02;
Sp_stiffness=1E7;
heel_shift=0.0; % this optional parameter just shifts forward heel probes along the foot if contact is too anticipated but does not affect CoP calculation
% Ground-probes penetration (expressed in m) at foot-flat instant
d=pen_mm/1000;

%% Contact body names from GUI configuration
% Defaults match the original script, but the GUI can override them when the
% selected model uses different body names.
calcn_r_name = validateOpenSimBodyName(model, getCfgContactBodyName(cfg, 'CalcnRight', 'calcn_r'), 'right calcaneus');
calcn_l_name = validateOpenSimBodyName(model, getCfgContactBodyName(cfg, 'CalcnLeft',  'calcn_l'), 'left calcaneus');
toes_r_name  = validateOpenSimBodyName(model, getCfgContactBodyName(cfg, 'ToesRight',  'toes_r'),  'right toes');
toes_l_name  = validateOpenSimBodyName(model, getCfgContactBodyName(cfg, 'ToesLeft',   'toes_l'),  'left toes');

if FullBody
    hand_r_name  = validateOpenSimBodyName(model, getCfgContactBodyName(cfg, 'HandRight', 'hand_r'), 'right hand');
    hand_l_name  = validateOpenSimBodyName(model, getCfgContactBodyName(cfg, 'HandLeft',  'hand_l'), 'left hand');
else
    hand_r_name = getCfgContactBodyName(cfg, 'HandRight', 'hand_r');
    hand_l_name = getCfgContactBodyName(cfg, 'HandLeft',  'hand_l');
end
disp('Contact elements calibration in progress, please wait ...')
%% Execution of BK to obtain CoM info (positions,velocities,accelerations) segments
BKout=BKanalysis(model,modelPath,IKFolder, IKResult,Freq,timeStart,timeEnd,path_BK,path_Solution,FullBody,n);%% Contact analysis 
%% Finding calcn and toes scaling factors
Calcn_r_SF=model.getBodySet.get(calcn_r_name).get_attached_geometry(0).get_scale_factors.getAsMat;
Calcn_l_SF=model.getBodySet.get(calcn_l_name).get_attached_geometry(0).get_scale_factors.getAsMat;
Toes_r_SF=model.getBodySet.get(toes_r_name).get_attached_geometry(0).get_scale_factors.getAsMat;
Toes_l_SF=model.getBodySet.get(toes_l_name).get_attached_geometry(0).get_scale_factors.getAsMat;

% determining the spheres baseline position on the foot
PosSp=...
        [(0+heel_shift) 0.03 -0.01; % calcn #1
        (0+heel_shift) 0.03 0.01;% calcn #2
        (0.035+heel_shift) 0.03 -0.02;% calcn #3
        (0.035+heel_shift) 0.03 0.02;% calcn #4
        0.037 0.03 -0.01; %toes  #5 0.034 0.03 -0.01
        0.037 0.03 0.02;%toes #6    0.034 0.03 -0.01
        0.06 0.03 -0.015;% calcn #7
        0.06 0.03 0.035;% calcn #8
        0.105 0.03 -0.005;% calcn #9
        0.105 0.03 0.045;% calcn #10
        0.14 0.03 -0.005;% calcn #11
        0.14 0.03 0.045;% calcn #12
        0.0 0.03 -0.005;%toes #13
        0.0 0.03 0.03];%toes #14
% adding contact spheres (probes) on the model
side=['R','L'];
for ii=1:size(PosSp,1)
    for ss=1:2
        % creating spheres and ground contact
        SpName=['Sphere_Foot_',num2str(ii),'_',side(ss)];
        Sphere=org.opensim.modeling.ContactSphere();
        Sphere.setName(SpName);
        if ii<=4 || ii>=7 && ii<=12 % just selecting the spheres on calcaneus
            if strcmp(side(ss),'R')
                Body=model.getBodySet.get(calcn_r_name);
                Sphere.setLocation(org.opensim.modeling.Vec3.createFromMat(PosSp(ii,:).*Calcn_r_SF'));
            else
                Body=model.getBodySet.get(calcn_l_name);
                PosSp(ii,3)=-PosSp(ii,3);
                Sphere.setLocation(org.opensim.modeling.Vec3.createFromMat(PosSp(ii,:).*Calcn_l_SF'));
            end
        elseif strcmp(side(ss),'R')
            Body=model.getBodySet.get(toes_r_name);
            Sphere.setLocation(org.opensim.modeling.Vec3.createFromMat(PosSp(ii,:).*Toes_r_SF'));
        else
            Body=model.getBodySet.get(toes_l_name);
            PosSp(ii,3)=-PosSp(ii,3);
            Sphere.setLocation(org.opensim.modeling.Vec3.createFromMat(PosSp(ii,:).*Toes_l_SF'));
        end
        Frame=org.opensim.modeling.PhysicalFrame.safeDownCast(Body);
        Sphere.setFrame(Frame);
        Sphere.setRadius(Sp_radius);
        model.addContactGeometry(Sphere)
        model.finalizeConnections
    end
end

if FullBody
    Sphere_Hand_R=org.opensim.modeling.ContactSphere();
    Sphere_Hand_R.setName('Sphere_Hand_R');
    Body=model.getBodySet.get(hand_r_name);
    Frame=org.opensim.modeling.PhysicalFrame.safeDownCast(Body);
    Sphere_Hand_R.setFrame(Frame);
    Sphere_Hand_R.setLocation(Body.get_mass_center);
    Sphere_Hand_R.setRadius(0.08);
    model.addContactGeometry(Sphere_Hand_R);

    Sphere_Hand_L=org.opensim.modeling.ContactSphere();
    Sphere_Hand_L.setName('Sphere_Hand_L');
    Body=model.getBodySet.get(hand_l_name);
    Frame=org.opensim.modeling.PhysicalFrame.safeDownCast(Body);
    Sphere_Hand_L.setFrame(Frame);
    Sphere_Hand_L.setLocation(Body.get_mass_center);
    Sphere_Hand_L.setRadius(0.08);
    model.addContactGeometry(Sphere_Hand_L);
end

%% creation of the 4 contact planes (2 per foot)
% Planes A and B are used per each foot. For level motor tasks
% Default Plane A and B heights are set to 0; while for those involving
% stairs, Plane A must be lower and Plane B the higher (set to step height).

groundCont_Foot_L_A=org.opensim.modeling.ContactHalfSpace();
groundCont_Foot_L_A.setName('ground_Foot_L_A');
groundCont_Foot_L_A.setFrame(model.getGround);
groundCont_Foot_L_A.set_location(org.opensim.modeling.Vec3.createFromMat(Ground_L_Foot_T_A));
groundCont_Foot_L_A.set_orientation(org.opensim.modeling.Vec3.createFromMat(Ground_Rot));
model.addContactGeometry(groundCont_Foot_L_A);

groundCont_Foot_R_A=org.opensim.modeling.ContactHalfSpace();
groundCont_Foot_R_A.setName('ground_Foot_R_A');
groundCont_Foot_R_A.setFrame(model.getGround);
groundCont_Foot_R_A.set_location(org.opensim.modeling.Vec3.createFromMat(Ground_R_Foot_T_A));
groundCont_Foot_R_A.set_orientation(org.opensim.modeling.Vec3.createFromMat(Ground_Rot));
model.addContactGeometry(groundCont_Foot_R_A);
% adding both right and left contact B plane Half Space
groundCont_Foot_L_B=org.opensim.modeling.ContactHalfSpace();
groundCont_Foot_L_B.setName('ground_Foot_L_B');
groundCont_Foot_L_B.setFrame(model.getGround);
groundCont_Foot_L_B.set_location(org.opensim.modeling.Vec3.createFromMat(Ground_L_Foot_T_B));
groundCont_Foot_L_B.set_orientation(org.opensim.modeling.Vec3.createFromMat(Ground_Rot));
model.addContactGeometry(groundCont_Foot_L_B);

groundCont_Foot_R_B=org.opensim.modeling.ContactHalfSpace();
groundCont_Foot_R_B.setName('ground_Foot_R_B');
groundCont_Foot_R_B.setFrame(model.getGround);
groundCont_Foot_R_B.set_location(org.opensim.modeling.Vec3.createFromMat(Ground_R_Foot_T_B));
groundCont_Foot_R_B.set_orientation(org.opensim.modeling.Vec3.createFromMat(Ground_Rot));
model.addContactGeometry(groundCont_Foot_R_B);

if FullBody % adding contact planes for Hands if present
    groundCont_Hand_R=org.opensim.modeling.ContactHalfSpace();
    groundCont_Hand_R.setName('ground_Hand_R');
    groundCont_Hand_R.setFrame(model.getGround);
    groundCont_Hand_R.set_location(org.opensim.modeling.Vec3.createFromMat(Ground_R_Hand_T));
    groundCont_Hand_R.set_orientation(org.opensim.modeling.Vec3.createFromMat(Ground_Rot));
    model.addContactGeometry(groundCont_Hand_R);

    groundCont_Hand_L=org.opensim.modeling.ContactHalfSpace();
    groundCont_Hand_L.setName('ground_Hand_L');
    groundCont_Hand_L.setFrame(model.getGround)
    groundCont_Hand_L.set_location(org.opensim.modeling.Vec3.createFromMat(Ground_L_Hand_T));
    groundCont_Hand_L.set_orientation(org.opensim.modeling.Vec3.createFromMat(Ground_Rot));
    model.addContactGeometry(groundCont_Hand_L);
end
model.finalizeConnections;
% create contact forces between each foot sphere and the ground A and B
contact_plane=['A','B'];
for ii=1:size(PosSp,1) % per each sphere on the foot
    for ss=1:2 % per each side R and L
        for uu=1:2 % per each A and B contact plane
            Force_name=['ForceGround_Foot_',num2str(ii),'_',side(ss),'_',contact_plane(uu)];
            ForceGround = org.opensim.modeling.HuntCrossleyForce();
            ForceGround.setName(Force_name);
            ForceGround.set_appliesForce(true);
            CoupleArray=['ground_Foot_',side(ss),'_',contact_plane(uu),' ','Sphere_Foot_',num2str(ii),'_',side(ss)];
            ForceGround.addGeometry(CoupleArray);
            ForceGround.setStiffness(Sp_stiffness);
            ForceGround.setDissipation(0);
            ForceGround.setStaticFriction(0);
            ForceGround.setDynamicFriction(0);
            ForceGround.setViscousFriction(0);
            ForceGround.setTransitionVelocity(0.13);
            model.getForceSet.cloneAndAppend(ForceGround);
            %ForceGround_Foot1_R.print('es_contForce.xml')
        end
    end
end

if FullBody
    ForceGround_Hand_L = org.opensim.modeling.HuntCrossleyForce();
    ForceGround_Hand_L.setName('ForceGround_Hand_L');
    ForceGround_Hand_L.set_appliesForce(true);
    ForceGround_Hand_L.addGeometry('ground_Hand_L Sphere_Hand_L');
    ForceGround_Hand_L.setStiffness(Sp_stiffness);
    ForceGround_Hand_L.setDissipation(0);
    ForceGround_Hand_L.setStaticFriction(0);
    ForceGround_Hand_L.setDynamicFriction(0);
    ForceGround_Hand_L.setViscousFriction(0);
    ForceGround_Hand_L.setTransitionVelocity(0.13)
    ForceGround_Hand_R = org.opensim.modeling.HuntCrossleyForce();
    ForceGround_Hand_R.setName('ForceGround_Hand_R');
    ForceGround_Hand_R.set_appliesForce(true);
    ForceGround_Hand_R.addGeometry('ground_Hand_R Sphere_Hand_R');
    ForceGround_Hand_R.setStiffness(Sp_stiffness);
    ForceGround_Hand_R.setDissipation(0);
    ForceGround_Hand_R.setStaticFriction(0);
    ForceGround_Hand_R.setDynamicFriction(0);
    ForceGround_Hand_R.setViscousFriction(0);
    ForceGround_Hand_R.setTransitionVelocity(0.13)
    model.getForceSet.cloneAndAppend(ForceGround_Hand_R);
    model.getForceSet.cloneAndAppend(ForceGround_Hand_L);
end

model.finalizeConnections();
modelProcessed_path=fullfile(ModelFolder,"ModelProcessed.osim");
model.setName("ModelProcessed");
model.print(modelProcessed_path);
modelProcessed=org.opensim.modeling.Model(modelProcessed_path);
%% start of Foot spheres calibration
% spheres calibration phase 2: PK on the spheres with current position
ExecutePK(modelProcessed,modelProcessed_path,fullfile(IKFolder,IKResult),...
    Freq,timeStart,timeEnd,calcn_r_name, calcn_l_name, toes_r_name, ...
    toes_l_name, heel_shift,path_PK,path_Solution);
% inizializing vectors of spheres positions correction
Sp_delta_r=zeros(size(BKout.timeBK,1),size(PosSp,1));
Sp_delta_l=zeros(size(BKout.timeBK,1),size(PosSp,1));
% finding position of each sphere with respect to the ground @ foot flat instant

for ii=1:size(PosSp,1)
    PK_Sp_Res_r=load_sto(fullfile(path_PK,['_PointKinematics_PK_sp',num2str(ii),'_r_pos.sto']));
    Sp_delta_r(:,ii)=dot(PK_Sp_Res_r(BKout.position_r,2:4)-Ground_R_Foot_T_A,n)-Sp_radius+d; % direction n 
    PK_Sp_Res_l=load_sto(fullfile(path_PK,['_PointKinematics_PK_sp',num2str(ii),'_l_pos.sto']));
    Sp_delta_l(:,ii)=dot(PK_Sp_Res_l(BKout.position_l,2:4)-Ground_L_Foot_T_A,n)-Sp_radius+d; % direction n 
end
% 

% updating the new spheres locations into the model
UpdateSphereLocs(modelProcessed,modelProcessed_path,Sp_delta_r,Sp_delta_l,n);

% execute PK with updated spheres locations
ExecutePK(modelProcessed,modelProcessed_path,fullfile(IKFolder,IKResult),...
    Freq,timeStart,timeEnd,calcn_r_name, calcn_l_name, toes_r_name, ...
    toes_l_name, heel_shift,path_PK,path_Solution);

% finding minumum position over the entire task
for ii=1:size(PosSp,1)
    PK_Sp_Res_r2=load_sto(fullfile(path_PK,['_PointKinematics_PK_sp',num2str(ii),'_r_pos.sto']));
    Sp_delta_r2(:,ii)=min(PK_Sp_Res_r2(:,2:4)*n)-dot(PK_Sp_Res_r2(BKout.position_r,2:4),n);
    PK_Sp_Res_l2=load_sto(fullfile(path_PK,['_PointKinematics_PK_sp',num2str(ii),'_l_pos.sto']));
     Sp_delta_l2(:,ii)=min(PK_Sp_Res_l2(:,2:4)*n)-dot(PK_Sp_Res_l2(BKout.position_l,2:4),n);
end
  % updating the new spheres locations into the model after calibrated with the minumum location over time
UpdateSphereLocs(modelProcessed,modelProcessed_path,Sp_delta_r2,Sp_delta_l2,n);
 %% end of calibration
%% calculation of penetrations 
% finding the contact intervals and penetrations for the second plane (B)
% finding the interval where the contact is active: just when calcn and
% toes are > of the contact plane and a velocity condition is met

if Ground_R_Foot_T_A(2)~=Ground_R_Foot_T_B(2)
    t_cont_foot_r_B=BKout.timeBK(all([BKout.Y_Calcn_r>Ground_R_Foot_T_B(2),BKout.Y_Toes_r>Ground_R_Foot_T_B(2),BKout.v_calcn_r_n<=0.35],2));
else
    t_cont_foot_r_B=BKout.timeBK;
end
if Ground_L_Foot_T_A(2)~=Ground_L_Foot_T_B(2)
    t_cont_foot_l_B=BKout.timeBK(all([BKout.Y_Calcn_l>Ground_L_Foot_T_B(2),BKout.Y_Toes_l>Ground_L_Foot_T_B(2),BKout.v_calcn_l_n<=0.35],2));
else
    t_cont_foot_l_B=BKout.timeBK;
end
ind_t_cont_foot_r_B=find(ismember(BKout.timeBK,t_cont_foot_r_B)); % find the position of this contact inside the time vector
ind_t_cont_foot_l_B=find(ismember(BKout.timeBK,t_cont_foot_l_B));
% Execution of Force Reporter tool
ExecuteFR(modelProcessed,modelProcessed_path,fullfile(IKFolder,IKResult),...
    Freq,timeStart,timeEnd,path_FR,path_Solution);
% analyze Force Reporter Results
[ForceReport,HeadFR]=load_sto(fullfile(path_FR,'\_ForceReporter_forces.sto'));
Cont_Foot_L_A=zeros(size(PosSp,1),length(BKout.timeBK));
Cont_Foot_R_A=Cont_Foot_L_A;
Cont_Foot_R_B=Cont_Foot_R_A;
Cont_Foot_L_B=Cont_Foot_R_B;

for ii=1:size(PosSp,1)
    sel_force_vec=['ForceGround_Foot_', num2str(ii),'_L_A.ground.force.Y'];
    Cont_Foot_L_A(ii,:)=ForceReport(:,strcmp(sel_force_vec,HeadFR));
    sel_force_vec=['ForceGround_Foot_', num2str(ii),'_R_A.ground.force.Y'];
    Cont_Foot_R_A(ii,:)=ForceReport(:,strcmp(sel_force_vec,HeadFR));
    sel_force_vec=['ForceGround_Foot_', num2str(ii),'_L_B.ground.force.Y'];
    Cont_Foot_L_B(ii,ind_t_cont_foot_l_B)=ForceReport(ind_t_cont_foot_l_B,strcmp(sel_force_vec,HeadFR));
    sel_force_vec=['ForceGround_Foot_', num2str(ii),'_R_B.ground.force.Y'];
    Cont_Foot_R_B(ii,ind_t_cont_foot_r_B)=ForceReport(ind_t_cont_foot_r_B,strcmp(sel_force_vec,HeadFR));
end

if Ground_L_Foot_T_A(2)==Ground_L_Foot_T_B(2) % If both plane A and B are on the same level I choose the level B contact forces cause have the controls on position and velocities
    Cont_Foot_L_A=Cont_Foot_L_B;
end
if Ground_R_Foot_T_A(2)==Ground_R_Foot_T_B(2) % If both plane A and B are on the same level I choose the level B contact forces cause have the controls on position and velocities
    Cont_Foot_R_A=Cont_Foot_R_B;
end

% putting together the A and B contact forces per each sphere

% the final contact forces
Cont_Foot_R=sum(Cont_Foot_R_A)+sum(Cont_Foot_R_B);
Cont_Foot_L=sum(Cont_Foot_L_A)+sum(Cont_Foot_L_B);


%% retrieving Spheres location to calculate CoP
ExecutePK(modelProcessed,modelProcessed_path,fullfile(IKFolder,IKResult),...
    Freq,timeStart,timeEnd,calcn_r_name, calcn_l_name, toes_r_name, ...
    toes_l_name, heel_shift,path_PK,path_Solution);

for ii=1:size(PosSp,1)
    PK_Sp_Res_r=load_sto(fullfile(path_PK,['_PointKinematics_PK_sp',num2str(ii),'_r_pos.sto']));
    X_sp_r(:,ii)=PK_Sp_Res_r(:,2);
    Y_sp_r(:,ii)=PK_Sp_Res_r(:,3);
    Z_sp_r(:,ii)=PK_Sp_Res_r(:,4);
    PK_Sp_Res_l=load_sto(fullfile(path_PK,['_PointKinematics_PK_sp',num2str(ii),'_l_pos.sto']));
    X_sp_l(:,ii)=PK_Sp_Res_l(:,2);
    Y_sp_l(:,ii)=PK_Sp_Res_l(:,3);
    Z_sp_l(:,ii)=PK_Sp_Res_l(:,4);
end

% determining CoP bounds (for future implementation)
% CoP_X_Calcn_l_max=max(X_sp_l');
% CoP_X_Calcn_r_max=max(X_sp_r');
% CoP_X_Calcn_l_min=min(X_sp_l');
% CoP_X_Calcn_r_min=min(X_sp_r');
% 
% CoP_Z_Calcn_l_max=max(Z_sp_l');
% CoP_Z_Calcn_r_max=max(Z_sp_r');
% CoP_Z_Calcn_l_min=min(Z_sp_l');
% CoP_Z_Calcn_r_min=min(Z_sp_r');%

% CoP is calculated as a weighted mean of probes penetrations and their
% centre positions
COP_Calcn_x_r_A=dot(Cont_Foot_R_A,X_sp_r')./sum(Cont_Foot_R_A);
COP_Calcn_y_r_A=dot(Cont_Foot_R_A,Y_sp_r')./sum(Cont_Foot_R_A);
COP_Calcn_z_r_A=dot(Cont_Foot_R_A,Z_sp_r')./sum(Cont_Foot_R_A);
COP_Calcn_x_l_A=dot(Cont_Foot_L_A,X_sp_l')./sum(Cont_Foot_L_A);
COP_Calcn_y_l_A=dot(Cont_Foot_L_A,Y_sp_l')./sum(Cont_Foot_L_A);
COP_Calcn_z_l_A=dot(Cont_Foot_L_A,Z_sp_l')./sum(Cont_Foot_L_A);

COP_Calcn_x_r_B=dot(Cont_Foot_R_B,X_sp_r')./sum(Cont_Foot_R_B);
COP_Calcn_y_r_B=dot(Cont_Foot_R_B,Y_sp_r')./sum(Cont_Foot_R_B);
COP_Calcn_z_r_B=dot(Cont_Foot_R_B,Z_sp_r')./sum(Cont_Foot_R_B);
COP_Calcn_x_l_B=dot(Cont_Foot_L_B,X_sp_l')./sum(Cont_Foot_L_B);
COP_Calcn_y_l_B=dot(Cont_Foot_L_B,Y_sp_l')./sum(Cont_Foot_L_B);
COP_Calcn_z_l_B=dot(Cont_Foot_L_B,Z_sp_l')./sum(Cont_Foot_L_B);

COP_Calcn_x_l_A(isnan(COP_Calcn_x_l_A))=0;
COP_Calcn_z_l_A(isnan(COP_Calcn_z_l_A))=0;
COP_Calcn_x_r_A(isnan(COP_Calcn_x_r_A))=0;
COP_Calcn_z_r_A(isnan(COP_Calcn_z_r_A))=0;
COP_Calcn_y_r_A(isnan(COP_Calcn_y_r_A))=0;
COP_Calcn_y_l_A(isnan(COP_Calcn_y_l_A))=0;

COP_Calcn_x_l_A(~isfinite(COP_Calcn_x_l_A))=0;
COP_Calcn_z_l_A(~isfinite(COP_Calcn_z_l_A))=0;
COP_Calcn_x_r_A(~isfinite(COP_Calcn_x_r_A))=0;
COP_Calcn_z_r_A(~isfinite(COP_Calcn_z_r_A))=0;
COP_Calcn_y_l_A(~isfinite(COP_Calcn_y_l_A))=0;
COP_Calcn_y_r_A(~isfinite(COP_Calcn_y_r_A))=0;

COP_Calcn_x_l_B(isnan(COP_Calcn_x_l_B))=0;
COP_Calcn_z_l_B(isnan(COP_Calcn_z_l_B))=0;
COP_Calcn_x_r_B(isnan(COP_Calcn_x_r_B))=0;
COP_Calcn_z_r_B(isnan(COP_Calcn_z_r_B))=0;
COP_Calcn_y_r_B(isnan(COP_Calcn_y_r_B))=0;
COP_Calcn_y_l_B(isnan(COP_Calcn_y_l_B))=0;

COP_Calcn_x_l_B(~isfinite(COP_Calcn_x_l_B))=0;
COP_Calcn_z_l_B(~isfinite(COP_Calcn_z_l_B))=0;
COP_Calcn_x_r_B(~isfinite(COP_Calcn_x_r_B))=0;
COP_Calcn_z_r_B(~isfinite(COP_Calcn_z_r_B))=0;
COP_Calcn_y_l_B(~isfinite(COP_Calcn_y_l_B))=0;
COP_Calcn_y_r_B(~isfinite(COP_Calcn_y_r_B))=0;

if FullBody
    COP_Hand_x_l=BKout.X_Hand_l;
    COP_Hand_y_l=BKout.Y_Hand_l;
    COP_Hand_z_l=BKout.Z_Hand_l;
    COP_Hand_x_r=BKout.X_Hand_r;
    COP_Hand_y_r=BKout.Y_Hand_r;
    COP_Hand_z_r=BKout.Z_Hand_r;
    COP_Hand_x_l(isnan(COP_Hand_x_l))=0;
    COP_Hand_y_l(isnan(COP_Hand_y_l))=0;
    COP_Hand_z_l(isnan(COP_Hand_z_l))=0;
    COP_Hand_x_r(isnan(COP_Hand_x_r))=0;
    COP_Hand_y_r(isnan(COP_Hand_y_r))=0;
    COP_Hand_z_r(isnan(COP_Hand_z_r))=0;
    COP_Hand_x_l(~isfinite(COP_Hand_x_l))=0;
    COP_Hand_y_l(~isfinite(COP_Hand_y_l))=0;
    COP_Hand_z_l(~isfinite(COP_Hand_z_l))=0;
    COP_Hand_x_r(~isfinite(COP_Hand_x_r))=0;
    COP_Hand_y_r(~isfinite(COP_Hand_y_r))=0;
    COP_Hand_z_r(~isfinite(COP_Hand_z_r))=0;
end
%% projecting CoP onto the contact surfaces
Proj_COP_Calcn_l=zeros(length(BKout.timeBK),3);
Proj_COP_Calcn_r=zeros(length(BKout.timeBK),3);

for i=1:length(BKout.timeBK)
    if ~sum(Cont_Foot_L_B(:,i))==0 && sum(Cont_Foot_L_A(:,i))==0
        Proj_COP_Calcn_l(i,:)=[COP_Calcn_x_l_B(i)' COP_Calcn_y_l_B(i)' COP_Calcn_z_l_B(i)']-(dot([COP_Calcn_x_l_B(i)' COP_Calcn_y_l_B(i)' COP_Calcn_z_l_B(i)']-Ground_L_Foot_T_B,n))*n'; % if I am in a time interval where the contact B is met I have to consider its contact plane, otherwise consider the contact A
    elseif ~sum(Cont_Foot_L_A(:,i))==0 && sum(Cont_Foot_L_B(:,i))==0
        Proj_COP_Calcn_l(i,:)=[COP_Calcn_x_l_A(i)' COP_Calcn_y_l_A(i)' COP_Calcn_z_l_A(i')]-(dot([COP_Calcn_x_l_A(i)' COP_Calcn_y_l_A(i)' COP_Calcn_z_l_A(i)']-Ground_L_Foot_T_A,n))*n';
    else
        Proj_COP_Calcn_l(i,:)=[COP_Calcn_x_l_B(i)' COP_Calcn_y_l_B(i)' COP_Calcn_z_l_B(i)']-(dot([COP_Calcn_x_l_B(i)' COP_Calcn_y_l_B(i)' COP_Calcn_z_l_B(i)']-Ground_L_Foot_T_B,n))*n'; % if I am in a time interval where the contact B is met I have to consider its contact plane, otherwise consider the contact A
    end
    if ~sum(Cont_Foot_R_B(:,i))==0 && sum(Cont_Foot_R_A(:,i))==0
        Proj_COP_Calcn_r(i,:)=[COP_Calcn_x_r_B(i)' COP_Calcn_y_r_B(i)' COP_Calcn_z_r_B(i)']-(dot([COP_Calcn_x_r_B(i)' COP_Calcn_y_r_B(i)' COP_Calcn_z_r_B(i)']-Ground_R_Foot_T_B,n))*n'; % if I am in a time interval where the contact B is met I have to consider its contact plane, otherwise consider the contact A
    elseif ~sum(Cont_Foot_R_A(:,i))==0 && sum(Cont_Foot_R_B(:,i))==0
        Proj_COP_Calcn_r(i,:)=[COP_Calcn_x_r_A(i)' COP_Calcn_y_r_A(i)' COP_Calcn_z_r_A(i')]-(dot([COP_Calcn_x_r_A(i)' COP_Calcn_y_r_A(i)' COP_Calcn_z_r_A(i)']-Ground_R_Foot_T_A,n))*n';
    else
        Proj_COP_Calcn_r(i,:)=[COP_Calcn_x_r_B(i)' COP_Calcn_y_r_B(i)' COP_Calcn_z_r_B(i)']-(dot([COP_Calcn_x_r_B(i)' COP_Calcn_y_r_B(i)' COP_Calcn_z_r_B(i)']-Ground_R_Foot_T_B,n))*n'; % if I am in a time interval where the contact B is met I have to consider its contact plane, otherwise consider the contact A
    end
end

if FullBody
    Cont_Hand_R=ForceReport(:,strcmp('ForceGround_Hand_R.ground.force.Y',HeadFR));
    Cont_Hand_L=ForceReport(:,strcmp('ForceGround_Hand_L.ground.force.Y',HeadFR));
    for i=1:length(BKout.timeBK)
        Proj_COP_Hand_r(i,:)=[COP_Hand_x_r(i,:) COP_Hand_y_r(i,:) COP_Hand_z_r(i,:)]-(dot([COP_Hand_x_r(i,:) COP_Hand_y_r(i,:) COP_Hand_z_r(i,:)]-Ground_R_Hand_T,n))*n';
        Proj_COP_Hand_l(i,:)=[COP_Hand_x_l(i,:) COP_Hand_y_l(i,:) COP_Hand_z_l(i,:)]-(dot([COP_Hand_x_l(i,:) COP_Hand_y_l(i,:) COP_Hand_z_l(i,:)]-Ground_L_Hand_T,n))*n';
    end
end

Sph_COP_Calcn_l=Proj_COP_Calcn_l; 
Sph_COP_Calcn_r=Proj_COP_Calcn_r;

disp('CoP estimation successfully completed')


%% Check of Body model type for contact index
if FullBody
    Cont=[Cont_Foot_R',Cont_Foot_L',Cont_Hand_R,Cont_Hand_L];
    Cont_indx=zeros(length(BKout.timeBK),4);
    Cont_indx(Cont(:)~=0)=1;
else
    Cont=[Cont_Foot_R',Cont_Foot_L'];
    Cont_indx=zeros(length(BKout.timeBK),2);
    Cont_indx(Cont(:)~=0)=1;
end

%% find Single Stance (SS) left and right
SS_R=BKout.timeBK(and(abs(Cont_Foot_R)>0,Cont_Foot_L==0));
SS_L=BKout.timeBK(and(abs(Cont_Foot_L)>0,Cont_Foot_R==0));

%% Creating coordinate constaints list
if model.getConstraintSet.getSize ~= 0
for cs=0:model.getConstraintSet.getSize-1
    if ~isempty(org.opensim.modeling.CoordinateCouplerConstraint.safeDownCast(model.getConstraintSet.get(cs)))
   ConstSet(cs+1,:)=string(org.opensim.modeling.CoordinateCouplerConstraint.safeDownCast(model.getConstraintSet.get(cs)).getDependentCoordinateName); %SafeDownCasting the 
    end
end
else
    ConstSet="No Coupled Constrained Joints";
end

%% start of computation
disp('Starting GRF calculation ...')
time=zeros(length(BKout.timeBK)-1,1);

% actuators names
if FullBody
Act={'GRF_Foot_R_X','GRF_Foot_R_Y','GRF_Foot_R_Z','GRM_Foot_R_X','GRM_Foot_R_Y','GRM_Foot_R_Z','GRF_Foot_L_X','GRF_Foot_L_Y','GRF_Foot_L_Z','GRM_Foot_L_X','GRM_Foot_L_Y','GRM_Foot_L_Z','GRF_Hand_R_X','GRF_Hand_R_Y','GRF_Hand_R_Z','GRM_Hand_R_X','GRM_Hand_R_Y','GRM_Hand_R_Z','GRF_Hand_L_X','GRF_Hand_L_Y','GRF_Hand_L_Z','GRM_Hand_L_X','GRM_Hand_L_Y','GRM_Hand_L_Z'};
else
     Act={'GRF_Foot_R_X','GRF_Foot_R_Y','GRF_Foot_R_Z','GRM_Foot_R_X','GRM_Foot_R_Y','GRM_Foot_R_Z','GRF_Foot_L_X','GRF_Foot_L_Y','GRF_Foot_L_Z','GRM_Foot_L_X','GRM_Foot_L_Y','GRM_Foot_L_Z'};
end

model_orig=org.opensim.modeling.Model(modelPath);
% state0=model_orig.initSystem();
% motionStore = org.opensim.modeling.Storage(motPath);     % input kinematics for states

% adding actuators to the solver
ActuatorFile=org.opensim.modeling.ForceSet();
N_coord=0;
% coordinate actuators including pelvis
for u=0:model_orig.getJointSet.getSize-1 % for every joint
    for v=0:model_orig.getJointSet.get(u).numCoordinates-1 % get every coordinate from every joint
        coord=model_orig.getJointSet.get(u).get_coordinates(v);
        if ~coord.get_locked && ~contains(convertCharsToStrings(toString(coord).toCharArray),ConstSet)
            CoordActuator=org.opensim.modeling.CoordinateActuator();
            CoordActuator.setCoordinate(coord);
            CoordActuator.set_appliesForce(true);
            CoordActuator.setName(coord.toString);
            if contains(convertCharsToStrings(toString(coord).toCharArray),'pelvis')   % se pelvi
                CoordActuator.set_min_control(-PelvisHighestValue);
                CoordActuator.set_max_control(PelvisHighestValue);
                CoordActuator.set_optimal_force(PelvisForce)
                ActuatorFile.append(CoordActuator);
            else
                CoordActuator.set_min_control(-CoordValue);
                CoordActuator.set_max_control(CoordValue);
                CoordActuator.set_optimal_force(CoordForce);
                ActuatorFile.append(CoordActuator);
            end
            N_coord=N_coord+1;
        end
    end
end

% start of cycles for GRF&Ms estimation
for t=1:length(BKout.timeBK)-1

    SoStartIter=BKout.timeBK(t);
    SoEndIter=BKout.timeBK(t+1);
    crop_t_s=SoStartIter-0.5;
    crop_t_e=SoEndIter+0.5;
    motionIterStore=org.opensim.modeling.Storage(motPath);     % input kinematics for states
    motionIterStore.crop(crop_t_s,crop_t_e);
    
    model_orig.getForceSet.clearAndDestroy();
    model_orig.getAnalysisSet.clearAndDestroy();
    model_orig.set_ForceSet(ActuatorFile);

    
    % external actuators
    for i = 1:length(Act)
        if contains(Act{i},"Foot_R")
            body=calcn_r_name;
            Point=[Proj_COP_Calcn_r(t,1) Proj_COP_Calcn_r(t,2) Proj_COP_Calcn_r(t,3)];
        else if contains(Act{i},"Foot_L")
                body=calcn_l_name;
                Point=[Proj_COP_Calcn_l(t,1) Proj_COP_Calcn_l(t,2) Proj_COP_Calcn_l(t,3)];
        else if contains(Act{i},"Hand_R")
                body=hand_r_name;
                Point=[Proj_COP_Hand_r(t,1) Proj_COP_Hand_r(t,2) Proj_COP_Hand_r(t,3)];
        else if contains(Act{i},"Hand_L")
                body=hand_l_name;
                Point=[Proj_COP_Hand_l(t,1) Proj_COP_Hand_l(t,2) Proj_COP_Hand_l(t,3)];
        end
        end
        end
        end

        if contains(Act{i},"X")
            direction=Rot(:,1);
        else if contains(Act{i},"Y")
                direction=Rot(:,2);
        else
            direction=Rot(:,3);
        end
        end

        switch true
            case i>=1 && i<=6
                m=1;
            case i>=7 && i<=12
                m=2;
            case i>=13 && i<=18
                m=3;
            case i>=19 && i<=24
                m=4;
        end

        if contains(Act{i},'GRF')
            ExtForce=org.opensim.modeling.PointActuator();
            ExtForce.setName(Act{i})
            ExtForce.set_appliesForce(1);
            ExtForce.set_point_is_global(false)
            if Cont(t,m)
                if ismember(i,[2,8,14,20]) % Just for vertical direction
                    ExtForce.setOptimalForce(Force)
                    ExtForce.setMinControl(0);
                    ExtForce.setMaxControl(HighestValue);
                else
                    ExtForce.setOptimalForce(Force)
                    ExtForce.setMinControl(-HighestValue);
                    ExtForce.setMaxControl(HighestValue);
                end
            else
                ExtForce.setOptimalForce(PelvisForce)
                ExtForce.setMinControl(-PelvisHighestValue);
                ExtForce.setMaxControl(PelvisHighestValue);
            end
            ExtForce.set_direction(org.opensim.modeling.Vec3.createFromMat(direction));
            ExtForce.set_force_is_global(true);
            ExtForce.set_point_is_global(true);
            ExtForce.set_body(model.getBodySet.get(body).toString);
            ExtForce.set_point(org.opensim.modeling.Vec3.createFromMat(Point))
        else
            ExtForce=org.opensim.modeling.TorqueActuator();
            ExtForce.setName(Act{i});
            ExtForce.set_appliesForce(1);
            ExtForce.set_bodyA(body);
            if Cont(t,m)
                if ismember(i,[2,8,14,20]) % Just for vertical direction
                    ExtForce.setOptimalForce(Moment)
                    ExtForce.setMinControl(0);
                    ExtForce.setMaxControl(HighestValue);
                else
                    ExtForce.setOptimalForce(Moment);
                    ExtForce.setMinControl(-HighestValue);
                    ExtForce.setMaxControl(HighestValue);
                end
            else
                ExtForce.setOptimalForce(PelvisForce);
                ExtForce.setMinControl(-PelvisHighestValue);
                ExtForce.setMaxControl(PelvisHighestValue);
            end
            ExtForce.setBodyB(model.getGround)
            ExtForce.set_axis(org.opensim.modeling.Vec3.createFromMat(direction));
            ExtForce.set_torque_is_global(true);
        end
        % ActuatorFile.append(ExtForce);
        model_orig.getForceSet.append(ExtForce);
    end
    %model_orig.print(modelPath)

    % execute Statict opt

    state0=model_orig.initSystem();
    Antool = org.opensim.modeling.AnalyzeTool(model_orig);          % use the original model to run the analysis
    Antool.setInitialTime(SoStartIter);
    Antool.setFinalTime(SoEndIter);
    Antool.setSolveForEquilibrium(SolveForEquilibrium);

    if AutoFreq
        Antool.setLowpassCutoffFrequency(f_cutoff(t));
    else
        Antool.setLowpassCutoffFrequency(Freq);
    end

    Antool.setStatesFromMotion(state0,motionIterStore,true);
    Antool.setPrintResultFiles(false);

    if ~isempty(FextPath) % just if I select known External Forces
        Antool.setExternalLoadsFileName(FextPath)
    end

    % --- Static Optimization (Analysis)
    SOtool = org.opensim.modeling.StaticOptimization();
    SOtool.setOn(true);
    SOtool.setUseModelForceSet(true);
    SOtool.setUseMusclePhysiology(false);
    SOtool.setPrintResultFiles(false);
    Antool.updAnalysisSet.cloneAndAppend(SOtool);
    Antool.addAnalysisSetToModel();

    % --- Run in-process
    ok = Antool.run(false);
    if ~ok
        error('AnalyzeTool.run() failed.');
    end

    % retrieving the stat opt analysis
    soRan = [];
    for i = 0:Antool.getModel.getAnalysisSet.getSize()-1
        a = Antool.getModel.getAnalysisSet.get(i);
        if strcmp(char(a.getConcreteClassName()), 'StaticOptimization')
            soRan =org.opensim.modeling.StaticOptimization.safeDownCast(a);
            break
        end
    end
    if isempty(soRan)
        error('Error during StaticOptimization');
    end

    %soRan=Antool.getModel.getAnalysisSet.get(1)

    forceSto = soRan.getForceStorage();

    % loading result from memory
    [t_for, PredGRF_iter, HeadSTO] = storageToMat(forceSto);
    PredGRF(t,:)=[mean(PredGRF_iter,[1,length(PredGRF_iter)])];
    time(t)=t_for(end);
    disp(['Processing time: ', num2str(BKout.timeBK(t))]);
end
indxAct=contains(HeadSTO,Act);
GRFData=PredGRF(:,indxAct);
for s=1:length(Act)/3
    GRF_vec=GRFData(:,(s-1)*3+1:s*(3));
    GRFData(:,(s-1)*3+1:s*(3))=(Rot*GRF_vec')';
end
PredGRF(:,indxAct)=GRFData;
%% Writing MOT file
pelvis_list=PredGRF(:,strcmp(HeadSTO,'pelvis_list'));
pelvis_tilt=PredGRF(:,strcmp(HeadSTO,'pelvis_tilt'));
pelvis_rotation=PredGRF(:,strcmp(HeadSTO,'pelvis_rotation'));
pelvis_tx=PredGRF(:,strcmp(HeadSTO,'pelvis_tx'));
pelvis_ty=PredGRF(:,strcmp(HeadSTO,'pelvis_ty'));
pelvis_tz=PredGRF(:,strcmp(HeadSTO,'pelvis_tz'));

Calcn_l_Fx=PredGRF(:,strcmp(HeadSTO,'GRF_Foot_L_X'));
Calcn_l_Fy=PredGRF(:,strcmp(HeadSTO,'GRF_Foot_L_Y'));
Calcn_l_Fz=PredGRF(:,strcmp(HeadSTO,'GRF_Foot_L_Z'));

Calcn_r_Fx=PredGRF(:,strcmp(HeadSTO,'GRF_Foot_R_X'));
Calcn_r_Fy=PredGRF(:,strcmp(HeadSTO,'GRF_Foot_R_Y'));
Calcn_r_Fz=PredGRF(:,strcmp(HeadSTO,'GRF_Foot_R_Z'));

Calcn_l_Mx=PredGRF(:,strcmp(HeadSTO,'GRM_Foot_L_X'));
Calcn_l_My=PredGRF(:,strcmp(HeadSTO,'GRM_Foot_L_Y'));
Calcn_l_Mz=PredGRF(:,strcmp(HeadSTO,'GRM_Foot_L_Z'));

Calcn_r_Mx=PredGRF(:,strcmp(HeadSTO,'GRM_Foot_R_X'));
Calcn_r_My=PredGRF(:,strcmp(HeadSTO,'GRM_Foot_R_Y'));
Calcn_r_Mz=PredGRF(:,strcmp(HeadSTO,'GRM_Foot_R_Z'));
% storing var for COP computation
Calcn_r_Mz_COP=Calcn_r_Mz;
Calcn_r_Mx_COP=Calcn_r_Mx;
Calcn_l_Mz_COP=Calcn_l_Mz;
Calcn_l_Mx_COP=Calcn_l_Mx;
Calcn_l_Fy_COP=Calcn_l_Fy;
Calcn_r_Fy_COP=Calcn_r_Fy;
%
if FullBody
    Hand_l_Fx=PredGRF(:,strcmp(HeadSTO,'GRF_Hand_L_X'));
    Hand_l_Fy=PredGRF(:,strcmp(HeadSTO,'GRF_Hand_L_Y'));
    Hand_l_Fz=PredGRF(:,strcmp(HeadSTO,'GRF_Hand_L_Z'));

    Hand_r_Fx=PredGRF(:,strcmp(HeadSTO,'GRF_Hand_R_X'));
    Hand_r_Fy=PredGRF(:,strcmp(HeadSTO,'GRF_Hand_R_Y'));
    Hand_r_Fz=PredGRF(:,strcmp(HeadSTO,'GRF_Hand_R_Z'));

    Hand_l_Mx=PredGRF(:,strcmp(HeadSTO,'GRM_Hand_L_X'));
    Hand_l_My=PredGRF(:,strcmp(HeadSTO,'GRM_Hand_L_Y'));
    Hand_l_Mz=PredGRF(:,strcmp(HeadSTO,'GRM_Hand_L_Z'));

    Hand_r_Mx=PredGRF(:,strcmp(HeadSTO,'GRM_Hand_R_X'));
    Hand_r_My=PredGRF(:,strcmp(HeadSTO,'GRM_Hand_R_Y'));
    Hand_r_Mz=PredGRF(:,strcmp(HeadSTO,'GRM_Hand_R_Z'));
end
% filtering GRF
[b,a]=butter(4,Freq/(Fs/2),"low");
Calcn_l_Fx=filtfilt(b,a,Calcn_l_Fx);
Calcn_l_Fy=filtfilt(b,a,Calcn_l_Fy);
Calcn_l_Fz=filtfilt(b,a,Calcn_l_Fz);
Calcn_r_Fx=filtfilt(b,a,Calcn_r_Fx);
Calcn_r_Fy=filtfilt(b,a,Calcn_r_Fy);
Calcn_r_Fz=filtfilt(b,a,Calcn_r_Fz);
Calcn_l_Mx=filtfilt(b,a,Calcn_l_Mx);
Calcn_l_My=filtfilt(b,a,Calcn_l_My);
Calcn_l_Mz=filtfilt(b,a,Calcn_l_Mz);
Calcn_r_Mx=filtfilt(b,a,Calcn_r_Mx);
Calcn_r_My=filtfilt(b,a,Calcn_r_My);
Calcn_r_Mz=filtfilt(b,a,Calcn_r_Mz);
if FullBody
    Hand_l_Fx=filtfilt(b,a,Hand_l_Fx);
    Hand_l_Fy=filtfilt(b,a,Hand_l_Fy);
    Hand_l_Fz=filtfilt(b,a,Hand_l_Fz);
    Hand_r_Fx=filtfilt(b,a,Hand_r_Fx);
    Hand_r_Fy=filtfilt(b,a,Hand_r_Fy);
    Hand_r_Fz=filtfilt(b,a,Hand_r_Fz);
    Hand_l_Mx=filtfilt(b,a,Hand_l_Mx);
    Hand_l_My=filtfilt(b,a,Hand_l_My);
    Hand_l_Mz=filtfilt(b,a,Hand_l_Mz);
    Hand_r_Mx=filtfilt(b,a,Hand_r_Mx);
    Hand_r_My=filtfilt(b,a,Hand_r_My);
    Hand_r_Mz=filtfilt(b,a,Hand_r_Mz);
end


%% control on GRF: if not in contact with ground the GRF&M become 0
for i=1:length(time)
    if ~Cont_Foot_R(i)
        Calcn_r_Fx(i)=0;
        Calcn_r_Fy(i)=0;
        Calcn_r_Fz(i)=0;
        Calcn_r_Mx(i)=0;
        Calcn_r_My(i)=0;
        Calcn_r_Mz(i)=0;
    end
end
for i=1:length(time)
    if ~Cont_Foot_L(i)
        Calcn_l_Fx(i)=0;
        Calcn_l_Fy(i)=0;
        Calcn_l_Fz(i)=0;
        Calcn_l_Mx(i)=0;
        Calcn_l_My(i)=0;
        Calcn_l_Mz(i)=0;
    end
end
if FullBody
    for i=1:length(time)
        if ~Cont_Hand_L(i)
            Hand_l_Fx(i)=0;
            Hand_l_Fy(i)=0;
            Hand_l_Fz(i)=0;
            Hand_l_Mx(i)=0;
            Hand_l_My(i)=0;
            Hand_l_Mz(i)=0;
        end
    end
    for i=1:length(time)
        if ~Cont_Hand_R(i)
            Hand_r_Fx(i)=0;
            Hand_r_Fy(i)=0;
            Hand_r_Fz(i)=0;
            Hand_r_Mx(i)=0;
            Hand_r_My(i)=0;
            Hand_r_Mz(i)=0;
        end
    end
end

Corr_COP_Calcn_r_z=zeros(length(time),1);
Corr_COP_Calcn_r_x=zeros(length(time),1);
Corr_COP_Calcn_l_z=zeros(length(time),1);
Corr_COP_Calcn_l_x=zeros(length(time),1);
% fusing FCP and CoP from dynamics

if ~isempty(SS_R)
    indx_ss_r=ismember(time,SS_R);
end
if ~isempty(SS_L)
    indx_ss_l=ismember(time,SS_L);
end

if exist("indx_ss_r",'var')
    Corr_COP_Calcn_r_x=Calcn_r_Mz_COP./Calcn_r_Fy_COP;
    Corr_COP_Calcn_r_z=-Calcn_r_Mx_COP./Calcn_r_Fy_COP;
    [Proj_COP_Calcn_r(:,1),Proj_COP_Calcn_r(:,3)]=fuseCOP(Sph_COP_Calcn_r(:,1),Sph_COP_Calcn_r(:,3),Corr_COP_Calcn_r_x(1:length(time))+Sph_COP_Calcn_r(1:length(time),1),Corr_COP_Calcn_r_z(1:length(time))+Sph_COP_Calcn_r(1:length(time),3),indx_ss_r(1:length(time)));
end
if exist("indx_ss_l",'var')
    Corr_COP_Calcn_l_x=Calcn_l_Mz_COP./Calcn_l_Fy_COP;
    Corr_COP_Calcn_l_z=-Calcn_l_Mx_COP./Calcn_l_Fy_COP;
    [Proj_COP_Calcn_l(:,1),Proj_COP_Calcn_l(:,3)]=fuseCOP(Sph_COP_Calcn_l(:,1),Sph_COP_Calcn_l(:,3),Corr_COP_Calcn_l_x(1:length(time))+Sph_COP_Calcn_l(1:length(time),1),Corr_COP_Calcn_l_z(1:length(time))+Sph_COP_Calcn_l(1:length(time),3),indx_ss_l(1:length(time)));
end
%% computing free moment for left and right calcn
Calcn_l_My=Calcn_l_My-(Sph_COP_Calcn_l(1:length(time),1)-Proj_COP_Calcn_l(1:length(time),1)).*Calcn_l_Fz+(Sph_COP_Calcn_l(1:length(time),3)-Proj_COP_Calcn_l(1:length(time),3)).*Calcn_l_Fx;
Calcn_r_My=Calcn_r_My-(Sph_COP_Calcn_r(1:length(time),1)-Proj_COP_Calcn_r(1:length(time),1)).*Calcn_r_Fz+(Sph_COP_Calcn_r(1:length(time),3)-Proj_COP_Calcn_r(1:length(time),3)).*Calcn_r_Fx;
%% Writing Static Optimization .sto Result file
info_SO=string({'Actuation Force'; ['version=' num2str(1)];['nRows=' num2str(length(time))];['nColumns=' num2str(length(PredGRF(1,:)))];'inDegrees=yes';'endheader'});																																			
outFile_CMC=[info_SO;strjoin(string(HeadSTO'));string(num2str(PredGRF))];
fileID_CMC=fopen(fullfile(path_iter,'Force_eq.sto'),"w+");
for k=1:length(outFile_CMC)
fprintf(fileID_CMC,'%s',[outFile_CMC(k)]);
fprintf(fileID_CMC,'\n');
end
fclose(fileID_CMC);

%% Writing .MOT file
Data_calcn_l=[Calcn_l_Fx Calcn_l_Fy Calcn_l_Fz Proj_COP_Calcn_l(1:end-1,1) Proj_COP_Calcn_l(1:end-1,2) Proj_COP_Calcn_l(1:end-1,3) zeros(length(Calcn_l_Fy),1) Calcn_l_My zeros(length(Calcn_l_Fy),1)];
Data_calcn_r=[Calcn_r_Fx Calcn_r_Fy Calcn_r_Fz Proj_COP_Calcn_r(1:end-1,1) Proj_COP_Calcn_r(1:end-1,2) Proj_COP_Calcn_r(1:end-1,3) zeros(length(Calcn_l_Fy),1) Calcn_r_My zeros(length(Calcn_l_Fy),1)];


if FullBody
    Data_hand_l=[Hand_l_Fx Hand_l_Fy Hand_l_Fz COP_Hand_x_l(1:length(time))  COP_Hand_y_l(1:length(time)) COP_Hand_z_l(1:length(time)) Hand_l_Mx Hand_l_My Hand_l_Mz];
    Data_hand_r=[Hand_r_Fx Hand_r_Fy Hand_r_Fz COP_Hand_x_r(1:length(time)) COP_Hand_y_r(1:length(time)) COP_Hand_z_r(1:length(time)) Hand_r_Mx Hand_r_My Hand_r_Mz];
    Data=[time Data_calcn_l Data_calcn_r Data_hand_l Data_hand_r];
    header="time	ground_force_calcn_l_vx	ground_force_calcn_l_vy	ground_force_calcn_l_vz	ground_force_calcn_l_px	ground_force_calcn_l_py	ground_force_calcn_l_pz	ground_torque_calcn_l_vx	ground_torque_calcn_l_vy	ground_torque_calcn_l_vz	ground_force_calcn_r_vx	ground_force_calcn_r_vy	ground_force_calcn_r_vz	ground_force_calcn_r_px	ground_force_calcn_r_py	ground_force_calcn_r_pz	ground_torque_calcn_r_vx	ground_torque_calcn_r_vy	ground_torque_calcn_r_vz	ground_force_hand_l_vx	ground_force_hand_l_vy	ground_force_hand_l_vz	ground_force_hand_l_px	ground_force_hand_l_py	ground_force_hand_l_pz	ground_torque_hand_l_vx	ground_torque_hand_l_vy	ground_torque_hand_l_vz	ground_force_hand_r_vx	ground_force_hand_r_vy	ground_force_hand_r_vz	ground_force_hand_r_px	ground_force_hand_r_py	ground_force_hand_r_pz	ground_torque_hand_r_vx	ground_torque_hand_r_vy	ground_torque_hand_r_vz";
else
    Data=[time Data_calcn_l Data_calcn_r];
    header="time	ground_force_calcn_l_vx	ground_force_calcn_l_vy	ground_force_calcn_l_vz	ground_force_calcn_l_px	ground_force_calcn_l_py	ground_force_calcn_l_pz	ground_torque_calcn_l_vx	ground_torque_calcn_l_vy	ground_torque_calcn_l_vz	ground_force_calcn_r_vx	ground_force_calcn_r_vy	ground_force_calcn_r_vz	ground_force_calcn_r_px	ground_force_calcn_r_py	ground_force_calcn_r_pz	ground_torque_calcn_r_vx	ground_torque_calcn_r_vy	ground_torque_calcn_r_vz";
end
info=string({['GRF' IKResult]; ['version=' num2str(1)];['nRows=' num2str(length(time))];['nColumns=' num2str(length(Data(1,:)))];'inDegrees=yes';'endheader'});
outFile=[info;header;string(num2str(Data))];
fileID=fopen(fullfile(path_Solution,['Predicted_GRF_' IKResult]),"w+");
for k=1:length(outFile)
    fprintf(fileID,'%s',[outFile(k)]);
    fprintf(fileID,'\n');
end
fclose(fileID);

elpsTime=round(toc,1);

fprintf('Analysis successfully completed in %.1f seconds',elpsTime);
if nargout > 0
    output = struct();
    output.SolutionFolder = path_Solution;
    output.PredictedGRFFile = fullfile(path_Solution,['Predicted_GRF_' IKResult]);
    output.ModelProcessedPath = modelProcessed_path;
    output.ElapsedTimeSeconds = elpsTime;
    output.ExternalForceSetupPath = ExternalForceSetupPath;
end
end


%% helper functions
function bodyName = getCfgContactBodyName(cfg, fieldName, defaultName)
% Returns a contact body name from cfg.ContactBodies.
bodyName = string(defaultName);
if isfield(cfg, 'ContactBodies') && isstruct(cfg.ContactBodies) && isfield(cfg.ContactBodies, fieldName)
    value = string(cfg.ContactBodies.(fieldName));
    if strlength(strtrim(value)) > 0
        bodyName = strtrim(value);
    end
end
bodyName = char(bodyName);
end

function bodyName = validateOpenSimBodyName(model, requestedName, roleLabel)
% Validates a body name against model.getBodySet.
requestedName = string(strtrim(string(requestedName)));
if strlength(requestedName) == 0
    error('The contact body name for %s is empty.', roleLabel);
end

bodySet = model.getBodySet;
nBodies = bodySet.getSize;
available = strings(nBodies,1);
for ii = 0:nBodies-1
    available(ii+1) = convertCharsToStrings(bodySet.get(ii).toString.toCharArray);
end

matchIdx = find(strcmpi(requestedName, available), 1);
if isempty(matchIdx)
    preview = strjoin(available(1:min(numel(available),30)), ', ');
    if numel(available) > 30
        preview = preview + ', ...';
    end
    error(['Contact body name "%s" for %s was not found in the selected model. ' ...
        'Edit it in Advanced settings > Contact body names. Available body names include: %s'], ...
        char(requestedName), roleLabel, char(preview));
end

bodyName = char(available(matchIdx));
end
