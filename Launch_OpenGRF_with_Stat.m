clc
close all
clear all
import org.opensim.modeling.*
%% choosing the parsing folder
[MainFolder]=uigetdir("select folder containing all the subjects to parse");
Dir=dir(MainFolder);
% number of subjct =2
Summary=struct;
Stat=struct;
for ss=1:1

Subj_Fold=fullfile(MainFolder,Dir(ss+2).name); % aggiungo +2 per portarmi sui valori di iteresse nella dir

% uploading the kinematics
IKFolder=fullfile(Subj_Fold,'IK');

IK_Dir=dir(IKFolder);
Num_sim=0;
for jj=1:length(IK_Dir)
if contains(IK_Dir(jj).name,'.mot')
Num_sim=Num_sim+1;
end
end

for nn=1:Num_sim

% uploading the model
Subj_Dir=dir(Subj_Fold);
ModelFolder=Subj_Fold;

for mm=1:length(Subj_Dir)
    if contains(Subj_Dir(mm).name,'osim') && contains(Subj_Dir(mm).name,'AB')
        ModelFile=Subj_Dir(mm).name;
    end
end
modelPath=fullfile(ModelFolder,ModelFile);
model=Model(modelPath);

% uploading current kinematics
IKResult=IK_Dir(nn+2).name;
TaskName=erase(IKResult,'.mot');
%[IKResult,IKFolder]=uigetfile('*.mot', 'Choose IK Result file');
[Motion, HeadMotion]=load_mot(fullfile(IKFolder,IKResult));
%model.print(fullfile(ModelFolder,'OriginalModel.osim'));
Fs=1/(Motion(2,1)-Motion(1,1));

%% input data
InfoDir=dir(fullfile(Subj_Fold,'Info'));
for ii=1:length(InfoDir)
    if contains(InfoDir(ii).name,TaskName)
        InfoData=readcell(fullfile(Subj_Fold,'Info',InfoDir(ii).name));
    end
end

% Info=inputdlg({'Start time:', 'End time:','Low pass cut-off frequency (Hz):'},...
%     'Input data',[1 50; 1 50;1 50], {string(Motion(1,1)),string(Motion(end,1)),'6'});
timeStart=InfoData{1,1};
timeEnd=InfoData{1,2};
Freq=6;
%F_th_sp=str2double(Info{4});
%
%Ground_R_Foot_T=str2double({'-7.20298';'0.0885227';'-5.07797'}); To use in
%Ramp trial
% Ground_R_Foot_T_A=str2double({'0';'0.47';'0'}); % contact used for caibration
% 
Ground_R_Foot_T_A=[InfoData{2,1} InfoData{2,2} InfoData{2,3}]; % contact used for caibration
Ground_R_Foot_T_B=[InfoData{3,1} InfoData{3,2} InfoData{3,3}];

Ground_L_Foot_T_A=[InfoData{4,1} InfoData{4,2} InfoData{4,3}]; % contact used for caibration
Ground_L_Foot_T_B=[InfoData{5,1} InfoData{5,2} InfoData{5,3}];

% Ground_R_Foot_T_A=str2double({'0';'0';'0'}); % contact used for caibration
% Ground_R_Foot_T_B=str2double({'0';'0';'0'});
% 
% Ground_L_Foot_T_A=str2double({'0';'0';'0'}); % contact used for caibration
% Ground_L_Foot_T_B=str2double({'0';'0';'0'});

Ground_R_Hand_T=str2double({'0';'0';'0'});
Ground_L_Hand_T=str2double({'0';'0';'0'});
% Hp: the orientatation of the ground is the same per each body
%x_angle=str2double('-0.314159'); & to use in ramp trials
x_angle=InfoData{6,1};
z_angle=InfoData{6,2} -pi/2;

Rot_x=[1 0 0;
    0 cos(x_angle) -sin(x_angle);
    0 sin(x_angle) cos(x_angle)];
Rot_z=[-sin(z_angle) cos(z_angle) 0;
     cos(z_angle) -sin(z_angle) 0;
     0 0 1];
Rot=Rot_x*Rot_z; % 2 successive rotations, first around X and then around z
n=Rot(:,2); % the versor normal to the plane expressed in ground coordinates
Ground_Rot=[x_angle 0 z_angle];
%% Creation of storage directory for iterations 
path_Solution=fullfile(Subj_Fold,"Solution",TaskName);
path_BK=fullfile(path_Solution,"BK");
path_PK=fullfile(path_Solution,"PK");
path_FR=fullfile(path_Solution,"FR");
path_iter=fullfile(path_Solution,"Iterations");
mkdir(path_Solution);
mkdir(path_BK);
mkdir(path_PK);
mkdir(path_FR);
mkdir(path_iter);
%% check for typology of model (lower body of full body)
for i=0:model.getBodySet.getSize-1
if contains(string(model.getBodySet.get(i)),"hand")
    FullBody=1;
else 
    FullBody=0;
end
end
%% find cut-off frequence vector
% NFFT=1024;
% M=Motion(:,2:end);
% [b,a]=butter(4,Freq/(Fs/2),"low");
% M=filtfilt(b,a,M);
% M=M-mean(M);
% t_vect=Motion(:,1);
% window=hann(floor(size(M,1)/4.5));
% for j=1:size(M,2)
% % [S,f_S,t_S]=stft(M(all([(t_vect>=timeStart)';(t_vect<=timeEnd)']),j),Fs,"FFTLength",NFFT,"Window",window,"FrequencyRange","onesided");
% % mesh(t_S,f_S,real(abs(S))),xlabel('time'),ylabel('freq'),zlabel('abs(S)');
% [S,t_S,f_S]=STFT(M(all([(t_vect>=timeStart)';(t_vect<=timeEnd)']),j),window,Fs,NFFT);
% 
% for i=1:length(t_S)
% Freq_matrix(j,i)=sum(f_S'.*real(abs(S(:,i))))/sum(real(abs(S(:,i))));
% end
% end
% Freq_cutoff=max(Freq_matrix);
% %Freq_cutoff(Freq_cutoff>Freq)=Freq;
% Freq_cutoff=Freq*ones(1,length(t_S));
 %figure, plot(Freq_cutoff)
%% CMC input settings
CMCStartTime=timeStart;
CMCEndTime=timeEnd;
%
CoordForce=1000;
CoordValue=1;
%
Force=10000;
Moment=100;
HighestValue=1; %% do not modify
LowestValue=0;
%
PelvisForceBase=1; % Pelvis force excitation set by default to [-inf inf]
PelvisHighestValueBase=1000;
PelvisLowestValueBase=0;
%
PelvisForceFlight=1000;
PelvisHighestValueFlight=1;
PelvisLowestValueFlight=0;
%
TimeWindow=round(1/Fs,5);
SelectFastTarget=1;
IgnoreMuscles=true;
SolveForEquilibrium=0;
%
Sp_radius=0.02;
Sp_stiffness=1E7;
heel_shift=0.00;
%
% Pen=0.003;
% F_th_sp=Sp_stiffness*(Pen)^(3/2);
F_th_sp=400;
%d=(F_th_sp/Sp_stiffness)^(2/3);
%% Automating the contact geometry creation
% detect model body names of body in contact with ground
for i=0:model.getBodySet.getSize-1
    body_name=convertCharsToStrings(model.getBodySet.get(i).toString.toCharArray);
    if strcmpi(body_name,'calcn_r')
        calcn_r_name=convertCharsToStrings(model.getBodySet.get(i).toString.toCharArray);
    elseif strcmpi(body_name,'calcn_l')
            calcn_l_name=convertCharsToStrings(model.getBodySet.get(i).toString.toCharArray);
    elseif strcmpi(body_name,'toes_r')
            toes_r_name=convertCharsToStrings(model.getBodySet.get(i).toString.toCharArray);
    elseif strcmpi(body_name,'toes_l')
            toes_l_name=convertCharsToStrings(model.getBodySet.get(i).toString.toCharArray);
    elseif strcmpi(body_name,'hand_r')
            hand_r_name=convertCharsToStrings(model.getBodySet.get(i).toString.toCharArray);
    elseif strcmpi(body_name,'hand_l')
            hand_l_name=convertCharsToStrings(model.getBodySet.get(i).toString.toCharArray);
    end
end
%% Calibration of Contact Spheres on the model
disp('Contact elements calibration in progress, please wait ...')
%% BK executing
state=model.initSystem();
AnTool2=AnalyzeTool();
AnTool2.setModel(model);
AnTool2.setModelFilename(modelPath);
AnTool2.setCoordinatesFileName(fullfile(IKFolder,IKResult));
AnTool2.setLowpassCutoffFrequency(Freq);
AnTool2.setSolveForEquilibrium(1);
AnTool2.setStartTime(timeStart);
AnTool2.setFinalTime(timeEnd);
AnTool2.setResultsDir(path_BK);
BKTool=BodyKinematics();
BKTool.setStartTime(timeStart);
BKTool.setEndTime(timeEnd);
AnTool2.getAnalysisSet.cloneAndAppend(BKTool);
AnTool2.print(fullfile(path_Solution,'Setup_BK.xml'));
%AnTool2.run;

BK_execute=AnalyzeTool(fullfile(path_Solution,'Setup_BK.xml'));
BK_execute.run;
%% BK retrieving
[BodyPos,Head_BK]=load_sto(fullfile(path_BK,'_BodyKinematics_pos_global.sto'));
[BodyVel,Head_BK]=load_sto(fullfile(path_BK,'_BodyKinematics_vel_global.sto'));
[BodyAcc,Head_BK]=load_sto(fullfile(path_BK,'_BodyKinematics_acc_global.sto'));
timeBK=round(BodyPos(:,1),8);


if FullBody
X_Hand_l=BodyPos(:,strcmp(Head_BK,'hand_l_X'));
Y_Hand_l=BodyPos(:,strcmp(Head_BK,'hand_l_Y'));
Z_Hand_l=BodyPos(:,strcmp(Head_BK,'hand_l_Z'));
X_Hand_r=BodyPos(:,strcmp(Head_BK,'hand_r_X'));
Y_Hand_r=BodyPos(:,strcmp(Head_BK,'hand_r_Y'));
Z_Hand_r=BodyPos(:,strcmp(Head_BK,'hand_r_Z'));
end
%% toes p,v,a
X_Toes_l=BodyPos(:,strcmp(Head_BK,'toes_l_X'));
Y_Toes_l=BodyPos(:,strcmp(Head_BK,'toes_l_Y'));
Z_Toes_l=BodyPos(:,strcmp(Head_BK,'toes_l_Z'));
X_Toes_r=BodyPos(:,strcmp(Head_BK,'toes_r_X'));
Y_Toes_r=BodyPos(:,strcmp(Head_BK,'toes_r_Y'));
Z_Toes_r=BodyPos(:,strcmp(Head_BK,'toes_r_Z'));

v_toes_l_x=BodyVel(:,strcmp(Head_BK,'toes_l_X'));
v_toes_l_y=BodyVel(:,strcmp(Head_BK,'toes_l_Y'));
v_toes_l_z=BodyVel(:,strcmp(Head_BK,'toes_l_Z'));
v_toes_l=sqrt(v_toes_l_x.^2+v_toes_l_y.^2+v_toes_l_z.^2);

a_toes_l_x=BodyAcc(:,strcmp(Head_BK,'toes_l_X'));
a_toes_l_y=BodyAcc(:,strcmp(Head_BK,'toes_l_Y'));
a_toes_l_z=BodyAcc(:,strcmp(Head_BK,'toes_l_Z'));
a_toes_l=sqrt(a_toes_l_x.^2+a_toes_l_y.^2+a_toes_l_z.^2);

v_toes_r_x=BodyVel(:,strcmp(Head_BK,'toes_r_X'));
v_toes_r_y=BodyVel(:,strcmp(Head_BK,'toes_r_Y'));
v_toes_r_z=BodyVel(:,strcmp(Head_BK,'toes_r_Z'));
v_toes_r=sqrt(v_toes_r_x.^2+v_toes_r_y.^2+v_toes_r_z.^2);

a_toes_r_x=BodyAcc(:,strcmp(Head_BK,'toes_r_X'));
a_toes_r_y=BodyAcc(:,strcmp(Head_BK,'toes_r_Y'));
a_toes_r_z=BodyAcc(:,strcmp(Head_BK,'toes_r_Z'));
a_toes_r=sqrt(a_toes_r_x.^2+a_toes_r_y.^2+a_toes_r_z.^2);
% finding toes velocities normalized at the max value
v_toes_r_n=v_toes_r/max(v_toes_r);
v_toes_l_n=v_toes_l/max(v_toes_l);
%% calcn p,v,a
X_Calcn_l=BodyPos(:,strcmp(Head_BK,'calcn_l_X'));
Y_Calcn_l=BodyPos(:,strcmp(Head_BK,'calcn_l_Y'));
Z_Calcn_l=BodyPos(:,strcmp(Head_BK,'calcn_l_Z'));
X_Calcn_r=BodyPos(:,strcmp(Head_BK,'calcn_r_X'));
Y_Calcn_r=BodyPos(:,strcmp(Head_BK,'calcn_r_Y'));
Z_Calcn_r=BodyPos(:,strcmp(Head_BK,'calcn_r_Z'));

v_calcn_l_x=BodyVel(:,strcmp(Head_BK,'calcn_l_X'));
v_calcn_l_y=BodyVel(:,strcmp(Head_BK,'calcn_l_Y'));
v_calcn_l_z=BodyVel(:,strcmp(Head_BK,'calcn_l_Z'));
v_calcn_l=sqrt(v_calcn_l_x.^2+v_calcn_l_y.^2+v_calcn_l_z.^2);

a_calcn_l_x=BodyAcc(:,strcmp(Head_BK,'calcn_l_X'));
a_calcn_l_y=BodyAcc(:,strcmp(Head_BK,'calcn_l_Y'));
a_calcn_l_z=BodyAcc(:,strcmp(Head_BK,'calcn_l_Z'));
a_calcn_l=sqrt(a_calcn_l_x.^2+a_calcn_l_y.^2+a_calcn_l_z.^2);

v_calcn_r_x=BodyVel(:,strcmp(Head_BK,'calcn_r_X'));
v_calcn_r_y=BodyVel(:,strcmp(Head_BK,'calcn_r_Y'));
v_calcn_r_z=BodyVel(:,strcmp(Head_BK,'calcn_r_Z'));
v_calcn_r=sqrt(v_calcn_r_x.^2+v_calcn_r_y.^2+v_calcn_r_z.^2);

a_calcn_r_x=BodyAcc(:,strcmp(Head_BK,'calcn_r_X'));
a_calcn_r_y=BodyAcc(:,strcmp(Head_BK,'calcn_r_Y'));
a_calcn_r_z=BodyAcc(:,strcmp(Head_BK,'calcn_r_Z'));
a_calcn_r=sqrt(a_calcn_r_x.^2+a_calcn_r_y.^2+a_calcn_r_z.^2);
% finding calcn velocities normalized at the max value
v_calcn_r_n=v_calcn_r/max(v_calcn_r);
v_calcn_l_n=v_calcn_l/max(v_calcn_l);
%figure, plot(abs(a_calcn_r_y/1000)), hold on, plot(p_calcn_r_y);
%figure, plot(abs(a_calcn_l_y/1000)), hold on, plot(p_calcn_l_y);
% 
% figure,plot(X_Calcn_l), hold on, plot(Y_Calcn_l),hold on, plot(Z_Calcn_l)
% figure,plot(v_calcn_l_x), hold on, plot(v_calcn_l_y),hold on, plot(v_calcn_l_z)
% figure,plot(a_calcn_l_x), hold on, plot(a_calcn_l_y),hold on, plot(a_calcn_l_z)
% % 
%  figure,plot(timeBK,v_calcn_l_n)
%  figure,plot(timeBK,v_toes_l_n)


%figure, plot(v_calcn_l_y)
for i=1:length(X_Calcn_r)
proj_r(i)=dot([X_Calcn_r(i),Y_Calcn_r(i),Z_Calcn_r(i)],n);
proj_l(i)=dot([X_Calcn_l(i),Y_Calcn_l(i),Z_Calcn_l(i)],n);
end
range_pos_r=proj_r<min(proj_r)+min(proj_r)*1/1000;
range_pos_l=proj_l<min(proj_l)+min(proj_l)*1/1000;


min_p_calcn_r_y=find(range_pos_r);
min_p_calcn_l_y=find(range_pos_l);


[val_pos_r,pos_r]=min(abs(a_calcn_r_y(min_p_calcn_r_y)));
[val_pos_l,pos_l]=min(abs(a_calcn_l_y(min_p_calcn_l_y))); % nello spazio delle altezze minime ( min, min+1/30*min) scelgo il punto con accelerazione minimo

position_r=min_p_calcn_r_y(pos_r); % the position of interest is the one with min acc whitin the lowest range of height
position_l=min_p_calcn_l_y(pos_l);

% finding calcn and toes scaling factors
Calcn_r_SF=model.getBodySet.get('calcn_r').get_attached_geometry(0).get_scale_factors.getAsMat;
Calcn_l_SF=model.getBodySet.get('calcn_l').get_attached_geometry(0).get_scale_factors.getAsMat;
Toes_r_SF=model.getBodySet.get('toes_r').get_attached_geometry(0).get_scale_factors.getAsMat;
Toes_l_SF=model.getBodySet.get('toes_l').get_attached_geometry(0).get_scale_factors.getAsMat;

% determining position of contact shperes
PosSp1R=[(0+heel_shift)*Calcn_r_SF(1) 0.03*Calcn_r_SF(2) -0.01*Calcn_r_SF(3)];
PosSp1L=[(0+heel_shift)*Calcn_l_SF(1) 0.03*Calcn_l_SF(2) -0.01*Calcn_l_SF(3)];
PosSp2R=[(0+heel_shift)*Calcn_r_SF(1) 0.03*Calcn_r_SF(2) 0.01*Calcn_r_SF(3)];
PosSp2L=[(0+heel_shift)*Calcn_l_SF(1) 0.03*Calcn_l_SF(2) 0.01*Calcn_l_SF(3)];
PosSp3R=[(0.035+heel_shift)*Calcn_r_SF(1) 0.03*Calcn_r_SF(2) -0.02*Calcn_r_SF(3)];
PosSp3L=[(0.035+heel_shift)*Calcn_l_SF(1) 0.03*Calcn_l_SF(2) -0.02*Calcn_l_SF(3)];
PosSp4R=[(0.035+heel_shift)*Calcn_r_SF(1) 0.03*Calcn_r_SF(2) 0.02*Calcn_r_SF(3)];
PosSp4L=[(0.035+heel_shift)*Calcn_l_SF(1) 0.03*Calcn_l_SF(2) 0.02*Calcn_l_SF(3)];
PosSp5R=[0.026*Toes_r_SF(1) 0.03*Toes_r_SF(2) -0.01*Toes_r_SF(3)];
PosSp5L=[0.026*Toes_l_SF(1) 0.03*Toes_l_SF(2) -0.02*Toes_l_SF(3)];
PosSp6R=[0.026*Toes_r_SF(1) 0.03*Toes_r_SF(2) 0.02*Toes_r_SF(3)];
PosSp6L=[0.026*Toes_l_SF(1) 0.03*Toes_l_SF(2) 0.01*Toes_l_SF(3)];
PosSp7R=[0.07*Calcn_r_SF(1) 0.03*Calcn_r_SF(2) -0.015*Calcn_r_SF(3)];
PosSp7L=[0.07*Calcn_l_SF(1) 0.03*Calcn_l_SF(2) -0.035*Calcn_l_SF(3)];
PosSp8R=[0.07*Calcn_r_SF(1) 0.03*Calcn_r_SF(2) 0.035*Calcn_r_SF(3)];
PosSp8L=[0.07*Calcn_l_SF(1) 0.03*Calcn_l_SF(2) 0.015*Calcn_l_SF(3)];
PosSp9R=[0.105*Calcn_r_SF(1) 0.03*Calcn_r_SF(2) -0.005*Calcn_r_SF(3)];
PosSp9L=[0.105*Calcn_l_SF(1) 0.03*Calcn_l_SF(2) -0.045*Calcn_l_SF(3)];
PosSp10R=[0.105*Calcn_r_SF(1) 0.03*Calcn_r_SF(2) 0.045*Calcn_r_SF(3)];
PosSp10L=[0.105*Calcn_l_SF(1) 0.03*Calcn_l_SF(2) 0.005*Calcn_l_SF(3)];
PosSp11R=[0.14*Calcn_r_SF(1) 0.03*Calcn_r_SF(2) -0.005*Calcn_r_SF(3)];
PosSp11L=[0.14*Calcn_l_SF(1) 0.03*Calcn_l_SF(2) -0.045*Calcn_l_SF(3)];
PosSp12R=[0.14*Calcn_r_SF(1) 0.03*Calcn_r_SF(2) 0.045*Calcn_r_SF(3)];
PosSp12L=[0.14*Calcn_l_SF(1) 0.03*Calcn_l_SF(2) 0.005*Calcn_l_SF(3)];
PosSp13R=[0.0*Toes_r_SF(1) 0.03*Toes_r_SF(2) -0.005*Toes_r_SF(3)];
PosSp13L=[0.0*Toes_l_SF(1) 0.03*Toes_l_SF(2) -0.03*Toes_l_SF(3)];
PosSp14R=[0.0*Toes_r_SF(1) 0.03*Toes_r_SF(2) 0.03*Toes_r_SF(3)];
PosSp14L=[0.0*Toes_l_SF(1) 0.03*Toes_l_SF(2) 0.005*Toes_l_SF(3)];
% creating spheres and ground contact
Sphere_Foot_1_R=ContactSphere();
Sphere_Foot_1_R.setName('Sphere_Foot_1_R');
Body=model.getBodySet.get(calcn_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_1_R.setFrame(Frame);
Sphere_Foot_1_R.setLocation(Vec3.createFromMat(PosSp1R));
Sphere_Foot_1_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_1_R)

Sphere_Foot_1_L=ContactSphere();
Sphere_Foot_1_L.setName('Sphere_Foot_1_L');
Body=model.getBodySet.get(calcn_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_1_L.setFrame(Frame);
Sphere_Foot_1_L.setLocation(Vec3.createFromMat(PosSp1L));
Sphere_Foot_1_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_1_L)

Sphere_Foot_2_R=ContactSphere();
Sphere_Foot_2_R.setName('Sphere_Foot_2_R');
Body=model.getBodySet.get(calcn_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_2_R.setFrame(Frame);
Sphere_Foot_2_R.setLocation(Vec3.createFromMat(PosSp2R));
Sphere_Foot_2_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_2_R)

Sphere_Foot_2_L=ContactSphere();
Sphere_Foot_2_L.setName('Sphere_Foot_2_L');
Body=model.getBodySet.get(calcn_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_2_L.setFrame(Frame);
Sphere_Foot_2_L.setLocation(Vec3.createFromMat(PosSp2L));
Sphere_Foot_2_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_2_L)

Sphere_Foot_3_R=ContactSphere();
Sphere_Foot_3_R.setName('Sphere_Foot_3_R');
Body=model.getBodySet.get(calcn_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_3_R.setFrame(Frame);
Sphere_Foot_3_R.setLocation(Vec3.createFromMat(PosSp3R));
Sphere_Foot_3_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_3_R)

Sphere_Foot_3_L=ContactSphere();
Sphere_Foot_3_L.setName('Sphere_Foot_3_L');
Body=model.getBodySet.get(calcn_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_3_L.setFrame(Frame);
Sphere_Foot_3_L.setLocation(Vec3.createFromMat(PosSp3L));
Sphere_Foot_3_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_3_L)

Sphere_Foot_4_L=ContactSphere();
Sphere_Foot_4_L.setName('Sphere_Foot_4_L');
Body=model.getBodySet.get(calcn_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_4_L.setFrame(Frame);
Sphere_Foot_4_L.setLocation(Vec3.createFromMat(PosSp4L));
Sphere_Foot_4_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_4_L)

Sphere_Foot_4_R=ContactSphere();
Sphere_Foot_4_R.setName('Sphere_Foot_4_R');
Body=model.getBodySet.get(calcn_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_4_R.setFrame(Frame);
Sphere_Foot_4_R.setLocation(Vec3.createFromMat(PosSp4R));
Sphere_Foot_4_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_4_R)

Sphere_Foot_5_L=ContactSphere();
Sphere_Foot_5_L.setName('Sphere_Foot_5_L');
Body=model.getBodySet.get(toes_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_5_L.setFrame(Frame);
Sphere_Foot_5_L.setLocation(Vec3.createFromMat(PosSp5L));
Sphere_Foot_5_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_5_L)

Sphere_Foot_5_R=ContactSphere();
Sphere_Foot_5_R.setName('Sphere_Foot_5_R');
Body=model.getBodySet.get(toes_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_5_R.setFrame(Frame);
Sphere_Foot_5_R.setLocation(Vec3.createFromMat(PosSp5R));
Sphere_Foot_5_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_5_R)

Sphere_Foot_6_L=ContactSphere();
Sphere_Foot_6_L.setName('Sphere_Foot_6_L');
Body=model.getBodySet.get(toes_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_6_L.setFrame(Frame);
Sphere_Foot_6_L.setLocation(Vec3.createFromMat(PosSp6L));
Sphere_Foot_6_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_6_L)

Sphere_Foot_6_R=ContactSphere();
Sphere_Foot_6_R.setName('Sphere_Foot_6_R');
Body=model.getBodySet.get(toes_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_6_R.setFrame(Frame);
Sphere_Foot_6_R.setLocation(Vec3.createFromMat(PosSp6R));
Sphere_Foot_6_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_6_R)


Sphere_Foot_7_L=ContactSphere();
Sphere_Foot_7_L.setName('Sphere_Foot_7_L');
Body=model.getBodySet.get(calcn_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_7_L.setFrame(Frame);
Sphere_Foot_7_L.setLocation(Vec3.createFromMat(PosSp7L));
Sphere_Foot_7_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_7_L)

Sphere_Foot_7_R=ContactSphere();
Sphere_Foot_7_R.setName('Sphere_Foot_7_R');
Body=model.getBodySet.get(calcn_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_7_R.setFrame(Frame);
Sphere_Foot_7_R.setLocation(Vec3.createFromMat(PosSp7R));
Sphere_Foot_7_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_7_R)

Sphere_Foot_8_L=ContactSphere();
Sphere_Foot_8_L.setName('Sphere_Foot_8_L');
Body=model.getBodySet.get(calcn_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_8_L.setFrame(Frame);
Sphere_Foot_8_L.setLocation(Vec3.createFromMat(PosSp8L));
Sphere_Foot_8_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_8_L)

Sphere_Foot_8_R=ContactSphere();
Sphere_Foot_8_R.setName('Sphere_Foot_8_R');
Body=model.getBodySet.get(calcn_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_8_R.setFrame(Frame);
Sphere_Foot_8_R.setLocation(Vec3.createFromMat(PosSp8R));
Sphere_Foot_8_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_8_R)


Sphere_Foot_9_L=ContactSphere();
Sphere_Foot_9_L.setName('Sphere_Foot_9_L');
Body=model.getBodySet.get(calcn_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_9_L.setFrame(Frame);
Sphere_Foot_9_L.setLocation(Vec3.createFromMat(PosSp9L));
Sphere_Foot_9_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_9_L)

Sphere_Foot_9_R=ContactSphere();
Sphere_Foot_9_R.setName('Sphere_Foot_9_R');
Body=model.getBodySet.get(calcn_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_9_R.setFrame(Frame);
Sphere_Foot_9_R.setLocation(Vec3.createFromMat(PosSp9R));
Sphere_Foot_9_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_9_R)


Sphere_Foot_10_L=ContactSphere();
Sphere_Foot_10_L.setName('Sphere_Foot_10_L');
Body=model.getBodySet.get(calcn_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_10_L.setFrame(Frame);
Sphere_Foot_10_L.setLocation(Vec3.createFromMat(PosSp10L));
Sphere_Foot_10_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_10_L)

Sphere_Foot_10_R=ContactSphere();
Sphere_Foot_10_R.setName('Sphere_Foot_10_R');
Body=model.getBodySet.get(calcn_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_10_R.setFrame(Frame);
Sphere_Foot_10_R.setLocation(Vec3.createFromMat(PosSp10R));
Sphere_Foot_10_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_10_R)

Sphere_Foot_11_L=ContactSphere();
Sphere_Foot_11_L.setName('Sphere_Foot_11_L');
Body=model.getBodySet.get(calcn_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_11_L.setFrame(Frame);
Sphere_Foot_11_L.setLocation(Vec3.createFromMat(PosSp11L));
Sphere_Foot_11_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_11_L)

Sphere_Foot_11_R=ContactSphere();
Sphere_Foot_11_R.setName('Sphere_Foot_11_R');
Body=model.getBodySet.get(calcn_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_11_R.setFrame(Frame);
Sphere_Foot_11_R.setLocation(Vec3.createFromMat(PosSp11R));
Sphere_Foot_11_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_11_R)

Sphere_Foot_12_L=ContactSphere();
Sphere_Foot_12_L.setName('Sphere_Foot_12_L');
Body=model.getBodySet.get(calcn_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_12_L.setFrame(Frame);
Sphere_Foot_12_L.setLocation(Vec3.createFromMat(PosSp12L));
Sphere_Foot_12_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_12_L)

Sphere_Foot_12_R=ContactSphere();
Sphere_Foot_12_R.setName('Sphere_Foot_12_R');
Body=model.getBodySet.get(calcn_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_12_R.setFrame(Frame);
Sphere_Foot_12_R.setLocation(Vec3.createFromMat(PosSp12R));
Sphere_Foot_12_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_12_R)

Sphere_Foot_13_L=ContactSphere();
Sphere_Foot_13_L.setName('Sphere_Foot_13_L');
Body=model.getBodySet.get(toes_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_13_L.setFrame(Frame);
Sphere_Foot_13_L.setLocation(Vec3.createFromMat(PosSp13L));
Sphere_Foot_13_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_13_L)

Sphere_Foot_13_R=ContactSphere();
Sphere_Foot_13_R.setName('Sphere_Foot_13_R');
Body=model.getBodySet.get(toes_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_13_R.setFrame(Frame);
Sphere_Foot_13_R.setLocation(Vec3.createFromMat(PosSp13R));
Sphere_Foot_13_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_13_R)

Sphere_Foot_14_L=ContactSphere();
Sphere_Foot_14_L.setName('Sphere_Foot_14_L');
Body=model.getBodySet.get(toes_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_14_L.setFrame(Frame);
Sphere_Foot_14_L.setLocation(Vec3.createFromMat(PosSp14L));
Sphere_Foot_14_L.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_14_L)

Sphere_Foot_14_R=ContactSphere();
Sphere_Foot_14_R.setName('Sphere_Foot_14_R');
Body=model.getBodySet.get(toes_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Foot_14_R.setFrame(Frame);
Sphere_Foot_14_R.setLocation(Vec3.createFromMat(PosSp14R));
Sphere_Foot_14_R.setRadius(Sp_radius);
model.addContactGeometry(Sphere_Foot_14_R)
if FullBody
Sphere_Hand_R=ContactSphere();
Sphere_Hand_R.setName('Sphere_Hand_R');
Body=model.getBodySet.get(hand_r_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Hand_R.setFrame(Frame);
Sphere_Hand_R.setLocation(Body.get_mass_center);
Sphere_Hand_R.setRadius(0.08);
model.addContactGeometry(Sphere_Hand_R)

Sphere_Hand_L=ContactSphere();
Sphere_Hand_L.setName('Sphere_Hand_L');
Body=model.getBodySet.get(hand_l_name);
Frame=PhysicalFrame.safeDownCast(Body);
Sphere_Hand_L.setFrame(Frame);
Sphere_Hand_L.setLocation(Body.get_mass_center);
Sphere_Hand_L.setRadius(0.08);
model.addContactGeometry(Sphere_Hand_L)
end
groundCont_Foot_L_A=ContactHalfSpace();
groundCont_Foot_L_A.setName('ground_Foot_L_A');
groundCont_Foot_L_A.setFrame(model.getGround)
groundCont_Foot_L_A.set_location(Vec3.createFromMat(Ground_L_Foot_T_A))
groundCont_Foot_L_A.set_orientation(Vec3.createFromMat(Ground_Rot))
model.addContactGeometry(groundCont_Foot_L_A)

groundCont_Foot_R_A=ContactHalfSpace();
groundCont_Foot_R_A.setName('ground_Foot_R_A');
groundCont_Foot_R_A.setFrame(model.getGround)
groundCont_Foot_R_A.set_location(Vec3.createFromMat(Ground_R_Foot_T_A))
groundCont_Foot_R_A.set_orientation(Vec3.createFromMat(Ground_Rot))
model.addContactGeometry(groundCont_Foot_R_A)
% adding both right and left contact B plane Half Space
groundCont_Foot_L_B=ContactHalfSpace();
groundCont_Foot_L_B.setName('ground_Foot_L_B');
groundCont_Foot_L_B.setFrame(model.getGround)
groundCont_Foot_L_B.set_location(Vec3.createFromMat(Ground_L_Foot_T_B))
groundCont_Foot_L_B.set_orientation(Vec3.createFromMat(Ground_Rot))
model.addContactGeometry(groundCont_Foot_L_B)

groundCont_Foot_R_B=ContactHalfSpace();
groundCont_Foot_R_B.setName('ground_Foot_R_B');
groundCont_Foot_R_B.setFrame(model.getGround)
groundCont_Foot_R_B.set_location(Vec3.createFromMat(Ground_R_Foot_T_B))
groundCont_Foot_R_B.set_orientation(Vec3.createFromMat(Ground_Rot))
model.addContactGeometry(groundCont_Foot_R_B)
if FullBody
groundCont_Hand_R=ContactHalfSpace();
groundCont_Hand_R.setName('ground_Hand_R');
groundCont_Hand_R.setFrame(model.getGround)
groundCont_Hand_R.set_location(Vec3.createFromMat(Ground_R_Hand_T))
groundCont_Hand_R.set_orientation(Vec3.createFromMat(Ground_Rot))
model.addContactGeometry(groundCont_Hand_R)

groundCont_Hand_L=ContactHalfSpace();
groundCont_Hand_L.setName('ground_Hand_L');
groundCont_Hand_L.setFrame(model.getGround)
groundCont_Hand_L.set_location(Vec3.createFromMat(Ground_L_Hand_T))
groundCont_Hand_L.set_orientation(Vec3.createFromMat(Ground_Rot))
model.addContactGeometry(groundCont_Hand_L)
end
% create contact forces
ForceGround_Foot_1_R_A = HuntCrossleyForce();
ForceGround_Foot_1_R_A.setName('ForceGround_Foot_1_R_A');
ForceGround_Foot_1_R_A.set_appliesForce(true);
ForceGround_Foot_1_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_1_R')
ForceGround_Foot_1_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_1_R_A.setDissipation(0);
ForceGround_Foot_1_R_A.setStaticFriction(0);
ForceGround_Foot_1_R_A.setDynamicFriction(0);
ForceGround_Foot_1_R_A.setViscousFriction(0);
ForceGround_Foot_1_R_A.setTransitionVelocity(0.13)
%ForceGround_Foot1_R.print('es_contForce.xml')
ForceGround_Foot_2_R_A = HuntCrossleyForce();
ForceGround_Foot_2_R_A.setName('ForceGround_Foot_2_R_A');
ForceGround_Foot_2_R_A.set_appliesForce(true);
ForceGround_Foot_2_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_2_R')
ForceGround_Foot_2_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_2_R_A.setDissipation(0);
ForceGround_Foot_2_R_A.setStaticFriction(0);
ForceGround_Foot_2_R_A.setDynamicFriction(0);
ForceGround_Foot_2_R_A.setViscousFriction(0);
ForceGround_Foot_2_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_1_L_A = HuntCrossleyForce();
ForceGround_Foot_1_L_A.setName('ForceGround_Foot_1_L_A');
ForceGround_Foot_1_L_A.set_appliesForce(true);
ForceGround_Foot_1_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_1_L')
ForceGround_Foot_1_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_1_L_A.setDissipation(0);
ForceGround_Foot_1_L_A.setStaticFriction(0);
ForceGround_Foot_1_L_A.setDynamicFriction(0);
ForceGround_Foot_1_L_A.setViscousFriction(0);
ForceGround_Foot_1_L_A.setTransitionVelocity(0.13)
%ForceGround_Foot1_R.print('es_contForce.xml')
ForceGround_Foot_2_L_A = HuntCrossleyForce();
ForceGround_Foot_2_L_A.setName('ForceGround_Foot_2_L_A');
ForceGround_Foot_2_L_A.set_appliesForce(true);
ForceGround_Foot_2_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_2_L')
ForceGround_Foot_2_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_2_L_A.setDissipation(0);
ForceGround_Foot_2_L_A.setStaticFriction(0);
ForceGround_Foot_2_L_A.setDynamicFriction(0);
ForceGround_Foot_2_L_A.setViscousFriction(0);
ForceGround_Foot_2_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_3_R_A = HuntCrossleyForce();
ForceGround_Foot_3_R_A.setName('ForceGround_Foot_3_R_A');
ForceGround_Foot_3_R_A.set_appliesForce(true);
ForceGround_Foot_3_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_3_R');
ForceGround_Foot_3_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_3_R_A.setDissipation(0);
ForceGround_Foot_3_R_A.setStaticFriction(0);
ForceGround_Foot_3_R_A.setDynamicFriction(0);
ForceGround_Foot_3_R_A.setViscousFriction(0);
ForceGround_Foot_3_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_3_L_A=HuntCrossleyForce();
ForceGround_Foot_3_L_A.setName('ForceGround_Foot_3_L_A');
ForceGround_Foot_3_L_A.set_appliesForce(true);
ForceGround_Foot_3_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_3_L')
ForceGround_Foot_3_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_3_L_A.setDissipation(0);
ForceGround_Foot_3_L_A.setStaticFriction(0);
ForceGround_Foot_3_L_A.setDynamicFriction(0);
ForceGround_Foot_3_L_A.setViscousFriction(0);
ForceGround_Foot_3_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_4_R_A = HuntCrossleyForce();
ForceGround_Foot_4_R_A.setName('ForceGround_Foot_4_R_A');
ForceGround_Foot_4_R_A.set_appliesForce(true);
ForceGround_Foot_4_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_4_R');
ForceGround_Foot_4_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_4_R_A.setDissipation(0);
ForceGround_Foot_4_R_A.setStaticFriction(0);
ForceGround_Foot_4_R_A.setDynamicFriction(0);
ForceGround_Foot_4_R_A.setViscousFriction(0);
ForceGround_Foot_4_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_4_L_A=HuntCrossleyForce();
ForceGround_Foot_4_L_A.setName('ForceGround_Foot_4_L_A');
ForceGround_Foot_4_L_A.set_appliesForce(true);
ForceGround_Foot_4_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_4_L')
ForceGround_Foot_4_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_4_L_A.setDissipation(0);
ForceGround_Foot_4_L_A.setStaticFriction(0);
ForceGround_Foot_4_L_A.setDynamicFriction(0);
ForceGround_Foot_4_L_A.setViscousFriction(0);
ForceGround_Foot_4_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_5_R_A = HuntCrossleyForce();
ForceGround_Foot_5_R_A.setName('ForceGround_Foot_5_R_A');
ForceGround_Foot_5_R_A.set_appliesForce(true);
ForceGround_Foot_5_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_5_R');
ForceGround_Foot_5_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_5_R_A.setDissipation(0);
ForceGround_Foot_5_R_A.setStaticFriction(0);
ForceGround_Foot_5_R_A.setDynamicFriction(0);
ForceGround_Foot_5_R_A.setViscousFriction(0);
ForceGround_Foot_5_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_5_L_A=HuntCrossleyForce();
ForceGround_Foot_5_L_A.setName('ForceGround_Foot_5_L_A');
ForceGround_Foot_5_L_A.set_appliesForce(true);
ForceGround_Foot_5_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_5_L')
ForceGround_Foot_5_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_5_L_A.setDissipation(0);
ForceGround_Foot_5_L_A.setStaticFriction(0);
ForceGround_Foot_5_L_A.setDynamicFriction(0);
ForceGround_Foot_5_L_A.setViscousFriction(0);
ForceGround_Foot_5_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_6_R_A = HuntCrossleyForce();
ForceGround_Foot_6_R_A.setName('ForceGround_Foot_6_R_A');
ForceGround_Foot_6_R_A.set_appliesForce(true);
ForceGround_Foot_6_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_6_R');
ForceGround_Foot_6_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_6_R_A.setDissipation(0);
ForceGround_Foot_6_R_A.setStaticFriction(0);
ForceGround_Foot_6_R_A.setDynamicFriction(0);
ForceGround_Foot_6_R_A.setViscousFriction(0);
ForceGround_Foot_6_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_6_L_A=HuntCrossleyForce();
ForceGround_Foot_6_L_A.setName('ForceGround_Foot_6_L_A');
ForceGround_Foot_6_L_A.set_appliesForce(true);
ForceGround_Foot_6_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_6_L')
ForceGround_Foot_6_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_6_L_A.setDissipation(0);
ForceGround_Foot_6_L_A.setStaticFriction(0);
ForceGround_Foot_6_L_A.setDynamicFriction(0);
ForceGround_Foot_6_L_A.setViscousFriction(0);
ForceGround_Foot_6_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_7_R_A = HuntCrossleyForce();
ForceGround_Foot_7_R_A.setName('ForceGround_Foot_7_R_A');
ForceGround_Foot_7_R_A.set_appliesForce(true);
ForceGround_Foot_7_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_7_R');
ForceGround_Foot_7_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_7_R_A.setDissipation(0);
ForceGround_Foot_7_R_A.setStaticFriction(0);
ForceGround_Foot_7_R_A.setDynamicFriction(0);
ForceGround_Foot_7_R_A.setViscousFriction(0);
ForceGround_Foot_7_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_7_L_A=HuntCrossleyForce();
ForceGround_Foot_7_L_A.setName('ForceGround_Foot_7_L_A');
ForceGround_Foot_7_L_A.set_appliesForce(true);
ForceGround_Foot_7_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_7_L')
ForceGround_Foot_7_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_7_L_A.setDissipation(0);
ForceGround_Foot_7_L_A.setStaticFriction(0);
ForceGround_Foot_7_L_A.setDynamicFriction(0);
ForceGround_Foot_7_L_A.setViscousFriction(0);
ForceGround_Foot_7_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_8_R_A = HuntCrossleyForce();
ForceGround_Foot_8_R_A.setName('ForceGround_Foot_8_R_A');
ForceGround_Foot_8_R_A.set_appliesForce(true);
ForceGround_Foot_8_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_8_R');
ForceGround_Foot_8_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_8_R_A.setDissipation(0);
ForceGround_Foot_8_R_A.setStaticFriction(0);
ForceGround_Foot_8_R_A.setDynamicFriction(0);
ForceGround_Foot_8_R_A.setViscousFriction(0);
ForceGround_Foot_8_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_8_L_A=HuntCrossleyForce();
ForceGround_Foot_8_L_A.setName('ForceGround_Foot_8_L_A');
ForceGround_Foot_8_L_A.set_appliesForce(true);
ForceGround_Foot_8_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_8_L')
ForceGround_Foot_8_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_8_L_A.setDissipation(0);
ForceGround_Foot_8_L_A.setStaticFriction(0);
ForceGround_Foot_8_L_A.setDynamicFriction(0);
ForceGround_Foot_8_L_A.setViscousFriction(0);
ForceGround_Foot_8_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_9_R_A = HuntCrossleyForce();
ForceGround_Foot_9_R_A.setName('ForceGround_Foot_9_R_A');
ForceGround_Foot_9_R_A.set_appliesForce(true);
ForceGround_Foot_9_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_9_R');
ForceGround_Foot_9_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_9_R_A.setDissipation(0);
ForceGround_Foot_9_R_A.setStaticFriction(0);
ForceGround_Foot_9_R_A.setDynamicFriction(0);
ForceGround_Foot_9_R_A.setViscousFriction(0);
ForceGround_Foot_9_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_9_L_A=HuntCrossleyForce();
ForceGround_Foot_9_L_A.setName('ForceGround_Foot_9_L_A');
ForceGround_Foot_9_L_A.set_appliesForce(true);
ForceGround_Foot_9_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_9_L')
ForceGround_Foot_9_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_9_L_A.setDissipation(0);
ForceGround_Foot_9_L_A.setStaticFriction(0);
ForceGround_Foot_9_L_A.setDynamicFriction(0);
ForceGround_Foot_9_L_A.setViscousFriction(0);
ForceGround_Foot_9_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_10_R_A = HuntCrossleyForce();
ForceGround_Foot_10_R_A.setName('ForceGround_Foot_10_R_A');
ForceGround_Foot_10_R_A.set_appliesForce(true);
ForceGround_Foot_10_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_10_R');
ForceGround_Foot_10_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_10_R_A.setDissipation(0);
ForceGround_Foot_10_R_A.setStaticFriction(0);
ForceGround_Foot_10_R_A.setDynamicFriction(0);
ForceGround_Foot_10_R_A.setViscousFriction(0);
ForceGround_Foot_10_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_10_L_A=HuntCrossleyForce();
ForceGround_Foot_10_L_A.setName('ForceGround_Foot_10_L_A');
ForceGround_Foot_10_L_A.set_appliesForce(true);
ForceGround_Foot_10_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_10_L')
ForceGround_Foot_10_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_10_L_A.setDissipation(0);
ForceGround_Foot_10_L_A.setStaticFriction(0);
ForceGround_Foot_10_L_A.setDynamicFriction(0);
ForceGround_Foot_10_L_A.setViscousFriction(0);
ForceGround_Foot_10_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_11_R_A = HuntCrossleyForce();
ForceGround_Foot_11_R_A.setName('ForceGround_Foot_11_R_A');
ForceGround_Foot_11_R_A.set_appliesForce(true);
ForceGround_Foot_11_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_11_R');
ForceGround_Foot_11_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_11_R_A.setDissipation(0);
ForceGround_Foot_11_R_A.setStaticFriction(0);
ForceGround_Foot_11_R_A.setDynamicFriction(0);
ForceGround_Foot_11_R_A.setViscousFriction(0);
ForceGround_Foot_11_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_11_L_A=HuntCrossleyForce();
ForceGround_Foot_11_L_A.setName('ForceGround_Foot_11_L_A');
ForceGround_Foot_11_L_A.set_appliesForce(true);
ForceGround_Foot_11_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_11_L')
ForceGround_Foot_11_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_11_L_A.setDissipation(0);
ForceGround_Foot_11_L_A.setStaticFriction(0);
ForceGround_Foot_11_L_A.setDynamicFriction(0);
ForceGround_Foot_11_L_A.setViscousFriction(0);
ForceGround_Foot_11_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_12_R_A = HuntCrossleyForce();
ForceGround_Foot_12_R_A.setName('ForceGround_Foot_12_R_A');
ForceGround_Foot_12_R_A.set_appliesForce(true);
ForceGround_Foot_12_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_12_R');
ForceGround_Foot_12_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_12_R_A.setDissipation(0);
ForceGround_Foot_12_R_A.setStaticFriction(0);
ForceGround_Foot_12_R_A.setDynamicFriction(0);
ForceGround_Foot_12_R_A.setViscousFriction(0);
ForceGround_Foot_12_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_12_L_A=HuntCrossleyForce();
ForceGround_Foot_12_L_A.setName('ForceGround_Foot_12_L_A');
ForceGround_Foot_12_L_A.set_appliesForce(true);
ForceGround_Foot_12_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_12_L')
ForceGround_Foot_12_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_12_L_A.setDissipation(0);
ForceGround_Foot_12_L_A.setStaticFriction(0);
ForceGround_Foot_12_L_A.setDynamicFriction(0);
ForceGround_Foot_12_L_A.setViscousFriction(0);
ForceGround_Foot_12_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_13_R_A = HuntCrossleyForce();
ForceGround_Foot_13_R_A.setName('ForceGround_Foot_13_R_A');
ForceGround_Foot_13_R_A.set_appliesForce(true);
ForceGround_Foot_13_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_13_R');
ForceGround_Foot_13_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_13_R_A.setDissipation(0);
ForceGround_Foot_13_R_A.setStaticFriction(0);
ForceGround_Foot_13_R_A.setDynamicFriction(0);
ForceGround_Foot_13_R_A.setViscousFriction(0);
ForceGround_Foot_13_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_13_L_A=HuntCrossleyForce();
ForceGround_Foot_13_L_A.setName('ForceGround_Foot_13_L_A');
ForceGround_Foot_13_L_A.set_appliesForce(true);
ForceGround_Foot_13_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_13_L')
ForceGround_Foot_13_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_13_L_A.setDissipation(0);
ForceGround_Foot_13_L_A.setStaticFriction(0);
ForceGround_Foot_13_L_A.setDynamicFriction(0);
ForceGround_Foot_13_L_A.setViscousFriction(0);
ForceGround_Foot_13_L_A.setTransitionVelocity(0.13)

ForceGround_Foot_14_R_A = HuntCrossleyForce();
ForceGround_Foot_14_R_A.setName('ForceGround_Foot_14_R_A');
ForceGround_Foot_14_R_A.set_appliesForce(true);
ForceGround_Foot_14_R_A.addGeometry('ground_Foot_R_A Sphere_Foot_14_R');
ForceGround_Foot_14_R_A.setStiffness(Sp_stiffness);
ForceGround_Foot_14_R_A.setDissipation(0);
ForceGround_Foot_14_R_A.setStaticFriction(0);
ForceGround_Foot_14_R_A.setDynamicFriction(0);
ForceGround_Foot_14_R_A.setViscousFriction(0);
ForceGround_Foot_14_R_A.setTransitionVelocity(0.13)

ForceGround_Foot_14_L_A=HuntCrossleyForce();
ForceGround_Foot_14_L_A.setName('ForceGround_Foot_14_L_A');
ForceGround_Foot_14_L_A.set_appliesForce(true);
ForceGround_Foot_14_L_A.addGeometry('ground_Foot_L_A Sphere_Foot_14_L')
ForceGround_Foot_14_L_A.setStiffness(Sp_stiffness);
ForceGround_Foot_14_L_A.setDissipation(0);
ForceGround_Foot_14_L_A.setStaticFriction(0);
ForceGround_Foot_14_L_A.setDynamicFriction(0);
ForceGround_Foot_14_L_A.setViscousFriction(0);
ForceGround_Foot_14_L_A.setTransitionVelocity(0.13)

if FullBody
ForceGround_Hand_L = HuntCrossleyForce();
ForceGround_Hand_L.setName('ForceGround_Hand_L');
ForceGround_Hand_L.set_appliesForce(true);
ForceGround_Hand_L.addGeometry('ground_Hand_L Sphere_Hand_L')
ForceGround_Hand_L.setStiffness(Sp_stiffness);
ForceGround_Hand_L.setDissipation(0);
ForceGround_Hand_L.setStaticFriction(0);
ForceGround_Hand_L.setDynamicFriction(0);
ForceGround_Hand_L.setViscousFriction(0);
ForceGround_Hand_L.setTransitionVelocity(0.13)

ForceGround_Hand_R = HuntCrossleyForce();
ForceGround_Hand_R.setName('ForceGround_Hand_R');
ForceGround_Hand_R.set_appliesForce(true);
ForceGround_Hand_R.addGeometry('ground_Hand_R Sphere_Hand_R')
ForceGround_Hand_R.setStiffness(Sp_stiffness);
ForceGround_Hand_R.setDissipation(0);
ForceGround_Hand_R.setStaticFriction(0);
ForceGround_Hand_R.setDynamicFriction(0);
ForceGround_Hand_R.setViscousFriction(0);
ForceGround_Hand_R.setTransitionVelocity(0.13)
end
% Automate the Force reporter creation
model.getForceSet.cloneAndAppend(ForceGround_Foot_1_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_2_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_1_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_2_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_3_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_3_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_4_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_4_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_5_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_5_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_6_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_6_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_7_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_7_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_8_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_8_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_9_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_9_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_10_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_10_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_11_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_11_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_12_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_12_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_13_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_13_L_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_14_R_A);
model.getForceSet.cloneAndAppend(ForceGround_Foot_14_L_A);
if FullBody
model.getForceSet.cloneAndAppend(ForceGround_Hand_R);
model.getForceSet.cloneAndAppend(ForceGround_Hand_L);
end
model.finalizeConnections()
modelProcessed_path=fullfile(ModelFolder,"ModelProcessed.osim");
model.setName("ModelProcessed");
model.print(modelProcessed_path);
modelProcessed=Model(modelProcessed_path);
%% Analyzing the Force reporter output to detect the contact with ground load IK file
AnTool=AnalyzeTool();
AnTool.setModel(modelProcessed);
AnTool.setModelFilename(fullfile(ModelFolder,"ModelProcessed.osim"));
AnTool.setCoordinatesFileName(fullfile(IKFolder,IKResult));
AnTool.setLowpassCutoffFrequency(Freq);
AnTool.setSolveForEquilibrium(1);
AnTool.setStartTime(timeStart);
AnTool.setFinalTime(timeEnd);
AnTool.setResultsDir(path_FR);
FR_An=ForceReporter();
FR_An.setStartTime(timeStart);
FR_An.setEndTime(timeEnd)
AnTool.getAnalysisSet.cloneAndAppend(FR_An);
AnTool.print(fullfile(path_Solution,'Setup_ForceReporter.xml'));
%AnTool.run;

%ForceTool=AnalyzeTool(fullfile(IKFolder,'Setup_ForceReporter.xml'));
delta=0.001; % decrement for sphere y location
counter=0;
% iterating untill each sphere feels contact
while true

IterForceTool=AnalyzeTool(fullfile(path_Solution,'Setup_ForceReporter.xml'));

IterForceTool.run;
% analyze Force Reporter Results
[ForceReport,HeadFR]=load_sto(fullfile(path_FR,'_ForceReporter_forces.sto'));

Cont_Foot_1_L_A=ForceReport(:,strcmp('ForceGround_Foot_1_L_A.ground.force.Y',HeadFR));
Cont_Foot_2_L_A=ForceReport(:,strcmp('ForceGround_Foot_2_L_A.ground.force.Y',HeadFR));
Cont_Foot_3_L_A=ForceReport(:,strcmp('ForceGround_Foot_3_L_A.ground.force.Y',HeadFR));
Cont_Foot_4_L_A=ForceReport(:,strcmp('ForceGround_Foot_4_L_A.ground.force.Y',HeadFR));
Cont_Foot_5_L_A=ForceReport(:,strcmp('ForceGround_Foot_5_L_A.ground.force.Y',HeadFR));
Cont_Foot_6_L_A=ForceReport(:,strcmp('ForceGround_Foot_6_L_A.ground.force.Y',HeadFR));
Cont_Foot_7_L_A=ForceReport(:,strcmp('ForceGround_Foot_7_L_A.ground.force.Y',HeadFR));
Cont_Foot_8_L_A=ForceReport(:,strcmp('ForceGround_Foot_8_L_A.ground.force.Y',HeadFR));
Cont_Foot_9_L_A=ForceReport(:,strcmp('ForceGround_Foot_9_L_A.ground.force.Y',HeadFR));
Cont_Foot_10_L_A=ForceReport(:,strcmp('ForceGround_Foot_10_L_A.ground.force.Y',HeadFR));
Cont_Foot_11_L_A=ForceReport(:,strcmp('ForceGround_Foot_11_L_A.ground.force.Y',HeadFR));
Cont_Foot_12_L_A=ForceReport(:,strcmp('ForceGround_Foot_12_L_A.ground.force.Y',HeadFR));
Cont_Foot_13_L_A=ForceReport(:,strcmp('ForceGround_Foot_13_L_A.ground.force.Y',HeadFR));
Cont_Foot_14_L_A=ForceReport(:,strcmp('ForceGround_Foot_14_L_A.ground.force.Y',HeadFR));
Cont_Foot_1_R_A=ForceReport(:,strcmp('ForceGround_Foot_1_R_A.ground.force.Y',HeadFR));
Cont_Foot_2_R_A=ForceReport(:,strcmp('ForceGround_Foot_2_R_A.ground.force.Y',HeadFR));
Cont_Foot_3_R_A=ForceReport(:,strcmp('ForceGround_Foot_3_R_A.ground.force.Y',HeadFR));
Cont_Foot_4_R_A=ForceReport(:,strcmp('ForceGround_Foot_4_R_A.ground.force.Y',HeadFR));
Cont_Foot_5_R_A=ForceReport(:,strcmp('ForceGround_Foot_5_R_A.ground.force.Y',HeadFR));
Cont_Foot_6_R_A=ForceReport(:,strcmp('ForceGround_Foot_6_R_A.ground.force.Y',HeadFR));
Cont_Foot_7_R_A=ForceReport(:,strcmp('ForceGround_Foot_7_R_A.ground.force.Y',HeadFR));
Cont_Foot_8_R_A=ForceReport(:,strcmp('ForceGround_Foot_8_R_A.ground.force.Y',HeadFR));
Cont_Foot_9_R_A=ForceReport(:,strcmp('ForceGround_Foot_9_R_A.ground.force.Y',HeadFR));
Cont_Foot_10_R_A=ForceReport(:,strcmp('ForceGround_Foot_10_R_A.ground.force.Y',HeadFR));
Cont_Foot_11_R_A=ForceReport(:,strcmp('ForceGround_Foot_11_R_A.ground.force.Y',HeadFR));
Cont_Foot_12_R_A=ForceReport(:,strcmp('ForceGround_Foot_12_R_A.ground.force.Y',HeadFR));
Cont_Foot_13_R_A=ForceReport(:,strcmp('ForceGround_Foot_13_R_A.ground.force.Y',HeadFR));
Cont_Foot_14_R_A=ForceReport(:,strcmp('ForceGround_Foot_14_R_A.ground.force.Y',HeadFR));

if abs(Cont_Foot_1_L_A(position_l))<F_th_sp/3
    PosSp1L=PosSp1L-delta*n';
    fl_1=0;
else
    F_min_L(1)=min(Cont_Foot_1_L_A);
    fl_1=1;
end
if abs(Cont_Foot_1_R_A(position_r))<F_th_sp/3
    PosSp1R=PosSp1R-delta*n';
    fl_2=0;
else
    F_min_R(1)=min(Cont_Foot_1_R_A);
fl_2=1;
end
if abs(Cont_Foot_2_L_A(position_l))<F_th_sp/3
    PosSp2L=PosSp2L-delta*n';
    fl_3=0;
else
    F_min_L(2)=min(Cont_Foot_2_L_A);
    fl_3=1;
end
if abs(Cont_Foot_2_R_A(position_r))<F_th_sp/3
    PosSp2R=PosSp2R-delta*n';
    fl_4=0;
else
    F_min_R(2)=min(Cont_Foot_2_R_A);
    fl_4=1;
end
if abs(Cont_Foot_3_L_A(position_l))<F_th_sp/3
    PosSp3L=PosSp3L-delta*n';
    fl_5=0;
else
    F_min_L(3)=min(Cont_Foot_3_L_A);
    fl_5=1;
end
if abs(Cont_Foot_3_R_A(position_r))<F_th_sp/3
    PosSp3R=PosSp3R-delta*n';
    fl_6=0;
else
    F_min_R(3)=min(Cont_Foot_3_R_A);
      fl_6=1;
end
if abs(Cont_Foot_4_L_A(position_l))<F_th_sp/3
    PosSp4L=PosSp4L-delta*n';
    fl_7=0;
else
    F_min_L(4)=min(Cont_Foot_4_L_A);
   fl_7=1;
end
if abs(Cont_Foot_4_R_A(position_r))<F_th_sp/3
    PosSp4R=PosSp4R-delta*n';
    fl_8=0;
else
   F_min_R(4)=min(Cont_Foot_4_R_A);
   fl_8=1;
end
if abs(Cont_Foot_5_L_A(position_l))<F_th_sp
    PosSp5L=PosSp5L-delta*n';
    fl_9=0;
else
    F_min_L(5)=min(Cont_Foot_5_L_A);
   fl_9=1;
end
if abs(Cont_Foot_5_R_A(position_r))<F_th_sp
    PosSp5R=PosSp5R-delta*n';
    fl_10=0;
else
    F_min_R(5)=min(Cont_Foot_5_R_A);
    fl_10=1;
end
if abs(Cont_Foot_6_L_A(position_l))<F_th_sp
    PosSp6L=PosSp6L-delta*n';
    fl_11=0;
else
    F_min_L(6)=min(Cont_Foot_6_L_A);
    fl_11=1;
end
if abs(Cont_Foot_6_R_A(position_r))<F_th_sp
    PosSp6R=PosSp6R-delta*n';
    fl_12=0;
else
    F_min_R(6)=min(Cont_Foot_6_R_A);
    fl_12=1;
end
if abs(Cont_Foot_7_L_A(position_l))<F_th_sp
    PosSp7L=PosSp7L-delta*n';
    fl_13=0;
else
    F_min_L(7)=min(Cont_Foot_7_L_A);

    fl_13=1;
end
if abs(Cont_Foot_7_R_A(position_r))<F_th_sp
    PosSp7R=PosSp7R-delta*n';
    fl_14=0;
else
    F_min_R(7)=min(Cont_Foot_7_R_A);
    fl_14=1;
end
if abs(Cont_Foot_8_L_A(position_l))<F_th_sp
    PosSp8L=PosSp8L-delta*n';
    fl_15=0;
else
    F_min_L(8)=min(Cont_Foot_8_L_A);
    fl_15=1;
end
if abs(Cont_Foot_8_R_A(position_r))<F_th_sp
    PosSp8R=PosSp8R-delta*n';
    fl_16=0;
else
    F_min_R(8)=min(Cont_Foot_8_R_A);
   fl_16=1;
end
if abs(Cont_Foot_9_L_A(position_l))<F_th_sp
    PosSp9L=PosSp9L-delta*n';
    fl_17=0;
else
    F_min_L(9)=min(Cont_Foot_9_L_A);
    fl_17=1;
end
if abs(Cont_Foot_9_R_A(position_r))<F_th_sp
    PosSp9R=PosSp9R-delta*n';
    fl_18=0;
else
    F_min_R(9)=min(Cont_Foot_9_R_A);
   fl_18=1;
end
if abs(Cont_Foot_10_L_A(position_l))<F_th_sp
    PosSp10L=PosSp10L-delta*n';
    fl_19=0;
else
    F_min_L(10)=min(Cont_Foot_10_L_A);
    fl_19=1;
end
if abs(Cont_Foot_10_R_A(position_r))<F_th_sp
    PosSp10R=PosSp10R-delta*n';
    fl_20=0;
else
    F_min_R(10)=min(Cont_Foot_10_R_A);
   fl_20=1;
end
if abs(Cont_Foot_11_L_A(position_l))<F_th_sp
    PosSp11L=PosSp11L-delta*n';
    fl_21=0;
else
    F_min_L(11)=min(Cont_Foot_11_L_A);
    fl_21=1;
end
if abs(Cont_Foot_11_R_A(position_r))<F_th_sp
    PosSp11R=PosSp11R-delta*n';
    fl_22=0;
else
    F_min_R(11)=min(Cont_Foot_11_R_A);
   fl_22=1;
end
if abs(Cont_Foot_12_L_A(position_l))<F_th_sp
    PosSp12L=PosSp12L-delta*n';
    fl_23=0;
else
    F_min_L(12)=min(Cont_Foot_12_L_A);
    fl_23=1;
end
if abs(Cont_Foot_12_R_A(position_r))<F_th_sp
    PosSp12R=PosSp12R-delta*n';
    fl_24=0;
else
    F_min_R(12)=min(Cont_Foot_12_R_A);
   fl_24=1;
end
if abs(Cont_Foot_13_L_A(position_l))<F_th_sp
    PosSp13L=PosSp13L-delta*n';
    fl_25=0;
else
    F_min_L(13)=min(Cont_Foot_4_L_A);
    fl_25=1;
end
if abs(Cont_Foot_13_R_A(position_r))<F_th_sp
    PosSp13R=PosSp13R-delta*n';
    fl_26=0;
else
    F_min_R(13)=min(Cont_Foot_13_R_A);
   fl_26=1;
end
if abs(Cont_Foot_14_L_A(position_l))<F_th_sp
    PosSp14L=PosSp14L-delta*n';
    fl_27=0;
else
    F_min_L(14)=min(Cont_Foot_14_L_A);
    fl_27=1;
end
if abs(Cont_Foot_14_R_A(position_r))<F_th_sp
    PosSp14R=PosSp14R-delta*n';
    fl_28=0;
else
    F_min_R(14)=min(Cont_Foot_14_R_A);
   fl_28=1;
end

Sphere_Foot_1_L.setLocation(Vec3.createFromMat(PosSp1L));
Sphere_Foot_1_R.setLocation(Vec3.createFromMat(PosSp1R));
Sphere_Foot_2_L.setLocation(Vec3.createFromMat(PosSp2L));
Sphere_Foot_2_R.setLocation(Vec3.createFromMat(PosSp2R));
Sphere_Foot_3_L.setLocation(Vec3.createFromMat(PosSp3L));
Sphere_Foot_3_R.setLocation(Vec3.createFromMat(PosSp3R));
Sphere_Foot_4_L.setLocation(Vec3.createFromMat(PosSp4L));
Sphere_Foot_4_R.setLocation(Vec3.createFromMat(PosSp4R));
Sphere_Foot_5_L.setLocation(Vec3.createFromMat(PosSp5L));
Sphere_Foot_5_R.setLocation(Vec3.createFromMat(PosSp5R));
Sphere_Foot_6_L.setLocation(Vec3.createFromMat(PosSp6L));
Sphere_Foot_6_R.setLocation(Vec3.createFromMat(PosSp6R));
Sphere_Foot_7_L.setLocation(Vec3.createFromMat(PosSp7L));
Sphere_Foot_7_R.setLocation(Vec3.createFromMat(PosSp7R));
Sphere_Foot_8_L.setLocation(Vec3.createFromMat(PosSp8L));
Sphere_Foot_8_R.setLocation(Vec3.createFromMat(PosSp8R));
Sphere_Foot_9_L.setLocation(Vec3.createFromMat(PosSp9L));
Sphere_Foot_9_R.setLocation(Vec3.createFromMat(PosSp9R));
Sphere_Foot_10_L.setLocation(Vec3.createFromMat(PosSp10L));
Sphere_Foot_10_R.setLocation(Vec3.createFromMat(PosSp10R));
Sphere_Foot_11_L.setLocation(Vec3.createFromMat(PosSp11L));
Sphere_Foot_11_R.setLocation(Vec3.createFromMat(PosSp11R));
Sphere_Foot_12_L.setLocation(Vec3.createFromMat(PosSp12L));
Sphere_Foot_12_R.setLocation(Vec3.createFromMat(PosSp12R));
Sphere_Foot_13_L.setLocation(Vec3.createFromMat(PosSp13L));
Sphere_Foot_13_R.setLocation(Vec3.createFromMat(PosSp13R));
Sphere_Foot_14_L.setLocation(Vec3.createFromMat(PosSp14L));
Sphere_Foot_14_R.setLocation(Vec3.createFromMat(PosSp14R));
model.finalizeConnections()
model.print(modelProcessed_path);
counter=counter+1;
if all([fl_1 fl_2 fl_3 fl_4 fl_5 fl_6 fl_7 fl_8 fl_9 fl_10 fl_11 fl_12 fl_13 fl_14 fl_15 fl_16 fl_17 fl_18 fl_19 fl_20 fl_21 fl_22 fl_23 fl_24 fl_25 fl_26 fl_27 fl_28])  
model.finalizeConnections()
model.print(modelProcessed_path); 
   break
end
end

%% spheres calibration phase 2: PK on the spheres with current position

AnTool_PK=AnalyzeTool();
AnTool_PK.setModel(modelProcessed);
AnTool_PK.setModelFilename(fullfile(ModelFolder,"ModelProcessed.osim"));
AnTool_PK.setCoordinatesFileName(fullfile(IKFolder,IKResult));
AnTool_PK.setLowpassCutoffFrequency(Freq);
AnTool_PK.setStartTime(timeStart);
AnTool_PK.setFinalTime(timeEnd);
AnTool_PK.setResultsDir(path_PK);
PK_sp1_r=PointKinematics();
PK_sp1_r.setPointName('PK_sp1_r');
PK_sp1_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp1_r.setPoint(Vec3.createFromMat(PosSp1R-[heel_shift 0 0]))
PK_sp1_r.setRelativeToBody(model.getGround)
PK_sp2_r=PointKinematics();
PK_sp2_r.setPointName('PK_sp2_r');
PK_sp2_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp2_r.setPoint(Vec3.createFromMat(PosSp2R-[heel_shift 0 0]))
PK_sp2_r.setRelativeToBody(model.getGround)
PK_sp3_r=PointKinematics();
PK_sp3_r.setPointName('PK_sp3_r');
PK_sp3_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp3_r.setPoint(Vec3.createFromMat(PosSp3R-[heel_shift 0 0]))
PK_sp3_r.setRelativeToBody(model.getGround)

PK_sp4_r=PointKinematics();
PK_sp4_r.setPointName('PK_sp4_r');
PK_sp4_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp4_r.setPoint(Vec3.createFromMat(PosSp4R-[heel_shift 0 0]))
PK_sp4_r.setRelativeToBody(model.getGround)

PK_sp5_r=PointKinematics();
PK_sp5_r.setPointName('PK_sp5_r');
PK_sp5_r.setBody(model.getBodySet.get(toes_r_name))
PK_sp5_r.setPoint(Vec3.createFromMat(PosSp5R))
PK_sp5_r.setRelativeToBody(model.getGround)

PK_sp6_r=PointKinematics();
PK_sp6_r.setPointName('PK_sp6_r');
PK_sp6_r.setBody(model.getBodySet.get(toes_r_name))
PK_sp6_r.setPoint(Vec3.createFromMat(PosSp6R))
PK_sp6_r.setRelativeToBody(model.getGround)

PK_sp7_r=PointKinematics();
PK_sp7_r.setPointName('PK_sp7_r');
PK_sp7_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp7_r.setPoint(Vec3.createFromMat(PosSp7R))
PK_sp7_r.setRelativeToBody(model.getGround)
PK_sp8_r=PointKinematics();
PK_sp8_r.setPointName('PK_sp8_r');
PK_sp8_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp8_r.setPoint(Vec3.createFromMat(PosSp8R))
PK_sp8_r.setRelativeToBody(model.getGround)

PK_sp9_r=PointKinematics();
PK_sp9_r.setPointName('PK_sp9_r');
PK_sp9_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp9_r.setPoint(Vec3.createFromMat(PosSp9R))
PK_sp9_r.setRelativeToBody(model.getGround)

PK_sp10_r=PointKinematics();
PK_sp10_r.setPointName('PK_sp10_r');
PK_sp10_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp10_r.setPoint(Vec3.createFromMat(PosSp10R))
PK_sp10_r.setRelativeToBody(model.getGround)

PK_sp11_r=PointKinematics();
PK_sp11_r.setPointName('PK_sp11_r');
PK_sp11_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp11_r.setPoint(Vec3.createFromMat(PosSp11R))
PK_sp11_r.setRelativeToBody(model.getGround)

PK_sp12_r=PointKinematics();
PK_sp12_r.setPointName('PK_sp12_r');
PK_sp12_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp12_r.setPoint(Vec3.createFromMat(PosSp12R))
PK_sp12_r.setRelativeToBody(model.getGround)

PK_sp13_r=PointKinematics();
PK_sp13_r.setPointName('PK_sp13_r');
PK_sp13_r.setBody(model.getBodySet.get(toes_r_name))
PK_sp13_r.setPoint(Vec3.createFromMat(PosSp13R))
PK_sp13_r.setRelativeToBody(model.getGround)

PK_sp14_r=PointKinematics();
PK_sp14_r.setPointName('PK_sp14_r');
PK_sp14_r.setBody(model.getBodySet.get(toes_r_name))
PK_sp14_r.setPoint(Vec3.createFromMat(PosSp14R))
PK_sp14_r.setRelativeToBody(model.getGround)

PK_sp1_l=PointKinematics();
PK_sp1_l.setPointName('PK_sp1_l');
PK_sp1_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp1_l.setPoint(Vec3.createFromMat(PosSp1L-[heel_shift 0 0]))
PK_sp1_l.setRelativeToBody(model.getGround)

PK_sp2_l=PointKinematics();
PK_sp2_l.setPointName('PK_sp2_l');
PK_sp2_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp2_l.setPoint(Vec3.createFromMat(PosSp2L-[heel_shift 0 0]))
PK_sp2_l.setRelativeToBody(model.getGround)

PK_sp3_l=PointKinematics();
PK_sp3_l.setPointName('PK_sp3_l');
PK_sp3_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp3_l.setPoint(Vec3.createFromMat(PosSp3L-[heel_shift 0 0]))
PK_sp3_l.setRelativeToBody(model.getGround)

PK_sp4_l=PointKinematics();
PK_sp4_l.setPointName('PK_sp4_l');
PK_sp4_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp4_l.setPoint(Vec3.createFromMat(PosSp4L-[heel_shift 0 0]))
PK_sp4_l.setRelativeToBody(model.getGround)

PK_sp5_l=PointKinematics();
PK_sp5_l.setPointName('PK_sp5_l');
PK_sp5_l.setBody(model.getBodySet.get(toes_l_name))
PK_sp5_l.setPoint(Vec3.createFromMat(PosSp5L))
PK_sp5_l.setRelativeToBody(model.getGround)

PK_sp6_l=PointKinematics();
PK_sp6_l.setPointName('PK_sp6_l');
PK_sp6_l.setBody(model.getBodySet.get(toes_l_name))
PK_sp6_l.setPoint(Vec3.createFromMat(PosSp6L))
PK_sp6_l.setRelativeToBody(model.getGround)

PK_sp7_l=PointKinematics();
PK_sp7_l.setPointName('PK_sp7_l');
PK_sp7_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp7_l.setPoint(Vec3.createFromMat(PosSp7L))
PK_sp7_l.setRelativeToBody(model.getGround)

PK_sp8_l=PointKinematics();
PK_sp8_l.setPointName('PK_sp8_l');
PK_sp8_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp8_l.setPoint(Vec3.createFromMat(PosSp8L))
PK_sp8_l.setRelativeToBody(model.getGround)

PK_sp9_l=PointKinematics();
PK_sp9_l.setPointName('PK_sp9_l');
PK_sp9_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp9_l.setPoint(Vec3.createFromMat(PosSp9L))
PK_sp9_l.setRelativeToBody(model.getGround)

PK_sp10_l=PointKinematics();
PK_sp10_l.setPointName('PK_sp10_l');
PK_sp10_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp10_l.setPoint(Vec3.createFromMat(PosSp10L))
PK_sp10_l.setRelativeToBody(model.getGround)

PK_sp11_l=PointKinematics();
PK_sp11_l.setPointName('PK_sp11_l');
PK_sp11_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp11_l.setPoint(Vec3.createFromMat(PosSp11L))
PK_sp11_l.setRelativeToBody(model.getGround)

PK_sp12_l=PointKinematics();
PK_sp12_l.setPointName('PK_sp12_l');
PK_sp12_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp12_l.setPoint(Vec3.createFromMat(PosSp12L))
PK_sp12_l.setRelativeToBody(model.getGround)

PK_sp13_l=PointKinematics();
PK_sp13_l.setPointName('PK_sp13_l');
PK_sp13_l.setBody(model.getBodySet.get(toes_l_name))
PK_sp13_l.setPoint(Vec3.createFromMat(PosSp13L))
PK_sp13_l.setRelativeToBody(model.getGround)

PK_sp14_l=PointKinematics();
PK_sp14_l.setPointName('PK_sp14_l');
PK_sp14_l.setBody(model.getBodySet.get(toes_l_name))
PK_sp14_l.setPoint(Vec3.createFromMat(PosSp14L))
PK_sp14_l.setRelativeToBody(model.getGround)

AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp1_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp2_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp3_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp4_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp5_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp6_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp7_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp8_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp9_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp10_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp11_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp12_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp13_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp14_r);

AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp1_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp2_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp3_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp4_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp5_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp6_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp7_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp8_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp9_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp10_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp11_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp12_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp13_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp14_l);

AnTool_PK.print(fullfile(path_Solution,'SetupPK_sp.xml'));
AnalyzeTool(fullfile(path_Solution,'SetupPK_sp.xml')).run;
PK_res_sp_1_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp1_r_pos.sto'));
PK_res_sp_2_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp2_r_pos.sto'));
PK_res_sp_3_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp3_r_pos.sto'));
PK_res_sp_4_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp4_r_pos.sto'));
PK_res_sp_5_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp5_r_pos.sto'));
PK_res_sp_6_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp6_r_pos.sto'));
PK_res_sp_7_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp7_r_pos.sto'));
PK_res_sp_8_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp8_r_pos.sto'));
PK_res_sp_9_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp9_r_pos.sto'));
PK_res_sp_10_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp10_r_pos.sto'));
PK_res_sp_11_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp11_r_pos.sto'));
PK_res_sp_12_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp12_r_pos.sto'));
PK_res_sp_13_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp13_r_pos.sto'));
PK_res_sp_14_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp14_r_pos.sto'));

PK_res_sp_1_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp1_l_pos.sto'));
PK_res_sp_2_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp2_l_pos.sto'));
PK_res_sp_3_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp3_l_pos.sto'));
PK_res_sp_4_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp4_l_pos.sto'));
PK_res_sp_5_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp5_l_pos.sto'));
PK_res_sp_6_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp6_l_pos.sto'));
PK_res_sp_7_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp7_l_pos.sto'));
PK_res_sp_8_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp8_l_pos.sto'));
PK_res_sp_9_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp9_l_pos.sto'));
PK_res_sp_10_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp10_l_pos.sto'));
PK_res_sp_11_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp11_l_pos.sto'));
PK_res_sp_12_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp12_l_pos.sto'));
PK_res_sp_13_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp13_l_pos.sto'));
PK_res_sp_14_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp14_l_pos.sto'));

for i=1:length(timeBK)
dist_sp_L(i,1)=dot(PK_res_sp_1_l(i,2:4),n);
dist_sp_L(i,2)=dot(PK_res_sp_2_l(i,2:4),n);
dist_sp_L(i,3)=dot(PK_res_sp_3_l(i,2:4),n);
dist_sp_L(i,4)=dot(PK_res_sp_4_l(i,2:4),n);
dist_sp_L(i,5)=dot(PK_res_sp_5_l(i,2:4),n);
dist_sp_L(i,6)=dot(PK_res_sp_6_l(i,2:4),n);
dist_sp_L(i,7)=dot(PK_res_sp_7_l(i,2:4),n);
dist_sp_L(i,8)=dot(PK_res_sp_8_l(i,2:4),n);
dist_sp_L(i,9)=dot(PK_res_sp_9_l(i,2:4),n);
dist_sp_L(i,10)=dot(PK_res_sp_10_l(i,2:4),n);
dist_sp_L(i,11)=dot(PK_res_sp_11_l(i,2:4),n);
dist_sp_L(i,12)=dot(PK_res_sp_12_l(i,2:4),n);
dist_sp_L(i,13)=dot(PK_res_sp_13_l(i,2:4),n);
dist_sp_L(i,14)=dot(PK_res_sp_14_l(i,2:4),n);

dist_sp_R(i,1)=dot(PK_res_sp_1_r(i,2:4),n);
dist_sp_R(i,2)=dot(PK_res_sp_2_r(i,2:4),n);
dist_sp_R(i,3)=dot(PK_res_sp_3_r(i,2:4),n);
dist_sp_R(i,4)=dot(PK_res_sp_4_r(i,2:4),n);
dist_sp_R(i,5)=dot(PK_res_sp_5_r(i,2:4),n);
dist_sp_R(i,6)=dot(PK_res_sp_6_r(i,2:4),n);
dist_sp_R(i,7)=dot(PK_res_sp_7_r(i,2:4),n);
dist_sp_R(i,8)=dot(PK_res_sp_8_r(i,2:4),n);
dist_sp_R(i,9)=dot(PK_res_sp_9_r(i,2:4),n);
dist_sp_R(i,10)=dot(PK_res_sp_10_r(i,2:4),n);
dist_sp_R(i,11)=dot(PK_res_sp_11_r(i,2:4),n);
dist_sp_R(i,12)=dot(PK_res_sp_12_r(i,2:4),n);
dist_sp_R(i,13)=dot(PK_res_sp_13_r(i,2:4),n);
dist_sp_R(i,14)=dot(PK_res_sp_14_r(i,2:4),n);
end

%figure, plot(Y_sp_R)

PosSp1R=PosSp1R-[0 max(min(dist_sp_R(:,1)),min(dist_sp_R(:,2)))-dist_sp_R(position_r,1) 0];
PosSp2R=PosSp2R-[0 max(min(dist_sp_R(:,1)),min(dist_sp_R(:,2)))-dist_sp_R(position_r,2) 0];
PosSp3R=PosSp3R-[0 max(min(dist_sp_R(:,3)),min(dist_sp_R(:,4)))-dist_sp_R(position_r,3) 0];
PosSp4R=PosSp4R-[0 max(min(dist_sp_R(:,3)),min(dist_sp_R(:,4)))-dist_sp_R(position_r,4) 0];
PosSp5R=PosSp5R-[0 max(min(dist_sp_R(:,5)),min(dist_sp_R(:,6)))-dist_sp_R(position_r,5) 0];
PosSp6R=PosSp6R-[0 max(min(dist_sp_R(:,5)),min(dist_sp_R(:,6)))-dist_sp_R(position_r,6) 0];
PosSp7R=PosSp7R-[0 max(min(dist_sp_R(:,7)),min(dist_sp_R(:,8)))-dist_sp_R(position_r,7) 0];
PosSp8R=PosSp8R-[0 max(min(dist_sp_R(:,7)),min(dist_sp_R(:,8)))-dist_sp_R(position_r,8) 0];
PosSp9R=PosSp9R-[0 max(min(dist_sp_R(:,9)),min(dist_sp_R(:,10)))-dist_sp_R(position_r,9) 0];
PosSp10R=PosSp10R-[0 max(min(dist_sp_R(:,9)),min(dist_sp_R(:,10)))-dist_sp_R(position_r,10) 0];
PosSp11R=PosSp11R-[0 max(min(dist_sp_R(:,11)),min(dist_sp_R(:,12)))-dist_sp_R(position_r,11) 0];
PosSp12R=PosSp12R-[0 max(min(dist_sp_R(:,11)),min(dist_sp_R(:,12)))-dist_sp_R(position_r,12) 0];
PosSp13R=PosSp13R-[0 max(min(dist_sp_R(:,13)),min(dist_sp_R(:,14)))-dist_sp_R(position_r,13) 0];
PosSp14R=PosSp14R-[0 max(min(dist_sp_R(:,13)),min(dist_sp_R(:,14)))-dist_sp_R(position_r,14) 0];

PosSp1L=PosSp1L-[0 max(min(dist_sp_L(:,1)),min(dist_sp_L(:,2)))-dist_sp_L(position_l,1) 0];
PosSp2L=PosSp2L-[0 max(min(dist_sp_L(:,1)),min(dist_sp_L(:,2)))-dist_sp_L(position_l,2) 0];
PosSp3L=PosSp3L-[0 max(min(dist_sp_L(:,3)),min(dist_sp_L(:,4)))-dist_sp_L(position_l,3) 0];
PosSp4L=PosSp4L-[0 max(min(dist_sp_L(:,3)),min(dist_sp_L(:,4)))-dist_sp_L(position_l,4) 0];
PosSp5L=PosSp5L-[0 max(min(dist_sp_L(:,5)),min(dist_sp_L(:,6)))-dist_sp_L(position_l,5) 0];
PosSp6L=PosSp6L-[0 max(min(dist_sp_L(:,5)),min(dist_sp_L(:,6)))-dist_sp_L(position_l,6) 0];
PosSp7L=PosSp7L-[0 max(min(dist_sp_L(:,7)),min(dist_sp_L(:,8)))-dist_sp_L(position_l,7) 0];
PosSp8L=PosSp8L-[0 max(min(dist_sp_L(:,7)),min(dist_sp_L(:,8)))-dist_sp_L(position_l,8) 0];
PosSp9L=PosSp9L-[0 max(min(dist_sp_L(:,9)),min(dist_sp_L(:,10)))-dist_sp_L(position_l,9) 0];
PosSp10L=PosSp10L-[0 max(min(dist_sp_L(:,9)),min(dist_sp_L(:,10)))-dist_sp_L(position_l,10) 0];
PosSp11L=PosSp11L-[0 max(min(dist_sp_L(:,11)),min(dist_sp_L(:,12)))-dist_sp_L(position_l,11) 0];
PosSp12L=PosSp12L-[0 max(min(dist_sp_L(:,11)),min(dist_sp_L(:,12)))-dist_sp_L(position_l,12) 0];
PosSp13L=PosSp13L-[0 max(min(dist_sp_L(:,13)),min(dist_sp_L(:,14)))-dist_sp_L(position_l,13) 0];
PosSp14L=PosSp14L-[0 max(min(dist_sp_L(:,13)),min(dist_sp_L(:,14)))-dist_sp_L(position_l,14) 0];

%%
Sphere_Foot_1_L.setLocation(Vec3.createFromMat(PosSp1L));
Sphere_Foot_1_R.setLocation(Vec3.createFromMat(PosSp1R));
Sphere_Foot_2_L.setLocation(Vec3.createFromMat(PosSp2L));
Sphere_Foot_2_R.setLocation(Vec3.createFromMat(PosSp2R));
Sphere_Foot_3_L.setLocation(Vec3.createFromMat(PosSp3L));
Sphere_Foot_3_R.setLocation(Vec3.createFromMat(PosSp3R));
Sphere_Foot_4_L.setLocation(Vec3.createFromMat(PosSp4L));
Sphere_Foot_4_R.setLocation(Vec3.createFromMat(PosSp4R));
Sphere_Foot_5_L.setLocation(Vec3.createFromMat(PosSp5L));
Sphere_Foot_5_R.setLocation(Vec3.createFromMat(PosSp5R));
Sphere_Foot_6_L.setLocation(Vec3.createFromMat(PosSp6L));
Sphere_Foot_6_R.setLocation(Vec3.createFromMat(PosSp6R));
Sphere_Foot_7_L.setLocation(Vec3.createFromMat(PosSp7L));
Sphere_Foot_7_R.setLocation(Vec3.createFromMat(PosSp7R));
Sphere_Foot_8_L.setLocation(Vec3.createFromMat(PosSp8L));
Sphere_Foot_8_R.setLocation(Vec3.createFromMat(PosSp8R));
Sphere_Foot_9_L.setLocation(Vec3.createFromMat(PosSp9L));
Sphere_Foot_9_R.setLocation(Vec3.createFromMat(PosSp9R));
Sphere_Foot_10_L.setLocation(Vec3.createFromMat(PosSp10L));
Sphere_Foot_10_R.setLocation(Vec3.createFromMat(PosSp10R));
Sphere_Foot_11_L.setLocation(Vec3.createFromMat(PosSp11L));
Sphere_Foot_11_R.setLocation(Vec3.createFromMat(PosSp11R));
Sphere_Foot_12_L.setLocation(Vec3.createFromMat(PosSp12L));
Sphere_Foot_12_R.setLocation(Vec3.createFromMat(PosSp12R));
Sphere_Foot_13_L.setLocation(Vec3.createFromMat(PosSp13L));
Sphere_Foot_13_R.setLocation(Vec3.createFromMat(PosSp13R));
Sphere_Foot_14_L.setLocation(Vec3.createFromMat(PosSp14L));
Sphere_Foot_14_R.setLocation(Vec3.createFromMat(PosSp14R));
model.finalizeConnections()
model.print(modelProcessed_path);
    
ForceTool=AnalyzeTool(fullfile(path_Solution,'Setup_ForceReporter.xml'));
ForceTool.run;
% analyze Force Reporter Results
[ForceReport,HeadFR]=load_sto(fullfile(path_FR,'\_ForceReporter_forces.sto'));
Cont_Foot_1_L_A=ForceReport(:,strcmp('ForceGround_Foot_1_L_A.ground.force.Y',HeadFR));
Cont_Foot_2_L_A=ForceReport(:,strcmp('ForceGround_Foot_2_L_A.ground.force.Y',HeadFR));
Cont_Foot_3_L_A=ForceReport(:,strcmp('ForceGround_Foot_3_L_A.ground.force.Y',HeadFR));
Cont_Foot_4_L_A=ForceReport(:,strcmp('ForceGround_Foot_4_L_A.ground.force.Y',HeadFR));
Cont_Foot_5_L_A=ForceReport(:,strcmp('ForceGround_Foot_5_L_A.ground.force.Y',HeadFR));
Cont_Foot_6_L_A=ForceReport(:,strcmp('ForceGround_Foot_6_L_A.ground.force.Y',HeadFR));
Cont_Foot_7_L_A=ForceReport(:,strcmp('ForceGround_Foot_7_L_A.ground.force.Y',HeadFR));
Cont_Foot_8_L_A=ForceReport(:,strcmp('ForceGround_Foot_8_L_A.ground.force.Y',HeadFR));
Cont_Foot_9_L_A=ForceReport(:,strcmp('ForceGround_Foot_9_L_A.ground.force.Y',HeadFR));
Cont_Foot_10_L_A=ForceReport(:,strcmp('ForceGround_Foot_10_L_A.ground.force.Y',HeadFR));
Cont_Foot_11_L_A=ForceReport(:,strcmp('ForceGround_Foot_11_L_A.ground.force.Y',HeadFR));
Cont_Foot_12_L_A=ForceReport(:,strcmp('ForceGround_Foot_12_L_A.ground.force.Y',HeadFR));
Cont_Foot_13_L_A=ForceReport(:,strcmp('ForceGround_Foot_13_L_A.ground.force.Y',HeadFR));
Cont_Foot_14_L_A=ForceReport(:,strcmp('ForceGround_Foot_14_L_A.ground.force.Y',HeadFR));
Cont_Foot_1_R_A=ForceReport(:,strcmp('ForceGround_Foot_1_R_A.ground.force.Y',HeadFR));
Cont_Foot_2_R_A=ForceReport(:,strcmp('ForceGround_Foot_2_R_A.ground.force.Y',HeadFR));
Cont_Foot_3_R_A=ForceReport(:,strcmp('ForceGround_Foot_3_R_A.ground.force.Y',HeadFR));
Cont_Foot_4_R_A=ForceReport(:,strcmp('ForceGround_Foot_4_R_A.ground.force.Y',HeadFR));
Cont_Foot_5_R_A=ForceReport(:,strcmp('ForceGround_Foot_5_R_A.ground.force.Y',HeadFR));
Cont_Foot_6_R_A=ForceReport(:,strcmp('ForceGround_Foot_6_R_A.ground.force.Y',HeadFR));
Cont_Foot_7_R_A=ForceReport(:,strcmp('ForceGround_Foot_7_R_A.ground.force.Y',HeadFR));
Cont_Foot_8_R_A=ForceReport(:,strcmp('ForceGround_Foot_8_R_A.ground.force.Y',HeadFR));
Cont_Foot_9_R_A=ForceReport(:,strcmp('ForceGround_Foot_9_R_A.ground.force.Y',HeadFR));
Cont_Foot_10_R_A=ForceReport(:,strcmp('ForceGround_Foot_10_R_A.ground.force.Y',HeadFR));
Cont_Foot_11_R_A=ForceReport(:,strcmp('ForceGround_Foot_11_R_A.ground.force.Y',HeadFR));
Cont_Foot_12_R_A=ForceReport(:,strcmp('ForceGround_Foot_12_R_A.ground.force.Y',HeadFR));
Cont_Foot_13_R_A=ForceReport(:,strcmp('ForceGround_Foot_13_R_A.ground.force.Y',HeadFR));
Cont_Foot_14_R_A=ForceReport(:,strcmp('ForceGround_Foot_14_R_A.ground.force.Y',HeadFR));
Cont_Foot_L_A=Cont_Foot_1_L_A+Cont_Foot_2_L_A+Cont_Foot_3_L_A+Cont_Foot_4_L_A+Cont_Foot_5_L_A+Cont_Foot_6_L_A+Cont_Foot_7_L_A+Cont_Foot_8_L_A+Cont_Foot_9_L_A+Cont_Foot_10_L_A+Cont_Foot_11_L_A+Cont_Foot_12_L_A+Cont_Foot_13_L_A+Cont_Foot_14_L_A;
Cont_Foot_R_A=Cont_Foot_1_R_A+Cont_Foot_2_R_A+Cont_Foot_3_R_A+Cont_Foot_4_R_A+Cont_Foot_5_R_A+Cont_Foot_6_R_A+Cont_Foot_7_R_A+Cont_Foot_8_R_A+Cont_Foot_9_R_A+Cont_Foot_10_R_A+Cont_Foot_11_R_A+Cont_Foot_12_R_A+Cont_Foot_13_R_A+Cont_Foot_14_R_A;

%% finding the contact intervals and penetrations for the second plane (B)
% finding the interval where the contact is active: just when calcn and
% toes are > of the contact plane
Height_foot_R_B=Ground_R_Foot_T_B(2);
Height_foot_L_B=Ground_L_Foot_T_B(2);

if Ground_R_Foot_T_A(2)~=Ground_R_Foot_T_B(2)
    t_cont_foot_r_B=timeBK(all([Y_Calcn_r>Height_foot_R_B,Y_Toes_r>Height_foot_R_B,v_calcn_r_n<=0.35],2));
else
    t_cont_foot_r_B=timeBK;
end
if Ground_L_Foot_T_A(2)~=Ground_L_Foot_T_B(2)
    t_cont_foot_l_B=timeBK(all([Y_Calcn_l>Height_foot_L_B,Y_Toes_l>Height_foot_L_B,v_calcn_l_n<=0.35],2));
else
    t_cont_foot_l_B=timeBK;
end
ind_t_cont_foot_r_B=find(ismember(timeBK,t_cont_foot_r_B)); % find the position of this contact inside the time vector
ind_t_cont_foot_l_B=find(ismember(timeBK,t_cont_foot_l_B));

% creation of Contact B forces
ForceGround_Foot_1_R_B = HuntCrossleyForce();
ForceGround_Foot_1_R_B.setName('ForceGround_Foot_1_R_B');
ForceGround_Foot_1_R_B.set_appliesForce(true);
ForceGround_Foot_1_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_1_R')
ForceGround_Foot_1_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_1_R_B.setDissipation(0);
ForceGround_Foot_1_R_B.setStaticFriction(0);
ForceGround_Foot_1_R_B.setDynamicFriction(0);
ForceGround_Foot_1_R_B.setViscousFriction(0);
ForceGround_Foot_1_R_B.setTransitionVelocity(0.13)
%ForceGround_Foot1_R.print('es_contForce.xml')
ForceGround_Foot_2_R_B = HuntCrossleyForce();
ForceGround_Foot_2_R_B.setName('ForceGround_Foot_2_R_B');
ForceGround_Foot_2_R_B.set_appliesForce(true);
ForceGround_Foot_2_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_2_R')
ForceGround_Foot_2_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_2_R_B.setDissipation(0);
ForceGround_Foot_2_R_B.setStaticFriction(0);
ForceGround_Foot_2_R_B.setDynamicFriction(0);
ForceGround_Foot_2_R_B.setViscousFriction(0);
ForceGround_Foot_2_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_1_L_B = HuntCrossleyForce();
ForceGround_Foot_1_L_B.setName('ForceGround_Foot_1_L_B');
ForceGround_Foot_1_L_B.set_appliesForce(true);
ForceGround_Foot_1_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_1_L')
ForceGround_Foot_1_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_1_L_B.setDissipation(0);
ForceGround_Foot_1_L_B.setStaticFriction(0);
ForceGround_Foot_1_L_B.setDynamicFriction(0);
ForceGround_Foot_1_L_B.setViscousFriction(0);
ForceGround_Foot_1_L_B.setTransitionVelocity(0.13)
%ForceGround_Foot1_R.print('es_contForce.xml')
ForceGround_Foot_2_L_B = HuntCrossleyForce();
ForceGround_Foot_2_L_B.setName('ForceGround_Foot_2_L_B');
ForceGround_Foot_2_L_B.set_appliesForce(true);
ForceGround_Foot_2_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_2_L')
ForceGround_Foot_2_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_2_L_B.setDissipation(0);
ForceGround_Foot_2_L_B.setStaticFriction(0);
ForceGround_Foot_2_L_B.setDynamicFriction(0);
ForceGround_Foot_2_L_B.setViscousFriction(0);
ForceGround_Foot_2_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_3_R_B = HuntCrossleyForce();
ForceGround_Foot_3_R_B.setName('ForceGround_Foot_3_R_B');
ForceGround_Foot_3_R_B.set_appliesForce(true);
ForceGround_Foot_3_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_3_R');
ForceGround_Foot_3_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_3_R_B.setDissipation(0);
ForceGround_Foot_3_R_B.setStaticFriction(0);
ForceGround_Foot_3_R_B.setDynamicFriction(0);
ForceGround_Foot_3_R_B.setViscousFriction(0);
ForceGround_Foot_3_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_3_L_B=HuntCrossleyForce();
ForceGround_Foot_3_L_B.setName('ForceGround_Foot_3_L_B');
ForceGround_Foot_3_L_B.set_appliesForce(true);
ForceGround_Foot_3_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_3_L')
ForceGround_Foot_3_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_3_L_B.setDissipation(0);
ForceGround_Foot_3_L_B.setStaticFriction(0);
ForceGround_Foot_3_L_B.setDynamicFriction(0);
ForceGround_Foot_3_L_B.setViscousFriction(0);
ForceGround_Foot_3_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_4_R_B = HuntCrossleyForce();
ForceGround_Foot_4_R_B.setName('ForceGround_Foot_4_R_B');
ForceGround_Foot_4_R_B.set_appliesForce(true);
ForceGround_Foot_4_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_4_R');
ForceGround_Foot_4_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_4_R_B.setDissipation(0);
ForceGround_Foot_4_R_B.setStaticFriction(0);
ForceGround_Foot_4_R_B.setDynamicFriction(0);
ForceGround_Foot_4_R_B.setViscousFriction(0);
ForceGround_Foot_4_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_4_L_B=HuntCrossleyForce();
ForceGround_Foot_4_L_B.setName('ForceGround_Foot_4_L_B');
ForceGround_Foot_4_L_B.set_appliesForce(true);
ForceGround_Foot_4_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_4_L')
ForceGround_Foot_4_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_4_L_B.setDissipation(0);
ForceGround_Foot_4_L_B.setStaticFriction(0);
ForceGround_Foot_4_L_B.setDynamicFriction(0);
ForceGround_Foot_4_L_B.setViscousFriction(0);
ForceGround_Foot_4_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_5_R_B = HuntCrossleyForce();
ForceGround_Foot_5_R_B.setName('ForceGround_Foot_5_R_B');
ForceGround_Foot_5_R_B.set_appliesForce(true);
ForceGround_Foot_5_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_5_R');
ForceGround_Foot_5_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_5_R_B.setDissipation(0);
ForceGround_Foot_5_R_B.setStaticFriction(0);
ForceGround_Foot_5_R_B.setDynamicFriction(0);
ForceGround_Foot_5_R_B.setViscousFriction(0);
ForceGround_Foot_5_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_5_L_B=HuntCrossleyForce();
ForceGround_Foot_5_L_B.setName('ForceGround_Foot_5_L_B');
ForceGround_Foot_5_L_B.set_appliesForce(true);
ForceGround_Foot_5_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_5_L')
ForceGround_Foot_5_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_5_L_B.setDissipation(0);
ForceGround_Foot_5_L_B.setStaticFriction(0);
ForceGround_Foot_5_L_B.setDynamicFriction(0);
ForceGround_Foot_5_L_B.setViscousFriction(0);
ForceGround_Foot_5_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_6_R_B = HuntCrossleyForce();
ForceGround_Foot_6_R_B.setName('ForceGround_Foot_6_R_B');
ForceGround_Foot_6_R_B.set_appliesForce(true);
ForceGround_Foot_6_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_6_R');
ForceGround_Foot_6_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_6_R_B.setDissipation(0);
ForceGround_Foot_6_R_B.setStaticFriction(0);
ForceGround_Foot_6_R_B.setDynamicFriction(0);
ForceGround_Foot_6_R_B.setViscousFriction(0);
ForceGround_Foot_6_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_6_L_B=HuntCrossleyForce();
ForceGround_Foot_6_L_B.setName('ForceGround_Foot_6_L_B');
ForceGround_Foot_6_L_B.set_appliesForce(true);
ForceGround_Foot_6_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_6_L')
ForceGround_Foot_6_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_6_L_B.setDissipation(0);
ForceGround_Foot_6_L_B.setStaticFriction(0);
ForceGround_Foot_6_L_B.setDynamicFriction(0);
ForceGround_Foot_6_L_B.setViscousFriction(0);
ForceGround_Foot_6_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_7_R_B = HuntCrossleyForce();
ForceGround_Foot_7_R_B.setName('ForceGround_Foot_7_R_B');
ForceGround_Foot_7_R_B.set_appliesForce(true);
ForceGround_Foot_7_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_7_R');
ForceGround_Foot_7_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_7_R_B.setDissipation(0);
ForceGround_Foot_7_R_B.setStaticFriction(0);
ForceGround_Foot_7_R_B.setDynamicFriction(0);
ForceGround_Foot_7_R_B.setViscousFriction(0);
ForceGround_Foot_7_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_7_L_B=HuntCrossleyForce();
ForceGround_Foot_7_L_B.setName('ForceGround_Foot_7_L_B');
ForceGround_Foot_7_L_B.set_appliesForce(true);
ForceGround_Foot_7_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_7_L')
ForceGround_Foot_7_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_7_L_B.setDissipation(0);
ForceGround_Foot_7_L_B.setStaticFriction(0);
ForceGround_Foot_7_L_B.setDynamicFriction(0);
ForceGround_Foot_7_L_B.setViscousFriction(0);
ForceGround_Foot_7_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_8_R_B = HuntCrossleyForce();
ForceGround_Foot_8_R_B.setName('ForceGround_Foot_8_R_B');
ForceGround_Foot_8_R_B.set_appliesForce(true);
ForceGround_Foot_8_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_8_R');
ForceGround_Foot_8_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_8_R_B.setDissipation(0);
ForceGround_Foot_8_R_B.setStaticFriction(0);
ForceGround_Foot_8_R_B.setDynamicFriction(0);
ForceGround_Foot_8_R_B.setViscousFriction(0);
ForceGround_Foot_8_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_8_L_B=HuntCrossleyForce();
ForceGround_Foot_8_L_B.setName('ForceGround_Foot_8_L_B');
ForceGround_Foot_8_L_B.set_appliesForce(true);
ForceGround_Foot_8_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_8_L')
ForceGround_Foot_8_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_8_L_B.setDissipation(0);
ForceGround_Foot_8_L_B.setStaticFriction(0);
ForceGround_Foot_8_L_B.setDynamicFriction(0);
ForceGround_Foot_8_L_B.setViscousFriction(0);
ForceGround_Foot_8_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_9_R_B = HuntCrossleyForce();
ForceGround_Foot_9_R_B.setName('ForceGround_Foot_9_R_B');
ForceGround_Foot_9_R_B.set_appliesForce(true);
ForceGround_Foot_9_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_9_R');
ForceGround_Foot_9_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_9_R_B.setDissipation(0);
ForceGround_Foot_9_R_B.setStaticFriction(0);
ForceGround_Foot_9_R_B.setDynamicFriction(0);
ForceGround_Foot_9_R_B.setViscousFriction(0);
ForceGround_Foot_9_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_9_L_B=HuntCrossleyForce();
ForceGround_Foot_9_L_B.setName('ForceGround_Foot_9_L_B');
ForceGround_Foot_9_L_B.set_appliesForce(true);
ForceGround_Foot_9_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_9_L')
ForceGround_Foot_9_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_9_L_B.setDissipation(0);
ForceGround_Foot_9_L_B.setStaticFriction(0);
ForceGround_Foot_9_L_B.setDynamicFriction(0);
ForceGround_Foot_9_L_B.setViscousFriction(0);
ForceGround_Foot_9_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_10_R_B = HuntCrossleyForce();
ForceGround_Foot_10_R_B.setName('ForceGround_Foot_10_R_B');
ForceGround_Foot_10_R_B.set_appliesForce(true);
ForceGround_Foot_10_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_10_R');
ForceGround_Foot_10_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_10_R_B.setDissipation(0);
ForceGround_Foot_10_R_B.setStaticFriction(0);
ForceGround_Foot_10_R_B.setDynamicFriction(0);
ForceGround_Foot_10_R_B.setViscousFriction(0);
ForceGround_Foot_10_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_10_L_B=HuntCrossleyForce();
ForceGround_Foot_10_L_B.setName('ForceGround_Foot_10_L_B');
ForceGround_Foot_10_L_B.set_appliesForce(true);
ForceGround_Foot_10_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_10_L')
ForceGround_Foot_10_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_10_L_B.setDissipation(0);
ForceGround_Foot_10_L_B.setStaticFriction(0);
ForceGround_Foot_10_L_B.setDynamicFriction(0);
ForceGround_Foot_10_L_B.setViscousFriction(0);
ForceGround_Foot_10_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_11_R_B = HuntCrossleyForce();
ForceGround_Foot_11_R_B.setName('ForceGround_Foot_11_R_B');
ForceGround_Foot_11_R_B.set_appliesForce(true);
ForceGround_Foot_11_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_11_R');
ForceGround_Foot_11_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_11_R_B.setDissipation(0);
ForceGround_Foot_11_R_B.setStaticFriction(0);
ForceGround_Foot_11_R_B.setDynamicFriction(0);
ForceGround_Foot_11_R_B.setViscousFriction(0);
ForceGround_Foot_11_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_11_L_B=HuntCrossleyForce();
ForceGround_Foot_11_L_B.setName('ForceGround_Foot_11_L_B');
ForceGround_Foot_11_L_B.set_appliesForce(true);
ForceGround_Foot_11_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_11_L')
ForceGround_Foot_11_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_11_L_B.setDissipation(0);
ForceGround_Foot_11_L_B.setStaticFriction(0);
ForceGround_Foot_11_L_B.setDynamicFriction(0);
ForceGround_Foot_11_L_B.setViscousFriction(0);
ForceGround_Foot_11_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_12_R_B = HuntCrossleyForce();
ForceGround_Foot_12_R_B.setName('ForceGround_Foot_12_R_B');
ForceGround_Foot_12_R_B.set_appliesForce(true);
ForceGround_Foot_12_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_12_R');
ForceGround_Foot_12_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_12_R_B.setDissipation(0);
ForceGround_Foot_12_R_B.setStaticFriction(0);
ForceGround_Foot_12_R_B.setDynamicFriction(0);
ForceGround_Foot_12_R_B.setViscousFriction(0);
ForceGround_Foot_12_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_12_L_B=HuntCrossleyForce();
ForceGround_Foot_12_L_B.setName('ForceGround_Foot_12_L_B');
ForceGround_Foot_12_L_B.set_appliesForce(true);
ForceGround_Foot_12_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_12_L')
ForceGround_Foot_12_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_12_L_B.setDissipation(0);
ForceGround_Foot_12_L_B.setStaticFriction(0);
ForceGround_Foot_12_L_B.setDynamicFriction(0);
ForceGround_Foot_12_L_B.setViscousFriction(0);
ForceGround_Foot_12_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_13_R_B = HuntCrossleyForce();
ForceGround_Foot_13_R_B.setName('ForceGround_Foot_13_R_B');
ForceGround_Foot_13_R_B.set_appliesForce(true);
ForceGround_Foot_13_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_13_R');
ForceGround_Foot_13_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_13_R_B.setDissipation(0);
ForceGround_Foot_13_R_B.setStaticFriction(0);
ForceGround_Foot_13_R_B.setDynamicFriction(0);
ForceGround_Foot_13_R_B.setViscousFriction(0);
ForceGround_Foot_13_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_13_L_B=HuntCrossleyForce();
ForceGround_Foot_13_L_B.setName('ForceGround_Foot_13_L_B');
ForceGround_Foot_13_L_B.set_appliesForce(true);
ForceGround_Foot_13_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_13_L')
ForceGround_Foot_13_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_13_L_B.setDissipation(0);
ForceGround_Foot_13_L_B.setStaticFriction(0);
ForceGround_Foot_13_L_B.setDynamicFriction(0);
ForceGround_Foot_13_L_B.setViscousFriction(0);
ForceGround_Foot_13_L_B.setTransitionVelocity(0.13)

ForceGround_Foot_14_R_B = HuntCrossleyForce();
ForceGround_Foot_14_R_B.setName('ForceGround_Foot_14_R_B');
ForceGround_Foot_14_R_B.set_appliesForce(true);
ForceGround_Foot_14_R_B.addGeometry('ground_Foot_R_B Sphere_Foot_14_R');
ForceGround_Foot_14_R_B.setStiffness(Sp_stiffness);
ForceGround_Foot_14_R_B.setDissipation(0);
ForceGround_Foot_14_R_B.setStaticFriction(0);
ForceGround_Foot_14_R_B.setDynamicFriction(0);
ForceGround_Foot_14_R_B.setViscousFriction(0);
ForceGround_Foot_14_R_B.setTransitionVelocity(0.13)

ForceGround_Foot_14_L_B=HuntCrossleyForce();
ForceGround_Foot_14_L_B.setName('ForceGround_Foot_14_L_B');
ForceGround_Foot_14_L_B.set_appliesForce(true);
ForceGround_Foot_14_L_B.addGeometry('ground_Foot_L_B Sphere_Foot_14_L')
ForceGround_Foot_14_L_B.setStiffness(Sp_stiffness);
ForceGround_Foot_14_L_B.setDissipation(0);
ForceGround_Foot_14_L_B.setStaticFriction(0);
ForceGround_Foot_14_L_B.setDynamicFriction(0);
ForceGround_Foot_14_L_B.setViscousFriction(0);
ForceGround_Foot_14_L_B.setTransitionVelocity(0.13)

% Automate the Force reporter creation
model.getForceSet.cloneAndAppend(ForceGround_Foot_1_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_2_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_1_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_2_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_3_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_3_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_4_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_4_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_5_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_5_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_6_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_6_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_7_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_7_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_8_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_8_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_9_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_9_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_10_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_10_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_11_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_11_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_12_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_12_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_13_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_13_L_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_14_R_B);
model.getForceSet.cloneAndAppend(ForceGround_Foot_14_L_B);
model.finalizeConnections()
model.print(modelProcessed_path);
% launching the force reporter tool for the contact B for left Foot
AnTool_L_B=AnalyzeTool();
AnTool_L_B.setModel(modelProcessed);
AnTool_L_B.setModelFilename(fullfile(ModelFolder,"ModelProcessed.osim"));
AnTool_L_B.setCoordinatesFileName(fullfile(IKFolder,IKResult));
AnTool_L_B.setLowpassCutoffFrequency(Freq);
AnTool_L_B.setSolveForEquilibrium(1);
AnTool_L_B.setStartTime(t_cont_foot_l_B(1));
AnTool_L_B.setFinalTime(t_cont_foot_l_B(end));
AnTool_L_B.setResultsDir(fullfile(path_FR,'Cont_L_B'));
FR_An_L_B=ForceReporter();
FR_An_L_B.setStartTime(t_cont_foot_l_B(1));
FR_An_L_B.setEndTime(t_cont_foot_l_B(end))
AnTool_L_B.getAnalysisSet.cloneAndAppend(FR_An_L_B);
AnTool_L_B.print(fullfile(path_Solution,'Setup_ForceReporter.xml'));
ForceTool=AnalyzeTool(fullfile(path_Solution,'Setup_ForceReporter.xml'));
ForceTool.run;
[ForceReport_L_B,HeadFR]=load_sto(fullfile(path_FR,'Cont_L_B','_ForceReporter_forces.sto'));
Time_Cont_Foot_L_B=ForceReport_L_B(:,strcmp('time',HeadFR));
temp_Cont_Foot_1_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_1_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_2_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_2_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_3_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_3_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_4_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_4_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_5_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_5_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_6_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_6_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_7_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_7_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_8_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_8_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_9_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_9_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_10_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_10_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_11_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_11_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_12_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_12_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_13_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_13_L_B.ground.force.Y',HeadFR));
temp_Cont_Foot_14_L_B=ForceReport_L_B(:,strcmp('ForceGround_Foot_14_L_B.ground.force.Y',HeadFR));

% launching the force reporter tool for the contact B for right Foot
AnTool_R_B=AnalyzeTool();
AnTool_R_B.setModel(modelProcessed);
AnTool_R_B.setModelFilename(fullfile(ModelFolder,"ModelProcessed.osim"));
AnTool_R_B.setCoordinatesFileName(fullfile(IKFolder,IKResult));
AnTool_R_B.setLowpassCutoffFrequency(Freq);
AnTool_R_B.setSolveForEquilibrium(1);
AnTool_R_B.setStartTime(t_cont_foot_r_B(1));
AnTool_R_B.setFinalTime(t_cont_foot_r_B(end));
AnTool_R_B.setResultsDir(fullfile(path_FR,'Cont_R_B'));
FR_An_R_B=ForceReporter();
FR_An_R_B.setStartTime(t_cont_foot_r_B(1));
FR_An_R_B.setEndTime(t_cont_foot_r_B(end))
AnTool_R_B.getAnalysisSet.cloneAndAppend(FR_An_R_B);
AnTool_R_B.print(fullfile(path_Solution,'Setup_ForceReporter.xml'));
ForceTool=AnalyzeTool(fullfile(path_Solution,'Setup_ForceReporter.xml'));
ForceTool.run;
[ForceReport_R_B,HeadFR]=load_sto(fullfile(path_FR,'Cont_R_B','_ForceReporter_forces.sto'));
Time_Cont_Foot_R_B=ForceReport_R_B(:,strcmp('time',HeadFR));
temp_Cont_Foot_1_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_1_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_2_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_2_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_3_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_3_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_4_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_4_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_5_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_5_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_6_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_6_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_7_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_7_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_8_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_8_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_9_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_9_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_10_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_10_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_11_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_11_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_12_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_12_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_13_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_13_R_B.ground.force.Y',HeadFR));
temp_Cont_Foot_14_R_B=ForceReport_R_B(:,strcmp('ForceGround_Foot_14_R_B.ground.force.Y',HeadFR));

% creating a unique contact vector: inizialize the B contact force vector 

Cont_Foot_1_L_B=zeros(length(timeBK),1);
Cont_Foot_2_L_B=zeros(length(timeBK),1);
Cont_Foot_3_L_B=zeros(length(timeBK),1);
Cont_Foot_4_L_B=zeros(length(timeBK),1);
Cont_Foot_5_L_B=zeros(length(timeBK),1);
Cont_Foot_6_L_B=zeros(length(timeBK),1);
Cont_Foot_7_L_B=zeros(length(timeBK),1);
Cont_Foot_8_L_B=zeros(length(timeBK),1);
Cont_Foot_9_L_B=zeros(length(timeBK),1);
Cont_Foot_10_L_B=zeros(length(timeBK),1);
Cont_Foot_11_L_B=zeros(length(timeBK),1);
Cont_Foot_12_L_B=zeros(length(timeBK),1);
Cont_Foot_13_L_B=zeros(length(timeBK),1);
Cont_Foot_14_L_B=zeros(length(timeBK),1);
Cont_Foot_1_R_B=zeros(length(timeBK),1);
Cont_Foot_2_R_B=zeros(length(timeBK),1);
Cont_Foot_3_R_B=zeros(length(timeBK),1);
Cont_Foot_4_R_B=zeros(length(timeBK),1);
Cont_Foot_5_R_B=zeros(length(timeBK),1);
Cont_Foot_6_R_B=zeros(length(timeBK),1);
Cont_Foot_7_R_B=zeros(length(timeBK),1);
Cont_Foot_8_R_B=zeros(length(timeBK),1);
Cont_Foot_9_R_B=zeros(length(timeBK),1);
Cont_Foot_10_R_B=zeros(length(timeBK),1);
Cont_Foot_11_R_B=zeros(length(timeBK),1);
Cont_Foot_12_R_B=zeros(length(timeBK),1);
Cont_Foot_13_R_B=zeros(length(timeBK),1);
Cont_Foot_14_R_B=zeros(length(timeBK),1);
% filling the B contact forces with the contact B forces at the spcified
% time with the temp Contact forces
Cont_Foot_1_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_1_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B)); % I also have to make sure that the time vector of FR results is consistent with the instants detected before: it can happen that the time of FR has more samples than detected interval
Cont_Foot_2_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_2_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_3_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_3_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_4_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_4_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_5_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_5_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_6_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_6_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_7_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_7_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_8_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_8_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_9_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_9_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_10_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_10_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_11_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_11_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_12_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_12_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_13_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_13_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));
Cont_Foot_14_L_B(ind_t_cont_foot_l_B)=temp_Cont_Foot_14_L_B(ismember(Time_Cont_Foot_L_B,t_cont_foot_l_B));

Cont_Foot_1_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_1_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_2_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_2_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_3_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_3_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_4_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_4_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_5_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_5_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_6_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_6_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_7_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_7_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_8_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_8_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_9_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_9_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_10_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_10_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_11_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_11_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_12_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_12_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_13_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_13_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));
Cont_Foot_14_R_B(ind_t_cont_foot_r_B)=temp_Cont_Foot_14_R_B(ismember(Time_Cont_Foot_R_B,t_cont_foot_r_B));

if Ground_L_Foot_T_A(2)==Ground_L_Foot_T_B(2) % If both plane A and B are on the same level I choose the level B contact forces cause have the controls on position and velocities
Cont_Foot_1_L_A=Cont_Foot_1_L_B;
Cont_Foot_2_L_A=Cont_Foot_2_L_B;
Cont_Foot_3_L_A=Cont_Foot_3_L_B;
Cont_Foot_4_L_A=Cont_Foot_4_L_B;
Cont_Foot_5_L_A=Cont_Foot_5_L_B;
Cont_Foot_6_L_A=Cont_Foot_6_L_B;
Cont_Foot_7_L_A=Cont_Foot_7_L_B;
Cont_Foot_8_L_A=Cont_Foot_8_L_B;
Cont_Foot_9_L_A=Cont_Foot_9_L_B;
Cont_Foot_10_L_A=Cont_Foot_10_L_B;
Cont_Foot_11_L_A=Cont_Foot_11_L_B;
Cont_Foot_12_L_A=Cont_Foot_12_L_B;
Cont_Foot_13_L_A=Cont_Foot_13_L_B;
Cont_Foot_14_L_A=Cont_Foot_14_L_B;
end
if Ground_R_Foot_T_A(2)==Ground_R_Foot_T_B(2) % If both plane A and B are on the same level I choose the level B contact forces cause have the controls on position and velocities
Cont_Foot_1_R_A=Cont_Foot_1_R_B;
Cont_Foot_2_R_A=Cont_Foot_2_R_B;
Cont_Foot_3_R_A=Cont_Foot_3_R_B;
Cont_Foot_4_R_A=Cont_Foot_4_R_B;
Cont_Foot_5_R_A=Cont_Foot_5_R_B;
Cont_Foot_6_R_A=Cont_Foot_6_R_B;
Cont_Foot_7_R_A=Cont_Foot_7_R_B;
Cont_Foot_8_R_A=Cont_Foot_8_R_B;
Cont_Foot_9_R_A=Cont_Foot_9_R_B;
Cont_Foot_10_R_A=Cont_Foot_10_R_B;
Cont_Foot_11_R_A=Cont_Foot_11_R_B;
Cont_Foot_12_R_A=Cont_Foot_12_R_B;
Cont_Foot_13_R_A=Cont_Foot_13_R_B;
Cont_Foot_14_R_A=Cont_Foot_14_R_B;
end

Cont_Foot_L_B=Cont_Foot_1_L_B+Cont_Foot_2_L_B+Cont_Foot_3_L_B+Cont_Foot_4_L_B+Cont_Foot_5_L_B+Cont_Foot_6_L_B+Cont_Foot_7_L_B+Cont_Foot_8_L_B+Cont_Foot_9_L_B+Cont_Foot_10_L_B+Cont_Foot_11_L_B+Cont_Foot_12_L_B+Cont_Foot_13_L_B+Cont_Foot_14_L_B;
Cont_Foot_R_B=Cont_Foot_1_R_B+Cont_Foot_2_R_B+Cont_Foot_3_R_B+Cont_Foot_4_R_B+Cont_Foot_5_R_B+Cont_Foot_6_R_B+Cont_Foot_7_R_B+Cont_Foot_8_R_B+Cont_Foot_9_R_B+Cont_Foot_10_R_B+Cont_Foot_11_R_B+Cont_Foot_12_R_B+Cont_Foot_13_R_B+Cont_Foot_14_R_B;
% putting together the A and B contact forces per each sphere
Cont_Foot_1_R=Cont_Foot_1_R_A+Cont_Foot_1_R_B;
Cont_Foot_2_R=Cont_Foot_2_R_A+Cont_Foot_2_R_B;
Cont_Foot_3_R=Cont_Foot_3_R_A+Cont_Foot_3_R_B;
Cont_Foot_4_R=Cont_Foot_4_R_A+Cont_Foot_4_R_B;
Cont_Foot_5_R=Cont_Foot_5_R_A+Cont_Foot_5_R_B;
Cont_Foot_6_R=Cont_Foot_6_R_A+Cont_Foot_6_R_B;
Cont_Foot_7_R=Cont_Foot_7_R_A+Cont_Foot_7_R_B;
Cont_Foot_8_R=Cont_Foot_8_R_A+Cont_Foot_8_R_B;
Cont_Foot_9_R=Cont_Foot_9_R_A+Cont_Foot_9_R_B;
Cont_Foot_10_R=Cont_Foot_10_R_A+Cont_Foot_10_R_B;
Cont_Foot_11_R=Cont_Foot_11_R_A+Cont_Foot_11_R_B;
Cont_Foot_12_R=Cont_Foot_12_R_A+Cont_Foot_12_R_B;
Cont_Foot_13_R=Cont_Foot_13_R_A+Cont_Foot_13_R_B;
Cont_Foot_14_R=Cont_Foot_14_R_A+Cont_Foot_14_R_B;

Cont_Foot_1_L=Cont_Foot_1_L_A+Cont_Foot_1_L_B;
Cont_Foot_2_L=Cont_Foot_2_L_A+Cont_Foot_2_L_B;
Cont_Foot_3_L=Cont_Foot_3_L_A+Cont_Foot_3_L_B;
Cont_Foot_4_L=Cont_Foot_4_L_A+Cont_Foot_4_L_B;
Cont_Foot_5_L=Cont_Foot_5_L_A+Cont_Foot_5_L_B;
Cont_Foot_6_L=Cont_Foot_6_L_A+Cont_Foot_6_L_B;
Cont_Foot_7_L=Cont_Foot_7_L_A+Cont_Foot_7_L_B;
Cont_Foot_8_L=Cont_Foot_8_L_A+Cont_Foot_8_L_B;
Cont_Foot_9_L=Cont_Foot_9_L_A+Cont_Foot_9_L_B;
Cont_Foot_10_L=Cont_Foot_10_L_A+Cont_Foot_10_L_B;
Cont_Foot_11_L=Cont_Foot_11_L_A+Cont_Foot_11_L_B;
Cont_Foot_12_L=Cont_Foot_12_L_A+Cont_Foot_12_L_B;
Cont_Foot_13_L=Cont_Foot_13_L_A+Cont_Foot_13_L_B;
Cont_Foot_14_L=Cont_Foot_14_L_A+Cont_Foot_14_L_B;

% implementing the weights of the contacts forces
w_sp_L=[min(F_min_L(1),F_min_L(2)),min(F_min_L(3),F_min_L(4)),min(F_min_L(5),F_min_L(6)),min(F_min_L(7),F_min_L(8)),min(F_min_L(9),F_min_L(10)),min(F_min_L(11),F_min_L(12)),min(F_min_L(13),F_min_L(14)),]./max(F_min_L);
w_sp_R=[min(F_min_R(1),F_min_R(2)),min(F_min_R(3),F_min_R(4)),min(F_min_R(5),F_min_R(6)),min(F_min_R(7),F_min_R(8)),min(F_min_R(9),F_min_R(10)),min(F_min_R(11),F_min_R(12)),min(F_min_R(13),F_min_R(14)),]./max(F_min_R);
Amp=[3 12 4 6 1 1 6]; %normal plantar foot weights

Cont_Foot_1_R=Cont_Foot_1_R*Amp(1)/w_sp_R(1);
Cont_Foot_2_R=Cont_Foot_2_R*Amp(1)/w_sp_R(1);
Cont_Foot_3_R=Cont_Foot_3_R*Amp(2)/w_sp_R(2);
Cont_Foot_4_R=Cont_Foot_4_R*Amp(2)/w_sp_R(2);
Cont_Foot_5_R=Cont_Foot_5_R*Amp(3)/w_sp_R(3);
Cont_Foot_6_R=Cont_Foot_6_R*Amp(3)/w_sp_R(3);
Cont_Foot_7_R=Cont_Foot_7_R*Amp(4)/w_sp_R(4);
Cont_Foot_8_R=Cont_Foot_8_R*Amp(4)/w_sp_R(4);
Cont_Foot_9_R=Cont_Foot_9_R*Amp(5)/w_sp_R(5);
Cont_Foot_10_R=Cont_Foot_10_R*Amp(5)/w_sp_R(5);
Cont_Foot_11_R=Cont_Foot_11_R*Amp(6)/w_sp_R(6);
Cont_Foot_12_R=Cont_Foot_12_R*Amp(6)/w_sp_R(6);
Cont_Foot_13_R=Cont_Foot_13_R*Amp(7)/w_sp_R(7);
Cont_Foot_14_R=Cont_Foot_14_R*Amp(7)/w_sp_R(7);

Cont_Foot_1_L=Cont_Foot_1_L*Amp(1)/w_sp_L(1);
Cont_Foot_2_L=Cont_Foot_2_L*Amp(1)/w_sp_L(1);
Cont_Foot_3_L=Cont_Foot_3_L*Amp(2)/w_sp_L(2);
Cont_Foot_4_L=Cont_Foot_4_L*Amp(2)/w_sp_L(2);
Cont_Foot_5_L=Cont_Foot_5_L*Amp(3)/w_sp_L(3);
Cont_Foot_6_L=Cont_Foot_6_L*Amp(3)/w_sp_L(3);
Cont_Foot_7_L=Cont_Foot_7_L*Amp(4)/w_sp_L(4);
Cont_Foot_8_L=Cont_Foot_8_L*Amp(4)/w_sp_L(4);
Cont_Foot_9_L=Cont_Foot_9_L*Amp(5)/w_sp_L(5);
Cont_Foot_10_L=Cont_Foot_10_L*Amp(5)/w_sp_L(5);
Cont_Foot_11_L=Cont_Foot_11_L*Amp(6)/w_sp_L(6);
Cont_Foot_12_L=Cont_Foot_12_L*Amp(6)/w_sp_L(6);
Cont_Foot_13_L=Cont_Foot_13_L*Amp(7)/w_sp_L(7);
Cont_Foot_14_L=Cont_Foot_14_L*Amp(7)/w_sp_L(7);


% the final contact forces
Cont_Foot_R=Cont_Foot_1_R+Cont_Foot_2_R+Cont_Foot_3_R+Cont_Foot_4_R+Cont_Foot_5_R+Cont_Foot_6_R+Cont_Foot_7_R+Cont_Foot_8_R+Cont_Foot_9_R+Cont_Foot_10_R+Cont_Foot_11_R+Cont_Foot_12_R+Cont_Foot_13_R+Cont_Foot_14_R;
Cont_Foot_L=Cont_Foot_1_L+Cont_Foot_2_L+Cont_Foot_3_L+Cont_Foot_4_L+Cont_Foot_5_L+Cont_Foot_6_L+Cont_Foot_7_L+Cont_Foot_8_L+Cont_Foot_9_L+Cont_Foot_10_L+Cont_Foot_11_L+Cont_Foot_12_L+Cont_Foot_13_L+Cont_Foot_14_L;


%% retrieving Spheres location to calculate CoP
AnTool_PK=AnalyzeTool();
AnTool_PK.setModel(modelProcessed);
AnTool_PK.setModelFilename(fullfile(ModelFolder,"ModelProcessed.osim"));
AnTool_PK.setCoordinatesFileName(fullfile(IKFolder,IKResult));
AnTool_PK.setLowpassCutoffFrequency(Freq);
AnTool_PK.setStartTime(timeStart);
AnTool_PK.setFinalTime(timeEnd);
AnTool_PK.setResultsDir(path_PK);
PK_sp1_r=PointKinematics();
PK_sp1_r.setPointName('PK_sp1_r');
PK_sp1_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp1_r.setPoint(Vec3.createFromMat(PosSp1R-[heel_shift 0 0]))
PK_sp1_r.setRelativeToBody(model.getGround)
PK_sp2_r=PointKinematics();
PK_sp2_r.setPointName('PK_sp2_r');
PK_sp2_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp2_r.setPoint(Vec3.createFromMat(PosSp2R-[heel_shift 0 0]))
PK_sp2_r.setRelativeToBody(model.getGround)
PK_sp3_r=PointKinematics();
PK_sp3_r.setPointName('PK_sp3_r');
PK_sp3_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp3_r.setPoint(Vec3.createFromMat(PosSp3R-[heel_shift 0 0]))
PK_sp3_r.setRelativeToBody(model.getGround)

PK_sp4_r=PointKinematics();
PK_sp4_r.setPointName('PK_sp4_r');
PK_sp4_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp4_r.setPoint(Vec3.createFromMat(PosSp4R-[heel_shift 0 0]))
PK_sp4_r.setRelativeToBody(model.getGround)

PK_sp5_r=PointKinematics();
PK_sp5_r.setPointName('PK_sp5_r');
PK_sp5_r.setBody(model.getBodySet.get(toes_r_name))
PK_sp5_r.setPoint(Vec3.createFromMat(PosSp5R))
PK_sp5_r.setRelativeToBody(model.getGround)

PK_sp6_r=PointKinematics();
PK_sp6_r.setPointName('PK_sp6_r');
PK_sp6_r.setBody(model.getBodySet.get(toes_r_name))
PK_sp6_r.setPoint(Vec3.createFromMat(PosSp6R))
PK_sp6_r.setRelativeToBody(model.getGround)

PK_sp7_r=PointKinematics();
PK_sp7_r.setPointName('PK_sp7_r');
PK_sp7_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp7_r.setPoint(Vec3.createFromMat(PosSp7R))
PK_sp7_r.setRelativeToBody(model.getGround)
PK_sp8_r=PointKinematics();
PK_sp8_r.setPointName('PK_sp8_r');
PK_sp8_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp8_r.setPoint(Vec3.createFromMat(PosSp8R))
PK_sp8_r.setRelativeToBody(model.getGround)

PK_sp9_r=PointKinematics();
PK_sp9_r.setPointName('PK_sp9_r');
PK_sp9_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp9_r.setPoint(Vec3.createFromMat(PosSp9R))
PK_sp9_r.setRelativeToBody(model.getGround)

PK_sp10_r=PointKinematics();
PK_sp10_r.setPointName('PK_sp10_r');
PK_sp10_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp10_r.setPoint(Vec3.createFromMat(PosSp10R))
PK_sp10_r.setRelativeToBody(model.getGround)

PK_sp11_r=PointKinematics();
PK_sp11_r.setPointName('PK_sp11_r');
PK_sp11_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp11_r.setPoint(Vec3.createFromMat(PosSp11R))
PK_sp11_r.setRelativeToBody(model.getGround)

PK_sp12_r=PointKinematics();
PK_sp12_r.setPointName('PK_sp12_r');
PK_sp12_r.setBody(model.getBodySet.get(calcn_r_name))
PK_sp12_r.setPoint(Vec3.createFromMat(PosSp12R))
PK_sp12_r.setRelativeToBody(model.getGround)

PK_sp13_r=PointKinematics();
PK_sp13_r.setPointName('PK_sp13_r');
PK_sp13_r.setBody(model.getBodySet.get(toes_r_name))
PK_sp13_r.setPoint(Vec3.createFromMat(PosSp13R))
PK_sp13_r.setRelativeToBody(model.getGround)

PK_sp14_r=PointKinematics();
PK_sp14_r.setPointName('PK_sp14_r');
PK_sp14_r.setBody(model.getBodySet.get(toes_r_name))
PK_sp14_r.setPoint(Vec3.createFromMat(PosSp14R))
PK_sp14_r.setRelativeToBody(model.getGround)

PK_sp1_l=PointKinematics();
PK_sp1_l.setPointName('PK_sp1_l');
PK_sp1_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp1_l.setPoint(Vec3.createFromMat(PosSp1L-[heel_shift 0 0]))
PK_sp1_l.setRelativeToBody(model.getGround)

PK_sp2_l=PointKinematics();
PK_sp2_l.setPointName('PK_sp2_l');
PK_sp2_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp2_l.setPoint(Vec3.createFromMat(PosSp2L-[heel_shift 0 0]))
PK_sp2_l.setRelativeToBody(model.getGround)

PK_sp3_l=PointKinematics();
PK_sp3_l.setPointName('PK_sp3_l');
PK_sp3_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp3_l.setPoint(Vec3.createFromMat(PosSp3L-[heel_shift 0 0]))
PK_sp3_l.setRelativeToBody(model.getGround)

PK_sp4_l=PointKinematics();
PK_sp4_l.setPointName('PK_sp4_l');
PK_sp4_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp4_l.setPoint(Vec3.createFromMat(PosSp4L-[heel_shift 0 0]))
PK_sp4_l.setRelativeToBody(model.getGround)

PK_sp5_l=PointKinematics();
PK_sp5_l.setPointName('PK_sp5_l');
PK_sp5_l.setBody(model.getBodySet.get(toes_l_name))
PK_sp5_l.setPoint(Vec3.createFromMat(PosSp5L))
PK_sp5_l.setRelativeToBody(model.getGround)

PK_sp6_l=PointKinematics();
PK_sp6_l.setPointName('PK_sp6_l');
PK_sp6_l.setBody(model.getBodySet.get(toes_l_name))
PK_sp6_l.setPoint(Vec3.createFromMat(PosSp6L))
PK_sp6_l.setRelativeToBody(model.getGround)

PK_sp7_l=PointKinematics();
PK_sp7_l.setPointName('PK_sp7_l');
PK_sp7_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp7_l.setPoint(Vec3.createFromMat(PosSp7L))
PK_sp7_l.setRelativeToBody(model.getGround)

PK_sp8_l=PointKinematics();
PK_sp8_l.setPointName('PK_sp8_l');
PK_sp8_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp8_l.setPoint(Vec3.createFromMat(PosSp8L))
PK_sp8_l.setRelativeToBody(model.getGround)

PK_sp9_l=PointKinematics();
PK_sp9_l.setPointName('PK_sp9_l');
PK_sp9_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp9_l.setPoint(Vec3.createFromMat(PosSp9L))
PK_sp9_l.setRelativeToBody(model.getGround)

PK_sp10_l=PointKinematics();
PK_sp10_l.setPointName('PK_sp10_l');
PK_sp10_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp10_l.setPoint(Vec3.createFromMat(PosSp10L))
PK_sp10_l.setRelativeToBody(model.getGround)

PK_sp11_l=PointKinematics();
PK_sp11_l.setPointName('PK_sp11_l');
PK_sp11_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp11_l.setPoint(Vec3.createFromMat(PosSp11L))
PK_sp11_l.setRelativeToBody(model.getGround)

PK_sp12_l=PointKinematics();
PK_sp12_l.setPointName('PK_sp12_l');
PK_sp12_l.setBody(model.getBodySet.get(calcn_l_name))
PK_sp12_l.setPoint(Vec3.createFromMat(PosSp12L))
PK_sp12_l.setRelativeToBody(model.getGround)

PK_sp13_l=PointKinematics();
PK_sp13_l.setPointName('PK_sp13_l');
PK_sp13_l.setBody(model.getBodySet.get(toes_l_name))
PK_sp13_l.setPoint(Vec3.createFromMat(PosSp13L))
PK_sp13_l.setRelativeToBody(model.getGround)

PK_sp14_l=PointKinematics();
PK_sp14_l.setPointName('PK_sp14_l');
PK_sp14_l.setBody(model.getBodySet.get(toes_l_name))
PK_sp14_l.setPoint(Vec3.createFromMat(PosSp14L))
PK_sp14_l.setRelativeToBody(model.getGround)

AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp1_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp2_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp3_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp4_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp5_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp6_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp7_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp8_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp9_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp10_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp11_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp12_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp13_r);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp14_r);

AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp1_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp2_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp3_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp4_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp5_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp6_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp7_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp8_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp9_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp10_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp11_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp12_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp13_l);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp14_l);

AnTool_PK.print(fullfile(path_Solution,'SetupPK_sp.xml'));
AnalyzeTool(fullfile(path_Solution,'SetupPK_sp.xml')).run;
PK_res_sp_1_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp1_r_pos.sto'));
PK_res_sp_2_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp2_r_pos.sto'));
PK_res_sp_3_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp3_r_pos.sto'));
PK_res_sp_4_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp4_r_pos.sto'));
PK_res_sp_5_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp5_r_pos.sto'));
PK_res_sp_6_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp6_r_pos.sto'));
PK_res_sp_7_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp7_r_pos.sto'));
PK_res_sp_8_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp8_r_pos.sto'));
PK_res_sp_9_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp9_r_pos.sto'));
PK_res_sp_10_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp10_r_pos.sto'));
PK_res_sp_11_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp11_r_pos.sto'));
PK_res_sp_12_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp12_r_pos.sto'));
PK_res_sp_13_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp13_r_pos.sto'));
PK_res_sp_14_r=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp14_r_pos.sto'));

PK_res_sp_1_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp1_l_pos.sto'));
PK_res_sp_2_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp2_l_pos.sto'));
PK_res_sp_3_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp3_l_pos.sto'));
PK_res_sp_4_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp4_l_pos.sto'));
PK_res_sp_5_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp5_l_pos.sto'));
PK_res_sp_6_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp6_l_pos.sto'));
PK_res_sp_7_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp7_l_pos.sto'));
PK_res_sp_8_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp8_l_pos.sto'));
PK_res_sp_9_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp9_l_pos.sto'));
PK_res_sp_10_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp10_l_pos.sto'));
PK_res_sp_11_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp11_l_pos.sto'));
PK_res_sp_12_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp12_l_pos.sto'));
PK_res_sp_13_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp13_l_pos.sto'));
PK_res_sp_14_l=load_sto(fullfile(path_PK,'_PointKinematics_PK_sp14_l_pos.sto'));

X_sp_1_r=PK_res_sp_1_r(:,2);
Z_sp_1_r=PK_res_sp_1_r(:,4);
X_sp_2_r=PK_res_sp_2_r(:,2);
Z_sp_2_r=PK_res_sp_2_r(:,4);
X_sp_3_r=PK_res_sp_3_r(:,2);
Z_sp_3_r=PK_res_sp_3_r(:,4);
X_sp_4_r=PK_res_sp_4_r(:,2);
Z_sp_4_r=PK_res_sp_4_r(:,4);
X_sp_5_r=PK_res_sp_5_r(:,2);
Z_sp_5_r=PK_res_sp_5_r(:,4);
X_sp_6_r=PK_res_sp_6_r(:,2);
Z_sp_6_r=PK_res_sp_6_r(:,4);
X_sp_7_r=PK_res_sp_7_r(:,2);
Z_sp_7_r=PK_res_sp_7_r(:,4);
X_sp_8_r=PK_res_sp_8_r(:,2);
Z_sp_8_r=PK_res_sp_8_r(:,4);
X_sp_9_r=PK_res_sp_9_r(:,2);
Z_sp_9_r=PK_res_sp_9_r(:,4);
X_sp_10_r=PK_res_sp_10_r(:,2);
Z_sp_10_r=PK_res_sp_10_r(:,4);
X_sp_11_r=PK_res_sp_11_r(:,2);
Z_sp_11_r=PK_res_sp_11_r(:,4);
X_sp_12_r=PK_res_sp_12_r(:,2);
Z_sp_12_r=PK_res_sp_12_r(:,4);
X_sp_13_r=PK_res_sp_13_r(:,2);
Z_sp_13_r=PK_res_sp_13_r(:,4);
X_sp_14_r=PK_res_sp_14_r(:,2);
Z_sp_14_r=PK_res_sp_14_r(:,4);

X_sp_1_l=PK_res_sp_1_l(:,2);
Z_sp_1_l=PK_res_sp_1_l(:,4);
X_sp_2_l=PK_res_sp_2_l(:,2);
Z_sp_2_l=PK_res_sp_2_l(:,4);
X_sp_3_l=PK_res_sp_3_l(:,2);
Z_sp_3_l=PK_res_sp_3_l(:,4);
X_sp_4_l=PK_res_sp_4_l(:,2);
Z_sp_4_l=PK_res_sp_4_l(:,4);
X_sp_5_l=PK_res_sp_5_l(:,2);
Z_sp_5_l=PK_res_sp_5_l(:,4);
X_sp_6_l=PK_res_sp_6_l(:,2);
Z_sp_6_l=PK_res_sp_6_l(:,4);
X_sp_7_l=PK_res_sp_7_l(:,2);
Z_sp_7_l=PK_res_sp_7_l(:,4);
X_sp_8_l=PK_res_sp_8_l(:,2);
Z_sp_8_l=PK_res_sp_8_l(:,4);
X_sp_9_l=PK_res_sp_9_l(:,2);
Z_sp_9_l=PK_res_sp_9_l(:,4);
X_sp_10_l=PK_res_sp_10_l(:,2);
Z_sp_10_l=PK_res_sp_10_l(:,4);
X_sp_11_l=PK_res_sp_11_l(:,2);
Z_sp_11_l=PK_res_sp_11_l(:,4);
X_sp_12_l=PK_res_sp_12_l(:,2);
Z_sp_12_l=PK_res_sp_12_l(:,4);
X_sp_13_l=PK_res_sp_13_l(:,2);
Z_sp_13_l=PK_res_sp_13_l(:,4);
X_sp_14_l=PK_res_sp_14_l(:,2);
Z_sp_14_l=PK_res_sp_14_l(:,4);

Y_sp_1_l=PK_res_sp_1_l(:,3);
Y_sp_2_l=PK_res_sp_2_l(:,3);
Y_sp_3_l=PK_res_sp_3_l(:,3);
Y_sp_4_l=PK_res_sp_4_l(:,3);
Y_sp_5_l=PK_res_sp_5_l(:,3);
Y_sp_6_l=PK_res_sp_6_l(:,3);
Y_sp_7_l=PK_res_sp_7_l(:,3);
Y_sp_8_l=PK_res_sp_8_l(:,3);
Y_sp_9_l=PK_res_sp_9_l(:,3);
Y_sp_10_l=PK_res_sp_10_l(:,3);
Y_sp_11_l=PK_res_sp_11_l(:,3);
Y_sp_12_l=PK_res_sp_12_l(:,3);
Y_sp_13_l=PK_res_sp_13_l(:,3);
Y_sp_14_l=PK_res_sp_14_l(:,3);

Y_sp_1_r=PK_res_sp_1_r(:,3);
Y_sp_2_r=PK_res_sp_2_r(:,3);
Y_sp_3_r=PK_res_sp_3_r(:,3);
Y_sp_4_r=PK_res_sp_4_r(:,3);
Y_sp_5_r=PK_res_sp_5_r(:,3);
Y_sp_6_r=PK_res_sp_6_r(:,3);
Y_sp_7_r=PK_res_sp_7_r(:,3);
Y_sp_8_r=PK_res_sp_8_r(:,3);
Y_sp_9_r=PK_res_sp_9_r(:,3);
Y_sp_10_r=PK_res_sp_10_r(:,3);
Y_sp_11_r=PK_res_sp_11_r(:,3);
Y_sp_12_r=PK_res_sp_12_r(:,3);
Y_sp_13_r=PK_res_sp_13_r(:,3);
Y_sp_14_r=PK_res_sp_14_r(:,3);

COP_Calcn_x_r_A=(X_sp_1_r.*Cont_Foot_1_R_A+X_sp_2_r.*Cont_Foot_2_R_A+X_sp_3_r.*Cont_Foot_3_R_A+X_sp_4_r.*Cont_Foot_4_R_A+X_sp_5_r.*Cont_Foot_5_R_A+X_sp_6_r.*Cont_Foot_6_R_A+X_sp_7_r.*Cont_Foot_7_R_A+X_sp_8_r.*Cont_Foot_8_R_A+X_sp_9_r.*Cont_Foot_9_R_A+X_sp_10_r.*Cont_Foot_10_R_A+X_sp_11_r.*Cont_Foot_11_R_A+X_sp_12_r.*Cont_Foot_12_R_A+X_sp_13_r.*Cont_Foot_13_R_A+X_sp_14_r.*Cont_Foot_14_R_A)./Cont_Foot_R_A;
COP_Calcn_x_l_A=(X_sp_1_l.*Cont_Foot_1_L_A+X_sp_2_l.*Cont_Foot_2_L_A+X_sp_3_l.*Cont_Foot_3_L_A+X_sp_4_l.*Cont_Foot_4_L_A+X_sp_5_l.*Cont_Foot_5_L_A+X_sp_6_l.*Cont_Foot_6_L_A+X_sp_7_l.*Cont_Foot_7_L_A+X_sp_8_l.*Cont_Foot_8_L_A+X_sp_9_l.*Cont_Foot_9_L_A+X_sp_10_l.*Cont_Foot_10_L_A+X_sp_11_l.*Cont_Foot_11_L_A+X_sp_12_l.*Cont_Foot_12_L_A+X_sp_13_l.*Cont_Foot_13_L_A+X_sp_14_l.*Cont_Foot_14_L_A)./Cont_Foot_L_A;
COP_Calcn_y_r_A=(Y_sp_1_r.*Cont_Foot_1_R_A+Y_sp_2_r.*Cont_Foot_2_R_A+Y_sp_3_r.*Cont_Foot_3_R_A+Y_sp_4_r.*Cont_Foot_4_R_A+Y_sp_5_r.*Cont_Foot_5_R_A+Y_sp_6_r.*Cont_Foot_6_R_A+Y_sp_7_r.*Cont_Foot_7_R_A+Y_sp_8_r.*Cont_Foot_8_R_A+Y_sp_9_r.*Cont_Foot_9_R_A+Y_sp_10_r.*Cont_Foot_10_R_A+Y_sp_11_r.*Cont_Foot_11_R_A+Y_sp_12_r.*Cont_Foot_12_R_A+Y_sp_13_r.*Cont_Foot_13_R_A+Y_sp_14_r.*Cont_Foot_14_R_A)./Cont_Foot_R_A;
COP_Calcn_y_l_A=(Y_sp_1_l.*Cont_Foot_1_L_A+Y_sp_2_l.*Cont_Foot_2_L_A+Y_sp_3_l.*Cont_Foot_3_L_A+Y_sp_4_l.*Cont_Foot_4_L_A+Y_sp_5_l.*Cont_Foot_5_L_A+Y_sp_6_l.*Cont_Foot_6_L_A+Y_sp_7_l.*Cont_Foot_7_L_A+Y_sp_8_l.*Cont_Foot_8_L_A+Y_sp_9_l.*Cont_Foot_9_L_A+Y_sp_10_l.*Cont_Foot_10_L_A+Y_sp_11_l.*Cont_Foot_11_L_A+Y_sp_12_l.*Cont_Foot_12_L_A+Y_sp_13_l.*Cont_Foot_13_L_A+Y_sp_14_l.*Cont_Foot_14_L_A)./Cont_Foot_L_A;
COP_Calcn_z_r_A=(Z_sp_1_r.*Cont_Foot_1_R_A+Z_sp_2_r.*Cont_Foot_2_R_A+Z_sp_3_r.*Cont_Foot_3_R_A+Z_sp_4_r.*Cont_Foot_4_R_A+Z_sp_5_r.*Cont_Foot_5_R_A+Z_sp_6_r.*Cont_Foot_6_R_A+Z_sp_7_r.*Cont_Foot_7_R_A+Z_sp_8_r.*Cont_Foot_8_R_A+Z_sp_9_r.*Cont_Foot_9_R_A+Z_sp_10_r.*Cont_Foot_10_R_A+Z_sp_11_r.*Cont_Foot_11_R_A+Z_sp_12_r.*Cont_Foot_12_R_A+Z_sp_13_r.*Cont_Foot_13_R_A+Z_sp_14_r.*Cont_Foot_14_R_A)./Cont_Foot_R_A;
COP_Calcn_z_l_A=(Z_sp_1_l.*Cont_Foot_1_L_A+Z_sp_2_l.*Cont_Foot_2_L_A+Z_sp_3_l.*Cont_Foot_3_L_A+Z_sp_4_l.*Cont_Foot_4_L_A+Z_sp_5_l.*Cont_Foot_5_L_A+Z_sp_6_l.*Cont_Foot_6_L_A+Z_sp_7_l.*Cont_Foot_7_L_A+Z_sp_8_l.*Cont_Foot_8_L_A+Z_sp_9_l.*Cont_Foot_9_L_A+Z_sp_10_l.*Cont_Foot_10_L_A+Z_sp_11_l.*Cont_Foot_11_L_A+Z_sp_12_l.*Cont_Foot_12_L_A+Z_sp_13_l.*Cont_Foot_13_L_A+Z_sp_14_l.*Cont_Foot_14_L_A)./Cont_Foot_L_A;

COP_Calcn_x_r_B=(X_sp_1_r.*Cont_Foot_1_R_B+X_sp_2_r.*Cont_Foot_2_R_B+X_sp_3_r.*Cont_Foot_3_R_B+X_sp_4_r.*Cont_Foot_4_R_B+X_sp_5_r.*Cont_Foot_5_R_B+X_sp_6_r.*Cont_Foot_6_R_B+X_sp_7_r.*Cont_Foot_7_R_B+X_sp_8_r.*Cont_Foot_8_R_B+X_sp_9_r.*Cont_Foot_9_R_B+X_sp_10_r.*Cont_Foot_10_R_B+X_sp_11_r.*Cont_Foot_11_R_B+X_sp_12_r.*Cont_Foot_12_R_B+X_sp_13_r.*Cont_Foot_13_R_B+X_sp_14_r.*Cont_Foot_14_R_B)./Cont_Foot_R_B;
COP_Calcn_x_l_B=(X_sp_1_l.*Cont_Foot_1_L_B+X_sp_2_l.*Cont_Foot_2_L_B+X_sp_3_l.*Cont_Foot_3_L_B+X_sp_4_l.*Cont_Foot_4_L_B+X_sp_5_l.*Cont_Foot_5_L_B+X_sp_6_l.*Cont_Foot_6_L_B+X_sp_7_l.*Cont_Foot_7_L_B+X_sp_8_l.*Cont_Foot_8_L_B+X_sp_9_l.*Cont_Foot_9_L_B+X_sp_10_l.*Cont_Foot_10_L_B+X_sp_11_l.*Cont_Foot_11_L_B+X_sp_12_l.*Cont_Foot_12_L_B+X_sp_13_l.*Cont_Foot_13_L_B+X_sp_14_l.*Cont_Foot_14_L_B)./Cont_Foot_L_B;
COP_Calcn_y_r_B=(Y_sp_1_r.*Cont_Foot_1_R_B+Y_sp_2_r.*Cont_Foot_2_R_B+Y_sp_3_r.*Cont_Foot_3_R_B+Y_sp_4_r.*Cont_Foot_4_R_B+Y_sp_5_r.*Cont_Foot_5_R_B+Y_sp_6_r.*Cont_Foot_6_R_B+Y_sp_7_r.*Cont_Foot_7_R_B+Y_sp_8_r.*Cont_Foot_8_R_B+Y_sp_9_r.*Cont_Foot_9_R_B+Y_sp_10_r.*Cont_Foot_10_R_B+Y_sp_11_r.*Cont_Foot_11_R_B+Y_sp_12_r.*Cont_Foot_12_R_B+Y_sp_13_r.*Cont_Foot_13_R_B+Y_sp_14_r.*Cont_Foot_14_R_B)./Cont_Foot_R_B;
COP_Calcn_y_l_B=(Y_sp_1_l.*Cont_Foot_1_L_B+Y_sp_2_l.*Cont_Foot_2_L_B+Y_sp_3_l.*Cont_Foot_3_L_B+Y_sp_4_l.*Cont_Foot_4_L_B+Y_sp_5_l.*Cont_Foot_5_L_B+Y_sp_6_l.*Cont_Foot_6_L_B+Y_sp_7_l.*Cont_Foot_7_L_B+Y_sp_8_l.*Cont_Foot_8_L_B+Y_sp_9_l.*Cont_Foot_9_L_B+Y_sp_10_l.*Cont_Foot_10_L_B+Y_sp_11_l.*Cont_Foot_11_L_B+Y_sp_12_l.*Cont_Foot_12_L_B+Y_sp_13_l.*Cont_Foot_13_L_B+Y_sp_14_l.*Cont_Foot_14_L_B)./Cont_Foot_L_B;
COP_Calcn_z_r_B=(Z_sp_1_r.*Cont_Foot_1_R_B+Z_sp_2_r.*Cont_Foot_2_R_B+Z_sp_3_r.*Cont_Foot_3_R_B+Z_sp_4_r.*Cont_Foot_4_R_B+Z_sp_5_r.*Cont_Foot_5_R_B+Z_sp_6_r.*Cont_Foot_6_R_B+Z_sp_7_r.*Cont_Foot_7_R_B+Z_sp_8_r.*Cont_Foot_8_R_B+Z_sp_9_r.*Cont_Foot_9_R_B+Z_sp_10_r.*Cont_Foot_10_R_B+Z_sp_11_r.*Cont_Foot_11_R_B+Z_sp_12_r.*Cont_Foot_12_R_B+Z_sp_13_r.*Cont_Foot_13_R_B+Z_sp_14_r.*Cont_Foot_14_R_B)./Cont_Foot_R_B;
COP_Calcn_z_l_B=(Z_sp_1_l.*Cont_Foot_1_L_B+Z_sp_2_l.*Cont_Foot_2_L_B+Z_sp_3_l.*Cont_Foot_3_L_B+Z_sp_4_l.*Cont_Foot_4_L_B+Z_sp_5_l.*Cont_Foot_5_L_B+Z_sp_6_l.*Cont_Foot_6_L_B+Z_sp_7_l.*Cont_Foot_7_L_B+Z_sp_8_l.*Cont_Foot_8_L_B+Z_sp_9_l.*Cont_Foot_9_L_B+Z_sp_10_l.*Cont_Foot_10_L_B+Z_sp_11_l.*Cont_Foot_11_L_B+Z_sp_12_l.*Cont_Foot_12_L_B+Z_sp_13_l.*Cont_Foot_13_L_B+Z_sp_14_l.*Cont_Foot_14_L_B)./Cont_Foot_L_B;


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
COP_Hand_x_l=X_Hand_l;
COP_Hand_y_l=Y_Hand_l;
COP_Hand_z_l=Z_Hand_l;
COP_Hand_x_r=X_Hand_r;
COP_Hand_y_r=Y_Hand_r;
COP_Hand_z_r=Z_Hand_r;
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
Proj_COP_Calcn_l=zeros(length(timeBK),3);
Proj_COP_Calcn_r=zeros(length(timeBK),3);

for i=1:length(timeBK)
    if ~Cont_Foot_L_B(i)==0 && Cont_Foot_L_A(i)==0
Proj_COP_Calcn_l(i,:)=[COP_Calcn_x_l_B(i,:) COP_Calcn_y_l_B(i,:) COP_Calcn_z_l_B(i,:)]-(dot([COP_Calcn_x_l_B(i,:) COP_Calcn_y_l_B(i,:) COP_Calcn_z_l_B(i,:)]-Ground_L_Foot_T_B,n))*n'; % if I am in a time interval where the contact B is met I have to consider its contact plane, otherwise consider the contact A 
    elseif ~Cont_Foot_L_A(i)==0 && Cont_Foot_L_B(i)==0
Proj_COP_Calcn_l(i,:)=[COP_Calcn_x_l_A(i,:) COP_Calcn_y_l_A(i,:) COP_Calcn_z_l_A(i,:)]-(dot([COP_Calcn_x_l_A(i,:) COP_Calcn_y_l_A(i,:) COP_Calcn_z_l_A(i,:)]-Ground_L_Foot_T_A,n))*n';
    else
Proj_COP_Calcn_l(i,:)=[COP_Calcn_x_l_B(i,:) COP_Calcn_y_l_B(i,:) COP_Calcn_z_l_B(i,:)]-(dot([COP_Calcn_x_l_B(i,:) COP_Calcn_y_l_B(i,:) COP_Calcn_z_l_B(i,:)]-Ground_L_Foot_T_B,n))*n'; % if I am in a time interval where the contact B is met I have to consider its contact plane, otherwise consider the contact A 
    end
    if ~Cont_Foot_R_B(i)==0 && Cont_Foot_R_A(i)==0
Proj_COP_Calcn_r(i,:)=[COP_Calcn_x_r_B(i,:) COP_Calcn_y_r_B(i,:) COP_Calcn_z_r_B(i,:)]-(dot([COP_Calcn_x_r_B(i,:) COP_Calcn_y_r_B(i,:) COP_Calcn_z_r_B(i,:)]-Ground_R_Foot_T_B,n))*n'; % if I am in a time interval where the contact B is met I have to consider its contact plane, otherwise consider the contact A 
    elseif ~Cont_Foot_R_A(i)==0 && Cont_Foot_R_B(i)==0
Proj_COP_Calcn_r(i,:)=[COP_Calcn_x_r_A(i,:) COP_Calcn_y_r_A(i,:) COP_Calcn_z_r_A(i,:)]-(dot([COP_Calcn_x_r_A(i,:) COP_Calcn_y_r_A(i,:) COP_Calcn_z_r_A(i,:)]-Ground_R_Foot_T_A,n))*n';
    else
Proj_COP_Calcn_r(i,:)=[COP_Calcn_x_r_B(i,:) COP_Calcn_y_r_B(i,:) COP_Calcn_z_r_B(i,:)]-(dot([COP_Calcn_x_r_B(i,:) COP_Calcn_y_r_B(i,:) COP_Calcn_z_r_B(i,:)]-Ground_R_Foot_T_B,n))*n'; % if I am in a time interval where the contact B is met I have to consider its contact plane, otherwise consider the contact A 
    end
 end

 if FullBody
Cont_Hand_R=ForceReport(:,strcmp('ForceGround_Hand_R.ground.force.Y',HeadFR));
Cont_Hand_L=ForceReport(:,strcmp('ForceGround_Hand_L.ground.force.Y',HeadFR));
     for i=1:length(timeBK)
    Proj_COP_Hand_r(i,:)=[COP_Hand_x_r(i,:) COP_Hand_y_r(i,:) COP_Hand_z_r(i,:)]-(dot([COP_Hand_x_r(i,:) COP_Hand_y_r(i,:) COP_Hand_z_r(i,:)]-Ground_R_Hand_T',n))*n';
    Proj_COP_Hand_l(i,:)=[COP_Hand_x_l(i,:) COP_Hand_y_l(i,:) COP_Hand_z_l(i,:)]-(dot([COP_Hand_x_l(i,:) COP_Hand_y_l(i,:) COP_Hand_z_l(i,:)]-Ground_L_Hand_T',n))*n';
     end
 end


% figure, plot(timeBK,Proj_COP_Calcn_r(:,1)), hold on, plot(timeBK,COP_Calcn_x_r)
% figure, plot(timeBK,Proj_COP_Calcn_l), hold on, plot(timeBK,COP_Calcn_y_l)

disp('CoP estimation successfully completed')

% Proj_COP_Calcn_r=[COP_Calcn_x_r COP_Calcn_y_r COP_Calcn_z_r];
% Proj_COP_Calcn_l=[COP_Calcn_x_l COP_Calcn_y_l COP_Calcn_z_l];


% control on Calcn_l and calcn_r velocity


% v_th=0.5; % velocity threshold for outliers detectors
% for i=1:length(Cont_Foot_R)
%     if  Cont_Foot_R(i)~=0 && v_calcn_r_n(i)>v_th % if the contact is achieved but with high velocity contact is not considered
%          Cont_Foot_R(i)=0;
%     elseif Cont_Foot_L(i)~=0 && v_calcn_l_n(i)>v_th
%          Cont_Foot_L(i)=0;
%     end
% 
% end

if FullBody
Cont=[Cont_Foot_R,Cont_Foot_L,Cont_Hand_R,Cont_Hand_L];

else
    Cont=[Cont_Foot_R,Cont_Foot_L];
end
%HeadCont={'Foot_R','Foot_L','Hand_R','Hand_L'};
%HeadCont={'Foot_R','Foot_L','Hand_R','Hand_L'};
%% Excitation setup
%LowestValue=0.001;


% preallocation
MinControl= zeros(length(timeBK),4); 

% define max and min controls for GRF actuators based on contacts with ground
MinControl(Cont(:)==0)=-LowestValue; 
MinControl(Cont(:)~=0)=-HighestValue;
MaxControl=-MinControl;

C=ischange(MinControl);
D=circshift(C,-1); % 1 temporal increments as transition
E=C+D;
E(1,:)=ones(1,4);
E(end,:)=ones(1,4); % contains the temporal instants of interest : the time increments to consider in the Control matrix

CntrSet=ControlSet();
if FullBody
Act={'GRF_Foot_R_X','GRF_Foot_R_Y','GRF_Foot_R_Z','GRM_Foot_R_X','GRM_Foot_R_Y','GRM_Foot_R_Z','GRF_Foot_L_X','GRF_Foot_L_Y','GRF_Foot_L_Z','GRM_Foot_L_X','GRM_Foot_L_Y','GRM_Foot_L_Z','GRF_Hand_R_X','GRF_Hand_R_Y','GRF_Hand_R_Z','GRM_Hand_R_X','GRM_Hand_R_Y','GRM_Hand_R_Z','GRF_Hand_L_X','GRF_Hand_L_Y','GRF_Hand_L_Z','GRM_Hand_L_X','GRM_Hand_L_Y','GRM_Hand_L_Z'};

else
    Act={'GRF_Foot_R_X','GRF_Foot_R_Y','GRF_Foot_R_Z','GRM_Foot_R_X','GRM_Foot_R_Y','GRM_Foot_R_Z','GRF_Foot_L_X','GRF_Foot_L_Y','GRF_Foot_L_Z','GRM_Foot_L_X','GRM_Foot_L_Y','GRM_Foot_L_Z'};
end 
for k=1:length(Act)
    CntrLin=ControlLinear();
    CntrLin.setName([Act{k},'.excitation'])
    CntrLin.setIsModelControl(true);
    CntrLin.setExtrapolate(true);
    CntrLin.setDefaultParameterMin(-1);
    CntrLin.setDefaultParameterMax(2);
    CntrLin.setFilterOn(false);
    CntrLin.setUseSteps(false);
    switch true
        case ismember(k,1:6)
            j=1;
        case  ismember(k,7:12)
            j=2;
        case ismember(k,13:18)
            j=3;
        case ismember(k,19:24)
            j=4;
    end
    for i=length(timeBK):-1:1
        if E(i,j)==1 % select time instants of interest
            MinCntrNode=ControlLinearNode();
            MinCntrNode.setTime(timeBK(i))
            MaxCntrNode=ControlLinearNode();
            MaxCntrNode.setTime(timeBK(i))
            if ~ismember(k,[2,8,14,20]) % se sono in direzione verticale
                MinCntrNode.setValue(MinControl(i,j))
                CntrLin.insertNewMinNode(0,MinCntrNode)
                MaxCntrNode.setValue(MaxControl(i,j))
                CntrLin.insertNewMaxNode(0,MaxCntrNode)
            else
                %ismember(k,[2,8,14,20])
                MinCntrNode.setValue(0)
                CntrLin.insertNewMinNode(0,MinCntrNode)
                MaxCntrNode.setValue(MaxControl(i,j))
                CntrLin.insertNewMaxNode(0,MaxCntrNode)
            end
        end
    end
    CntrSet.cloneAndAppend(CntrLin);
    clear CntrLin
end

%set the Pelvis control
Flight=zeros(1,length(timeBK));
Flight(sum(Cont')==0)=1;  % where 1 means flight times
F_change=ischange(Flight);
F=F_change+circshift(F_change,-1);
F(1)=1;
F(end)=1;

%Preallocation
MaxPelvisControl= zeros(length(timeBK),1); 
% define max and min controls based on contacts with ground
%PelvisLowestValue=0.03;


MaxPelvisControl(Flight(:)==0)=PelvisHighestValueBase; 
MaxPelvisControl(Flight(:)==1)=PelvisHighestValueFlight; % if in flight instant pelvis Activity is more likely to act
MinPelvisControl=-MaxPelvisControl;

%if sum(Flight)~=0 % if flight time exixts
    for u=0:model.getJointSet.getSize-1 % for every joint
        for v=0:model.getJointSet.get(u).numCoordinates-1 % get every coordinate from every joint
            coord=model.getJointSet.get(u).get_coordinates(v);
            if contains(convertCharsToStrings(toString(coord).toCharArray),'pelvis')   % se pelvi
                PelvisCntr=ControlLinear();
                PelvisCntr.setName(append(convertCharsToStrings(toString(coord).toCharArray),".excitation"));
                PelvisCntr.setIsModelControl(true);
                PelvisCntr.setExtrapolate(true);
                PelvisCntr.setDefaultParameterMin(-1);
                PelvisCntr.setDefaultParameterMax(2);
                PelvisCntr.setFilterOn(false);
                PelvisCntr.setUseSteps(false);
                for i=length(timeBK):-1:1
                    if F(i)==1
                        MinCntrNode=ControlLinearNode();
                        MinCntrNode.setTime(timeBK(i))
                        MaxCntrNode=ControlLinearNode();
                        MaxCntrNode.setTime(timeBK(i));
                        MinCntrNode.setValue(MinPelvisControl(i));
                        PelvisCntr.insertNewMinNode(0,MinCntrNode);
                        MaxCntrNode.setValue(MaxPelvisControl(i));
                        PelvisCntr.insertNewMaxNode(0,MaxCntrNode);
                    end
                end
                CntrSet.cloneAndAppend(PelvisCntr);
                clear PelvisCntr;
            end
        end
    end
%end
CntrSet.print(fullfile(path_Solution,"excitations_set.xml"));
%% Creating coordinate constaints list
if model.getConstraintSet.getSize ~= 0
for cs=0:model.getConstraintSet.getSize-1
   ConstSet(cs+1,:)=string(CoordinateCouplerConstraint.safeDownCast(model.getConstraintSet.get(cs)).getDependentCoordinateName); %SafeDownCasting the 
end
else
    ConstSet="No Coupled Constrained Joints";
end

%% Coordinate Task file creation
CMCTaskFile=CMC_TaskSet();
weight=50;
for u=0:model.getJointSet.getSize-1 % for every joint
    for v=0:model.getJointSet.get(u).numCoordinates-1 % get every coordinate from every joint
        coord=model.getJointSet.get(u).get_coordinates(v);
        if ~coord.get_locked && ~contains(convertCharsToStrings(toString(coord).toCharArray),ConstSet) % not considering locked joints nor patella joint
            CMCJoint=CMC_Joint();
            CMCJoint.setName((convertCharsToStrings(toString(coord).toCharArray)))
            CMCJoint.setWeight(weight)
            CMCJoint.setKP(100,1,1);
            CMCJoint.setKV(20,1,1);
            CMCJoint.setKA(1,1,1);
            CMCJoint.setActive(1,0,0);
            CMCJoint.setExpressBodyName("-1");
            CMCJoint.setWRTBodyName("-1");
            CMCJoint.setCoordinateName((convertCharsToStrings(toString(coord).toCharArray)))
            if contains(convertCharsToStrings(toString(coord).toCharArray),'pelvis')  % se pelvi
                CMCjoint.setOn=false;
                CMCJoint.setWeight(2*weight);
                CMCTaskFile.cloneAndAppend(CMCJoint);
            else
                CMCjoint.setOn=1;
                CMCTaskFile.cloneAndAppend(CMCJoint);
            end
            clear CMCjoint
        end
    end
end
CMCTaskFile.print(fullfile(path_Solution,'Task_coordinates.xml'));
% [ActFile,ActFold]=uigetfile('*.xml','Pick up the External actuators file');
 %[TaskFile,TaskFold]=uigetfile('*.xml','Pick up the Coordinate tasks file');
disp('Starting GRF calculation ...')
PredGRF=[];
KinOut=[];
Perr=[];
for t=1:length(timeBK)-1
     if Flight(t)==0
        PelvisForce=PelvisForceBase;
        PelvisHighestValue=PelvisHighestValueBase;
        PelvisLowestValue=PelvisLowestValueBase;
     else 
        PelvisForce=PelvisForceFlight;
        PelvisHighestValue=PelvisHighestValueFlight;
        PelvisLowestValue=PelvisLowestValueFlight;
     end
    CMCStartIter=timeBK(t);
    CMCEndIter=timeBK(t+1);
ActuatorFile=ForceSet();
% coordinate actuators including pelvis
for u=0:model.getJointSet.getSize-1 % for every joint
    for v=0:model.getJointSet.get(u).numCoordinates-1 % get every coordinate from every joint
        coord=model.getJointSet.get(u).get_coordinates(v);
        if ~coord.get_locked && ~contains(convertCharsToStrings(toString(coord).toCharArray),ConstSet)
            CoordActuator=CoordinateActuator();
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
        end
    end
end
% external actuators
%bodies=["Foot_R","Foot_L","Hand_R","Hand_L"];
%side={'R','L'}
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
    if contains(Act{i},'GRF')
        ExtForce=PointActuator();
        ExtForce.setName(Act{i})
        ExtForce.set_appliesForce(1);
        ExtForce.set_point_is_global(false)
        ExtForce.setOptimalForce(Force)
        ExtForce.setMinControl(-HighestValue);
        ExtForce.setMaxControl(HighestValue);
        ExtForce.set_direction(Vec3.createFromMat(direction));
        ExtForce.set_force_is_global(true);
        ExtForce.set_point_is_global(true);
        ExtForce.set_body(model.getBodySet.get(body).toString);
        ExtForce.set_point(Vec3.createFromMat(Point))
    else
        ExtForce=TorqueActuator();
        ExtForce.setName(Act{i});
        ExtForce.set_appliesForce(1);
        ExtForce.set_bodyA(body);
        ExtForce.setMinControl(-HighestValue);
        ExtForce.setMaxControl(HighestValue);
        ExtForce.setBodyB(model.getGround)
        ExtForce.set_axis(Vec3.createFromMat(direction));
        ExtForce.set_torque_is_global(true);
        ExtForce.setOptimalForce(Moment);
     end
    ActuatorFile.append(ExtForce);
end
ActuatorFile.print(fullfile(path_Solution,'CoordActuator.xml'));
%% create and launch CMC tool
CMCtool=CMCTool();
CMCtool.setModel(model);
CMCtool.setModelFilename(fullfile(ModelFolder,ModelFile));
CMCtool.setReplaceForceSet(IgnoreMuscles);
ForceStrArray=ArrayStr();
ForceStrArray.append('CoordActuator.xml');
%ForceStrArray.append(TaskFile);
CMCtool.setForceSetFiles(ForceStrArray);
CMCtool.setResultsDir(fullfile(path_iter,['Iter_',num2str(t)]));
CMCtool.setInitialTime(CMCStartIter);
CMCtool.setFinalTime(CMCEndIter);
CMCtool.setConstraintsFileName(fullfile(path_Solution,"excitations_set.xml"));
CMCtool.setSolveForEquilibrium(SolveForEquilibrium);
CMCtool.setDesiredKinematicsFileName(fullfile(IKFolder,IKResult));
CMCtool.setLowpassCutoffFrequency(Freq);
CMCtool.setTimeWindow(TimeWindow);
CMCtool.setMaximumNumberOfSteps(80000);
CMCtool.setTaskSetFileName(fullfile(path_Solution,'Task_coordinates.xml'));
CMCtool.setUseFastTarget(SelectFastTarget);
CMCtool.print(fullfile(path_Solution,'CMC_Setup.xml'));
dos(['opensim-cmd -o off run-tool ',char(fullfile(path_Solution,'CMC_Setup.xml'))]);
[PredGRF_iter,HeadSTO]=load_sto(fullfile(path_iter,['Iter_',num2str(t)],'_Actuation_force.sto'));
PredGRF=[PredGRF;mean(PredGRF_iter,[1,length(PredGRF_iter)])];
KinOut_iter=load_mot(fullfile(path_iter,['Iter_',num2str(t)],'_Kinematics_q.sto'));
KinOut=[KinOut;mean(KinOut_iter,[1,length(KinOut_iter)])];
pErr_iter=load_sto(fullfile(path_iter,['Iter_',num2str(t)],'_pErr.sto'));
pErr=[Perr;mean(pErr_iter,[1,length(pErr_iter)])];
disp(['Processing time: ', num2str(timeBK(t))]);
end

indxAct=contains(HeadSTO,Act);
GRFData=PredGRF(:,indxAct);
for s=1:length(Act)/3
    GRF_vec=GRFData(:,(s-1)*3+1:s*(3));
    GRFData(:,(s-1)*3+1:s*(3))=(Rot*GRF_vec')';
end
PredGRF(:,indxAct)=GRFData;
%% Writing MOT file
time=PredGRF(:,1);
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
Calcn_r_Mz=PredGRF(:,strcmp(HeadSTO,'GRM_Foot_R_X'));
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
    Hand_r_Mz=PredGRF(:,strcmp(HeadSTO,'GRM_Hand_R_X'));
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
        % COP_Calcn_x_r(i)= 0;
        % COP_Calcn_z_r(i)= 0;
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
        % COP_Calcn_x_l(i)= 0;
        % COP_Calcn_z_l(i)= 0;
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
        % COP_Hand_x_l(i)= 0;
        % COP_Hand_z_l(i)= 0;
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
        % COP_Hand_x_r(i)= 0;
        % COP_Hand_z_r(i)= 0;
        Hand_r_Fx(i)=0;
        Hand_r_Fy(i)=0;
        Hand_r_Fz(i)=0;
        Hand_r_Mx(i)=0;
        Hand_r_My(i)=0;
        Hand_r_Mz(i)=0;
    end
end
end
%% Writing CMC.sto Result file
info_CMC=string({'Actuation Force'; ['version=' num2str(1)];['nRows=' num2str(length(time))];['nColumns=' num2str(length(PredGRF(1,:)))];'inDegrees=yes';'endheader'});																																			
outFile_CMC=[info_CMC;strjoin(string(HeadSTO'));string(num2str(PredGRF))];
fileID_CMC=fopen(fullfile(path_iter,'Force_eq.sto'),"w+");
for k=1:length(outFile_CMC)
fprintf(fileID_CMC,'%s',[outFile_CMC(k)]);
fprintf(fileID_CMC,'\n');
end
fclose(fileID_CMC);
%% Writing kinematics.mot Result file
info_kin=string({'Coordinates'; ['version=' num2str(1)];['nRows=' num2str(length(time))];['nColumns=' num2str(length(KinOut(1,:)))];'inDegrees=yes';'endheader'});																																			
outFile_kin=[info_kin;strjoin(string(HeadMotion'));string(num2str(KinOut))];
fileID_kin=fopen(fullfile(path_iter,'output_kinematics.mot'),"w+");
for k=1:length(outFile_kin)
fprintf(fileID_kin,'%s',[outFile_kin(k)]);
fprintf(fileID_kin,'\n');
end
fclose(fileID_kin);


%% Writing .MOT file

Data_calcn_l=[Calcn_l_Fx Calcn_l_Fy Calcn_l_Fz Proj_COP_Calcn_l(1:end-1,1) Proj_COP_Calcn_l(1:end-1,2) Proj_COP_Calcn_l(1:end-1,3) Calcn_l_Mx Calcn_l_My Calcn_l_Mz];
Data_calcn_r=[Calcn_r_Fx Calcn_r_Fy Calcn_r_Fz Proj_COP_Calcn_r(1:end-1,1) Proj_COP_Calcn_r(1:end-1,2) Proj_COP_Calcn_r(1:end-1,3) Calcn_r_Mx Calcn_r_My Calcn_r_Mz];
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

disp('Analysis successfully completed')

%% START of statistics
SubjField=Dir(ss+2).name; % name of the subject
% initializing field of structure
if contains(TaskName,'levelground') % TaskField
TaskField='Levelground';
elseif contains(TaskName,'A_ramp')
TaskField='Ramp_Ascent';
elseif contains(TaskName,'D_ramp')
TaskField='Ramp_Descent';
elseif contains(TaskName,'A_stair')
TaskField='Stair_Ascent';
elseif contains(TaskName,'D_stair')
TaskField='Stair_Descent';
end
TrialField=['Trial_',num2str(nn)];

BW=9.81*InfoData{1,5};
BH=InfoData{1,6};
Fs_GRF=1000;
fs_ID=200;
Np=100; %resampling points

% loading FP data
[FP_data_all,FP_Header]=load_mot(fullfile(Subj_Fold,'FP',IKResult));
time_FP=FP_data_all(:,1);
FP_time_sel=time_FP(all([time_FP>=time(1),time_FP<=time(end)],2));
FP_data=FP_data_all(all([time_FP>=time(1),time_FP<=time(end)],2),contains(FP_Header,InfoData{1,4})); % select just the component of Forceplate of interest at the interest time interval
% filter 
[bb,aa]=butter(4,Freq/(Fs_GRF/2),"low");
FP_data(:,[1 2 3 7 8 9])=filtfilt(bb,aa,FP_data(:,[1 2 3 7 8 9]));  %filtering all data except for CoP
%
%
if strcmp(InfoData{1,3},'L')
Pred_data=Data_calcn_l;
elseif strcmp(InfoData{1,3},'R')
    Pred_data=Data_calcn_r;
end

% change of sign based on the side
if  strcmp(InfoData{1,3},'R')
    FP_data(:,1)=-FP_data(:,1); % change of Fx (Medio-lateral) force between R and L
    Pred_data(:,1)=-Pred_data(:,1);
end

%ResTime_FP=time(1):(time(end)-time(1))/Np:time(end);
% Res_FP_data=interp1(1:size(FP_data,1),FP_data,(1:size(FP_data,1))/Np:size(FP_data,1));
% Res_Pred_data=interp1(0:size(Pred_data,1),Pred_data,0:size(Pred_data,1)/Np:size(Pred_data,1));
FP_time_res=FP_time_sel(1):(FP_time_sel(end)-FP_time_sel(1))/Np:FP_time_sel(end);
Pred_time_res=time(1):(time(end)-time(1))/Np:time(end);
Res_FP_data=interp1(FP_time_sel,FP_data,FP_time_res);
Res_Pred_data=interp1(time,Pred_data,Pred_time_res);

Res_FP_data_norm(:,1:3)=Res_FP_data(:,1:3)/BW;
Res_FP_data_norm(:,7:9)=Res_FP_data(:,7:9)/(BW*BH);
Res_Pred_data_norm(:,1:3)=Res_Pred_data(:,1:3)/BW;
Res_Pred_data_norm(:,7:9)=Res_Pred_data(:,7:9)/(BW*BH);

%figure, plot(Res_FP_data(:,2)), hold on, plot(Res_Pred_data(:,2))
GRF_Pred=Res_Pred_data_norm(:,1:3);
COP_Pred=Res_Pred_data(:,4:6);
GRM_Pred=Res_Pred_data_norm(:,7:9);

GRF_FP=Res_FP_data_norm(:,1:3);
COP_FP=Res_FP_data(:,4:6);
GRM_FP=Res_FP_data_norm(:,7:9);

Summary.(SubjField).(TaskField).(TrialField).('Predicted').('GRF')=GRF_Pred;
Summary.(SubjField).(TaskField).(TrialField).('Predicted').('COP')=COP_Pred;
Summary.(SubjField).(TaskField).(TrialField).('Predicted').('GRM')=GRM_Pred;

Summary.(SubjField).(TaskField).(TrialField).('ForcePlate').('GRF')=GRF_FP;
Summary.(SubjField).(TaskField).(TrialField).('ForcePlate').('COP')=COP_FP;
Summary.(SubjField).(TaskField).(TrialField).('ForcePlate').('GRM')=GRM_FP;
% % compute Mean
% mean_GRF_Pred_x=mean(GRF_Pred(:,1));
% mean_GRF_Pred_y=mean(GRF_Pred(:,2));
% mean_GRF_Pred_z=mean(GRF_Pred(:,3));
% mean_GRM_Pred_x=mean(GRM_Pred(:,1));
% mean_GRM_Pred_y=mean(GRM_Pred(:,2));
% mean_GRM_Pred_z=mean(GRM_Pred(:,3));
% 
% mean_GRF_FP_x=mean(GRF_FP(:,1));
% mean_GRF_FP_y=mean(GRF_FP(:,2));
% mean_GRF_FP_z=mean(GRF_FP(:,3));
% mean_GRM_FP_x=mean(GRM_FP(:,1));
% mean_GRM_FP_y=mean(GRM_FP(:,2));
% mean_GRM_FP_z=mean(GRM_FP(:,3));
% 
% SD_GRF_Pred_x=std(mean(GRF_Pred(:,1)));
% compute RMSE
rRMSE_GRF_x=rmse(GRF_Pred(:,1),GRF_FP(:,1))/sqrt(sum(GRF_FP(:,1).^2))*100;
rRMSE_GRF_y=rmse(GRF_Pred(:,2),GRF_FP(:,2))/sqrt(sum(GRF_FP(:,2).^2))*100;
rRMSE_GRF_z=rmse(GRF_Pred(:,3),GRF_FP(:,3))/sqrt(sum(GRF_FP(:,3).^2))*100;

rRMSE_GRM_x=rmse(GRM_Pred(:,1),GRM_FP(:,1))/sqrt(sum(GRM_FP(:,1).^2))*100;
rRMSE_GRM_y=rmse(GRM_Pred(:,2),GRM_FP(:,2))/sqrt(sum(GRM_FP(:,2).^2))*100;
rRMSE_GRM_z=rmse(GRM_Pred(:,3),GRM_FP(:,3))/sqrt(sum(GRM_FP(:,3).^2))*100;

diff_COP=(COP_Pred-COP_FP);
dist_COP=sqrt(diff_COP(:,1).^2+diff_COP(:,2).^2+diff_COP(:,3).^2);
mean_dist_COP=mean(dist_COP);
RMSE_COP=sqrt(sum((dist_COP).^2)/length(COP_FP));

% % compute Wilcoxon-test
% [Pval_GRF_x,H_GRF_x]=ranksum(GRF_Pred(:,1),GRF_FP(:,1));
% [Pval_GRF_y,H_GRF_y]=ranksum(GRF_Pred(:,2),GRF_FP(:,2));
% [Pval_GRF_z,H_GRF_z]=ranksum(GRF_Pred(:,3),GRF_FP(:,3));
% [Pval_GRM_x,H_GRM_x]=ranksum(GRM_Pred(:,1),GRM_FP(:,1));
% [Pval_GRM_y,H_GRM_y]=ranksum(GRM_Pred(:,2),GRM_FP(:,2));
% [Pval_GRM_z,H_GRM_z]=ranksum(GRM_Pred(:,3),GRM_FP(:,3));
% [Pval_COP_x,H_COP_x]=ranksum(COP_Pred(:,1),COP_FP(:,1));
% [Pval_COP_y,H_COP_y]=ranksum(COP_Pred(:,2),COP_FP(:,2));
% [Pval_COP_z,H_COP_z]=ranksum(COPPred(:,3),COP_FP(:,3));
% compute pearson coefficient
rho=corrcoef(GRF_Pred(:,1),GRF_FP(:,1));
rho_GRF_x=rho(1,2);
rho=corrcoef(GRF_Pred(:,2),GRF_FP(:,2));
rho_GRF_y=rho(1,2);
rho=corrcoef(GRF_Pred(:,3),GRF_FP(:,3));
rho_GRF_z=rho(1,2);

rho=corrcoef(GRM_Pred(:,1),GRM_FP(:,1));
rho_GRM_x=rho(1,2);
rho=corrcoef(GRM_Pred(:,2),GRM_FP(:,2));
rho_GRM_y=rho(1,2);
rho=corrcoef(GRM_Pred(:,3),GRM_FP(:,3));
rho_GRM_z=rho(1,2);

rho=corrcoef(COP_Pred(:,1),COP_FP(:,1));
rho_COP_x=rho(1,2);
rho=corrcoef(COP_Pred(:,2),COP_FP(:,2));
rho_COP_y=rho(1,2);
rho=corrcoef(COP_Pred(:,3),COP_FP(:,3));
rho_COP_z=rho(1,2);
%

Stat.(SubjField).(TaskField).(TrialField).('GRF')=[[rRMSE_GRF_x rRMSE_GRF_y rRMSE_GRF_z];[rho_GRF_x rho_GRF_y rho_GRF_z]];
Stat.(SubjField).(TaskField).(TrialField).('GRM')=[[rRMSE_GRM_x rRMSE_GRM_y rRMSE_GRM_z];[rho_GRF_x rho_GRF_y rho_GRF_z]];
Stat.(SubjField).(TaskField).(TrialField).('COP')=mean_dist_COP;
if norm([rRMSE_GRF_x rRMSE_GRF_y rRMSE_GRF_z]) < 2
    disp('evvai rRMSE GRF minore del 2%, festeggiamo!!!!! :)')
end
%Stat.SubjField.TaskField.nn.('rho')=[GRF_rho;free_M_rho];

save(fullfile(MainFolder,'Stat_Results.mat'),'Stat')
save(fullfile(MainFolder,'Summary_Results.mat'),'Summary')

clearvars -except Dir IK_Dir IKFolder jj Num_sim ss nn MainFolder Stat Summary Subj_Fold
end
end
