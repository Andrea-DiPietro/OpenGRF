function ExecutePK(model,ModelPath,IKpath,Freq,timeStart,timeEnd,calcn_r_name, calcn_l_name, toes_r_name, toes_l_name, heel_shift,path_PK,path_Solution)

% Execute Point-Kinematics for obtaining global probes positions, used for
% COP computation

% Author: Andrea Di Pietro, 2026

import org.opensim.modeling.*
AnTool_PK=org.opensim.modeling.AnalyzeTool();
AnTool_PK.setModel(model);
AnTool_PK.setModelFilename(ModelPath);
AnTool_PK.setCoordinatesFileName(IKpath);
AnTool_PK.setLowpassCutoffFrequency(Freq);
AnTool_PK.setStartTime(timeStart);
AnTool_PK.setFinalTime(timeEnd);
AnTool_PK.setResultsDir(path_PK);

for h=0:model.getContactGeometrySet.getSize-1
if contains(string(model.getContactGeometrySet.get(h)),'Sphere_Foot')
    [sp_num]=regexp(string(model.getContactGeometrySet.get(h)),"\d+","match"); % selecting the sphere number
    if contains(string(model.getContactGeometrySet.get(h)),'_R')
        point_name=['PK_sp',char(sp_num),'_r'];
        body_name=calcn_r_name;
        if double(sp_num)>=5 && double(sp_num)<=6 || double(sp_num)>=13 % selecting separately calcn and toes spheres
        body_name=toes_r_name;
        end
    else
        point_name=['PK_sp',char(sp_num),'_l'];
        body_name=calcn_l_name;
        if double(sp_num)>=5 && double(sp_num)<=6 || double(sp_num)>=13
        body_name=toes_l_name;
        end
    end
SpLoc=model.getContactGeometrySet.get(h).getLocation.getAsMat;                               
PK_sp=org.opensim.modeling.PointKinematics();
PK_sp.setPointName(point_name);
PK_sp.setBody(model.getBodySet.get(body_name));
if double(sp_num)<=4 % shift just for the first 4 spheres
PK_sp.setPoint(org.opensim.modeling.Vec3.createFromMat(SpLoc-[heel_shift; 0; 0]));
else
    PK_sp.setPoint(org.opensim.modeling.Vec3.createFromMat(SpLoc));
end
PK_sp.setRelativeToBody(model.getGround);
AnTool_PK.getAnalysisSet.cloneAndAppend(PK_sp);
clear PK_sp
end
end

AnTool_PK.print(fullfile(path_Solution,'SetupPK_sp.xml'));
AnalyzeTool(fullfile(path_Solution,'SetupPK_sp.xml')).run;
end