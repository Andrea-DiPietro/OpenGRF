function UpdateSphereLocs(model,modelPath,Corr_r,Corr_l,n)
% Update the new calibrated spheres locations onto the model

% Input: Corr_l and Corr_r are the array for left and right spheres conataining the spatial correction of the initial positions
%        n is contact planes orientation

% Output: updated spheres 

% Author: Andrea Di Pietro, 2025

import org.opensim.modeling.*
for h=0:model.getContactGeometrySet.getSize-1
if contains(string(model.getContactGeometrySet.get(h)),'Sphere_Foot')
    [sp_num]=regexp(string(model.getContactGeometrySet.get(h)),"\d+","match"); % selecting the sphere number
    if contains(string(model.getContactGeometrySet.get(h)),'_R')
        model.getContactGeometrySet.get(h).setLocation(org.opensim.modeling.Vec3.createFromMat((model.getContactGeometrySet.get(h).getLocation.getAsMat-Corr_r(double(sp_num))*n)));
    elseif contains(string(model.getContactGeometrySet.get(h)),'_L')
       model.getContactGeometrySet.get(h).setLocation(org.opensim.modeling.Vec3.createFromMat((model.getContactGeometrySet.get(h).getLocation.getAsMat-Corr_l(double(sp_num))*n)));
    end
end
end
model.finalizeConnections();
model.print(modelPath);
end