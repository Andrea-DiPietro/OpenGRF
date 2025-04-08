%-------------------------------------------------------------------------%
%    Author:   Luca Modenese,  2025                                       %
%    email:    l.modenese@unsw.edu.au                                     %
% ----------------------------------------------------------------------- %

function osimSphere = createContactSphere(sphereName, bodyName, aLocation, aRadius)

% import OpenSim libraries
import org.opensim.modeling.*

% define HuntCrossleyForce
osimSphere=ContactSphere();
osimSphere.setName(sphereName);
Body=model.getBodySet.get(bodyName);
Frame=PhysicalFrame.safeDownCast(Body);
osimSphere.setFrame(Frame);
%Vec3.createFromMat(aLocation)
osimSphere.setLocation(aLocation);
osimSphere.setRadius(aRadius);

end