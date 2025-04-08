%-------------------------------------------------------------------------%
%    Author:   Luca Modenese,  2025                                       %
%    email:    l.modenese@unsw.edu.au                                     %
% ----------------------------------------------------------------------- %
function osimHuntCrossleyForce = createHuntCrossleyForce(aForceName, aContactGeometry, aStiffness)

% import OpenSim libraries
import org.opensim.modeling.*

% define HuntCrossleyForce
osimHuntCrossleyForce=HuntCrossleyForce();
osimHuntCrossleyForce.setName('ForceGround_Foot_14_L_A');
osimHuntCrossleyForce.set_appliesForce(true);
osimHuntCrossleyForce.addGeometry('ground_Foot_L_A Sphere_Foot_14_L')
osimHuntCrossleyForce.setStiffness(Sp_stiffness);
osimHuntCrossleyForce.setDissipation(0);
osimHuntCrossleyForce.setStaticFriction(0);
osimHuntCrossleyForce.setDynamicFriction(0);
osimHuntCrossleyForce.setViscousFriction(0);
osimHuntCrossleyForce.setTransitionVelocity(0.13)
end