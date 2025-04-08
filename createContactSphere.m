function osimSphere = createContactSphere(sphereName, bodyName, aLocation, aRadius)

osimSphere=ContactSphere();
osimSphere.setName(sphereName);
Body=model.getBodySet.get(bodyName);
Frame=PhysicalFrame.safeDownCast(Body);
osimSphere.setFrame(Frame);
%Vec3.createFromMat(aLocation)
osimSphere.setLocation(aLocation);
osimSphere.setRadius(aRadius);

end