function  ExecuteFR(model,ModelPath,IKpath,Freq,timeStart,timeEnd,path_FR,path_Solution)

% Detect Contact between contact probes (Spheres) and Contact planes

% Author: Andrea Di Pietro, 2026

import org.opensim.modeling.*
AnTool=org.opensim.modeling.AnalyzeTool();
AnTool.setModel(model);
AnTool.setModelFilename(ModelPath);
AnTool.setCoordinatesFileName(IKpath);
AnTool.setLowpassCutoffFrequency(Freq);
AnTool.setSolveForEquilibrium(1);
AnTool.setStartTime(timeStart);
AnTool.setFinalTime(timeEnd);
AnTool.setResultsDir(path_FR);
FR_An=org.opensim.modeling.ForceReporter();
FR_An.setStartTime(timeStart);
FR_An.setEndTime(timeEnd);
AnTool.getAnalysisSet.cloneAndAppend(FR_An);
AnTool.print(fullfile(path_Solution,'Setup_ForceReporter.xml'));
ForceTool=AnalyzeTool(fullfile(path_Solution,'Setup_ForceReporter.xml'));
ForceTool.run;
end