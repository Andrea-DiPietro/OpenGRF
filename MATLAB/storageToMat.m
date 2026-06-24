function [t, Y, labels] = storageToMat(sto)
% transform an OpenSim storgage file (.sto) into Matrix 

% Author: Andrea Di Pietro, 2026

import org.opensim.modeling.*

% time
tArr = org.opensim.modeling.ArrayDouble();
sto.getTimeColumn(tArr);
N = tArr.getSize();
t = zeros(N,1);
for i = 0:N-1
    t(i+1) = tArr.get(i);
end

% labels
labArr = sto.getColumnLabels();
nLab = labArr.getSize()-1; % without time
labels = cell(nLab,1);
for j = 0:nLab-1
    labels{j+1} = char(labArr.get(j+1));
end

% data columns
nStates = sto.getSmallestNumberOfStates();
Y = zeros(N, nStates);
for j = 0:nStates-1
    col = ArrayDouble();
    sto.getDataColumn(j, col);
    for i = 0:N-1
        Y(i+1, j+1) = col.get(i);
    end
end
end
