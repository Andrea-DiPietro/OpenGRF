function [f_cut_off] = DetectFcut(Motion,Fs)

% Detect most suitable kinematics cut-off by leveraging continuous-Wavelet-Transform approach

% Input: Motion is a Matrix of OpenSim kinematic file

% Output: array of  kinematics cut-off frequency

% Author: Andrea Di Pietro, 2026

M=Motion(:,2:end);
M=M-mean(M);
% t_vect=Motion(:,1);
%% detect not constant joint kinematics
k=1;
for jj=1:size(M,2)
    if sum(M(:,jj))==M(1,jj)*size(M,1) % if the joint coordinate
        ind_const(k)=jj;
        k=k+1;
    end
end

if exist("ind_const","var")
    M(:,ind_const)=[];
end
%% Continuous time frequency analysis
for i=1:size(M,2)
    [wt, f] = cwt(M(:,i),'amor',Fs);
    dominant_freqs(:,i)=sum(f.*abs(wt),1)./sum(abs(wt),1);
    f_cut_off=max(dominant_freqs,[],2);
end
end