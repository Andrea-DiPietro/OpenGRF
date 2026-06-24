function [COPx_fused, COPz_fused] = fuseCOP(COPx_geom, COPz_geom,COPx_dyn, COPz_dyn,isSS)
% Merge the trajectories from probes locations (geom) and the dynamics-based one (Dyn) knowing the single stance times of the
% kinematics into the final CoP trajectory

% Input: CoP_geom: from spheres
%        CoP_dyn: from dynamics
% Output: final CoP components

% Author: Andrea Di Pietro, 2025

% Initialize COP with position coming from contact anslysis
COPx_fused = COPx_geom;
COPz_fused = COPz_geom;

% Find all single stance intervals
ss_diff = diff([0; isSS(:); 0]);
ss_starts = find(ss_diff == 1);
ss_ends   = find(ss_diff == -1) - 1;

for i = 1:length(ss_starts)
    t1 = ss_starts(i);
    t2 = ss_ends(i);
    len = t2 - t1 + 1;

    if len < 10
        continue;
    end

    win_len = round(0.3 * len);
    if win_len < 5  % too short for blending
        continue;
    end

    blend_start     = t1;
    blend_mid_start = t1 + win_len;
    blend_mid_end   = t2 - win_len;
    blend_end       = t2;

    % first window (descreasing cosine)--> trasition from kinematics to dynamics
    w_in = 0.5 * (1 + cos(pi * (0:win_len-1) / (win_len - 1)));

    COPx_fused(blend_start:blend_mid_start-1) = ...
        w_in' .* COPx_geom(blend_start:blend_mid_start-1) + ...
        (1 - w_in') .* COPx_dyn(blend_start:blend_mid_start-1);

    COPz_fused(blend_start:blend_mid_start-1) = ...
        w_in' .* COPz_geom(blend_start:blend_mid_start-1) + ...
        (1 - w_in') .* COPz_dyn(blend_start:blend_mid_start-1);

    % central window: just dynamic contribution
    COPx_fused(blend_mid_start:blend_mid_end) = COPx_dyn(blend_mid_start:blend_mid_end);
    COPz_fused(blend_mid_start:blend_mid_end) = COPz_dyn(blend_mid_start:blend_mid_end);

    % final window  ( increasing cosine) --> trasition from dynamics to kinematics 
    w_out = flip(w_in);
    COPx_fused(blend_mid_end+1:blend_end) = ...
        w_out' .* COPx_geom(blend_mid_end+1:blend_end) + ...
        (1 - w_out') .* COPx_dyn(blend_mid_end+1:blend_end);

    COPz_fused(blend_mid_end+1:blend_end) = ...
        w_out' .* COPz_geom(blend_mid_end+1:blend_end) + ...
        (1 - w_out') .* COPz_dyn(blend_mid_end+1:blend_end);
end
end
