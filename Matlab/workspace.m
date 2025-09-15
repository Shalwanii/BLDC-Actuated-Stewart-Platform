%% Stewart Platform Workspace (9 separate figs, ±45° limits + margin)
% Requires: your stewartIK(pose, params, thetaPrev) in path.
clear; clc;

%% -------- Fixed geometry (edit if needed) --------
base.rp   = 0.085103;
base.sp   = 0.024;
base.rb   = 0.095103;
base.sb   = 0.017;
base.beta = deg2rad([0 0 120 120 240 240]);   % reorder to match your build

%% -------- Joint angle limits & safety margin --------
degMargin  = 5;                  % require this many degrees slack from the limit
base.thMin = deg2rad(-45);
base.thMax = deg2rad(+90);
marginRad  = deg2rad(degMargin); % margin for feasibility

%% -------- 3×3 arm-length grid --------
la_vec = [0.05, 0.07, 0.09];     % proximal (m)
lb_vec = [0.18, 0.21, 0.24];     % distal   (m)

%% -------- Orientation (Orr-style: platform level) --------
roll = 0; pitch = 0; yaw = 0;

%% -------- Translational grid (metres) --------
% Keep this identical for all combos to compare fairly. Enlarge if you fear clipping.
x_range = linspace(-0.22, 0.22, 41);
y_range = linspace(-0.22, 0.22, 41);
z_range = linspace( 0.00, 0.50, 41);

% Consistent axes for all figures
xlims = [min(x_range), max(x_range)];
ylims = [min(y_range), max(y_range)];
zlims = [min(z_range), max(z_range)];

%% -------- Options --------
useParfor     = false;     % set true if you have Parallel Computing Toolbox
saveFigures   = false;     % set true to auto-save PNGs
figureView    = [135, 22]; % azimuth, elevation for hull plots

%% -------- Main sweep (9 separate figures) --------
fprintf('Grid: %dx%dx%d\n', numel(x_range), numel(y_range), numel(z_range));
fprintf('\n%-10s %-10s | %-12s %-12s | %-12s\n', 'l_a (mm)', 'l_b (mm)', 'V_voxel (m^3)', 'V_alpha (m^3)', 'Feasible pts');

for i = 1:3
  for j = 1:3
    params = base;
    params.la = la_vec(i);
    params.lb = lb_vec(j);

    % Evaluate feasible points with margin
    [xf, yf, zf, V_voxel] = evalFeasiblePointsAndVoxelVolume( ...
        params, x_range, y_range, z_range, roll, pitch, yaw, useParfor, marginRad);

    % Alpha-shape volume (shape-based, grid-agnostic)
    V_alpha = 0;
    if numel(xf) > 100
        shp = alphaShape(xf, yf, zf);
        try, shp.Alpha = criticalAlpha(shp); end
        V_alpha = volume(shp);
    end

    % Separate figure for this (l_a, l_b)
    f = figure('Color','w','Name',sprintf('Workspace la=%dmm lb=%dmm', round(params.la*1e3), round(params.lb*1e3)));
    if numel(xf) < 40
        scatter3(xf, yf, zf, 8, zf, 'filled');
    else
        try
            plot(shp);
        catch
            scatter3(xf, yf, zf, 8, zf, 'filled'); % fallback if alphaShape fails
        end
    end
    view(figureView);
    axis equal; grid on; box on;
    xlim(xlims); ylim(ylims); zlim(zlims);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title(sprintf('Hull: l_a=%.0f mm, l_b=%.0f mm', ...
          params.la*1e3, params.lb*1e3));

    % Optional: save
    if saveFigures
        outname = sprintf('workspace_la%03d_lb%03d_lim%02d_margin%02d.png', ...
            round(params.la*1e3), round(params.lb*1e3), degLimit, degMargin);
        exportgraphics(f, outname, 'Resolution', 300);
    end

    % Print summary row
    fprintf('%-10.0f %-10.0f | %-12.6f %-12.6f | %-12d\n', ...
        params.la*1e3, params.lb*1e3, V_voxel, V_alpha, numel(xf));
  end
end

%% ================== Helpers ==================
function [xf, yf, zf, V_voxel] = evalFeasiblePointsAndVoxelVolume( ...
        params, x_range, y_range, z_range, roll, pitch, yaw, useParfor, marginRad)

    [X,Y,Z] = ndgrid(x_range, y_range, z_range);
    N    = numel(X);
    feas = false(size(X));

    % Cheap reach prefilter (optional but recommended for speed)
    doPrefilter = true;

    if useParfor
        feasLocal = false(size(X)); %#ok<NASGU>
        parfor k = 1:N
            feasLocal_k = false; %#ok<NASGU>
            pose = [X(k), Y(k), Z(k), roll, pitch, yaw];
            if ~doPrefilter || quickReachable(pose, params)
                [theta, ok] = stewartIK(pose, params, zeros(1,6));
                if ok
                    % Require margin from limits
                    thAbsMax = max(abs(theta));
                    limitMax = max(abs([params.thMin, params.thMax]));
                    ok = (limitMax - thAbsMax) >= marginRad;
                end
                feasLocal_k = ok; %#ok<NASGU>
            end
            feas(k) = feasLocal_k; %#ok<PFOUS>
        end
    else
        for k = 1:N
            pose = [X(k), Y(k), Z(k), roll, pitch, yaw];
            if doPrefilter && ~quickReachable(pose, params)
                feas(k) = false; continue;
            end
            [theta, ok] = stewartIK(pose, params, zeros(1,6));
            if ok
                thAbsMax = max(abs(theta));
                limitMax = max(abs([params.thMin, params.thMax]));
                ok = (limitMax - thAbsMax) >= marginRad;
            end
            feas(k) = ok;
        end
    end

    xf = X(feas); yf = Y(feas); zf = Z(feas);

    dV = (x_range(2)-x_range(1))*(y_range(2)-y_range(1))*(z_range(2)-z_range(1));
    V_voxel = sum(feas(:)) * dV;
end

function tf = quickReachable(pose, params)
    % Very conservative prefilter using two-link distance band per leg:
    % distance from base joint to platform joint must lie in [|la-lb|, la+lb]
    x=pose(1); y=pose(2); z=pose(3);
    phi=pose(4); the=pose(5); psi=pose(6);
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Ry = [cos(the) 0 sin(the); 0 1 0; -sin(the) 0 cos(the)];
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R  = Rz*Ry*Rx;

    lo = abs(params.la - params.lb);
    hi = params.la + params.lb + 1e-9;

    for i = 1:6
        b  = params.beta(i);
        cb = cos(b); sb = sin(b);
        pp = [ params.rp*cb + params.sp*sb;
               params.rp*sb - params.sp*cb;
               0 ];
        bw = [ params.rb*cb + params.sb*sb;
               params.rb*sb - params.sb*cb;
               0 ];
        pw = [x;y;z] + R*pp;
        d = norm(pw - bw);
        if d < lo || d > hi
            tf = false; return;
        end
    end
    tf = true;
end
