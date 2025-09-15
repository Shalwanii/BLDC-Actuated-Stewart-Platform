function [theta, ok, whichBranch] = stewartIK(pose, params, thetaPrev)
% pose = [x y z roll pitch yaw] in metres & radians (Rz*Ry*Rx order)
% params fields: rp, sp, rb, sb, la, lb, beta(1x6), thMin, thMax
% thetaPrev (optional) used to pick the closest of the two branches
% Returns:
%   theta(1x6) [rad], ok (true if all joints valid), whichBranch(+1/-1 per joint)

if nargin < 3, thetaPrev = zeros(1,6); end

x=pose(1); y=pose(2); z=pose(3);
phi=pose(4); the=pose(5); psi=pose(6);

% Rotations: R = Rz(psi)*Ry(theta)*Rx(phi)
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(the) 0 sin(the); 0 1 0; -sin(the) 0 cos(the)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R  = Rz*Ry*Rx;

theta     = nan(1,6);
whichBranch = zeros(1,6);
ok = true;

for i = 1:6
    b  = params.beta(i);
    cb = cos(b); sb = sin(b);
    % platform & base attachment points (your definitions)
    pp = [ params.rp*cb + params.sp*sb;
           params.rp*sb - params.sp*cb;
           0 ];
    bw = [ params.rb*cb + params.sb*sb;
           params.rb*sb - params.sb*cb;
           0 ];
    pw = [x;y;z] + R*pp;
    L  = pw - bw;                          % Lw^i
    Lx=L(1); Ly=L(2); Lz=L(3);

    % coefficients a_i, b_i, c_i  (your equations)
    a = 2*params.la*Lz;
    bcoef = 2*params.la*( Lx*sb + Ly*cb );
    c = -params.la^2 + params.lb^2 - (Lx^2+Ly^2+Lz^2);

    denom = hypot(a,bcoef);                % sqrt(a^2+b^2)
    if denom < 1e-12
        ok=false; continue
    end
    xval = c/denom;
    if xval<-1 || xval>1
        ok=false; continue
    end

    base = atan2(a,bcoef);
    d    = acos( max(-1,min(1,xval)) );

    th1 = base + d;
    th2 = base - d;

    % choose branch: inside limits and closest to previous
    thMin = params.thMin; thMax = params.thMax;
    in1 = (th1>=thMin && th1<=thMax);
    in2 = (th2>=thMin && th2<=thMax);

    if in1 && in2
        e1 = angdiff(th1, thetaPrev(i));
        e2 = angdiff(th2, thetaPrev(i));
        if abs(e1) <= abs(e2), theta(i)=th1; whichBranch(i)=+1;
        else,                  theta(i)=th2; whichBranch(i)=-1; end
    elseif in1
        theta(i)=th1; whichBranch(i)=+1;
    elseif in2
        theta(i)=th2; whichBranch(i)=-1;
    else
        ok=false; % both out of bounds
    end
end
end

function d = angdiff(a,b) % wrap to [-pi,pi]
d = a-b;
d = mod(d+pi, 2*pi) - pi;
end
