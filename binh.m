function Q = IK_fulls(T)

% Link lengths
l1 = 180;l2= 200; l3=220;l4= 50;l5 =15;l6=100;

% Extract pose
Px = T(1,4);
Py = T(2,4);
Pz = T(3,4);

% Orientation constraint: q1 - q4 = theta_orient
theta_14 = atan2(T(1,2), T(1,1));  

tol = 1e-9; 
Q = []; % store solution

% Base radius 
R = sqrt(Px^2 + Py^2);

% q1 
if abs(R) < tol
    q1_base = 0;
else
    q1_base = atan2(Py, Px);
end

% two branches for q1
q1_set = [q1_base, q1_base + pi];

for q1 = q1_set

    % if q1 = atan2(Py,Px)     → R_eff = +R
    % if q1 = atan2(Py,Px)+pi  → R_eff = -R
    if abs(wrapToPi(q1 - q1_base)) < 1e-9
        R_eff = +R;
    else
        R_eff = -R;
    end

    % set varaible A and B 
    A = R_eff - l4;
    B = Pz - l1 + l5 + l6;

    % Set variable P
    P = sqrt(A^2 + B^2);

    % Solve for D
    num = l3^2 - A^2 - B^2 - l2^2;
    den = 2*l2;
    K = num / den;

    D = K / P;
    if abs(D) > 1 + tol
        continue;
    end
    D = max(min(D,1), -1);

    % q2 two branches
    phi   = atan2(B, A);
    alpha = asin(D);

    q2_set = [phi + alpha, phi + (pi - alpha)];

    % Loop q2 branches
    for q2 = q2_set

        % Solve q3
        cosq3 = (A + l2*sin(q2)) / l3;
        sinq3 = (l2*cos(q2) - B) / l3;

        % normalize
        mag = hypot(cosq3, sinq3);
        if abs(mag - 1) > 1e-5
            continue;
        end
        cosq3 = cosq3 / mag;
        sinq3 = sinq3 / mag;

        q3 = atan2(sinq3, cosq3);

        % Solve q4 from rotation
        q4 = q1 - theta_14;

        % Store
        Q = [Q; wrapToPi([q1 q2 q3 q4])];

    end

end

% Clean duplicates
Q = unique(round(Q, 10), 'rows');

% Convert to degrees
Q = rad2deg(Q);

end