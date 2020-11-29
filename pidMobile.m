function [vel, done] = pidMobile(cur,goal, Kv, Kw)
%PIDMOBILE Summary of this function goes here
% pose - [x, y, theta]
% goal - [x, y]
% Kv, Kw - [Kp, Ki, Kd]
tol = 0.5;

% Compute error
er = goal - cur(1:2);

% Compute v
v = Kv(1)*sqrt(er(1)^2 + er(2)^2);
if (abs(v) < tol)
    v = 0;
end

%Compute w
if (v < tol)
    w = 0;
else
    w = Kw(1)*real(asin((cos(cur(3))*er(2)-sin(cur(3))*er(1))/v))  ;
end

if (v == 0 && w == 0)
    done = 1;
else
    done = 0;
end
vel = [v,w];
end