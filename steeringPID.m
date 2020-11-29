%% Clear all
clear
close all
clc
%% Load all parameters
[dt, R, L] = parameters();
%% Position
curr = -pi;
last = curr;
%% Goal
goal = pi/2;
%% Error
er = 0;
er_1 = 0;
er_2 = 0;
%% Config plot
figure
grid on
xlim_graph = [0 5];
ylim_graph = [-pi pi];
pos_graph = animatedline('Marker','o');
xlim(xlim_graph);
ylim(ylim_graph);
title("Theta output");
%% PID params
Kp = 0.035;
Ti = 0.01;
Td = 0.1;
%% Computing
t = 0:dt:xlim_graph(2); % Init time sets
v = 2*ones(1,length(t)); % Init linear velocity sets
w = zeros(1,length(t)); % Init angular velocity sets
for i = 1:length(t)
    % Update error
    er_2 = er_1;
    er_1 = er;
    er = goal-curr;
    % Update position
    last = curr;
    curr = last + Kp*(er + dt/Ti*er_1 + Td/dt*(er-2*er_1+er_2));
    w(i)=(curr-last)/dt;
    % Plot
    addpoints(pos_graph,t(i), curr)
    drawnow limitrate
    pause(dt)
end
%% Plot velocity
figure
subplot(211);
plot(t,w);
title("Angular Velocity");
xlabel("Time");
ylabel("Angular Velocity");
subplot(212);
plot(t,v);
title("Linaer Velocity");
xlabel("Time");
ylabel("Linear Velocity");
%% Wheel velocity
wR = zeros(1,length(t));
wL = zeros(1,length(t));
for i = 1:length(t)
    [wR(i),wL(i)] = inverseKinematic(v(i), w(i));
end
figure
subplot(211);
plot(t,wR);
title("Wheel Right Velocity");
xlabel("Time");
ylabel("Velocity");
subplot(212);
plot(t,wL);
title("Wheel Left Velocity");
xlabel("Time");
ylabel("Velocity");