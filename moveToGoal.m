%% Load all parameters
[dt, R, L] = parameters();
%% init the fisrt position of Mobile Robot
pose = initPose();
%% Set goal for robot
goals = [[10,20];[40,70];[80,20]];
%% Set PID params of Mobile Robot
Kv = [0.5, 0, 0];   % only Kp
Kw = [1.2, 0, 0];   % Only Kp
%% Move to goal
pose_set = pose;   % Save all position of Mobile Robot
goal_shape = size(goals);
for i = 1:goal_shape(1)
    done = false;
    goal = goals(i,:);
    while (~done)
        [vel, done] = pidMobile(pose, goal, Kv, Kw);
        pose = updatePose(pose, vel, dt);
        pose_set = cat(1, pose_set, pose);
    end
end
%% Plot Movement of Mobile Robot
plotMobileRobot(pose_set, goals);