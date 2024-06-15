% Ensure the Robotics Toolbox is added to the MATLAB path
% addpath('path_to_robotics_toolbox'); % Uncomment and set the correct path

% Initialize theta
thea = 0;

name = 'shaking_hands';
% 设置视频文件名称和帧率
videoFileName = [name, '.avi']; % 视频文件名
frameRate = 2; % 每秒帧数

% 创建 VideoWriter 对象
video = VideoWriter(videoFileName);
video.FrameRate = frameRate;
open(video);

% Define the links for the first robot
L(1) = Link([0 0 0 pi/2], 'standard');
L(2) = Link([0 0 0 -pi*70/180], 'standard');
L(2).qlim = [0, 20];
L(2).jointtype = 'P';
L(3) = Link([0 3.04 0 pi/2], 'standard');
L(4) = Link([0 0 0 pi/2], 'standard');
L(5) = Link([0 2.81 0 -pi/2], 'standard');
L(6) = Link([0 0 0 pi/2], 'standard');
L(7) = Link([0 2.33 0 -pi/2], 'standard'); 
L(8) = Link([0 0 0 pi/2], 'standard');
L(9) = Link([0 1.531 0 0], 'standard');

% Create the first robot
robot = SerialLink(L, 'name', 'a', 'base', trotx(-pi/2));

% Define the joint configuration
thea = 0;

% Set offsets for specific links
L(3).offset = pi/2;
L(4).offset = pi/2;
L(2).offset = 8.60;

% Load data
data = load([name, '_rot.mat']);
rot = data.rotations;
rot = rot([5,10,20,134,20,134,20,134,10,5],:);

% Initialize fixed joint angles
fixed_q1 = 0; % Fixed joint 1 angle
fixed_q2 = 0.61; % Fixed joint 2 angle

% Initialize joint angles
q0 = [fixed_q1 fixed_q2 0 0 0 0 0 0 0];

% Precompute all inverse kinematics solutions
num_positions = length(rot);
q_solutions = zeros(num_positions, 9);

for i = 1:num_positions
    % -pi/2 + rot(i, 3), rot(i, 4)
    q_sol = [fixed_q1, fixed_q2, -pi*70/180 - rot(i, 1), rot(i, 2), 0, - pi + rot(i, 3), rot(i, 4), rot(i, 5), rot(i, 6)];
    % Store the solution
    q_solutions(i, :) = q_sol;
end

q_solutions_degrees = rad2deg(q_solutions);
for col = [3,6]
    % -180 ~ 180
    over_180 = q_solutions_degrees(:, col) < -180;
    q_solutions_degrees(over_180, col) = 360 + q_solutions_degrees(over_180, col);
end
for col = [4,7]
    % 0 ~ 180
    q_solutions_degrees(:, col) = mod(q_solutions_degrees(:, col), 360);
    over_180 = q_solutions_degrees(:, col) > 180;
    q_solutions_degrees(over_180, col) = 360 - q_solutions_degrees(over_180, col);
end
q_solutions = deg2rad(q_solutions_degrees);

% Display the robot in its initial configuration
figure;
robot.plot(q0);

[az, el] = view;
newAz = az - 90;
view(newAz, el);
% camzoom(3);

% Plot all frames at 20 frames per second
for i = 1:num_positions
    disp(['Position: ', num2str(i)]);
    fig = gcf;
    frame = getframe(fig);
    writeVideo(video, frame);
    robot.plot(q_solutions(i, :));
    pause(1/2); % 20 frames per second
end

robot.plot(q0)

fig = gcf;
frame = getframe(fig);
writeVideo(video, frame);

close(video);

disp('视频已保存');
