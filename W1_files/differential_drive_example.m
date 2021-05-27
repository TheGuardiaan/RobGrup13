%% Drive with robot
figure(1)
axis([-100 100 -200 200])
initpose = [0 0 0]; % [x y angle] Note – Pose for turtlebot is [x, y, θ] (in world coordinates)
rob = ExampleHelperDifferentialDriveRobot(initpose);

v = 20; % linear velocity (m/s)
w = 1; % angular velocity (rad/s)
for i=1:200,
    drive(rob, v, w);
    
    % decrease w
    w = w - 0.01;
end


%% Plot x-direction position, velocity, acceleration

% We can set/get time step of simulation
dt = rob.Dt 

% position
x = rob.Trajectory(:,1)
% velocity
vx = (x(2:end)-x(1:end-1)) / dt
% acceleration
ax = (vx(2:end)-vx(1:end-1)) / dt

figure()
subplot(311), plot(x), axis tight
xlabel('Time (s)'), ylabel('X-position (m)')
subplot(312), plot(vx)
xlabel('Time (s)'), ylabel('X-velocity (m/s)')
subplot(313), plot(ax)
xlabel('Time (s)'), ylabel('X-acceleration (m/s{^2})')


