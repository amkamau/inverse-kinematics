robot = importrobot('arm5.urdf');
robot.DataFormat = 'column';

% View the robot.
figure;
ax = axes;

show(robot,'Parent',ax);

% Specify a robot trajectory.
x = 0.5*zeros(1,4)+0.25;
y = 0.25*[-1 -1 1 1];
z = 0.25*[-1 1 -1 1] + 0.75;

hold(ax,'on');

plot3(ax,x,y,z,'--r','LineWidth',2);

hold(ax,'off');

view(ax,3);
axis(ax,'equal');
grid(ax,'on');


% ax = show(robot);
% 
% x = 0.5*zeros(1,4)+0.25;
% y = 0.25*[-1 -1 1 1];
% z = 0.25*[-1 1 -1 1] + 0.75;
% 
% hold on
% plot3(x,y,z,'--r','LineWidth',2,'Parent',ax)
% hold off