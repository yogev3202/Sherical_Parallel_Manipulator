% Create a 3D unit sphere
[x, y, z] = sphere(50);
unit_sphere = surf(x, y, z);
hold on;

% Define the initial point and rotation axis
initial_point = [1; 0; 0];  % Starting point on the unit sphere
rotation_axis = [0; 0; 1];  % Rotation axis

% Create a quaternion representing the rotation
angle = pi/4;  % Rotation angle (in radians)
quat = [cos(angle/2); rotation_axis*sin(angle/2)];  % Quaternion representation

% Perform quaternion rotation
rotated_point = quatrotate(quat, initial_point')';

% Plot the initial and rotated points
scatter3(initial_point(1), initial_point(2), initial_point(3), 'ro', 'filled');
scatter3(rotated_point(1), rotated_point(2), rotated_point(3), 'bo', 'filled');

% Connect the initial and rotated points with a line
plot3([initial_point(1), rotated_point(1)], [initial_point(2), rotated_point(2)], [initial_point(3), rotated_point(3)], 'k--');

% Set the axis limits and labels
axis([-1 1 -1 1 -1 1]);
xlabel('X');
ylabel('Y');
zlabel('Z');

% Set the title and legend
title('Quaternion Rotation on a 3D Unit Sphere');
legend('Initial Point', 'Rotated Point');

% Set the view perspective
view(45, 30);
