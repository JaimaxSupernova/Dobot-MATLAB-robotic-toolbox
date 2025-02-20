addpath rvctools
startup_rvc
rtbdemo

%clear terminal and environment to ensure clean start
clc
clear
%defining DH parameters
dh = [0 85 0 pi/2 ; 0 0 135 pi;0 0 147 0 ; 0 0 59.7 -pi/2; 0 0 0 0];
%creating and naming robot using its links (given in dh)
r = SerialLink(dh);
r.name='DoBot Magician';
%setting limits that dobot cannot reach due to physical limitation
r.qlim = [-pi/2 pi/2;  % Joint 1 (e.g., range -π/2 to π/2)
          [deg2rad(-5) deg2rad(90)];      % Joint 2 (e.g., range 0 to 1.5 meters)
          [0 pi/2];      % Joint 3 (e.g., range 0 to 1.5 meters)
          [-pi pi];
          [-pi pi]];

%Lines used to loop and create an array of inverse kinematics and forward
%kinematics, allowing for seamless plotting for comparision in matlab. 
%not essential to code functionality
% jointanglesarray = [30, 0,0,0; 45,40,0, 0 ; 30,0,20, 0; 60,0,30, 0; 45,45,45, 0; 0,45,45, 0; -10,45,45, 0; 30,0,0,0;15,60,50,0;-40,80,30,0
%     ];
% invarray=zeros(size(jointanglesarray));
% for i = 1:10
% jointangles=deg2rad(jointanglesarray(i,:))


%select/set joint angles of dobot
jointangles=deg2rad([-40, 80, 30 , 0 , 0]);

%calibrate end effector position, compensate for 2nd and 3rd joint
%co-dependant behaviour and set 2nd joint axis to match DoBot studio style
jointangles(2)=deg2rad(90)-jointangles(2);
jointangles(4)=-jointangles(3);
jointangles(3)=jointangles(3)+jointangles(2);

%create slider panel and plotting of joint angles
% r.teach()
% r.plot(jointangles)

%find forward kinematics and define studio adjusted FK(forward kinematics)
FK = r.fkine(jointangles);
FKstudio = FK;

%old code to compensate for 4th degree of freedom, no longer needed due to
%addition of 4th pseudo joint (the one that isnt a real joint and has no
%motor but controls end effector orientation)
% EEfectorLength = 59.7;
% x_offset = EEfectorLength*cos(jointangles(1));
% y_offset = EEfectorLength*sin(jointangles(1));
% FKstudio.t(1) = FKstudio.t(1) + x_offset;
% FKstudio.t(2) = FKstudio.t(2) + y_offset;
% jointangles(4)=-jointangles(3);

%adjust forward kinematics by height of first link to compensate for origin
%being on the 2nd link. and then print accurate FK
FKstudio.t(3) = FKstudio.t(3) - 85;
FKstudio

%find inverse kinematics using previous FK and desired joint angles
jointangles=r.ikcon(FK , jointangles);

%old function
%jointangles=r.ikine(FK , jointangles, 'mask',[1 1 1 1 0 0]);

%adjust joint angles to compensate for 2nd and 3rd joints being
%co-dependant and then print in degrees for readability
jointangles(3)=jointangles(3)-jointangles(2);
jointangles(2)=deg2rad(90)-jointangles(2);
rad2deg(jointangles)

%part of commented for loop for comparision
% invarray(i,:)=jointangles;

%restore joints from human readable format to matlab friendly
jointangles(2)=deg2rad(90)-jointangles(2);
jointangles(4)=-jointangles(3);
jointangles(3)=jointangles(3)+jointangles(2);

%end of comparision loop
% end
% invarray

%create slider panel and plotting of joint angles
r.teach();
r.plot(jointangles)

%splitting time into sections to split trajectory
t =0:.2:1;

%defining points 1-5
pickarr=deg2rad([-20 58.9 73.8 0;0 57.5 78.8 0;22 59 73 0;38 66 59 0;50 82 37 0]);
pickarr(:,5)=zeros;

%correcting for angle co-dependancy and end effector rotation to ensure
%cards will be aligned
for i = 1:5
pickarr(i, 2)=deg2rad(90)-pickarr(i, 2);
pickarr(i,4)=-pickarr(i,3);
pickarr(i ,3)=pickarr(i, 3)+pickarr(i, 2);
pickarr(i,5)=-pickarr(i,1);
end
%defining card drop off and safe route position
destdrop=deg2rad([16 84 38 0 0]);
obstacle_avoid=deg2rad([18 39 35 0 0]);

%correcting for angle dependancy and end effector rotation
destdrop(2)=deg2rad(90)-destdrop(2);
destdrop(4)=-obstacle_avoid(3);
destdrop(3)=destdrop(3)+destdrop(2);
destdrop(5)=-destdrop(1);

obstacle_avoid(2)=deg2rad(90)-obstacle_avoid(2);
obstacle_avoid(4)=-obstacle_avoid(3);
obstacle_avoid(3)=obstacle_avoid(3)+obstacle_avoid(2);
obstacle_avoid(5)=-obstacle_avoid(1);

%defining trajectories for each of the following points:

%getting to pos 1-5
go_pos1 = jtraj(obstacle_avoid, pickarr(1,:), t)
go_pos2 = jtraj(obstacle_avoid, pickarr(2,:), t)
go_pos3 = jtraj(obstacle_avoid, pickarr(3,:), t)
go_pos4 = jtraj(obstacle_avoid, pickarr(4,:), t)
go_pos5 = jtraj(obstacle_avoid, pickarr(5,:), t)

%getting back from pos 1-5
pick_pos1 = jtraj(pickarr(1,:), obstacle_avoid, t)
pick_pos2 = jtraj(pickarr(2,:), obstacle_avoid, t)
pick_pos3 = jtraj(pickarr(3,:), obstacle_avoid, t)
pick_pos4 = jtraj(pickarr(4,:), obstacle_avoid, t)
pick_pos5 = jtraj(pickarr(5,:), obstacle_avoid, t)

%placing card and going back after placing card
place = jtraj(obstacle_avoid, destdrop, t)
back2buffer = jtraj(destdrop, obstacle_avoid, t)

%plotting trajectories for animation on Matlab
r.plot(go_pos1)
r.plot(pick_pos1)
r.plot(place)
r.plot(back2buffer)
r.plot(go_pos2)
r.plot(pick_pos2)
r.plot(place)
r.plot(back2buffer)
r.plot(go_pos3)
r.plot(pick_pos3)
r.plot(place)
r.plot(back2buffer)
r.plot(go_pos4)
r.plot(pick_pos4)
r.plot(place)
r.plot(back2buffer)
r.plot(go_pos5)
r.plot(pick_pos5)
r.plot(place)
r.plot(back2buffer)

%GRAPH GENERATION
%SUBSECTION FOR GRAPHING

% combined trajectories for all movements
trajectories = {go_pos1, pick_pos1, go_pos2, pick_pos2, go_pos3, pick_pos3, ...
                go_pos4, pick_pos4, go_pos5, pick_pos5, place, back2buffer};

% Prepare a 3D plot on a new figure with axis labels and title 
figure;
hold on;
grid on;
xlabel('Time');
ylabel('Joint Number');
zlabel('Joint Angle');
title('Path and Smoothness of Joint Trajectories');

% Generate distinct colors for each trajectory
colors = lines(length(trajectories));

% Plot each trajectory
for trajnumber = 1:length(trajectories)
    traj = trajectories{trajnumber};
    [numSteps, numJoints] = size(traj);
    %plot each joints angles and colour it depending on the trajectory
    for jointnumber = 1:numJoints
        plot3(t, jointnumber * ones(size(t)), rad2deg(traj(:, jointnumber)), 'Color', colors(trajnumber, :));
    end
end

%nensure 3d view
view(3)
hold off;
figure;

%define joint angles for labelling
jointangles = ["30,0,0"; "45,40,0"; "30,0,20"; "60,0,30"; "45,45,45"; "0,45,45"; "-10,45,45"; "30,0,0";"15,60,50";"-40,80,30"
    ];
%define dobot and matlab coords for comparision of each angle
dobot_coords = [
    179.0074	103.35	135;
    207.519	207.519	103.416;
    171.33	98.9174	84.723;
    93.5018	161.9517	61.5;
    183.2141	183.2142	-8.4852;
    259.1041	0.002	-8.4853;
    255.1677	-44.993	-8.4853;
    179.0074	-103.35	135;
    261.8657	70.1667	-45.1085;
    245	-205.663	-50.0575;];
Matlab_coords = [
    179	103.3	135;
    207.5	207.5	103.4;
    171.3	98.92	84.72;
    93.5	162	61.5;
    183.2	183.2	-8.485;
    259	0	-8.485;
    255.2	-44.99	-8.485;
    179	-103.3	135;
    261.9	70.17	-45.11;
    245.1	-205.7	-50.06;];

hold on;


% Plot Dobot and matlab joints in 3D
scatter3(dobot_coords(:,1), dobot_coords(:,2), dobot_coords(:,3), 60, 'o', ...
    'DisplayName', 'Dobot Joints');

scatter3(Matlab_coords(:,1), Matlab_coords(:,2), Matlab_coords(:,3), 60, 'x', ...
    'DisplayName', 'MATLAB Joints', 'LineWidth', 1.5);

% Label joint angles for easy identification
for i = 1:size(Matlab_coords, 1)
    text(Matlab_coords(i,1), Matlab_coords(i,2), Matlab_coords(i,3), ...
        jointangles(i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 10);
end

% Add xyz axes, title and legend for identification
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
title('3D Scatter Plot of Dobot and MATLAB Joints for comparision');
legend show;
view(3); 

figure;

%define joint angles to be compared to inverse kinematics joint angles
jointangles = [30,0,0; 45,40,0; 30,0,20; 60,0,30; 45,45,45; 0,45,45; -10,45,45; 30,0,0;15,60,50;-40,80,30
    ];
%define joint angles given by inverse kinematics
invkinangles = rad2deg([
0.5236	0	0	0;
0.7854	0.6981	0	0;
0.5236	0.2679	0.2679	0.0000;
1.0472	0.4116	0.4116	0.0000;
0.7854	0.7854	0.7854	0;
0	0.7854	0.7854	0;
-0.1745	0.7854	0.7854	0;
0.5236	0	0	0;
0.2618	1.0472	0.8727	0;
-0.6981	1.3963	0.5236	0;]);


% Plot Dobot joint angles in 3d
scatter3(jointangles(:,1), jointangles(:,2), jointangles(:,3), 60, 'o', ...
    'DisplayName', 'input angles');
hold on;
% Plot MATLAB joint angles in 3D
scatter3(invkinangles(:,1), invkinangles(:,2), invkinangles(:,3), 60, 'x', ...
    'DisplayName', 'Inverse Kinematic angles', 'LineWidth', 1.5);
title('3D Scatter Plot of input and inverse kinematic generated joint angles');
legend show;
hold off;
