% 2020/10/22 Calculating equation of motion of fully actuated 5 linked
% bipedal robot model with relative angles (floating point)

% PELIT Mustafa Melih

clear all
close all

% relative angles (floating point)
syms x_st y_st  dx_st dy_st ddx_st ddy_st % position of the stance foot
syms th1 th2 th3 th4 th5 real
syms dth1 dth2 dth3 dth4 dth5 real
syms ddth1 ddth2 ddth3 ddth4 ddth5 real
syms m1 m2 m3 m4 m5 real
syms I1 I2 I3 I4 I5 real
syms l1 l2 l3 l4 l5 real
syms gravi real

q = [x_st; y_st; th1; th2; th3; th4; th5];
dq = [dx_st; dy_st; dth1; dth2; dth3; dth4; dth5];
ddq = [ddx_st; ddy_st; ddth1; ddth2; ddth3; ddth4; ddth5];

m = [m1; m2; m3; m4; m5];

% CoM of Links
xG = [
    x_st + (l1/2)*cos(th1);
    x_st + l1*cos(th1) + (l2/2)*cos(th1 + th2);
    x_st + l1*cos(th1) + l2*cos(th1 + th2) + (l3/2)*cos(th1 + th2 + th3);
    x_st + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + (l4/2)*cos(th1 + th2 + th3 + th4);
    x_st + l1*cos(th1) + l2*cos(th1 + th2) + (l5/2)*cos(th1 + th2 + th5)];

zG = [
    y_st + (l1/2)*sin(th1);
    y_st + l1*sin(th1) + (l2/2)*sin(th1 + th2);
    y_st + l1*sin(th1) + l2*sin(th1 + th2) + (l3/2)*sin(th1 + th2 + th3);
    y_st + l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + (l4/2)*sin(th1 + th2 + th3 + th4);
    y_st + l1*sin(th1) + l2*sin(th1 + th2) + (l5/2)*sin(th1 + th2 + th5)];

%% Velocities of links' CoMs
for i=1:length(m)
    dxG(i) = jacobian(xG(i), q)*dq;
    dzG(i) = jacobian(zG(i), q)*dq;
end

%% Calculating kinetic energy due to CoM velocities and potential energies
KE_vel = 0;
PE = 0;
for i=1:length(m)
    KE_vel = KE_vel + 0.5*m(i)*(dxG(i)^2 + dzG(i)^2);
    PE = PE + m(i)*gravi*zG(i);
end

KE_inertia = 0.5*(I1*dth1^2 + I2*(dth1 + dth2)^2 + I3*(dth1 + dth2 + dth3)^2 + I4*(dth1 + dth2 + dth3 + dth4)^2 + I5*(dth1 + dth2 + dth5)^2);
KE = simplify(KE_vel + KE_inertia); % total KE
PE = simplify(PE);

%% Calculating Lagrangian
Lag =  simplify(KE - PE);
q_dq = [q; dq];
dq_ddq = [dq; ddq];

for i = 1:1:length(q)
Ldq(i,:) = diff(Lag,dq(i,:));
Ldq_dt(i,:) = jacobian(Ldq(i,:),q_dq)*dq_ddq;
Lq(i,:) = diff(Lag,q(i,:));
tau(i,:)= Ldq_dt(i,:) - Lq(i,:);
end

M = simplify(jacobian(tau,ddq));
G = simplify(collect(tau-subs(tau,gravi,0),gravi));
M_line = M*ddq;
C = simplify(tau-M_line-G);
H = simplify(G + C);

%% input torques mapping matrix
S = [
    0,0,0,0,0;
    0,0,0,0,0;
    1,0,0,0,0;
    0,1,0,0,0;
    0,0,1,0,0;
    0,0,0,1,0;
    0,0,-1,0,1;];

syms u1 u2 u3 u4 u5
U = [u1; u2; u3; u4; u5];

%% Single stance phase contraints
J_c_SS(1,:) = [1, 0, 0, 0, 0, 0, 0];
J_c_SS(2,:) = [0, 1, 0, 0, 0, 0, 0];

dJ_c_SS(1,:) = [0, 0, 0, 0, 0, 0, 0];
dJ_c_SS(2,:) = [0, 0, 0, 0, 0, 0, 0];

%% Double stance phase constraint
% x and y positions of the swing foot
x_sw = x_st + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + l4*cos(th1 + th2 + th3 + th4);
y_sw = y_st + l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + l4*sin(th1 + th2 + th3 + th4);

% vector of the constrained points
x_constrained = [x_st; y_st; x_sw; y_sw];

% constraint jacobian in the double stance phase
J_c_DS = jacobian(x_constrained, q);

jacobSize = size(J_c_DS);
dJ_c_DS = sym(zeros(jacobSize(1), jacobSize(2)));
for i = 1:1:jacobSize(1)
    for j = 1:1:jacobSize(2)
        dJ_c_DS(i,j) = simplify(jacobian(J_c_DS(i, j), q) * dq);
    end
end

%% Write matrices to the text file
% Convert the symbolic matrix to a string representation
M_str = char(M);
M_str = "MM = " + M_str + ";";

H_str = char(H);
H_str = "HH = " + H_str + ";";

J_c_SS_str = char(J_c_SS);
J_c_SS_str = "J_c_SS = " + J_c_SS_str + ";";
dJ_c_SS_str = char(dJ_c_SS);
dJ_c_SS_str = "dJ_c_SS = " + dJ_c_SS_str + ";";

J_c_DS_str = char(J_c_DS);
J_c_DS_str = "J_c_DS = " + J_c_DS_str + ";";
dJ_c_DS_str = char(dJ_c_DS);
dJ_c_DS_str = "dJ_c_DS = " + dJ_c_DS_str + ";";

dxG_str = char(dxG);
dxG_str = "dXG = " + dxG_str + ";";

dzG_str = char(dzG);
dzG_str = "dzG = " + dzG_str + ";";

% Specify the file name and open the text file for writing
filename = 'symbolic_matrix.txt';
fileID = fopen(filename, 'w');

% Check if the file was opened successfully
if fileID == -1
    error('Unable to open the file for writing.');
end

% Write the string representation of the matrix to the file
fprintf(fileID, '%s', M_str);
fprintf(fileID, '\n');
fprintf(fileID, '%s', H_str);
fprintf(fileID, '\n');
fprintf(fileID, '%s', J_c_DS_str);
fprintf(fileID, '\n');
fprintf(fileID, '%s', dJ_c_DS_str);
fprintf(fileID, '\n');
fprintf(fileID, '%s', dxG_str);
fprintf(fileID, '\n');
fprintf(fileID, '%s', dzG_str);
fprintf(fileID, '\n');

% Close the file
fclose(fileID);

disp('Symbolic matrix has been written to the file successfully.');

