%% ============================================================
%  BVH-BASED HUMANOID RECONSTRUCTION & ANIMATION (SINGLE FILE)
%  Uses joint angles extracted from a BVH file (relative to neck)
% ============================================================

clc;
clear;
close all;

%% ================= USER SETTINGS =============================
csvFile = 'Capoeira_joint_angles_relative_to_neck.csv'; % input
fps = 30;          % animation speed
frameStep = 3;     % skip frames for speed

%% ================= LOAD ANGLE DATA ===========================
data = readmatrix(csvFile);
time = data(:,2);

jointNames = { ...
    'Neck', ...
    'RightArm','RightForeArm','RightHand', ...
    'LeftArm','LeftForeArm','LeftHand', ...
    'RightUpLeg','RightLeg','RightFoot', ...
    'LeftUpLeg','LeftLeg','LeftFoot'};

numJoints = length(jointNames);

Angles = struct();
for j = 1:numJoints
    idx = 3 + (j-1)*3;
    Angles.(jointNames{j}).rx = deg2rad(data(:,idx+1));
    Angles.(jointNames{j}).ry = deg2rad(data(:,idx+2));
    Angles.(jointNames{j}).rz = deg2rad(data(:,idx+3));
end

%% ================= HUMANOID LINK LENGTHS =====================
L.shoulder = 0.20;
L.elbow    = 0.30;
L.arm      = 0.25;
L.hip      = 0.15;
L.thigh    = 0.40;
L.leg      = 0.40;
L.body     = 0.50;

%% ================= ANIMATION LOOP =============================
figure('Color','w');

for f = 1:frameStep:length(time)
    clf; hold on; axis equal;
    
    J = humanoidFK(f, Angles, L);
    
    % Upper body
    drawSeg(J.Neck, J.RShoulder,'r');
    drawSeg(J.RShoulder, J.RElbow,'g');
    drawSeg(J.RElbow, J.RWrist,'b');

    drawSeg(J.Neck, J.LShoulder,'r');
    drawSeg(J.LShoulder, J.LElbow,'g');
    drawSeg(J.LElbow, J.LWrist,'b');

    % Lower body
    drawSeg(J.Neck, J.RHip,'k');
    drawSeg(J.RHip, J.RKnee,'g');
    drawSeg(J.RKnee, J.RAnkle,'b');

    drawSeg(J.Neck, J.LHip,'k');
    drawSeg(J.LHip, J.LKnee,'g');
    drawSeg(J.LKnee, J.LAnkle,'b');

    view(40,25);
    xlim([-1 1]); ylim([-1 1]); zlim([-1.2 0.8]);
    grid on;
    title(sprintf('Humanoid Reconstruction – Frame %d', f));
    pause(1/fps);
end

disp('Animation complete');

%% ================= FORWARD KINEMATICS ========================
function J = humanoidFK(f, A, L)

p0 = [0;0;0];

R0 = rotZ(A.Neck.rz(f))*rotY(A.Neck.ry(f))*rotX(A.Neck.rx(f));

% Shoulders
pRS = p0 + R0*[0;-L.shoulder;0];
pLS = p0 + R0*[0; L.shoulder;0];

RrA = R0 * rotZ(A.RightArm.rz(f))*rotY(A.RightArm.ry(f))*rotX(A.RightArm.rx(f));
RlA = R0 * rotZ(A.LeftArm.rz(f))*rotY(A.LeftArm.ry(f))*rotX(A.LeftArm.rx(f));

pRE = pRS + RrA*[L.elbow;0;0];
pLE = pLS + RlA*[L.elbow;0;0];

RrF = RrA * rotY(A.RightForeArm.ry(f));
RlF = RlA * rotY(A.LeftForeArm.ry(f));

pRW = pRE + RrF*[L.arm;0;0];
pLW = pLE + RlF*[L.arm;0;0];

% Hips
pRH = p0 + R0*[0;-L.hip;-L.body];
pLH = p0 + R0*[0; L.hip;-L.body];

RrT = R0 * rotZ(A.RightUpLeg.rz(f))*rotY(A.RightUpLeg.ry(f))*rotX(A.RightUpLeg.rx(f));
RlT = R0 * rotZ(A.LeftUpLeg.rz(f))*rotY(A.LeftUpLeg.ry(f))*rotX(A.LeftUpLeg.rx(f));

pRK = pRH + RrT*[L.thigh;0;0];
pLK = pLH + RlT*[L.thigh;0;0];

RrL = RrT * rotY(A.RightLeg.ry(f));
RlL = RlT * rotY(A.LeftLeg.ry(f));

pRA = pRK + RrL*[L.leg;0;0];
pLA = pLK + RlL*[L.leg;0;0];

J = struct( ...
    'Neck',p0,...
    'RShoulder',pRS,'RElbow',pRE,'RWrist',pRW,...
    'LShoulder',pLS,'LElbow',pLE,'LWrist',pLW,...
    'RHip',pRH,'RKnee',pRK,'RAnkle',pRA,...
    'LHip',pLH,'LKnee',pLK,'LAnkle',pLA);
end

%% ================= DRAW SEGMENT ==============================
function drawSeg(p1,p2,color)
plot3([p1(1) p2(1)], ...
      [p1(2) p2(2)], ...
      [p1(3) p2(3)], ...
      color,'LineWidth',3);
end

%% ================= ROTATION MATRICES =========================
function R = rotX(t)
R = [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
end

function R = rotY(t)
R = [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
end

function R = rotZ(t)
R = [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
end
