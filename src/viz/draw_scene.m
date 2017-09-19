% generate N synthetic cameras
close all;
addpath(genpath('/Users/BlacKay/Documents/Projects/Open Source/SFMedu2'))

N = 15;
ang = 2 * pi / N;
radius = 3;
f = 1400;
w = 640;
h = 480;

for i = 1 : N
    cos_ang = cos(i * ang);
    sin_ang = sin(i * ang);
    R = [-sin_ang,  cos_ang, 0;...
          0,        0,      -1;... 
         -cos_ang, -sin_ang, 0];
    c = radius * [cos_ang, sin_ang, 0]';
    
    % implementation 1
%     Rt = [R, -R*c];
%     drawCamera(Rt, w, h, f, 0.05, 2);

    % implementation 2
%     K = [f 0 w/2; 0 f h/2; 0 0 1];
%     t = -R*c;
%     draw_camera(K, R, t);
    
    % implementation 3
    K = [f 0 w/2; 0 f h/2; 0 0 1];
    camera(i).w = w;
    camera(i).h = h;
    camera(i).K = K;
    camera(i).R = R;
    camera(i).c = c; % camera center in world coordinate system
end

showcamera(camera);

xlabel('x'); ylabel('y'); zlabel('z');
axis equal;
view(0, 30);
title('implementation 3');
