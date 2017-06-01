function draw_camera(K, R, t)

alpha_c = K(1, 2);
fc(1) = K(1, 1);
fc(2) = K(2, 2);
cc(1) = K(1, 3);
cc(2) = K(2, 3);
nx = 640;
ny = 480;
dX = 30;
dY = 30;

IP = 5*dX*[1 -alpha_c 0;0 1 0;0 0 1]*[1/fc(1) 0 0;0 1/fc(2) 0;0 0 1]*[1 0 -cc(1);0 1 -cc(2);0 0 1]*[0 nx-1 nx-1 0 0 ; 0 0 ny-1 ny-1 0;1 1 1 1 1];
BASE = 5*dX*([0 1 0 0 0 0;0 0 0 1 0 0;0 0 0 0 0 1]);
IP = reshape([IP;BASE(:,1)*ones(1,5);IP],3,15);

IP = R' * (IP - t * ones(1, 5));
BASE = R' * (IP - t * ones(1, 15));

plot3(BASE(1,:),BASE(3,:),-BASE(2,:),'b-','linewidth',2');
hold on;
plot3(IP(1,:),IP(3,:),-IP(2,:),'r-','linewidth',2);
text(BASE(1,2),BASE(3,2),-BASE(2,2),'X','HorizontalAlignment','center','FontWeight','bold');
text(BASE(1,6),BASE(3,6),-BASE(2,6),'Z','HorizontalAlignment','center','FontWeight','bold');
text(BASE(1,4),BASE(3,4),-BASE(2,4),'Y','HorizontalAlignment','center','FontWeight','bold');
text(BASE(1,1),BASE(3,1),-BASE(2,1),'Left Camera','HorizontalAlignment','center','FontWeight','bold');

end