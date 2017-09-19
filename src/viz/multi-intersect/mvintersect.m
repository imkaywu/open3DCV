%_____________________________________________ MULTI PHOTO INTERSECTION ________________________________________
% the mathematical model is based on the collinearity equations in Photogrammetry and machine vision topics. 
% Input: 1-Orienation of the cameras "three angles [rad] and three coordinates[m] for each camera"
%        2- focal length f [mm] - assume xo=yo=lens distortion=0
%        3- measured photo coordinates in [mm] in both photos
% Note: for pixel coordinates: transform to p.p. system by knowing the pixel size and image dimensions       
% Output: 
% 3D metric coordinates of the image points by least square adjustment
% code prepared by [Bashar S. Alsadik] 2012.  
%____________________________________________________________________________________________________________
function[XYZ,sx,sy,sz]=mvintersect(wpk,f,xp,yp)
 
  format short g 
  XYZ=[0 0 0 ];
  
 no=size(wpk,1); % no. of images match
   ext=wpk ;
  
for i=1:no

omega(i,1)=ext(i,1);phi(i,1)=ext(i,2);kappa(i,1)=ext(i,3);xo(i,1)=ext(i,4);yo(i,1)=ext(i,5);zo(i,1)=ext(i,6);
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Rotation Matrix M %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mw=[1 0 0;0 cos(omega(i,1)) sin(omega(i,1)) ;0 -sin(omega(i,1)) cos(omega(i,1))];
mp=[cos(phi(i,1)) 0 -sin(phi(i,1));0 1 0;sin(phi(i,1)) 0 cos(phi(i,1))];
mk=[cos(kappa(i,1)) sin(kappa(i,1)) 0;-sin(kappa(i,1)) cos(kappa(i,1)) 0;0 0 1];
m=mk*mp*mw;   
   m11(i,1)=  m(1,1);
   m12(i,1)=  m(1,2);
   m13(i,1)=  m(1,3);
   m21(i,1)=  m(2,1);
   m22(i,1)=  m(2,2);
   m23(i,1)=  m(2,3);
   m31(i,1)=  m(3,1);
   m32(i,1)=  m(3,2);
   m33(i,1)=  m(3,3);
end
xl=xp;yl=yp;B=[0 0 0];F=0;
[xp,yp];
for i=1:size(xp,1)
    b11(i,1)=(xl(i,1)*m31(i,1)+(f*m11(i,1)));
    b12(i,1)=(xl(i,1)*m32(i,1)+(f*m12(i,1)));
    b13(i,1)=(xl(i,1)*m33(i,1)+(f*m13(i,1)));
    b21(i,1)=(yl(i,1)*m31(i,1)+(f*m21(i,1)));
    b22(i,1)=(yl(i,1)*m32(i,1)+(f*m22(i,1)));
    b23(i,1)=(yl(i,1)*m33(i,1)+(f*m23(i,1)));
    fx1(i,1)=-((-f*m11(i,1)-xl(i,1)*m31(i,1))*xo(i,1)-(f*m13(i,1)+xl(i,1)*m33(i,1))*zo(i,1)-(f*m12(i,1)+xl(i,1)*m32(i,1))*yo(i,1)) ;
    fy1(i,1)=-((-f*m21(i,1)-yl(i,1)*m31(i,1))*xo(i,1)-(f*m23(i,1)+yl(i,1)*m33(i,1))*zo(i,1)-(f*m22(i,1)+yl(i,1)*m32(i,1))*yo(i,1)) ;
    B=[B;[b11(i,1) b12(i,1) b13(i,1);b21(i,1) b22(i,1) b23(i,1)]];
    F=[F;fx1(i,1);fy1(i,1)];
end
F(1,:)=[];B(1,:)=[];
 ff=F;b=B; 
 
%   [B F];
 
 %  %%%%%%%%%%%%%%%%%%%%%   least square     %%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
btb=inv(b'*b); 
btf=b'* ff;
delta=btb*btf  ;
res=b*delta-ff;
sigm=(res'*res)/(size(b,1)-size(b,2));
vcov=  (btb);

sx=sqrt( (vcov(1,1)));
sy=sqrt( (vcov(2,2)));
sz=sqrt( (vcov(3,3)));
 
j=1;
xx = delta(j,1);
yy = delta(j+1,1);
zz = delta(j+2,1);
 
  XYZ=[XYZ;xx,yy,zz];b=[];ff=[];ql=[];sl=[];rl=[]; 
 
XYZ(1,:)=[];
