
%_____________________________________________ MULTI-VIEW INTERSECTION ____________________________________________________
% the mathematical model is based on the collinearity equations in
% Photogrammetry and machine vision topics. 
% Input: 1- orienation of the multi cameras "three angles [deg.] and three coordinates[m]"
%        2- camera calibration parameters (or EXIF data)  [height;width;psize;focal length] 
% 
% Output: 
% 3D metric coordinates of the image points by least square adjustment
% code prepared by PhD. Bashar S. Alsadik  
% University of Twente  - ITC - Netherlands - 2013
%__________________________________________________________________________________________________________________________

clc
close all
clear
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
wpk=    [  71       17.344      -6.5556       1.4958       105.03       39.584      -11.184
           73       14.458      -8.9916       1.6298       102.65       28.551      -5.3021
           75       7.3108      -11.304       1.2747       101.39       3.4826     0.011981
           77       1.2008      -11.041       1.2178       100.73      -21.638         3.83
           79      -2.6456      -7.6125       1.2657        103.1      -36.118       10.769];
       
Tx= wpk(:,2);Ty=wpk(:,3); Tz=wpk(:,4);w2=(wpk(:,5)*pi/180) ;p2=wpk(:,6)*pi/180 ;k2= wpk(:,7)*pi/180;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% images load %%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pathname= fileparts(mfilename('fullpath')); %%pathfile of images
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:1:size(Tx,1)
      I1=imread([pathname ,'/IMG_00',num2str(wpk(i)),'.jpg']);
      imshow(I1)
      title('PLEASE TICK ONE POINT AND PRESS ENTER!!!!')
      [x(i,1),y(i,1)] = ginput; %%%%% interactive image observation 
hold on
close all
end

%%%%%%%%%%%%% load the 3d figure %%%%%%%%%%%%%%%%
openfig (sprintf('%s/pc.fig', fileparts(mfilename('fullpath'))));hold on
%%%%%%%%%%%%%%%%% IO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f=18; 
height=size(I1,2);width=size(I1,1);psize=0.0087246;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 xp =(x(:,1)-(height/2))*psize ; %TRANSFORMED TO PP.
 yp =((width/2)-y(:,1))*psize;
 EO=[w2,p2,k2,Tx,Ty,Tz];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[G,sx,sy,sz]=mvintersect(EO,f,xp,yp);
for i=1:size(x,1)
    plot3([Tx((i),1) G(1,1)] ,[Ty((i),1) G(1,2)],[Tz((i),1) G(1,3)],'r-');hold on
    plot3(G(1,1) , G(1,2), G(1,3),'ob','MarkerEdgeColor','k',...
                'MarkerFaceColor','r',...
                'MarkerSize',5);hold on
end
uicontrol('style','text','position',[110 560 370 27],'backgroundcolor',[.96 .92 .92],'FontWeight','bold','Fontsize',10,...
 'HorizontalAlignment','center','String','INTERSECTION POINT X,Y,Z');
text1=uicontrol('style','text','position',[110 550 370 15],'backgroundcolor',[.95 .95 .95],'FontWeight','bold','HorizontalAlignment','center');
G=round(G*1000)/1000; 
set( text1,'String', num2str(G)); 
uicontrol('style','text','position',[110 520 370 27],'backgroundcolor',[.96 .92 .92],'FontWeight','bold','Fontsize',10,...
 'HorizontalAlignment','center','String','ESTIMATED ERROR Sx,Sy,Sz [cm]');
text1=uicontrol('style','text','position',[110 510 370 15],'backgroundcolor',[.95 .95 .95],'FontWeight','bold','HorizontalAlignment','center');
s=round([sx,sy,sz]*1000)/1000;s=s*100;
set( text1,'String', num2str(s)); 
clc
 disp(' THANKS FOR USING MY CODE ...........')
 disp(' CLOSE RANGE PHOTOGRAMMETRY..........')
 disp(' BASHAR ALSADIK......................')

 

 