function SPM_Visual(Um,Wm,Vm,EA)




Qm = Rz(EA(1))*Ry(EA(2))*Rx(EA(3));

% Drawing 
 %Pyramid Dimension
 Length_Bottom_Pyramid_Edge = 1;  %[1x1] [m] Length of the Bottom Pyramid's Edge
 Length_Top_Pyramid_Edge = 1;  %[1x1] [m] Length of the Top Pyramid's Edge
 RotationCenter = [0 0 0.1311];
wmast = Wm';
wm = Wm';
vm = Vm';
um = Um';


% figure(2)
% clf

DrawRotationCenter( RotationCenter,Qm,wmast,wm) % Draw Coordinate System at the rotation center
DrawTopPyramid2(Length_Top_Pyramid_Edge*vm) % Draw Top Pyramid
DrawArc( Length_Bottom_Pyramid_Edge*um , Length_Bottom_Pyramid_Edge*wm )% Draw Base arms
DrawArc( Length_Bottom_Pyramid_Edge*wm,Length_Top_Pyramid_Edge*vm) % Draw Top arms
% title(['Time [sec]  ',(num2str(round(times,3))),',  Joint Angles [deg]  ' (num2str(round(tetav'*rd2dg))), ',  Euler Angles [deg]  ' (num2str(round(eav)))])
grid on
axis equal



end


function [  ] = DrawRotationCenter( Place_RotationCenter,qm,wm1,wm2)
%-- Draw Coordinate Axes of Space frame--%
rc = Place_RotationCenter;
lSs = 0.6; lBs = 0.5;
VectorArrow3D(rc,rc+[lSs 0 0],'m');hold on
%text(0.5,0,0,'X_S','FontSize',10)
VectorArrow3D(rc,rc+[0 lSs 0],'m');hold on
%text(0,0.5,0,'Y_S','FontSize',10)
VectorArrow3D(rc,rc+[0 0 lSs],'m');hold on
%text(0,0,0.5,'Z_S','FontSize',10)
%--------------------------%
%-- Draw Coordinate Axes of Body frame--%
VectorArrow3D(rc,rc+lBs*(qm(:,1))','k');hold on
VectorArrow3D(rc,rc+lBs*(qm(:,2))','k');hold on
VectorArrow3D(rc,rc+lBs*(qm(:,3))','k');hold on
%-- Draw Initial and Current Positions of Lower Arms on Horizontal plane--%
dum1v = [wm1(1,1) wm1(1,2)]/norm([wm1(1,1) wm1(1,2)]);
VectorArrow3D([0 0 -1],[0 0 -1]+1*[dum1v 0],'r--');hold on
dum2v = [wm1(2,1) wm1(2,2)]/norm([wm1(2,1) wm1(2,2)]);
VectorArrow3D([0 0 -1],[0 0 -1]+1*[dum2v 0],'g--');hold on
dum3v = [wm1(3,1) wm1(3,2)]/norm([wm1(3,1) wm1(3,2)]);
VectorArrow3D([0 0 -1],[0 0 -1]+1*[dum3v 0],'b--');hold on
 % 
dum1v = [wm2(1,1) wm2(1,2)]/norm([wm2(1,1) wm2(1,2)]);
VectorArrow3D([0 0 -1],[0 0 -1]+1*[dum1v 0],'r');hold on
dum2v = [wm2(2,1) wm2(2,2)]/norm([wm2(2,1) wm2(2,2)]);
VectorArrow3D([0 0 -1],[0 0 -1]+1*[dum2v 0],'g');hold on
dum3v = [wm2(3,1) wm2(3,2)]/norm([wm2(3,1) wm2(3,2)]);
VectorArrow3D([0 0 -1],[0 0 -1]+1*[dum3v 0],'b');hold on

end


function [  ] = DrawTopPyramid2( v )
global RotationCenter
l=norm(v(1,:)); % the size of v(1,:) vector ( ||v(1,:)|| )
n = cross(v(1,:)-v(2,:),v(1,:)-v(3,:))/norm(cross(v(1,:)-v(2,:),v(1,:)-v(3,:))); %find the normal to the top platform - the Line Of Sight
%--Draw Top Pyramid--%
plot3(v(1,1),v(1,2),v(1,3),'ro','MarkerFaceColor', 'r','MarkerSize',10);hold on
text(v(1,1)-0.1*l,v(1,2),v(1,3),'v_1','FontSize',14)
plot3(v(2,1),v(2,2),v(2,3),'go','MarkerFaceColor', 'g','MarkerSize',10);hold on
text(v(2,1)+0.1*l,v(2,2),v(2,3),'v_2','FontSize',14)
plot3(v(3,1),v(3,2),v(3,3),'bo','MarkerFaceColor', 'b','MarkerSize',10);hold on
text(v(3,1),v(3,2)+0.15*l,v(3,3),'v_3','FontSize',14)
% plot3(RotationCenter(1),RotationCenter(2),RotationCenter(3),'ko','MarkerFaceColor', 'k','MarkerSize',10);hold on
plot3([v(1,1) v(2,1) v(3,1) v(1,1)],[v(1,2) v(2,2) v(3,2) v(1,2)],[v(1,3) v(2,3) v(3,3) v(1,3)],'-k','LineWidth',2);hold on
%plot3([v(1,1) RotationCenter(1)],[v(1,2) RotationCenter(2)],[v(1,3) RotationCenter(3)],'-k','LineWidth',2);hold on
%plot3([v(2,1) RotationCenter(1)],[v(2,2) RotationCenter(2)],[v(2,3) RotationCenter(3)],'-k','LineWidth',2);hold on
%plot3([v(3,1) RotationCenter(1)],[v(3,2) RotationCenter(2)],[v(3,3) RotationCenter(3)],'-k','LineWidth',2);hold on

plot3(mean(v(:,1)),mean(v(:,2)),mean(v(:,3)),'k*','MarkerFaceColor', 'k','MarkerSize',10);hold on % Mark the center of the top platform
%quiver3(u(1,1) u(2,1) u(3,1),RotationCenter(1),RotationCenter(2),RotationCenter(3),10,'k','MarkerFaceColor', 'k','MarkerSize',100);hold on % Draw the normal to the top platform from the center of the top platform
 VectorArrow3D([mean(v(:,1)),mean(v(:,2)),mean(v(:,3))],n,'k');hold on % Draw the normal to the top platform from the center of the top platform
end

function [  ] = DrawArc( u,w )
plot3(w(1,1),w(1,2),w(1,3),'ro','MarkerFaceColor', 'r','MarkerSize',10);hold on
plot3(w(2,1),w(2,2),w(2,3),'go','MarkerFaceColor', 'g','MarkerSize',10);hold on
plot3(w(3,1),w(3,2),w(3,3),'bo','MarkerFaceColor', 'b','MarkerSize',10);hold on
% Draw an arc between two points 
acc = 100; % accuracy: ~ lines per arc
V = Find_Arc([u(1,1) w(1,1)],[u(1,2) w(1,2)],[u(1,3) w(1,3)],acc);
hold on, plot3(V(1,:),V(2,:),V(3,:),'-r','LineWidth',5)
V = Find_Arc([u(2,1) w(2,1)],[u(2,2) w(2,2)],[u(2,3) w(2,3)],acc);
hold on, plot3(V(1,:),V(2,:),V(3,:),'-g','LineWidth',5)
V = Find_Arc([u(3,1) w(3,1)],[u(3,2) w(3,2)],[u(3,3) w(3,3)],acc);
hold on, plot3(V(1,:),V(2,:),V(3,:),'-b','LineWidth',5)
end

function V = Find_Arc(x,y,z,acc)
v1 = [x(1:end-1);y(1:end-1);z(1:end-1)]; % Vector from center to 1st point
v2 = [x(2:end);y(2:end);z(2:end)]; % Vector from center to 2nd point
r = sqrt(sum([x(1); y(1); z(1)].^2,1));
v3a = cross(cross(v1,v2),v1); % v3 lies in plane of v1 & v2 and is orthog. to v1
v3 = r*v3a./repmat(sqrt(sum(v3a.^2,1)),3,1); % Make v3 of length r
% Let t range through the inner angle between v1 and v2
tmax = atan2(sqrt(sum(cross(v1,v2).^2,1)),dot(v1,v2));
V = zeros(3,sum(round(tmax*acc))); % preallocate
k = 0; % index in v
    for i = 1:length(tmax)
        steps = round(tmax(i)*acc)+1; %Edited +1
        k = (1:steps) + k(end);
        t = linspace(0,tmax(i),steps);
        V(:,k) = v1(:,i)*cos(t)+v3(:,i)*sin(t);
    end
end


function VectorArrow3D(p0,p1,Col)
%Arrowline 3-D vector plot.
%   vectarrow(p0,p1) plots a line vector with arrow pointing from point p0
%   to point p1. The function can plot both 2D and 3D vector with arrow
%   depending on the dimension of the input
% Col --> Color of the arrow
%
%   Example:
%       3D vector
%       p0 = [1 2 3];   % Coordinate of the first point p0
%       p1 = [4 5 6];   % Coordinate of the second point p1
%       vectarrow(p0,p1)
%
%       2D vector
%       p0 = [1 2];     % Coordinate of the first point p0
%       p1 = [4 5];     % Coordinate of the second point p1
%       vectarrow(p0,p1)
%
%   See also Vectline
%   Rentian Xiong 4-18-05
%   $Revision: 1.0
  if max(size(p0))==3
      if max(size(p1))==3
          x0 = p0(1);
          y0 = p0(2);
          z0 = p0(3);
          x1 = p1(1);
          y1 = p1(2);
          z1 = p1(3);
          plot3([x0;x1],[y0;y1],[z0;z1],Col,'MarkerFaceColor', 'y','MarkerSize',10);   % Draw a line between p0 and p1
          
          p = p1-p0;
          alpha = 0.1;  % Size of arrow head relative to the length of the vector
          beta = 0.1;  % Width of the base of the arrow head relative to the length
          
          hu = [x1-alpha*(p(1)+beta*(p(2)+eps)); x1; x1-alpha*(p(1)-beta*(p(2)+eps))];
          hv = [y1-alpha*(p(2)-beta*(p(1)+eps)); y1; y1-alpha*(p(2)+beta*(p(1)+eps))];
          hw = [z1-alpha*p(3);z1;z1-alpha*p(3)];
          
          hold on
          plot3(hu(:),hv(:),hw(:),Col,'MarkerFaceColor', 'y','MarkerSize',10)  % Plot arrow head
          grid on
          xlabel('x')
          ylabel('y')
          zlabel('z')
          hold off
      else
          error('p0 and p1 must have the same dimension')
      end
  elseif max(size(p0))==2
      if max(size(p1))==2
          x0 = p0(1);
          y0 = p0(2);
          x1 = p1(1);
          y1 = p1(2);
          plot([x0;x1],[y0;y1],Col,'MarkerFaceColor', 'y','MarkerSize',10);   % Draw a line between p0 and p1
          
          p = p1-p0;
          alpha = 0.1;  % Size of arrow head relative to the length of the vector
          beta = 0.1;  % Width of the base of the arrow head relative to the length
          
          hu = [x1-alpha*(p(1)+beta*(p(2)+eps)); x1; x1-alpha*(p(1)-beta*(p(2)+eps))];
          hv = [y1-alpha*(p(2)-beta*(p(1)+eps)); y1; y1-alpha*(p(2)+beta*(p(1)+eps))];
          
          hold on
          plot(hu(:),hv(:),Col,'MarkerFaceColor', 'y','MarkerSize',10)  % Plot arrow head
          grid on
          xlabel('x')
          ylabel('y')
          hold off
      else
          error('p0 and p1 must have the same dimension')
      end
  else
      error('this function only accepts 2D or 3D vector')
  end
end
