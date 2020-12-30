function plot_3d_cube(theta,varargin) %%theta in radian

H=[0 0.3 0 0.3 0 0.3 0 0.3; 0 0 0.3 0.3 0 0 0.3 0.3; 0 0 0 0 0.3 0.3 0.3 0.3]; %Vertices of the cube
S=[1 2 4 3; 1 2 6 5; 1 3 7 5; 3 4 8 7; 2 4 8 6; 5 6 8 7]; %Surfaces of the cube
figure(1)
hold on
H1 = zeros(size(S,1),4) ;
H2 = zeros(size(S,1),4) ;
H3 = zeros(size(S,1),4) ;
for i=1:size(S,1)    
    Si=S(i,:); 
   fill3(H(1,Si),H(2,Si),H(3,Si),'g','facealpha',0.6)
end
axis equal, axis on, hold off, view(20,10)
%% Rotation along x, y and z axes
if ~exist('theta','var')
  th = linspace(0,2*pi,10) ; 
else
  th = linspace(0,theta,10) ; %Specify the range of theta upto which rotation needs to be done
end
hold off
figure(2)

for i = 1:length(th)
    Rx = [1 0 0 ; 0 cos(th(i)) -sin(th(i)) ; 0 sin(th(i)) cos(th(i))] ; %rotation matrix for rotation along x-axis
    Ry = [cos(th(i)) 0 sin(th(i)) ; 0 1 0 ; -sin(th(i)) 0 cos(th(i))] ; %rotation matrix for rotation along y-axis
    Rz = [cos(th(i)) -sin(th(i)) 0 ; sin(th(i)) cos(th(i)) 0 ; 0 0 1];  %rotation matrix for rotation along z-axis
    %% Rotae the vertices 
    H1 = zeros(size(H)) ;
    for j = 1:size(H,2)
        H1(:,j) = Rx*H(:,j) ;       %make changes to rotate the particular axis
    end
    for k=1:size(S,1)    
        Si=S(k,:); 
        fill3(H1(1,Si),H1(2,Si),H1(3,Si),'g','facealpha',0.6)
        hold on
    end
    drawnow
    pause(1)
    hold off
end

end
