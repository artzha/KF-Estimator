function plotCube3Angles(roll, pitch, yaw)
 %I define the vertex of the rectangle
 vertex_matrix = [0 0 0
4 0 0
4 3 0
0 3 0
0 0 1
4 0 1
4 3 1
0 3 1];
 %I define the faces of the rectangle
faces_matrix = [1 2 6 5
2 3 7 6
3 4 8 7
4 1 5 8
1 2 3 4
5 6 7 8];
 
% subplot(1,3,1)
axis([0 5 0 5 0 5]); %axis limits
axis equal off; %serves to prevent Matlab from distorting the image to fit the window
cube = patch('Vertices',vertex_matrix,'Faces',faces_matrix,'FaceColor', 'blue'); %Create a 3d blue rectangle of the previous characteristics
rotate(cube, [1,0,0], roll); %rotate => 'object', 'rotation direction (of axis)', 'numeric value for rotation'
rotate(cube,[0,1,0], pitch);
rotate(cube, [0, 0, 1], yaw);
view(0,0); %view (0,0) places the camera completely horizontal so as not to disturb the perspective
 
end