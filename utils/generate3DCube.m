% INPUT ARGUMENTS
%   - Line length:
% OUTPUT ARGUMENTS
%   -p3d: (nx3) array of n 3D points 
function cube = generate3DCube(length)

p3d = generateCheckerboardPoints([5 5], length);
p3d_hole = length.*[0 0
            1 0
            2 0
            3 0
            0 1
            3 1
            0 2
            3 2
            0 3
            1 3
            2 3
            3 3];

% Add a z-coordinate to the world points.
zCoord = zeros(size(p3d,1),1);
p3d = [p3d zCoord];

zCoord = zeros(size(p3d_hole,1),1);
p3d_hole = [p3d_hole zCoord];

% shift the center of the bottom to (0,0)
p3d = p3d - repmat(1.5*[length length 0],size(p3d,1),1);
p3d_hole = p3d_hole - repmat(1.5*[length length 0],size(p3d_hole,1),1);

% construct the whold cube 
cube = p3d;
for i = 1:2
    cube = [cube; p3d_hole+repmat(i*[0 0 length], size(p3d_hole,1),1)];
end
cube = [cube; p3d+repmat(3*[0 0 length], size(p3d,1),1)];
cube = cube - [0, 0, 1.5 * length];
% cube = [length, length, length;
%         -length, -length, length;
%         length, -length, length;
%         -length, length, length;
%         length, length, -length;
%         -length, -length, -length;
%         length, -length, -length;
%         -length, length, -length];
