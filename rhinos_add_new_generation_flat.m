%% load coordinates from rhino
%% 3 trees
X=[-0.000024
0.922243
-0.921228
0.734227
0.387171
-0.307098
2.22713
1.576375
1.397113
-1.810225
-1.743467
-2.274906
-0.067771
-0.739995
-1.274175
4.00009
4.860434
3.164946
6.182968
5.525502
5.353729
5.671848
5.011844
4.475222
4.054552
3.379615
2.872351
3.308575
2.877258
2.191321
-0.000042
0.966774
-1.001024
1.954412
1.264264
0.780577
1.575486
0.96272
0.330535
-0.774993
-1.275641
-1.962991
-1.185716
-1.514135
-2.201178
    ];

Y=[-0.000034
0.380937
-0.398542
1.690903
1.091469
1.069342
0.021862
-0.221816
-0.893259
0.635211
-0.056798
-0.505318
-1.446046
-1.269997
-1.715084
2.000003
1.478269
2.550031
1.84431
2.070685
2.743821
0.380794
0.600108
0.158171
3.578268
3.413499
3.888431
1.16418
1.708282
1.610348
4.000066
4.261335
3.954322
5.183381
5.099605
5.598963
3.04579
3.371922
3.083361
5.28277
4.800501
4.90533
2.614606
3.227425
3.334529
];

Z=[0.000231
1.005235
1.004697
1.005
1.005175
1.00527
1.005
1.005
1.005
1.005
1.004999
1.005
1.005
1.004999
1.005
0.000494
1.004495
1.005062
1.005
1.004999
1.005
1.005
1.004999
1.005
1.005
1.005
1.005
1.00533
1.005171
1.005
0.000444
1.00492
1.004871
1.005
1.005
1.005
1.005
1.005193
1.005575
1.005
1.005
1.005
1.005
1.005
1.005
];

% make points
pt = [X(:), Y(:), Z(:)];
tree1= pt(1:15,:);
tree2= pt(16:30,:);
tree3= pt(31:45,:);
Points =tree3;

%% reorganize the tree
% organize input points so tops points are the daughters 
% seperate layers
grandparent=Points(1,:);
parents=Points(2:3, :);

% sister indices
sisterindices=[5,8,11,14];
sisters=Points(sisterindices,:);

% from 3D data, daughter indices are 
% 4,7,12,15,6,9,10,13
daughterindices=[4,7,12,15,6,9,10,13];
tips=Points(daughterindices,:);

%match points
%have to switch point 2/3 and 6/7 in tips 
%Swap points (rows) 2 and 3
temp1 = tips(2, :);      % Temporarily store row 2
tips(2, :) = tips(3, :); % Move row 3 to position of row 2
tips(3, :) = temp1;       % Move original row 2 to position of row 3

% Swap points (rows) and 5&7 
temp2 = tips(6, :);      % Temporarily store row 6
tips(6, :) = tips(7, :); % Move row 7 to position of row 6
tips(7, :) = temp2;       % Move original row 6 to position of row 7


% add sister nodes to tips 
tipsorg=[sisters(1,:)
    tips(1,:)
    sisters(1,:)
    tips(5,:)
    sisters(2,:)
    tips(3,:)
    sisters(2,:)
    tips(7,:)
    sisters(3,:)
    tips(2,:)
    sisters(3,:)
    tips(6,:)
    sisters(4,:)
    tips(4,:)
    sisters(4,:)
    tips(8,:)]; 
 
% add parent nodes to sisters
% sistersorg=[parents(1,:)
%     sisters(1,:)
%     parents(1,:)
%     sisters(2,:)
%     parents(2,:)
%     sisters(3,:)
%     parents(2,:)
%     sisters(4,:)
%     ];

% % after 90 rotation
sistersorg=[parents(1,:)
    sisters(1,:)
    parents(1,:)
    sisters(2,:)
    parents(2,:)
    sisters(3,:)
    parents(2,:)
    sisters(4,:)
    ];

%reassemble the points matrix
Points=[grandparent;parents(1,:);grandparent;parents(2,:);sistersorg;tipsorg];

% Plot the points
figure;
plot3(Points(:,1),Points(:,2),Points(:,3), 'o', 'MarkerSize', 8); % Plot points as circles
hold on;

% Connect the points with lines
plot3(Points(:,1),Points(:,2),Points(:,3), '-k', 'LineWidth', 2); % Connect points with black lines

% Labels and view adjustments
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
title('3D Object with Connected Points');
grid on;
axis equal;

%% add new generation
knownPoints = tips; % Top 8 points
%find their indices 
% Initialize an array for indices
topIndices = zeros(size(knownPoints, 1), 1);
% Loop through the points to find their indices
for i = 1:size(knownPoints, 1)
    topIndices(i) = find(ismember(Points, knownPoints(i, :), 'rows'));
end

% create direction vector 
vectors = zeros(size(knownPoints, 1), size(Points, 2));
xaxis=[1, 0, 0];
dotProduct = zeros();
magnitudeV = zeros();
thetavector = zeros();

% Calculate vectors and angle for each daughter
for i = 1:size(knownPoints,1)
    vectors(i, :) = Points(topIndices(i), :) - Points(topIndices(i)-1, :);
    dotProduct(i, :) = dot(vectors(i, :), xaxis);
    magnitudeV(i, :) = norm(vectors(i, :));
    thetavector(i, :) = acos(dotProduct(i, :) / magnitudeV(i, :));
end


% Define the new branch length 
distance = 0.5;
% Number of new points to generate (bifurcation)
numPoints = 2;
% Fixed polar angles (in radians)
theta = [-30, 30] * pi / 180; % Azimuthal angles: 0° and +30°

% Preallocate for new points
newPoints = [];


% Loop through each point in the list
for i = 1:size(knownPoints, 1)
    % Extract the current point
    currentPoint = knownPoints(i, :);
    % 
    % %conditions for tree1
    % if currentPoint(2)<parents(2,2)
    %     thetavector(i, :)=2*pi-thetavector(i, :);
    % end

    %  %conditions for tree2
    % if currentPoint(1)>parents(2,1) && currentPoint(2)<2
    %     thetavector(i, :)=2*pi-thetavector(i, :);
    % end

  %conditions for tree3
    if  currentPoint(2)<sisters(4,2)
        thetavector(i, :)=2*pi-thetavector(i, :);
    end

    % Calculate new points at the specified angles
    for t = theta
        x = currentPoint(1) + distance * cos(t+thetavector(i, :));
        y = currentPoint(2) + distance * sin(t+thetavector(i, :));
        z = currentPoint(3) ; % z remains unchanged
        newPoints = [newPoints; x, y, z];
    end
end


% % insertion sites
insert=topIndices;
%start at point 28 in the original 
branch1=[Points(insert(8), :);newPoints(15, :);Points(insert(8), :);newPoints(16, :)]; 
branch2=[Points(insert(7), :);newPoints(13, :);Points(insert(7), :);newPoints(14, :)];
branch3=[Points(insert(6), :);newPoints(11, :);Points(insert(6), :);newPoints(12, :)]; 
branch4=[Points(insert(5), :);newPoints(9, :);Points(insert(5), :);newPoints(10, :)];
branch5=[Points(insert(4), :);newPoints(7, :);Points(insert(4), :);newPoints(8, :)]; 
branch6=[Points(insert(3), :);newPoints(5, :);Points(insert(3), :);newPoints(6, :)]; 
branch7=[Points(insert(2), :);newPoints(3, :);Points(insert(2), :);newPoints(4, :)]; 
branch8 = [Points(insert(1), :);newPoints(1, :);Points(insert(1), :);newPoints(2, :)];

newgeneration = [Points;branch1; branch2; branch3; branch4; branch5; branch6; branch7; branch8];

% Plot the known points and the new points
figure;
plot3(newgeneration(:,1),newgeneration(:,2),newgeneration(:,3), 'o', 'MarkerSize', 8); % Plot points as circles
hold on;

% Connect the points with lines
plot3(newgeneration(:,1),newgeneration(:,2),newgeneration(:,3), '-k', 'LineWidth', 2); % Connect points with black lines

% Labels and view adjustments
xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
title('3D Object with next gen');
grid on;
axis equal;