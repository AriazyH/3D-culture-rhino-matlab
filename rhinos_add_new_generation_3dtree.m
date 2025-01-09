%% load coordinates from rhino
%% 3 trees
X=[1.7333e-10
0.526985
-0.526985
1.407451
0.806743
0.652691
1.333671
0.748639
0.530565
-0.530564
-0.748638
-1.33367
-0.65269
-0.806743
-1.407451
4.0
4.506236
3.493764
5.503882
4.89398
4.877445
5.13096
4.60023
4.259675
3.740325
3.39977
2.86904
3.122556
3.106021
2.496119
-1.417e-9
0.518693
-0.518694
1.471962
0.861685
0.784856
1.227716
0.669292
0.380244
-0.380246
-0.669293
-1.227716
-0.784857
-0.861686
-1.471964
    ];

Y=[1.1084e-10
-0.029594
0.029594
0.579937
0.473663
1.054126
-0.733871
-0.561009
-1.120574
1.120574
0.561009
0.733871
-1.054125
-0.473663
-0.579936
2.0
1.85047
2.14953
2.242131
2.27657
2.877794
0.9796
1.282076
0.786328
3.213673
2.717924
3.020401
1.122206
1.72343
1.75787
4.0
3.902065
4.097936
4.391934
4.36495
4.961474
3.098335
3.345984
2.818541
5.181459
4.654017
4.901666
3.038527
3.635051
3.608067
];

Z=[0.002171
1.426422
1.426422
2.503662
2.154185
2.504232
2.503662
2.154185
2.504232
2.504232
2.154185
2.503662
2.504232
2.154185
2.503662
0.004335
1.42836
1.42836
2.503675
2.155759
2.504238
2.503675
2.155759
2.504238
2.504238
2.155759
2.503675
2.504238
2.155759
2.503675
0.004335
1.42836
1.42836
2.503675
2.155759
2.504238
2.503675
2.155759
2.504238
2.504238
2.155758
2.503675
2.504238
2.155758
2.503675
];

% make points
pt = [X(:), Y(:), Z(:)];
tree1= pt(1:15,:);
tree2= pt(16:30,:);
tree3= pt(31:45,:);
Points =tree1;


%% add new generation
% organize input points so tops points are the daughters 
Points = sortrows(Points, 3);

% seperate layers
grandparent=Points(1,:);
parents=Points(2:3, :);
parents=sortrows(parents, 1);
sisters=Points(4:7, :);
%sort by x to organize 
sisters=sortrows(sisters, 1);
tips=Points(8:15, :);
%sort by x to organize
tips=sortrows(tips, 1);

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
    tips(2,:)
    sisters(2,:)
    tips(3,:)
    sisters(2,:)
    tips(4,:)
    sisters(3,:)
    tips(5,:)
    sisters(3,:)
    tips(6,:)
    sisters(4,:)
    tips(7,:)
    sisters(4,:)
    tips(8,:)]; 
 

% after 90 rotation
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


% find the tips of the first generation
[~, sortedIndices] = sort(Points(:,3), 'descend'); % Sort in descending order

% Get the top 8 highest points and their original indices
topIndices = sortedIndices(1:8); % Indices of the top 8 points
knownPoints = Points(topIndices, :); % Top 8 points

% Define the new branch length 
distance = 0.5;

% Number of new points to generate (bifurcation)
numPoints = 2;

% Preallocate an array for new points
newPoints = zeros(numPoints, 3);

% Preallocate a cell array to store new points for each known point
newPointsCell = cell(size(knownPoints, 1), 1);

% Generate new points for each known point
for k = 1:size(knownPoints, 1)
    % Get the current known point
    knownPoint = knownPoints(k, :);
    
    % Generate two new points
    for i = 1:numPoints
        % Calculate angles for evenly spaced points
        theta = 2 * pi * (i - 1) / numPoints; % azimuthal angle
        %  branch angle (30 degrees in radians)
        phi = pi/6; % polar angle (fixed to create points on a sphere)
        
        % Convert spherical coordinates to Cartesian coordinates
        x = knownPoint(1) + distance * sin(phi) * sin(theta);
        y = knownPoint(2) + distance * sin(phi) * cos(theta);
        z = knownPoint(3) + distance * cos(phi);
        
    % Store the new rotated point in the matrix
    newPoints((k-1)*numPoints + i, :) = [x, y, z];
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

