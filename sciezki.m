function [paths,pathNames,planningTime] = sciezki (ptCloudMap,res,startposition,goalposition,maxinclineangle)
chmurka=pcdownsample(ptCloudMap,'nonuniformGridSample',10);
%%
[plane1] = pcfitplane(chmurka,0.05);

if(plane1.Parameters(1,4)<0)
tform = normalRotation(plane1,[0 0 1]);
obrocona = pctransform(chmurka,tform);
przesuniecie = rigidtform3d([0 0 0],[0 0 -plane1.Parameters(1,4)]);
przesunieta = pctransform(obrocona,przesuniecie);
else
tform = normalRotation(plane1,[0 0 1]);
obrocona = pctransform(chmurka,tform);
przesuniecie = rigidtform3d([0 0 0],[0 0 plane1.Parameters(1,4)]);
przesunieta = pctransform(obrocona,przesuniecie);
end

indices=findPointsInROI(przesunieta,[-inf inf -inf inf -0.1 inf]);
zaznaczenie=select(przesunieta,indices);

figure
pcshow(zaznaczenie, VerticalAxis='Z', VerticalAxisDir='Up', ViewPlane='XZ', AxesVisibility='on');
xlabel("X(m)");
ylabel("Y(m)");
zlabel("Z(m)");
%%
zaznaczenie=pcdenoise(zaznaczenie,"NumNeighbors",15,"Threshold",0.5);
%%
groundPtsIdx = segmentGroundSMRF(zaznaczenie,0.01,ElevationThreshold=0.3,SlopeThreshold=0.1);
ptCloudWithGround = select(zaznaczenie,groundPtsIdx);
chwdp=pcdenoise(ptCloudWithGround,"NumNeighbors",40);
elevModel = pc2dem(chwdp,res,"CornerFillMethod","max");
figure
pcshow(chwdp.Location)
title("Ground Points")

%elevModel = flip(elevModel);

figure
imagesc(elevModel)
camroll(90)
colormap(gray)
title("Digital Terrain Model")
%%
Z=elevModel./res;
[X,Y]=meshgrid(1:size(Z,2),1:size(Z,1));

% [X,Y] = meshgrid(1:(1/res):size(Zinit,2),1:(1/res):size(Zinit,1));
% Z = interp2(Xinit,Yinit,Zinit,X,Y);

% Z(isnan(Z))=0;
[gx,gy] = gradient(Z);
figure
contour(X,Y,Z)
title("Terrain Slope")
hold on
quiver(X,Y,gx,gy)
camroll(-90)
axis equal
hold off

%%
maxSlope = tand(maxinclineangle); % Max preferred slope for vehicle
slope2cost = @(x)exampleHelperGradientToCost(maxSlope,flipud(x));
% gx(isnan(gx))=0;
% gy(isnan(gy))=0;
% Z(isnan(Z))=0;

%%
Z=Z.*res;
zLayerReal = mapLayer(flipud(Z),'Resolution',1/res,LayerName="Z");
dzdxReal = mapLayer(flipud(gx),Resolution=1/res,LayerName="dzdx");
dzdyReal = mapLayer(flipud(gy),Resolution=1/res,LayerName="dzdy");
xCostReal = mapLayer(slope2cost(flipud(gx)),Resolution=1/res,LayerName="xCost");
yCostReal = mapLayer(slope2cost(flipud(gy)),Resolution=1/res,LayerName="yCost");
diagCostReal = mapLayer(slope2cost(flipud(sqrt(gx.^2+gy.^2))),Resolution=1/res,LayerName="diagCost");

terrainObstaclesReal = binaryOccupancyMap(zeros(size(Z,1),size(Z,2)),'Resolution',1/res,LayerName="terrainObstacles");
terrainData=double(getMapData(terrainObstaclesReal));
mInvalidSlope = getMapData(xCostReal) > 1 | getMapData(yCostReal) > 1 | getMapData(diagCostReal) > 1;
terrainData(mInvalidSlope) = true;
terrainData(isnan(Z))= 1;

setMapData(terrainObstaclesReal,terrainData);
% inflate(terrainObstaclesReal,0.01);
%%
figure
show(terrainObstaclesReal)
title("Obstacles and Gradient-Restricted Regions")

costMapReal = multiLayerMap({zLayerReal dzdxReal dzdyReal terrainObstaclesReal xCostReal yCostReal diagCostReal});

gWeight = 1;
hWeight = 1;

defaultPlanner = plannerAStarGrid(getLayer(costMapReal,"terrainObstacles"));

heightAwarePlanner = plannerAStarGrid(getLayer(costMapReal,"terrainObstacles"), ...
    GCostFcn=@(s1,s2)exampleHelperZHeuristic(costMapReal,gWeight,s1,s2), ...
    HCostFcn=@(s1,s2)exampleHelperZHeuristic(costMapReal,hWeight,s1,s2));

gradientAwarePlanner = plannerAStarGrid(getLayer(costMapReal,"terrainObstacles"), ...
    GCostFcn=@(s1,s2)exampleHelperGradientHeuristic(costMapReal,gWeight,s1,s2), ...
    HCostFcn=@(s1,s2)exampleHelperGradientHeuristic(costMapReal,hWeight,s1,s2));

rolloverAwarePlanner = plannerAStarGrid(getLayer(costMapReal,"terrainObstacles"), ...
    GCostFcn=@(s1,s2)exampleHelperRolloverHeuristic(costMapReal,gWeight,s1,s2), ...
    HCostFcn=@(s1,s2)exampleHelperRolloverHeuristic(costMapReal,hWeight,s1,s2));

start = world2grid(costMapReal,startposition);
goal = world2grid(costMapReal,goalposition);

% Planners return a sequence of IJ cells
paths = cell(4,1);
pathNames = ["Domyślna","Zależna od wysokości","Zależna od kąta nachylenia","Zależna od kąta przechylenia robota"];
planningTime = nan(4,1);
tic;
fprintf('Planning path %d\n', 1);
paths{1} = plan(defaultPlanner,start,goal);
planningTime(1) = toc;
fprintf('Planning path %d\n', 2);
paths{2} = plan(heightAwarePlanner,start,goal);
planningTime(2) = toc-planningTime(1);
fprintf('Planning path %d\n', 3);
paths{3} = plan(gradientAwarePlanner,start,goal);
planningTime(3) = toc-planningTime(2);
fprintf('Planning path %d\n', 4);
paths{4} = plan(rolloverAwarePlanner,start,goal);
planningTime(4) = toc-planningTime(3);
end