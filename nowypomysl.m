    clear
    clc

    k=1;
    tabelakontrolna=zeros(1,40);


    filename='przejscie dlugie.bag';

    bag=rosbag(filename);

    depths=select(bag,"Topic","/device_0/sensor_0/Depth_0/image/data");
    colors=select(bag,"Topic","/device_0/sensor_1/Color_0/image/data");

    cfg = realsense.config();
    validateattributes(filename, {'char','string'}, {'scalartext', 'nonempty'}, '', 'filename', 1);
    % Tell pipeline to stream from the given rosbag file
    cfg.enable_device_from_file(filename);

    numerek=rosbag(filename);
    numerek=numerek.AvailableTopics.NumMessages(2);
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();

    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start(cfg);

    % Get streaming device's name
    dev = profile.get_device();
    streams=profile.get_streams();
    stream_profile=streams{1,1};
    stream_profile=stream_profile.as('video_stream_profile');
    raw_intrinsics=stream_profile.get_intrinsics();
    focalLength=[raw_intrinsics.fx raw_intrinsics.fy];
    principalPoint=[raw_intrinsics.ppx raw_intrinsics.ppy];
    imageSize=single([raw_intrinsics.height raw_intrinsics.width]);
    intrinsics=cameraIntrinsics(focalLength,principalPoint,imageSize);

    name = dev.get_info(realsense.camera_info.name);
    depthSensor = dev.first('depth_sensor');
    depthScale = depthSensor.get_depth_scale();

    K = [intrinsics.FocalLength(1,1) 0 intrinsics.PrincipalPoint(1,1); 0 intrinsics.FocalLength(1,2) intrinsics.PrincipalPoint(1,2); 0 0 1];
    zdjecia=cell(2,numerek);
    chmury=cell(1,numerek);
    downsample=cell(1,numerek);
    res=0.02;

    pipe.stop();
%%
%     pipe.start(cfg);
    %zdjecia=cell(2,numerek);
    depthmsgs=readMessages(depths);
    colormsgs=readMessages(colors);


    for i = 1:numerek
        depthMap = double(readImage(depthmsgs{i,1})) .* depthScale;
        colorMap = readImage(colormsgs{i,1});

        depthMap(depthMap>5)=0;
        depthMap(depthMap<0.5)=0;

        zdjecia(1,i)={depthMap};
        zdjecia(2,i)={colorMap};
        if (mod(i,10)==0),fprintf('Processing Image %d\n', i); end
%         fs = pipe.wait_for_frames();
%         pause(0.05);
%   depth = fs.get_depth_frame();
% %     if(depth.get_frame_number()<6467)
% %         fprintf('Skip %d\n', i);
% %         continue;
% %     end
%   color=fs.get_color_frame();
% 
%   depthWidth = depth.get_width();
%   depthHeight = depth.get_height();
%    
%   depthVector = depth.get_data();
%   colorVector = color.get_data();
%    
%   depthMap = double(transpose(reshape(depthVector, [depthWidth,depthHeight]))) .* depthScale;
%   colorMap = permute(reshape(colorVector',[3,color.get_width(),color.get_height()]),[3 2 1]);
%     zdjecia(1,i)={depthMap};
%     zdjecia(2,i)={depth.get_frame_number()};
%     zdjecia(3,i)={depth.get_timestamp()};
%     zdjecia(4,i)={colorMap};
%      fprintf('Processing Image %d\n', i);
%      fprintf('Frame number %d\n', zdjecia{2,i});
%      fprintf('\n');
 
    end
  %%
for i=1:length(zdjecia)
    depthMap=zdjecia{1,i};
    depthMap(depthMap>5)=0;
    len = size(depthMap,1)*size(depthMap,2);
    aa = 1:size(depthMap,2);
    bb = (1:size(depthMap,1))';
    xa = repmat(aa,size(depthMap,1),1);
    xb = repmat(bb,1,size(depthMap,2));
    x = ((xa - K(1,3)).* depthMap )/ K(1,1);
    y = ((xb - K(2,3)).* depthMap )/ K(2,2);
    x = x((1:len)');
    y = y((1:len)');
    depthMap = depthMap((1:len)');
    colorMap = zdjecia{2,i};
    %colorMap = reshape(colorMap,[],3);
    
    points3D = [x,y,depthMap];
    points3D = reshape(points3D,720,1280,3);
    points3D = pointCloud(points3D, Color=colorMap);

    downsamplik=pcdownsample(points3D,'gridAverage',0.01);
%     downsamplik=points3D;
%     downsample=pcdenoise(downsamplik,"NumNeighbors",20,"Threshold",0.4);
    chmury(1,i)={downsamplik};

    %chmury(1,i)={points3D};

    if (mod(i,10)==0),fprintf('Processing Cloud %d\n', i); end
end
clear zdjecia;
%%
    for i=1:length(chmury)
        %downsample{1,i} = pcdownsample(chmury{1,i},'gridAverage',0.01);
        %downsample{1,i} = pcdownsample(chmury{1,i},'random',0.01);
        %downsample{1,i} = pcdownsample(chmury{1,i},'nonuniformGridSample',100);
        downsample{1,i} = chmury{1,i};
        if (mod(i,10)==0),fprintf('Downsampling Cloud %d\n', i); end
    end
    clear chmury;
%%
clear vSet;

razem=downsample{1,1};
j=1;
relPose=rigidtform3d;
absPose=rigidtform3d;
vSet=pcviewset;
vSet = addView(vSet,1,absPose,PointCloud=downsample{1,1});
prevViewId = 1;
prevPtCloud = downsample{1,1};
loopDetector = scanContextLoopDetector;

rmses=zeros(1,round(length(chmury)/j));
    
regGridStep = 0.2;

% figure
% hold on
% xlabel("X(m)");
%     ylabel("Y(m)");
%     zlabel("Z(m)");
for i=2:j:460
    ptCloud=downsample{1,i};
     % Register new point cloud against the previous one.
    [relPose,~,rmse] = pcregisterndt(ptCloud,prevPtCloud,regGridStep,InitialTransform=relPose,Tolerance=[0.01 0.03]);
    %relPose = pcregistercorr(ptCloud,prevPtCloud,10,0.01);
    %relPose = pcregistericp(ptCloud,prevPtCloud,"Metric","pointToPoint");

    % Update the absolute pose.
    absPose = rigidtform3d(absPose.A * relPose.A);

    % Add new view and connection to the previous view.
    vSet = addView(vSet,i,absPose,PointCloud=ptCloud);
    vSet = addConnection(vSet,prevViewId,i,relPose);
    prevPtCloud = ptCloud;
    prevViewId = i;

    descriptor = scanContextDescriptor(ptCloud);
    addDescriptor(loopDetector,i,descriptor);
    [loopViewId,dists] = detectLoop(loopDetector,descriptor,'NumExcludedDescriptors',5);

    %ptCloud=pctransform(ptCloud,absPose);
    %razem=pcmerge(razem,ptCloud,0.02);

    rmses(i/j)=rmse;
        % A loop candidate was found

    if ~isempty(loopViewId)
        loopViewId = loopViewId(1);

        % Retrieve point cloud from view set
        loopView = findView(vSet, loopViewId);
        ptCloudOrig = loopView.PointCloud;

        % Use registration to estimate the relative pose
        [relTform, ~, rmse] = pcregisterndt(ptCloud, ptCloudOrig, ...
            regGridStep,Tolerance=[0.01 0.05]);

        acceptLoopClosure = (rmse <= 0.08);
        
        if acceptLoopClosure
            infoMat = 0.01 * eye(6);

            % Add a connection corresponding to a loop closure
            vSet = addConnection(vSet, loopViewId, i, relTform, infoMat);
        end
        rmses(i/j)=rmse;
        loopViewId=[];
    end

    
    %pcshow(razem, VerticalAxis='Z', VerticalAxisDir='Down', ViewPlane='YX', AxesVisibility='on')
    %drawnow;
    

    fprintf('Merging Cloud %d\n', i);
end

rmsem=mean(rmses);
tabelakontrolna(1,k)=regGridStep;
tabelakontrolna(2,k)=rmsem;
k=k+1;
%%
ptClouds = vSet.Views.PointCloud;
tforms   = vSet.Views.AbsolutePose;

ptCloudMap = pcalign(ptClouds,tforms);
pcshow(ptCloudMap, VerticalAxis='Y', VerticalAxisDir='Down', ViewPlane='YX', AxesVisibility='on');
%%
chmurka=pcdownsample(ptCloudMap,'nonuniformGridSample',30);
%%
figure
pcshowpair(chmurka,zaznaczenie)
xlabel("X(m)");
ylabel("Y(m)");
zlabel("Z(m)");
%%
[plane1,planeindices] = pcfitplane(chmurka,0.05);

if(plane1.Parameters(1,4)<0)
tform = normalRotation(plane1,[0 0 -1]);
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
plaszczyzna=select(przesunieta,planeindices);

%%
figure
pcshow(zaznaczenie, VerticalAxis='Z', VerticalAxisDir='Up', ViewPlane='YZ', AxesVisibility='on');
xlabel("X(m)");
ylabel("Y(m)");
zlabel("Z(m)");
%%
odszumiona=pcdenoise(zaznaczenie,"NumNeighbors",20,"Threshold",0.1);
%%
res=0.02;
groundPtsIdx = segmentGroundSMRF(zaznaczenie,0.01,ElevationThreshold=0.08,SlopeThreshold=0.05);
ptCloudWithGround = select(zaznaczenie,groundPtsIdx);
% chwdp=pcdenoise(ptCloudWithGround,"NumNeighbors",10,"Threshold",0.15);
chwdp=ptCloudWithGround;
elevModel = pc2dem(chwdp,res,"CornerFillMethod","mean");
figure
pcshow(chwdp.Location)
title("Ground Points")

%elevModel = flip(elevModel);
elevModel(abs(elevModel)<0.001)=0;
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
maxinclineangle = 35; % Degrees
prefinclineangle = 15;
maxSlope = tand(maxinclineangle); % Max preferred slope for vehicle
prefSlope = tand(prefinclineangle);
slope2cost = @(x)exampleHelperGradientToCost(maxSlope,prefSlope,flipud(x));
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
nachyleniex=gx.*(180/pi);
nachyleniey=gy.*(180/pi);
nachyleniediag=sqrt(gx.^2+gy.^2).*(180/pi);


terrainObstaclesReal = binaryOccupancyMap(zeros(size(Z,1),size(Z,2)),'Resolution',1/res,LayerName="terrainObstacles");
terrainData=double(getMapData(terrainObstaclesReal));
mInvalidSlope = isnan(getMapData(xCostReal)) & isnan(getMapData(yCostReal)) & isnan(getMapData(diagCostReal));
terrainData(mInvalidSlope) = true;
terrainData(isnan(Z))= 1;

setMapData(terrainObstaclesReal,terrainData);
inflate(terrainObstaclesReal,0.01);
%%
figure
show(terrainObstaclesReal)
title("Binarna mapa zajętości")
%%
clear costMapReal;
costMapReal = multiLayerMap({zLayerReal dzdxReal dzdyReal terrainObstaclesReal xCostReal yCostReal diagCostReal});
figure(5)

show(visualizationHelper,xCostReal,subplot(1,3,1),hold="on",title="X Slope");
surface(flipud(double(getMapData(terrainObstaclesReal))),FaceColor=[0 0 0])

show(visualizationHelper,yCostReal,subplot(1,3,2),hold="on",title="Y Slope",Threshold=1);
surface(flipud(double(getMapData(terrainObstaclesReal))),FaceColor=[0 0 0])

show(visualizationHelper,diagCostReal,subplot(1,3,3),hold="on",title="Diagonal Slope",Threshold=1);
% surface(flipud(double(getMapData(terrainObstaclesReal))),FaceColor=[0 0 0])
sgtitle("Unweighted Costs with Obstacles")
%%
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

customPlanner = plannerAStarGrid(getLayer(costMapReal,"terrainObstacles"), ...
    GCostFcn=@(s1,s2)customCost(costMapReal,20,0.1,0.1,s1,s2), ...
    HCostFcn=@(s1,s2)customCost(costMapReal,20,0.1,0.1,s1,s2));
%%
startposition = [2 10];
goalposition = [8.55 5.85];

start = world2grid(costMapReal,startposition);
goal = world2grid(costMapReal,goalposition);
%%
% Planners return a sequence of IJ cells
paths = cell(5,1);
pathNames = ["Domyślna","Zależna od wysokości","Zależna od kąta nachylenia","Zależna od kąta przechylenia robota","Własna 2","Własna 3","Własna 4","Własna 1","Własna 5","Własna 6"];
planningTime = nan(5,1);
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
%%
tic
fprintf('Planning path %d\n', 5);
paths{10} = plan(customPlanner,start,goal);
planningTime(10) = toc;
%%
% Define simulation parameters
scenarioParams.Gravity = 9.81;    % m/s^2
scenarioParams.UpdateRate = 1/60; % Hz

% Define vehicle parameters for estimating power consumption
scenarioParams.Robot.Mass = 70;% kg, no payload
scenarioParams.Robot.Velocity = 0.6;% m/s
scenarioParams.Robot.AmpHour = 24; % A-hr
scenarioParams.Robot.Vnom = 24;% V, Voltage
scenarioParams.Robot.RegenerativeBrakeEfficiency = 0;

% Display result
figure
animateResults = true;
statTable = exampleHelperCalculateMetrics(costMapReal,scenarioParams,paths,planningTime,pathNames)
%%
exampleHelperVisualizeResults(gca,statTable,scenarioParams,costMapReal,paths,pathNames,~animateResults);
%%

ptClouds = vSet.Views.PointCloud;
tforms   = vSet.Views.AbsolutePose;

ptCloudMap = pcalign(ptClouds,tforms);

%voxelSize = 1;
%ndtMap = pcmapndt(ptCloudMap,voxelSize);
%figure
pcshow(ptCloudMap, VerticalAxis='Y', VerticalAxisDir='Down', ViewPlane='YX', AxesVisibility='on');
%show(ndtMap)
%view(2);


%%
figure
plot(vSet,ShowViewIds='on')
view(3)






%{
pose=[0 0 0 1 0 0 0];
map3D = occupancyMap3D(0.01);

for i=1:length(chmury)
    fprintf('Processing Image %d\n', i);
    insertPointCloud(map3D,pose,chmury{1,i},10);
end
%}
%%
%{
figure;
show(map3D);
xlim([-2 2])
ylim([-2 2])
zlim([0 4])
view([-201 47])
%}
%}