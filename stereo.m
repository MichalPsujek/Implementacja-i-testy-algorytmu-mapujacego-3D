clc
clear
k=1;
tabelakontrolna=zeros(1,40);
[K,depths,colors,depthScale,numerek] = strimek('przejscie male.bag');
%%
%numerek=445;
maxRange=5;
zdjecia = zdjecia(depths,colors,numerek,maxRange,depthScale);
clear depths colors;
%%
grid=0.03;
chmury = chmury(zdjecia,K,maxRange,grid);
clear zdjecia;
%%
downsample=chmury;
clear chmury;
% downsample = downsamplowanie(chmury,'gridAverage',0.02);
%%
clear vSet;

razem=zaznaczenia{1,1};
relPose=rigidtform3d;
absPose=rigidtform3d;
vSet=pcviewset;
vSet = addView(vSet,1,absPose,PointCloud=zaznaczenia{1,1});
prevViewId = 1;
prevPtCloud = zaznaczenia{1,1};
loopDetector = scanContextLoopDetector;
viewId=2;
rmses=zeros(1,round(length(downsample)));
    
regGridStep = 0.2;

% figure
% hold on
% xlabel("X(m)");
%     ylabel("Y(m)");
%     zlabel("Z(m)");
frames(numerek) = struct('cdata',[],'colormap',[]);
i=2;
while i<numerek
    ptCloud=zaznaczenia{1,i};
    
     % Register new point cloud against the previous one.
    [relPose,~,rmse] = pcregisterndt(ptCloud,prevPtCloud,regGridStep,InitialTransform=relPose,Tolerance=[0.01 0.03]);
    %relPose = pcregistercorr(ptCloud,prevPtCloud,10,0.01);
    %relPose = pcregistericp(ptCloud,prevPtCloud,"Metric","pointToPoint");

    % Update the absolute pose.
    absPose = rigidtform3d(absPose.A * relPose.A);

    % Add new view and connection to the previous view.
    vSet = addView(vSet,viewId,absPose,PointCloud=ptCloud);
    vSet = addConnection(vSet,prevViewId,viewId,relPose);
    prevPtCloud = ptCloud;
    prevViewId = viewId;

    descriptor = scanContextDescriptor(ptCloud);
    addDescriptor(loopDetector,viewId,descriptor);
    [loopViewId,dists] = detectLoop(loopDetector,descriptor,'NumExcludedDescriptors',10);

    ptCloud=pctransform(ptCloud,absPose);
    razem=pcmerge(razem,ptCloud,0.02);
    

    rmses(round(viewId))=rmse;
        % A loop candidate was found

    if ~isempty(loopViewId)
        loopViewId = loopViewId(1);

        % Retrieve point cloud from view set
        loopView = findView(vSet, loopViewId);
        ptCloudOrig = loopView.PointCloud;

        % Use registration to estimate the relative pose
        [relTform, ~, rmse] = pcregisterndt(ptCloud, ptCloudOrig, ...
            regGridStep,Tolerance=[0.01 0.05]);

        acceptLoopClosure = (rmse <= 0.09);
        
        if acceptLoopClosure
            infoMat = 0.01 * eye(6);

            % Add a connection corresponding to a loop closure
            vSet = addConnection(vSet, loopViewId, viewId, relTform, infoMat);
        end
        loopViewId=[];
    end
%     if ptCloud.Count<8000000
%         i=i+1;
%     else
%         i=i+2;
%     end
%     if (i>270 && i<530) || (i>700 && i<1030)
%         regGridStep=0.12;
%     else
%         regGridStep=0.4;
%     end
    pcshow(razem,VerticalAxis='Z', VerticalAxisDir='Down', ViewPlane='YX', AxesVisibility='on');
    frames(i-1)=getframe(gcf);
    i=i+1;
    viewId=viewId+1;

    
    %pcshow(razem, VerticalAxis='Z', VerticalAxisDir='Down', ViewPlane='YX', AxesVisibility='on')
    %drawnow;
    

    fprintf('Merging Cloud %d\n', i-1);
end

rmsem=mean(rmses);
tabelakontrolna(1,k)=regGridStep;
tabelakontrolna(2,k)=rmsem;
k=k+1;
%%
ptClouds = vSet.Views.PointCloud;
tforms   = vSet.Views.AbsolutePose;

ptCloudMap = pcalign(ptClouds,tforms);
pcshow(ptCloudMap, VerticalAxis='Y', VerticalAxisDir='Down', ViewPlane='ZX', AxesVisibility='on');
%%
% res=0.01;
% startposition=[1.3 8.5];
% goalposition=[6 4];
% maxinclineangle=35;
% [paths,pathNames,planningTime] = sciezki (ptCloudMap,res,startposition,goalposition,maxinclineangle);