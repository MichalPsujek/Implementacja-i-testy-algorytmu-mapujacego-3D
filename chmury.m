function [chmury] = chmury (zdjecia,K,maxRange,grid)
    chmury=cell(1,10);
    
    for i=1:length(zdjecia)
    depthMap=zdjecia{1,i};
    depthMap(depthMap>maxRange)=0;
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

    downsamplik=pcdownsample(points3D,'gridAverage',grid);
%     downsamplik=points3D;
%     downsample=pcdenoise(downsamplik,"NumNeighbors",20,"Threshold",0.4);
    chmury(1,i)={downsamplik};

    %chmury(1,i)={points3D};

    if (mod(i,10)==0),fprintf('Processing Cloud %d\n', i); end
    end
end