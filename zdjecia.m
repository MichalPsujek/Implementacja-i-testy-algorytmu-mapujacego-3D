function [zdjecia] = zdjecia (depths,colors,numerek,maxRange,depthScale)
 
depthmsgs=readMessages(depths);
    colormsgs=readMessages(colors);
    zdjecia=cell(1,numerek);


    for i = 1:numerek
        depthMap = double(readImage(depthmsgs{i,1})) .* depthScale;
        colorMap = readImage(colormsgs{i,1});

        depthMap(depthMap>maxRange)=0;
        depthMap(depthMap<0.5)=0;

        zdjecia(1,i)={depthMap};
        zdjecia(2,i)={colorMap};
        if (mod(i,10)==0),fprintf('Processing Image %d\n', i); end
    end
end
