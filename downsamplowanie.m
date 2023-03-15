function [downsample] = downsamplowanie (chmury,typ,liczba)
narginchk(3,3);
mustBeText(typ);
downsample=cell(1,10);

    for i=1:length(chmury)
        downsample{1,i} = pcdownsample(chmury{1,i},typ,liczba);
        fprintf('Downsampling Cloud %d\n', i);
    end

end