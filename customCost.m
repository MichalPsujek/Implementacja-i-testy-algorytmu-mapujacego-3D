function cost=customCost(costMap,weightZ,weightgrad,weightroll,gridState1,gridState2)
assert(size(gridState1,1) == 1 && size(gridState2,1) == 1);
    
    % Get Z layer
    zMap = costMap.getLayer("Z");
    
    % Get XY states
    xyState = zMap.grid2world([gridState1(1:2);gridState2(1:2)]);

    % Get height difference
    Z = zMap.getMapData(xyState);

    Z(isnan(Z))=0;

    % Calculate cost
    cost1 = sqrt(sum([xyState(2,:)-xyState(1,:) (Z(2)-Z(1))].^2));

    xyState = costMap.grid2world([gridState1(1:2);gridState2(1:2)]);
    dXY = abs(xyState(2,:)-xyState(1,:));
    transitCost = sqrt(sum(dXY).^2);

    if isnan(transitCost)
        transitCost = 100;
    end

    % hill cost
    if all(dXY == 1)
        % Diagonal motion, check diagonal cost map for visit cost
        hillCost = getMapData(costMap,"diagCost",xyState(1,:));
    else
        vGrad = [getMapData(costMap,"xCost",xyState(1,:)) getMapData(costMap,"yCost",xyState(2,:))];
        vGrad(isnan(vGrad))=0;
        % Calculate a cost based on transverse slope
        hillCost = abs(dXY/norm(dXY)*vGrad');
    end

    if isnan(hillCost)
        hillCost = 100;
    end

    cost2 = transitCost + hillCost;

    % XY flipped to calculate perpendicularity
    vGradroll = [getMapData(costMap,"yCost",xyState(1,:)) getMapData(costMap,"xCost",xyState(1,:))];


    % Calculate a cost based on transverse slope
    rolloverCost = abs(dXY/norm(dXY)*vGradroll');
    
    if isnan(rolloverCost)
        rolloverCost = 100;
    end

    % Combine transit/rollover costs
    cost3 = transitCost + rolloverCost;

    cost=(weightZ*cost1+weightgrad*cost2+weightroll*cost3)/3;
end