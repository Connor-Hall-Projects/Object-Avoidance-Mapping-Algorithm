function [lamb_d, startXY] = planArenaPath(start, stop, map, show)
    % check start and stop positions arent in obstacle
    if map(start(1), start(2)) == 0 || map(stop(1), stop(2)) == 0
        error('start/stop in obstacle');
    end

    % Expand obstacles by 1 pixel to represent clearance zone
    expandedMap = expandObstacles(map, 1);

    % check if start/stop is in clearance zone
    if expandedMap(start(1), start(2)) == 0 || expandedMap(stop(1), stop(2)) == 0
        error('start/stop in clearance zone');
    end

    % calculate path
    [path, ~] = dijkstraPathPlanning(expandedMap, start, stop);

    if isempty(path)
        error('no path');
    end

    lamb_d = convertPathToGlobal(path);


    startXY = convertToGlobal(start);

    % save variables so can be used by simulink
    assignin('base', 'lamb_d', lamb_d);
    assignin('base', 'startXY', startXY);

    if show
        visualizePath(map, expandedMap, path);
    end
end

%% create clearance zones 
function expandedMap = expandObstacles(map, expansionRadius)
    [rows, cols] = size(map);
    expandedMap = map;  % Start with the original map

    [obsRows, obsCols] = find(map == 0); % Locate original obstacles

    % Expand obstacles by 1 pixel to create clearance zones
    for i = 1:length(obsRows)
        r = obsRows(i);
        c = obsCols(i);

        for dr = -expansionRadius:expansionRadius
            for dc = -expansionRadius:expansionRadius
                newR = r + dr;
                newC = c + dc;

                if newR >= 1 && newR <= rows && newC >= 1 && newC <= cols
                    expandedMap(newR, newC) = 0; % Mark as obstacle
                end
            end
        end
    end
end

%% calculate path
function [path, cost] = dijkstraPathPlanning(map, start, goal)
    moves = [0, 1; 1, 0; 0, -1; -1, 0; 1, 1; 1, -1; -1, -1; -1, 1];
    moveCost = [1, 1, 1, 1, 1.5, 1.5, 1.5, 1.5];

    [rows, cols] = size(map);
    infValue = inf;
    distMatrix = infValue * ones(rows, cols);
    parentR = zeros(rows, cols);
    parentC = zeros(rows, cols);

    queue = start';
    distMatrix(start(1), start(2)) = 0;

    while ~isempty(queue)
        [~, minIdx] = min(distMatrix(sub2ind(size(distMatrix), queue(:, 1), queue(:, 2))));
        row = queue(minIdx, 1);
        col = queue(minIdx, 2);
        queue(minIdx, :) = [];

        if row == goal(1) && col == goal(2)
            cost = distMatrix(row, col);
            path = backtrackPath(parentR, parentC, start, goal);
            return;
        end

        for i = 1:size(moves, 1)
            newRow = row + moves(i, 1);
            newCol = col + moves(i, 2);
            newCost = distMatrix(row, col) + moveCost(i);

            if isValidMove(newRow, newCol, map, rows, cols) && newCost < distMatrix(newRow, newCol)
                distMatrix(newRow, newCol) = newCost;
                parentR(newRow, newCol) = row;
                parentC(newRow, newCol) = col;
                queue = [queue; newRow, newCol];
            end
        end
    end
    path = [];
    cost = infValue;
end

function valid = isValidMove(row, col, map, numRows, numCols)
    valid = row >= 1 && row <= numRows && col >= 1 && col <= numCols && map(row, col) == 1;
end

function path = backtrackPath(parentR, parentC, start, goal)
    row = goal(1);
    col = goal(2);
    path = [row; col];

    while ~(row == start(1) && col == start(2))
        prevRow = parentR(row, col);
        prevCol = parentC(row, col);

        if prevRow == 0 || prevCol == 0
            break;
        end

        row = prevRow;
        col = prevCol;
        path = [[row; col], path];
    end
end

%% path conversion
function lamb_d = convertPathToGlobal(path)
    scale = 0.25;
    offset = 20.5;
    lamb_d = zeros(3, size(path, 2));
    for i = 1:size(path, 2)
        col = path(2,i);
        row = path(1,i);
        x = (col - offset) * scale;
        y = (offset - row) * scale;
        if i < size(path, 2)
            nextCol = path(2,i+1);
            nextRow = path(1,i+1);
            theta = atan2((offset - nextRow) - (offset - row), (nextCol-offset)-(col-offset));
            
        else
            theta = 0;
        end
        lamb_d(:,i) = [x;y;theta];
    end
end

function startXY = convertToGlobal(start)
    startXY = [(start(2) - 20.5) * 0.25; (20.5 - start(1)) * 0.25];
end
%% give graph
function visualizePath(map, expandedMap, path)
    figure;
    hold on;

    [rows, cols] = size(map);
    img = ones(rows, cols, 3); % Initialize as white (free space)

    % Set obstacles (black)
    obstacleIdx = find(map == 0);
    img(obstacleIdx) = 0; % Red channel
    img(obstacleIdx + numel(map)) = 0; % Green channel
    img(obstacleIdx + 2 * numel(map)) = 0; % Blue channel

    % Set clearance zones (red)
    clearanceIdx = find(map == 1 & expandedMap == 0);
    img(clearanceIdx) = 1; % Red channel
    img(clearanceIdx + numel(map)) = 0; % Green channel
    img(clearanceIdx + 2 * numel(map)) = 0; % Blue channel

    % Mark path (blue)
    for i = 1:size(path, 2)
        img(path(1, i), path(2, i), :) = [1 0 1]; % Blue
    end

    % Flip the image vertically to correct orientation
    img = flipud(img);

    image(img);
    axis equal;
    title('Path Planning Visualization');
    hold off;
end

