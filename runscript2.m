clc; clear; close all;


arenaFile = 'Arenas/arena02.mat'; 
if exist(arenaFile, 'file')
    load(arenaFile, 'map'); 
else
    error('invalid file');
end

% Define test cases (can do multiple at once for debugging)
testCases = {
    [4; 8], [33, 35];
};

% run tests
for i = 1:size(testCases, 1)
    start = testCases{i, 1};
    stop = testCases{i, 2};
    
    
    try
        [lamb_d, startXY] = planArenaPath(start, stop, map, true);
        
        
        % verify validity of path
        for j = 1:size(lamb_d, 2)
            x = lamb_d(1, j);
            y = lamb_d(2, j);
            
           
            col = round((x / 0.25) + 20);
            row = round(20 - (y / 0.25));
            
            
        end

        fprintf('Case %d PASSED.\n', i);
    end
    
    fprintf('----------------------------------------\n');
end
