function [outputStructure] = TriangulateBuildingsFinal(varargin)
% TRIANGULATEBUILDINGS Determines feasible regions of building localization
%   DATASET NOTES: x_0-3, where 0 = initial (== heading) direction. 4 is
%   ref image and discarded
addpath('Support Code');
%system('subst "z:" "H:\Vision Lab"')
close all;
[args dirs] = HandleArgs(varargin);

data = struct;
results = struct;
data.queryCounter = 0;
data.thetaCor = -81.251767103267881;            %Adjustment for mercator GE projection -> cartesian plane
%todo: clean up storage of stuff I don't need this iteration; half these error metrics are %collapsed into newer structures or discarded due to uselessness
results.gterror = [];
results.matchedBuilding = [];
results.matchedError = [];
results.centroids = [];
results.corners = [];
results.regionStats = [];
results.gtvalid = [];
results.matchedValid = [];
results.correctMatch = [];
results.maps = [];
data.xcart = args.xcart;
data.ycart = args.ycart;
for queryNum = args.qn'
    dirs.resultsDir = [dirs.resultsDir '\' int2str(queryNum) '\'];
    mkdir(dirs.resultsDir);
    tic %outer loop
    clf;
    data.queryNum = queryNum;
    data.nodeCounter = 0;
    results.maps = [];
    data.queryCounter = data.queryCounter + 1;
    fprintf('Query %d starting.\n', data.queryNum);
    data.figure = InitializeFigure();
    disp('Generating match indicies.');
    [args, dirs, data] = GenerateMatches(args, dirs, data);
    if(data.fullIndex == 0)
        continue;
    end
    [args dirs data results] = CreateMaps(args, dirs, data, results);
    disp('Loop through nodes.');
    if(length(data.fullIndex) > args.numNearest)
        data.fullIndex = data.fullIndex(1:args.numNearest);
    end
    %this is erroring on multiple passes
    for nodeNumber = data.fullIndex
        data.nodeCounter = data.nodeCounter + 1;
        data.nodeNumber = nodeNumber;
        [args dirs data results] = CreateLines(args, dirs, data, results);
        [args dirs data results] = CreateViewRegions(args, dirs, data, results);
        [args dirs data results] = WeightAndTruncate(args, dirs, data, results);
        results.maps(:,data.nodeCounter) = data.mapping;
    end
    [args dirs data results] = SumMaps(args, dirs, data, results);
    [args dirs data results] = FindRegions(args, dirs, data, results);
    %[args dirs data results] = RejectNodes(args, dirs, data, results);
    %todo: figure out how to reset between attempts
    [args dirs data results] = SetSubplotImages(args, dirs, data, results);
    [args dirs data results] = PlotRegions(args, dirs, data, results);

    
    dirs.resultsDir = [dirs.rootDir 'obj/' args.expername '/'];
    toc
end %%END: Query loop

outputStructure = struct('args', {args}, 'dirs', {dirs}, 'data', {data}, 'results', {results});
save([dirs.resultsDir '/output.mat'], 'outputStructure')
end

function [args dirs] = HandleArgs(varargin)
varargin = varargin{1};
%%%% FIX ORLANDO HEADING DISPARITY

%%%%%%%%%%%%%%%%
% ARGUMENT DEFINITION
sparse = 0;                     %[INDEV] display only reference images on the map
totalCov = 0;                   %[???] what does this do
testing = 0;                    %currently unused
numAreas = 1;                   %Number of areas to plot on map
expername = '';                 %Experiment name; automatically generated if not specified
numNearest = 30;                %Max # of nodes to consider for generating mapped regions
showPlacemarkText = 0;          %VISUAL: Show labels
numImages = 10;                 %VISUAL: Number of images to show
outputImages = 0;               %VISUAL: Display matched reference images
weightMode = 1;                 %Weighting modes. 1 = Unweighted, 2 = Direct, 3 = Quantized
plotOnlyContributing = 0;       %VISUAL: Only display nodes that contribute to the region in question
limitNodes = 0;                 %Number of nodes to limit post-mapping region to
voteThresh = 0;                 %INTERNAL: Does not change directly
voteThreshPer = 0;              %[INDEV] Percentage of votes of the 1st ranked node that a node must have to be considered in the formulation.
rootDir = '';                   %Root data directory. Defaults to Z:\Geolocation Data\GSV Pittsburgh 10k\
smoothed = '';                  %If '', use unsmoothed histograms
regMatch = 0;                   %Attempt to match to the nearest reference image in the dataset
maxDist = 0;                    %[INDEV] Maximum distance from a node that a placemark can be to be under consideration
maxCentrDist = 0;               %[INDEV] Maximum distance from the centroid of the found region that a node can be
rescaleDists = 0;               %[INDEV] Scale distances as a function of area.
enforceOpposites = 0;           %[INDEV] Ensure that there are orthogonal angles in the feasible region
multipleBuildings = 1;          %[INDEV] Number of buildings to consider
singleQ = '/';                  %[probably need to remove] single-building (not-full-frame) flag
qn = '';                        %Queries to test
dataType = 'r';                 %[INDEV] Type of data (governs use/non-use of histograms). 'r' = real, 'rs' = real-synth, 's' = synth
%%%%%%%%%%%%%%%%

%%%%%%% ARGUMENT HANDLING
for k=1:length(varargin)
    if(strcmp(varargin{k}, 'query') == 1)
        qn = varargin{k+1};
    end
    if(strcmp(varargin{k},'sparse') == 1)
        sparse = 1;
    end
    if(strcmp(varargin{k}, 'plotCoverage') == 1)
        totalCov = 1;
    end
    if(strcmp(varargin{k}, 'expname') == 1)
        expername = varargin{k+1};
    end
    if(strcmp(varargin{k}, 'test') == 1)
        testing = 1;
    end
    if(strcmp(varargin{k}, 'numAreas') == 1)
        numAreas = varargin{k+1};
    end
    if(strcmp(varargin{k}, 'NN') == 1)
        numNearest = varargin{k+1};
    end
    if(strcmp(varargin{k}, 'showPlacemarkText') == 1)
        showPlacemarkText = varargin{k+1};
    end
    if(strcmp(varargin{k}, 'showImages') == 1)
        outputImages = 1;
    end
    if(strcmp(varargin{k}, 'weightMode') == 1)
        weightMode = varargin{k+1};
    end
    if(strcmp(varargin{k}, 'plotContributing') == 1)
        plotOnlyContributing = 1;
    end
    if(strcmp(varargin{k}, 'rescaleDists') == 1)
        rescaleDists = 1;
    end
    if(strcmp(varargin{k}, 'limitNodes') == 1)
        limitNodes = varargin{k+1};
    end
    if(strcmp(varargin{k}, 'smoothedHistograms') == 1)
        smoothed = 'smoothed/';
    end
    if(strcmp(varargin{k}, 'singleQueries') == 1)
        singleQ = '/single/';
    end
    if(strcmp(varargin{k}, 'voteThresh') == 1)
        voteThreshPer = varargin{k+1};
    end
    if(strcmp(varargin{k}, 'dataset') == 1)
        
        if(strcmp(varargin{k+1}, 'PB') == 1)
            rootDir = 'Z:\Geolocation Data\GSV Pittsburgh 10k\';
            tempLoc = 'PB';
            xcart = 1;
            ycart = 3;
        else
            rootDir = 'Z:\Geolocation Data\GSV Orlando\';
            tempLoc = 'Orl';
            xcart = 1;
            ycart = 2;
        end
    end
    if(strcmp(varargin{k}, 'regionMatch') == 1)
        regMatch = 1;
    end
    if(strcmp(varargin{k}, 'maxDist') == 1)
        maxDist = varargin{k+1};
    end
    if(strcmp(varargin{k}, 'datatype') == 1)
        dataType = varargin{k+1};
    end
end

if(strcmp(rootDir, '') == 1)
    rootDir = 'Z:\Geolocation Data\GSV Pittsburgh 10k\';
    tempLoc = 'PB';
    xcart = 1;
    ycart = 3;
end
if(strcmp(expername, ''))
    expername = [tempLoc '-'];
    if(weightMode == 1)
        expername = [ expername 'Unweighted'];
    elseif(weightMode == 2)
        expername = [ expername 'DirectScaled'];
    elseif(weightMode == 3)
        expername = [ expername 'Quantized'];
    end
    expername = [ expername sprintf('-%s', dataType)];
    expername = [ expername sprintf('-%d NN', numNearest)];
    if(regMatch == 1)
        expername = [ expername '-RegionMatches'];
    end
    if(limitNodes ~= 0)
        expername = [ expername sprintf('-%d LimitNodes', limitNodes)];
    end
    if(maxDist ~= 0)
        expername = [ expername sprintf('-%d MaxDist', maxDist)];
    end
    
    if(maxCentrDist ~= 0)
        expername = [ expername sprintf('-%d MaxCentrDist', maxCentrDist)];
    end
    if(rescaleDists ~= 0)
        expername = [ expername sprintf('-%d RescaleDists', rescaleDists)];
    end
    if(voteThreshPer ~= 0)
        expername = [ expername sprintf('-%d VoteThreshPer', voteThreshPer)];
    end
    if(strcmp(smoothed, 'smoothed/') == 1)
        expername = [ expername '-Smoothed'];
    end
    if(strcmp(singleQ, '/single/') == 1)
        expername = [ expername '-Single'];
    end
end
resultsDir = [rootDir 'obj/' expername '/'];
dataDir = [rootDir 'data/'];
rawDir = [rootDir 'raw/'];
histoDir = [rootDir 'results/baseline/queries' singleQ 'histograms/' smoothed];
[a b c] = mkdir(resultsDir);
load([dataDir 'reference/pose2corheadings.mat']);
load([dataDir 'reference/pose2.txt']);
load([dataDir 'queries' singleQ 'buildingGT.mat']);
load([dataDir 'queries' singleQ 'queryGPS.txt']);

if(isempty(qn))
    qn = find(buildingGT(:,1) ~= 0);
end
%%% TODO: AUTOMATE CART

dirs = struct('rootDir', {rootDir}, 'rawDir', {rawDir}, 'dataDir', {dataDir}, 'histoDir', {histoDir}, 'resultsDir', {resultsDir});
args = struct('sparse', {sparse}, 'totalCov', {totalCov}, 'testing',{testing}, 'numAreas', {numAreas}, 'expername', {expername}, ...
    'numNearest', {numNearest}, 'showPlacemarkText', {showPlacemarkText}, 'numImages', {numImages}, 'outputImages', {outputImages}, ...
    'weightMode',{weightMode},'plotOnlyContributing', {plotOnlyContributing}, 'limitNodes', {limitNodes}, 'voteThresh', {voteThresh}, ...
    'voteThreshPer', {voteThreshPer}, 'smoothed',{smoothed}, 'regMatch', {regMatch}, 'maxDist', {maxDist}, 'enforceOpposites',{enforceOpposites}, ...
    'multipleBuildings', {multipleBuildings}, 'singleQ', {singleQ}, 'qn', {qn}, 'dataType', {dataType}, 'pose2', {pose2}, 'pose2cor', {pose2cor}, ...
    'bData', {bData}, 'buildingGT', {buildingGT}, 'queryGPS', {queryGPS(:,2:3)}, 'xcart', {xcart}, 'ycart', {ycart}, 'rescaleDists', {rescaleDists}, 'maxCentrDist',{maxCentrDist} );
end

function [args, dirs, data] = GenerateMatches(args, dirs, data)

if(strcmp(args.dataType, 'r') == 1)
    disp('Using REAL data.')
    hista = '';
    if(eval(sprintf('exist([dirs.histoDir ''histo%d.mat''])', data.queryNum)))          %Loading data + check for missing matches
        eval(sprintf('load([dirs.histoDir  ''histo%d.mat''])', data.queryNum));
    else
        fprintf('Couldn''t find %d histo', data.queryNum);
        data.fullIndex = 0;
        data.placemarkIndex = 0;
        return;
    end
    data.histogram = hista;
    [y ind] = sort(data.histogram, 'descend');
    if(args.voteThreshPer ~= 0)
        args.voteThresh = y(1) * args.voteThreshPer;
    end
    a = y > args.voteThresh;            %ensure logical indexing didn't break everything
    data.fullIndex = ind(a);
    if(length(data.fullIndex) < 4)
        data.fullIndex = ind(1:4);
    end
    data.placemarkIndex = floor(data.fullIndex/10);
    data.viewIndex = round((data.fullIndex/10 - data.placemarkIndex) * 10);
    data.votes = y;
    clear hista y ind;
elseif(strcmp(args.dataType, 'rs') == 1)
    disp('Using realistic synthetic data')
elseif(strcmp(args.dataType, 's') == 1)
    disp('Using synthetic data.')
else
    disp('Not a valid data parameter value, exiting.')
end


end

function [figureHandle] = InitializeFigure()
figureHandle = figure();
axes('DataAspectRatioMode', 'manual', 'DataAspectRatio', [1 1 1], 'PlotBoxAspectRatioMode', 'manual', 'PlotBoxAspectRatio', [1 1 1]);
hold on;
end

function [args dirs data results] = CreateMaps(args, dirs, data, results)

data.newMap = zeros((floor(max(args.pose2cor(:,data.xcart)) - min(args.pose2cor(:,data.xcart)))), floor((max(args.pose2cor(:,data.ycart)) - min(args.pose2cor(:,data.ycart)))));
data.xind = min(args.pose2cor(:,data.xcart)):min(args.pose2cor(:,data.xcart)) + size(data.newMap,data.xcart)-1;
data.yind = min(args.pose2cor(:,data.ycart)):min(args.pose2cor(:,data.ycart)) + size(data.newMap,2)-1;
data.xindold = size(data.xind);
data.yindold = size(data.yind);
data.xind = repmat(data.xind,size(data.yind));
data.yind = repmat(data.yind,data.xindold);
[data.yind sortind] = sort(data.yind, 'ascend');
data.xind = data.xind';
data.yind = data.yind';
data.xindold = data.xindold(2);
data.yindold = data.yindold(2);


end

function [args dirs data results] = CreateLines(args, dirs, data, results)

data.theta = (args.pose2cor(data.placemarkIndex(data.nodeCounter),4)+ data.viewIndex(data.nodeCounter) * 90) - data.thetaCor ;

data.headingX = args.pose2cor(data.placemarkIndex(data.nodeCounter), data.xcart) + -cosd(data.theta)*50;
data.headingY = args.pose2cor(data.placemarkIndex(data.nodeCounter), data.ycart) + sind(data.theta)*50;
data.plusX = args.pose2cor(data.placemarkIndex(data.nodeCounter), data.xcart) + -cosd(data.theta + 45)*50;
data.plusY = args.pose2cor(data.placemarkIndex(data.nodeCounter), data.ycart) + sind(data.theta + 45 )*50;
data.minusX = args.pose2cor(data.placemarkIndex(data.nodeCounter), data.xcart) + -cosd(data.theta - 45)*50;
data.minusY = args.pose2cor(data.placemarkIndex(data.nodeCounter), data.ycart) + sind(data.theta - 45 )*50;

end

function [args dirs data results] = PlotNodes(args, dirs, data, results)
[args dirs data results] = CreateLines(args, dirs, data, results);
plot( args.pose2cor(data.placemarkIndex(data.nodeCounter), data.xcart),args.pose2cor(data.placemarkIndex(data.nodeCounter), data.ycart), 'go', 'MarkerEdgeColor','k',  'MarkerFaceColor','g');
line([args.pose2cor(data.placemarkIndex(data.nodeCounter), data.xcart), data.headingX],[args.pose2cor(data.placemarkIndex(data.nodeCounter), data.ycart), data.headingY]);
line([args.pose2cor(data.placemarkIndex(data.nodeCounter), data.xcart),data.plusX],[args.pose2cor(data.placemarkIndex(data.nodeCounter), data.ycart), data.plusY],'Color', 'r', 'LineStyle', ':');
line([args.pose2cor(data.placemarkIndex(data.nodeCounter), data.xcart),data.minusX],[args.pose2cor(data.placemarkIndex(data.nodeCounter), data.ycart), data.minusY],'Color', 'r', 'LineStyle', ':');
end

function [args dirs data results] = CreateViewRegions(args, dirs, data, results)

    p1 = [args.pose2cor(data.placemarkIndex(data.nodeCounter),data.xcart), args.pose2cor(data.placemarkIndex(data.nodeCounter),data.ycart)];
    p2 = [data.plusX,data.plusY];
    if(args.sparse)
        p3 = [args.pose2cor(:,data.xcart), args.pose2cor(:,data.ycart)];
    else
        p3 = [data.xind data.yind];
    end
    data.mapping = ((p2(1) - p1(1)) * (p3(:,2) - p1(2)) - (p2(2) -p1(2)) * (p3(:,1) - p1(1))) >= 0;

    index = find(data.mapping == 1);
    p2 = [data.minusX,data.minusY];
    if(args.sparse)
        p3 = [args.pose2cor(:,data.xcart), args.pose2cor(:,data.ycart)];
    else
        p3 = [data.xind(index) data.yind(index)];
    end
    data.mapping(index) = ((p2(1) - p1(1)) * (p3(:,2) - p1(2)) - (p2(2) -p1(2)) * (p3(:,1) - p1(1))) <= 0;
    if(args.maxDist ~= 0)
        index = find(data.mapping == 1);
        outIndex = CalcThreshDist(data.mapping(index),
    end

end

function [args dirs data results] = WeightAndTruncate(args, dirs, data, results)
%todo: change fullindex. this needs to happen post truncation and
%mapping etc.
if(args.weightMode == 2)
    data.mapping = data.mapping * (length(data.fullIndex) + 1 - data.nodeCounter);
elseif(args.weightMode == 3)
    if(data.nodeCounter / length(data.fullIndex) <= .25)
        data.mapping = data.mapping * 4;
    elseif(data.nodeCounter / length(data.fullIndex) <= .5)
        data.mapping = data.mapping * 3;
    elseif(data.nodeCounter / length(data.fullIndex) <= .75)
        data.mapping = data.mapping * 2;
    end
end


end

function [args dirs data results] = SumMaps(args, dirs, data, results)

results.summedMaps(:,data.queryCounter) = sum(results.maps,2);
results.uniqueNumList = sort(unique(results.summedMaps(:,data.queryCounter)), 'descend');
index = results.summedMaps(:,data.queryCounter) == results.uniqueNumList(1);                    %select highest overlapping region
results.nnContributing = find(sum(results.maps(index,:),1) ~= 0);
if(args.limitNodes ~= 0 && args.limitNodes < length(results.nnContributing))
    results.nnContributing = results.nnContributing(1:args.limitNodes);
    
end
ind = [data.xind-min(args.pose2cor(:,data.xcart))+1,data.yind-min(args.pose2cor(:,data.ycart))+1];
results.logicalMap = reshape(results.summedMaps(:,data.queryCounter), max(ind(:,1)), max(ind(:,2)));

end

function [args dirs data results] = RejectNodes(args, dirs, data, results)
if(args.rescaleDists == 1)
    tmpMaps = results.maps;
    tCor = zeros(2,3);
    tCor(:,data.xcart) = data.xind(1:2);
    tCor(:,data.ycart) = data.yind(1:2);
    distU = pdist([cor2gps(tCor(1,:)); cor2gps(tCor(2,:))], 'euclidean');
    currentSelection = results.uniqueNumList(1);
    rejectionCount = 0;
    while(1)
        
        rejectionList = [];
        maximalDistance = sum(results.summedMaps == currentSelection)/4;
        for k=1:length(results.nnContributing)
            data.nodeCounter = results.nnContributing(k);
            nodeLoc = args.pose2cor(data.placemarkIndex(data.nodeCounter), :);
            nodeLoc = [nodeLoc(data.xcart) nodeLoc(data.ycart)];
            cent = gps2cor(results.centroids(data.queryCounter, 2:3));
            cent = [cent(data.xcart) cent(data.ycart)];
            realDist = pdist([cent;nodeLoc ],'euclidean' );
            if(realDist > maximalDistance*4)
                rejectionList = [rejectionList; k realDist];
            end
        end
        if(~isempty(rejectionList))
            rejectionCount = rejectionCount+1;
            [lst index] = sort(rejectionList(:,2));
            rejectionList = rejectionList(index,:);
            results.maps(rejectionList(1,1),:) = 0;
            [args dirs data results] = SumMaps(args, dirs, data, results);
            currentSelection = results.uniqueNumList(1);
            [args dirs data results] = FindRegions(args, dirs, data, results);
            disp('Rejecting a node.');
        else
            break;
        end
        %if(rejectionCount
        
       
            
    end
    
end
    
end
function [args dirs data results] = GetRegionStats(args, dirs, data, results)
end

function [args dirs data results] = GetSparseMatches(args, dirs, data, results)
end

function [args dirs data results] = SetSubplotImages(args, dirs, data, results)
if(args.outputImages == 1)                           %image output subplots
    numVert = 5;
    for imgInd = 1:args.numImages
        if(imgInd > length(results.nnContributing))
            break;
        end
        curVert = floor ((imgInd+1)/2);
        curHoriz = (mod(imgInd-1, 2) + 6 * curVert) - 1;
        subInd(imgInd) = subplot(numVert,6,curHoriz);
        imName = sprintf('%05d_%d.jpg', data.placemarkIndex(results.nnContributing(imgInd)), data.viewIndex(results.nnContributing(imgInd)));
        imshow([dirs.rawDir 'reference\' imName]);
        
    end
    subplot(numVert,6,[1 2 7 8]);
    imName = sprintf('q(%d).jpg', data.queryNum);
    imshow([dirs.rawDir 'queries/' imName]);
    plotting = [3 4 9 10];
    subplot(numVert, 6, plotting);
    hold on;
    
end
end

function [args dirs data results] = FindRegions(args, dirs, data, results)
if(args.multipleBuildings == 1)
    centroidTemp = [data.queryNum];
    errorGTTemp = [data.queryNum];
    errorMatchedTemp = [data.queryNum];
    validGTTemp = [data.queryNum];
    validMatchedTemp = [data.queryNum];
    matchedBuildingTemp = [data.queryNum];
    [bNums fnd] = find(args.buildingGT(data.queryNum, :)>0);
    if(isempty(bNums))
        bNums = 0;
        disp('No buildings identified for ground truth of selected query');
    end
    for areaNum = 1:args.numAreas
        areaIndex = find(results.summedMaps(:,data.queryCounter) == results.uniqueNumList(areaNum));
        %%% TO DO: INSERT SEPARABLE REGIONS CODE HERE
        corgps = zeros(1,3);
        corgps(data.xcart) =  mean(data.xind(areaIndex));
        corgps(data.ycart) = mean(data.yind(areaIndex));
        centGPS = cor2gps(corgps);
        centroidTemp = [centroidTemp centGPS];
        
        if(bNums ~= 0)
            boundingInGPS = args.bData{args.buildingGT(data.queryNum, bNums), 3};
            boundingInCart = gps2cor(boundingInGPS);
            boundingInCart = [boundingInCart(:,data.xcart), boundingInCart(:,data.ycart)];
            errorGTTemp = [errorGTTemp haversine(args.bData{args.buildingGT(data.queryNum, bNums), 2}, centGPS)];
            validGTTemp = [validGTTemp max(inpolygon(data.xind(areaIndex), data.yind(areaIndex), boundingInCart(:,1), boundingInCart(:,2)))];
        end
        matcherror = 0;
        bT = 0;
        for buildingIndex = 1:size(args.bData,1)
            tError = haversine(args.bData{buildingIndex,2}, centGPS);
            if( tError < matcherror || matcherror == 0)
                matcherror = tError;
                bT = buildingIndex;
            end
        end
        matchedBuildingTemp = [matchedBuildingTemp bT];
        errorMatchedTemp = [errorMatchedTemp matcherror];
        boundingInCart = gps2cor(args.bData{bT, 3});
        boundingInCart = [boundingInCart(:,data.xcart), boundingInCart(:,data.ycart)];
        validMatchedTemp = [validMatchedTemp max(inpolygon(data.xind(areaIndex), data.yind(areaIndex), boundingInCart(:,1), boundingInCart(:,2)))];
    end
    results.gterror(data.queryCounter,:) = [ errorGTTemp];
    results.matchedBuilding(data.queryCounter,:) = [ matchedBuildingTemp];
    results.matchedError(data.queryCounter,:) = [errorMatchedTemp];
    results.centroids(data.queryCounter,:) = [centroidTemp];
    results.gtvalid(data.queryCounter,:) = [ validGTTemp];
    results.matchedValid(data.queryCounter,:) = [ validMatchedTemp];
    results.correctMatch(data.queryCounter,:) = [(matchedBuildingTemp == args.buildingGT(data.queryNum, bNums))];
end

end

function [args dirs data results] = PlotRegions(args, dirs, data, results)

[bNums fnd] = find(args.buildingGT(data.queryNum, :)>0);
colormp = flipud(jet(length(results.uniqueNumList)));
hold on;
for i=length(results.uniqueNumList):-1:1
    
    ind = results.summedMaps(:,data.queryCounter) == results.uniqueNumList(i);
    axis([min(data.xind) max(data.xind) min(data.yind) max(data.yind)])
    set(gca,'XTickLabel','','YTickLabel','')
    plot(data.xind(ind), data.yind(ind), 'o', 'MarkerEdgeColor', colormp(i,:), 'MarkerFaceColor', colormp(i,:));
    
end
    for j=1:length(bNums)
        bCor = gps2cor(args.bData{bNums(j), 2});
        bCor = floor([bCor(:,data.xcart), bCor(:,data.ycart)]);
        plot(bCor(1), bCor(2), 'ro', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r')
        text(bCor(1), bCor(2), args.bData{bNums(j),1});
    end
    %this problem is in the creation of nn contributing. go find the error.
    %nodecounter IS NOT WHAT WE NEED TO BE MAPPING TO WHAT IS GOING ON
    %HERE.
    for k=1:length(results.nnContributing)
        data.nodeCounter = results.nnContributing(k);
        [args dirs data results] = PlotNodes(args, dirs, data, results);
    end
	filename = [dirs.resultsDir 'topView.jpg'];
    saveas(gcf,filename, 'jpeg')
    hold off;

surf(results.logicalMap, 'EdgeColor', 'none')
hold on;

for i=1:length(bNums)
    bCor = gps2cor(args.bData{bNums(i), 2});
    bCor = floor([bCor(:,data.xcart), bCor(:,data.ycart)]);
    bCorI = floor([bCor(:,1)-min(args.pose2cor(:,1))+1, bCor(:,2)-min(args.pose2cor(:,2))+1]);
    %line([bCor(1);bCor(1)], [bCor(2);bCor(2)], [40;50], 'Color', 'r', 'LineWidth', 2);
end
	filename = [dirs.resultsDir 'surface.jpg'];
    saveas(gcf,filename, 'jpeg')

end

function [outIndex] = CalcThreshDist(inputMatrix, distance, nodeLocation)


end
