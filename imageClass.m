%% Image Classification
%   In this matlab code we will be taking the data gathered from SLEAP and 
%   plotting to visualize our results. This code will produce a graph in
%   terms of positions for each instance and a graph of its relative angle.
%   We will also be graphing the velocity of the instance to find any
%   correlations between velocity and angle of head.
%
%   Last Modified on: 10/8/2020

%% Data Anaylsis
close all; clear;
inFile = 'C:/Users/Lobo/Documents/NCAT/Research/Image_Classification/experiment/experiment_small.analysis.h5';

occupancy_matrix = h5read(inFile,'/track_occupancy');

tracks_matrix = h5read(inFile,'/tracks');
% tracks_matrix is a 4D matrix labeled as such:
% [totalFrames, nodes , pos, tracks]

height = 1080;
width = 1920;
matrix_param = size(tracks_matrix);

totalFrames = matrix_param(1); % the amount of frames the vurrent video has
numNodes = matrix_param(2); % each instance has x amount of nodes
position = matrix_param(3); % 2D x,y for each instance
numTracks = matrix_param(4); % track that was determined by inferences

numInstances = 1;%size(tracks_matrix,3);
numFrames = size(tracks_matrix,1);

runFrames = 201;
lookAtTrack = 1;
determineTracks = 5;

%% Tracking Solution ==============================================================================================================

%to compute our own tracking determined by relative positions
selfTrack = nan(runFrames,2,2,determineTracks);

%distanceFrom = nan(numTracks,numTracks);
selfTrack(1,:,:,:) = tracks_matrix(1,1:2,:,1:determineTracks);
time = 1:runFrames;

for iterTrack = 1:determineTracks %iterate through each track for identify position for each frame
    for iterFrame = 2:runFrames %iterate through each frame to find the minimum distance
        % to compute our own tracking i will be finding the distance from each
        % head position on current Frame from the selected position of previous
        % frame. The minumum distance from the selected position will be the
        % next position of our cell.

        distanceFrom = sqrt((tracks_matrix(iterFrame,1,1,:)-selfTrack(iterFrame-1,1,1,iterTrack)).^2 + (tracks_matrix(iterFrame,1,2,:)-selfTrack(iterFrame-1,1,2,iterTrack)).^2) ;
         if (min(distanceFrom) > 180)
             break;
         end
         
        instance = find(distanceFrom == min(distanceFrom));
        nextPosition = tracks_matrix(iterFrame,1:2,:,instance);

       %selfTrack(iterFrame,:,:,iterTrack) = nextPosition;
    end
end

%% Plotting for visuals
% trackFig = figure();
% time = 1:runFrames;
% for iterTrack = lookAtTrack:determineTracks
%     plot3(time,selfTrack(1:runFrames,1,1,iterTrack),selfTrack(1:runFrames,1,2,iterTrack), '<-');
%     hold on;
% end
% xlabel('time');
% ylabel('x(t)');
% zlabel('y(t)');
% title('Matlab Tracking');
% lgd = legend;
% grid on;


%% Plotting SLEAP Tracking
sleapFig = figure();
time = 1:runFrames;
for iterTrack = lookAtTrack:determineTracks
    if(find(isnan(tracks_matrix(:,1,1,iterTrack))))
        hole = find(isnan(tracks_matrix(:,1,1,iterTrack)));
        tracks_matrix(hole(1):end,1,1,iterTrack) = nan;
    end
    plot3(time,tracks_matrix(1:runFrames,1,1,iterTrack),tracks_matrix(1:runFrames,1,2,iterTrack), '<-');
    hold on;
end

xlabel('time');
ylabel('x(t)');
zlabel('y(t)');
title('SLEAP Tracking');
lgd = legend;
grid on;


%% Angle via complex numbers ==============================================================================================================
% Takes the angle of the head and the body of the cell
% head = xHead, yHead
% body = xBody, yBody
% x = xBody-xHead
% y = yBody - yHead

angleOfhead = zeros(totalFrames,determineTracks);
for iterTrack = 1:determineTracks
    for iterFrame = 1:runFrames
        x = selfTrack(iterFrame,1,1,iterTrack)-selfTrack(iterFrame,2,1,iterTrack);
        y = selfTrack(iterFrame,1,2,iterTrack)-selfTrack(iterFrame,2,2,iterTrack);
        z = x+1i*y; %% 
        angleOfhead(iterFrame,iterTrack) = 180*angle(z)/pi;
    end
end

%% Angle via complex numbers using SLEAP Tracking
% Takes the angle of the head and the body of the cell
% head = xHead, yHead
% body = xBody, yBody
% x = xBody-xHead
% y = yBody - yHead

angleOfheadSLEAP = zeros(totalFrames,determineTracks);
for iterTrack = 1:determineTracks
    for iterFrame = 1:runFrames
        x = tracks_matrix(iterFrame,2,1,iterTrack)-tracks_matrix(iterFrame,1,1,iterTrack);
        y = tracks_matrix(iterFrame,2,2,iterTrack)-tracks_matrix(iterFrame,1,2,iterTrack);
        z = x+1i*y; %% 
        angleOfheadSLEAP(iterFrame,iterTrack) = 180*angle(z)/pi;
    end
end

%% Velocity of cell ============================================================================================================================
% velocity = zeros(totalFrames-1,determineTracks);
% dir = zeros(totalFrames-1,determineTracks);
% for iterTrack = 1:determineTracks
%     for iterFrame = 2:runFrames
%         dT = iterFrame - (iterFrame-1);
%         velocity(iterFrame,iterTrack) = sqrt((selfTrack(iterFrame,1,1,iterTrack)-selfTrack(iterFrame-1,1,1,iterTrack)).^2 - (selfTrack(iterFrame,1,2,iterTrack)-selfTrack(iterFrame-1,1,2,iterTrack)).^2)/dT;
%         dy =(selfTrack(iterFrame,1,2,iterTrack)-selfTrack(iterFrame-1,1,2,iterTrack));
%         dx =(selfTrack(iterFrame,1,1,iterTrack)-selfTrack(iterFrame-1,1,1,iterTrack));
%         z = dx+1i*dy;
%         dir(iterFrame,iterTrack) = 180*angle(z)/pi;
%     end
% end

%% Velocity of cell using SLEAP Tracking
velocitySLEAP = zeros(totalFrames-1,determineTracks);
dirSLEAP = zeros(totalFrames-1,determineTracks);
for iterTrack = 1:determineTracks
    for iterFrame = 2:runFrames
        dT = iterFrame - (iterFrame-1);
        velocitySLEAP(iterFrame,iterTrack) = sqrt((tracks_matrix(iterFrame,1,1,iterTrack)-tracks_matrix(iterFrame-1,1,1,iterTrack)).^2 - (tracks_matrix(iterFrame,1,2,iterTrack)-tracks_matrix(iterFrame-1,1,2,iterTrack)).^2)/dT;
        dy =(tracks_matrix(iterFrame,1,2,iterTrack)-tracks_matrix(iterFrame-1,1,2,iterTrack));
        dx =(tracks_matrix(iterFrame,1,1,iterTrack)-tracks_matrix(iterFrame-1,1,1,iterTrack));
        z = dx+1i*dy;
        dirSLEAP(iterFrame,iterTrack) = 180*angle(z)/pi;
    end
end

%% Cross Correlation SLEAP =======================================================================================================================
% need to do the cross correlation with a lag of
% (-numFrames+1):(numFrames-1)
cor_s = zeros(401,determineTracks);
lag = zeros(1,determineTracks);
for iterTrack = 1:determineTracks
    [cor_s(:,iterTrack) lags] = xcorr(angleOfheadSLEAP(:,iterTrack),dirSLEAP(:,iterTrack));
    nanFound = find(isnan(cor_s(:,iterTrack)));
    cor_s(nanFound,iterTrack) = 0;
    if max(cor_s(:,iterTrack)) == 0
        lag(1, iterTrack) = 0;
    else
        lag(1, iterTrack) = lags(find(cor_s(:,iterTrack) == max(cor_s(:,iterTrack))));
    end
    cor_s(isnan(cor_s(:,iterTrack)),iterTrack)=0;
end

sameTime = find(lags == 0); 

corrSLEAPFig = figure();

for iterTrack = 1:determineTracks
    plot(lags,cor_s(:,iterTrack));
    hold on;
end
xlabel('lag');
ylabel('Correlation Measure');
title('Cross Correlation between Angle of Head and Velocity Direction');
crosslgd = legend;
grid on;


%% Angle Difference =======================================================================================================================
angleDiff = zeros(201,5);
angleDiff2 = zeros(201,5);
angleDiffMin = zeros(201,5);
for iterTrack = 1:determineTracks
    for iterFrame= 2:runFrames
        angleDiff(iterFrame,iterTrack) = angleOfheadSLEAP(iterFrame,iterTrack)-dirSLEAP(iterFrame,iterTrack);
        angleDiff2(iterFrame,iterTrack) =dirSLEAP(iterFrame,iterTrack)-angleOfheadSLEAP(iterFrame,iterTrack);
        angleDiffMin(iterFrame,iterTrack) = min( angleDiff(iterFrame,iterTrack), abs(angleDiff2(iterFrame,iterTrack)) );
        if angleDiffMin(iterFrame,iterTrack) < -180
            angleDiffMin(iterFrame,iterTrack) = mod(angleDiffMin(iterFrame,iterTrack),180);
        elseif angleDiffMin(iterFrame,iterTrack) >= 180
            angleDiffMin(iterFrame,iterTrack) = angleDiffMin(iterFrame,iterTrack) -360;  
        end
    end
end

angleDiffMean = mean(mean(angleDiffMin(~isnan(angleDiffMin))));
angleDiffSTD = std(angleDiffMin)./mean(angleDiffMin);
angleDiffSTDMean = mean(mean(angleDiffSTD(~isnan(angleDiffSTD))));

angleDiffPlot = figure();
for iterTrack = 1:determineTracks
    plot(time,abs((angleDiffMin(:,iterTrack))));
    hold on;
end

xlabel('time');
ylabel('angle (deg)');
title('Angle Difference between angle of velocity and angle of head');
lgd = legend;
grid on;

%% Nomralize vectors
angleByNorm = zeros(runFrames,determineTracks);
for iterTrack = 1:determineTracks
    for iterFrame= 2:runFrames
        %vector of direction, 
        dy =(tracks_matrix(iterFrame,1,2,iterTrack)-tracks_matrix(iterFrame-1,1,2,iterTrack));
        dx =(tracks_matrix(iterFrame,1,1,iterTrack)-tracks_matrix(iterFrame-1,1,1,iterTrack));
        A = [dx; dy];
        aNorm = A/norm(A);
        %vector of angle
        x = tracks_matrix(iterFrame,1,1,iterTrack)-tracks_matrix(iterFrame,2,1,iterTrack);
        y = tracks_matrix(iterFrame,1,2,iterTrack)-tracks_matrix(iterFrame,2,2,iterTrack);
        B = [x; y];
        bNorm = B/norm(B);
        abDot = dot(aNorm,bNorm);
        aMag = sqrt((aNorm(1))^2 + (aNorm(2))^2); 
        bMag = sqrt((aNorm(1))^2 + (aNorm(2))^2);
        angleByNorm(iterFrame,iterTrack) = acosd(abDot);
        abDiff = A-B;
        abDiffAngle(iterFrame,iterTrack) = atand(abDiff(2)/abDiff(1));
    end
end

for iterTrack = 1:determineTracks
    plot(time,abDiffAngle(:,iterTrack), 'o');
    hold on;
end

% variantofDegree = std(angleByNorm);
% normFig = figure();
% for iterTrack = 1:determineTracks
%     plot(time,angleByNorm(:,iterTrack));
%     hold on;
% end
% xlabel('time');
% ylabel('angle (deg)');
% title('Angle by Normalizing angle and velocity direction');
% lgd = legend;
% grid on;


%% Histogram of angle difference ================================================================================================================
histFig = figure();
for iterTrack = 1:determineTracks
    histogram(angleDiff(:,iterTrack));
    hold on;
end

%% Standard Devation
stdArray = std(angleOfheadSLEAP,0,1);


%% Plotting angle of head
angleFig = figure();
time = 1:runFrames;
for iterTrack = lookAtTrack:determineTracks
    plot(time,angleOfhead(1:runFrames,iterTrack),'-');
    hold on;
end
xlabel('Time');
ylabel('Angle (deg)');
title('Angle of Head');
grid on;
axis([0 runFrames -200 200]);

%% Plotting angle of head from SLEAP
angleSLEAPFig = figure();
time = 1:runFrames;
for iterTrack = lookAtTrack:determineTracks
    plot(time,angleOfheadSLEAP(1:runFrames,iterTrack),'-');
    hold on;
end
xlabel('Time');
ylabel('Angle (deg)');
title('Angle of Head (SLEAP)');
grid on;
axis([0 runFrames -200 200]);

%% Plotting of velocity
% velFig = figure();
% time = 2;
% for iterTrack = 1:determineTracks
%     plot(time,velocity(2,iterTrack),'-');
%     hold on;
% end
% %axis([0 40 0  max(mean(velocity,2))]);
% xlabel('Time');
% ylabel('velocity');
% title('velocity of cell');
% grid on;

%% Plotting Dir
% dirFig = figure();
% time = 1:runFrames;
% for iterTrack = lookAtTrack:determineTracks
%     plot(time,dir(1:runFrames,iterTrack),'-');
%     hold on;
% end
% xlabel('time');
% ylabel('Angle (deg)');
% title('direction (SLEAP)');
% axis([0 runFrames -200 200]);

%% Plotting Dir SLEAP
dirSLEAPFig = figure();
time = 1:runFrames;
for iterTrack = lookAtTrack:determineTracks
    plot(time,dirSLEAP(1:runFrames,iterTrack),'-');
    hold on;
end
xlabel('time');
ylabel('Angle (deg)');
title('direction');
axis([0 runFrames -200 200]);

%% mean vel
% meanVelFig = figure();
% meanVel = mean(velocity,2);
% plot(1:runFrames, meanVel(1:runFrames));
% title('Mean Velocity');
% xlabel('time');
% ylabel('velocity');
