function videoInfo = labelInstances(trackMatrix, proximityRadius)
%function labeledTrackMatrix = labelInstances(trackMatrix)
%
% Example call labeledTrackMatrix = labelInstances(trackMatrix)
%
% This function labels the instances in each frame, ensuring that the 
%
% INPUT:
%   trackMatrix: Track Matrix provided by the .hd5 file
%   proximityRadius: The maximum distance at which the cell can be in next frame
%
% Output:
%  videoInfo: struct with the following fields
%           NIdentifiedInstances: Number of instances in frame
%           instanceIndex: Index of the instance in trackMatrix
%           positions: position of instances 
%           headAngle: angle of head-body line wrt to x-axis 
%           labels: Instance labels
%           motionAngle: angle of the head positions in two consecutive
%                   frames with respect to x-axis
%
% 
% Vijay Singh wrote this Nov 21 2020
% Vijay Singh modified this Nov 24 2020
%
%%
videoInfo = struct();
totalFrames = size(trackMatrix, 1);    % The number of frames in the video.

for iterFrames = 1: totalFrames
    frameInfo = getFrameInfo(trackMatrix, iterFrames);
    videoInfo.NIdentifiedInstances{iterFrames} = frameInfo.NIdentifiedInstances;
    videoInfo.instanceIndex{iterFrames} = frameInfo.instanceIndex;
    videoInfo.positions{iterFrames} = frameInfo.positions;
    videoInfo.headAngle{iterFrames} = frameInfo.headAngle;
end

videoInfo.labels{1} = [1:videoInfo.NIdentifiedInstances{1}];
videoInfo.motionAngle{1} = NaN(1,videoInfo.NIdentifiedInstances{1});

for iterFrames = 2:totalFrames
    % For every identified instance in the last frame, find the nearest
    % neighbour in the current frame. If it neighbour lies in the proximity
    % radius, call it the same label otherwise say there is no neighbour.
    % After all instances from previous frame are labeled, label the rest
    % of the instances in the current frame.
    NIdentifiedInstances = videoInfo.NIdentifiedInstances{iterFrames};
    positionsThisFrame = videoInfo.positions{iterFrames};
    positionsLastFrame = videoInfo.positions{iterFrames-1};
    videoInfo.labels{iterFrames} = zeros(1,videoInfo.NIdentifiedInstances{iterFrames});
    videoInfo.motionAngle{iterFrames} = NaN(1,videoInfo.NIdentifiedInstances{iterFrames});

    for iterInstances = 1 : NIdentifiedInstances
        distances = sum((squeeze(positionsLastFrame(1,:,:)) - ...
                            squeeze(positionsThisFrame(1,:,iterInstances))').^2);
        [minDistance, minPosition] = min(distances);
        
        if minDistance < proximityRadius^2
            videoInfo.labels{iterFrames}(iterInstances) = videoInfo.labels{iterFrames-1}(minPosition);
            zz = squeeze(videoInfo.positions{iterFrames}(1,:,iterInstances) - videoInfo.positions{iterFrames-1}(1,:,minPosition));
            videoInfo.motionAngle{iterFrames}(iterInstances) = angle(zz(1)+1i*zz(2));
        end
    end
    unlabeledInstances = find(videoInfo.labels{iterFrames}==0);
    videoInfo.labels{iterFrames}(unlabeledInstances) = max(videoInfo.labels{iterFrames-1}) + [1:length(unlabeledInstances)];
end
    


        
        
        
        
