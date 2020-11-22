function nIdentifiedInstances = labelInstances(trackMatrix, proximityRadius)
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
%  labeledTrackMatrix: track matrix with the correct labels
% 
% Vijay Singh wrote this Nov 21 2020
%
%%
% First find out the number of instances on each frame
nInstance = size(trackMatrix,4);            % Number of possible instances
nFrames = size(trackMatrix,1);              % Numebr of frames in video
nIdentifiedInstances = zeros(nFrames,nInstance); % Number of instances on the given frame

for iterFrames = 1:nFrames
    for iterInstance = 1:nInstance
        if ~isnan(max(max(squeeze(trackMatrix(iterFrames,:,:,iterInstance)))))
            nIdentifiedInstances(iterFrames,iterInstance) = 1;
        end
    end
end

labeledTrackMatrix = trackMatrix;   % intialization

labeledTrackMatrix = labeledTrackMatrix(:,:,:, ...
            [find(nIdentifiedInstances(1,:)) find(~nIdentifiedInstances(1,:))]);

for iterFrames = 2:nFrames
    % compare the number of instances in this frame as compared the last 
    % frame. 
    nIdentifiedInstances(iterFrames) 
end
    


        
        
        
        
