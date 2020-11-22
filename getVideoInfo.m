function videoInfo = getVideoInfo(trackMatrix)
%function videoInfo = getVideoInfo(trackMatrix)
% 
% Example call videoInfo = getVideoInfo(tracks_matrix);
%
% This function provides the information regarding the instances on each
% frame of the video information provided by matrix trackMatrix.
%
% INPUT:
%   trackMatrix: Track Matrix provided by the .hd5 file
%
% OUTPUT
%   videoInfo: Struct with fields. Each field is a cell.
%               NIdentifiedInstances: Number of instances in frame
%               instanceIndex: Index of the instance in trackMatrix
%               positions: position of instances 
%               headAngle: angle of head-body line wrt to x-axis 
%
% Written by Vijay Singh Nov 22 2020
%%
videoInfo = struct();
totalFrames = size(trackMatrix, 1); % The number of frames in the video.

for iterFrames = 1: totalFrames
    frameInfo = getFrameInfo(trackMatrix, iterFrames);
    videoInfo.NIdentifiedInstances{iterFrames} = frameInfo.NIdentifiedInstances;
    videoInfo.instanceIndex{iterFrames} = frameInfo.instanceIndex;
    videoInfo.positions{iterFrames} = frameInfo.positions;
    videoInfo.headAngle{iterFrames} = frameInfo.headAngle;
end