function NIdentifiedInstances = plotPositionsByFrame(trackMatrix, frameNumber, frameWidth, frameHeight)
%function NIdentifiedInstances = plotPositionsByFrame(trackMatrix, frameNumber, frameWidth, frameHeight)
%
% Example call NIdentifiedInstances = plotPositionsByFrame(tracks_matrix, 1, width, height);
%
% This function plots of the position of each instance on the frameNumber
% frame from the data provided in the matrix trackMatrix. The function also
% counts the actual number of instances identified in the frame. The first
% node on the instance (head) is plotted as a big dot. The rest of the
% nodes are plotted using connected line segments.
%
% INPUT:
%   trackMatrix: Track Matrix provided by the .hd5 file
%   frameNumber: Frame Number to be plotted;    Integer
%   frameWidth: Frame Width;                    Integer
%   frameHeight: Frame Height;                  Integer
%
% Output:
%  NIdentifiedInstances: Number of identified instances; Integer
% 
% Vijay Singh wrote this Nov 20 2020
%
%%
% Collect the data for the frame to be plotted.
frameData = squeeze(trackMatrix(frameNumber, :,:,:));

NInstance = size(frameData,3);      % Number of possible instances
NIdentifiedInstances = 0;           % Number of instances on the given frame

figure;
hold on;

for ii = 1:NInstance
    plot(squeeze(frameData(:,1,ii)),squeeze(frameData(:,2,ii)),'.-', 'MarkerSize',10, 'LineWidth',2);
    plot(squeeze(frameData(1,1,ii)),squeeze(frameData(1,2,ii)),'.', 'MarkerSize',20, 'LineWidth',2);
    if ~isnan(max(max(squeeze(frameData(:,:,ii)))))
        NIdentifiedInstances = NIdentifiedInstances + 1;
    end
end

box on;
xlim([0 frameWidth]);
ylim([0 frameHeight]);
title(['Frame Number ', num2str(frameNumber), ' Identified Instances ', num2str(NIdentifiedInstances)]);


