function testLabelInstancesByFrame(videoInfo, width, height, frameNumber)
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
%   videoInfo: Struct with fields. Each field is a cell.
%               NIdentifiedInstances: Number of instances in frame
%               instanceIndex: Index of the instance in trackMatrix
%               positions: position of instances 
%               headAngle: angle of head-body line wrt to x-axis 
%               labels: Instance labels
%
% Output:
%  NIdentifiedInstances: Number of identified instances; Integer
% 
% Vijay Singh wrote this Nov 20 2020
%
%% Test the positions and angles
refreshFigure = 0;
figure; hold on;
for iterFrames = frameNumber
    NIdentifiedInstances = videoInfo.NIdentifiedInstances{iterFrames};
    instanceIndex = videoInfo.instanceIndex{iterFrames};
    positions = videoInfo.positions{iterFrames};
    headAngle = videoInfo.headAngle{iterFrames};
    labels = videoInfo.labels{iterFrames};
    for ii = 1:NIdentifiedInstances
%         plot(squeeze(positions(:,1,ii)), ...
%             squeeze(positions(:,2,ii)),'.--', 'MarkerSize',20, 'LineWidth',2);
%         plot([squeeze(positions(2,1,ii)) squeeze(positions(2,1,ii))+10*cos(headAngle(ii))], ...
%             [squeeze(positions(2,2,ii)) squeeze(positions(2,2,ii))+10*sin(headAngle(ii))],'-', 'LineWidth',2);
        plot([squeeze(positions(1,1,ii))], ...
            [squeeze(positions(1,2,ii)) ],'.', 'MarkerSize',10);
        text(squeeze(positions(1,1,ii))+10,squeeze(positions(1,2,ii))+10, num2str(labels(ii)));
    end
    refreshFigure = refreshFigure + 1;
    if mod(refreshFigure,5) == 0
        pause();
        clf; hold on;
        xlim([1, width]);
        ylim([1, height]);
    end
end



