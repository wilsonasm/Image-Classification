function checkGetVideoInfo(videoInfo)

%% Test the positions and angles
for iterFrames = 1:size(videoInfo.NIdentifiedInstances,2)
    NIdentifiedInstances = videoInfo.NIdentifiedInstances{iterFrames};
    instanceIndex = videoInfo.instanceIndex{iterFrames};
    positions = videoInfo.positions{iterFrames};
    headAngle = videoInfo.headAngle{iterFrames};
    
    figure; hold on;
    for ii = 1:NIdentifiedInstances
        plot(squeeze(positions(:,1,ii)), ...
            squeeze(positions(:,2,ii)),'.--', 'MarkerSize',20, 'LineWidth',2);
        plot([squeeze(positions(2,1,ii)) squeeze(positions(2,1,ii))+10*cos(headAngle(ii))], ...
            [squeeze(positions(2,2,ii)) squeeze(positions(2,2,ii))+10*sin(headAngle(ii))],'-', 'LineWidth',2);
        plot([squeeze(positions(1,1,ii))], ...
            [squeeze(positions(1,2,ii)) ],'<', 'MarkerSize',10);
    end
    pause();
    close
end