function plotAnglesByInstanceNumber(videoInfo, instanceNumber)

nFrames = size(videoInfo.labels ,2);
headAngle = NaN(1, nFrames);
motionAngle = NaN(1, nFrames);

for iterFrames = 1:nFrames
    instanceIndex = find(videoInfo.labels{iterFrames} == instanceNumber);
    if ~isempty(instanceIndex)
        headAngle(1, iterFrames) = videoInfo.headAngle{iterFrames}(instanceIndex);
        motionAngle(1, iterFrames) = videoInfo.motionAngle{iterFrames}(instanceIndex);
    end
end

headAngle = headAngle*180/pi;
motionAngle = motionAngle*180/pi;
angleDifference = headAngle - motionAngle;
angleDifference(angleDifference >= 180) = 360 - angleDifference(angleDifference >= 180);
angleDifference(angleDifference <= -180) = angleDifference(angleDifference <= -180) - 360;


figure;
hold on;
histogram(angleDifference,[-180:10:180]);
% for iterFrames = 1:nFrames
%     plot(headAngle(iterFrames), motionAngle(iterFrames), '.');
%     text(headAngle(iterFrames), motionAngle(iterFrames), num2str(iterFrames));
% end