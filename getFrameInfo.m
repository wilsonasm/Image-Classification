function frameInfo = getFrameInfo(trackMatrix, frameNumber)
%function frameInfo = getFrameInfo(trackMatrix, frameNumber)
%
% Example call frameInfo = getFrameInfo(track_Matrix, 1);
%
% This function finds the head-body angle with respect to the x axis.
%
% INPUT:
%   trackMatrix: Track Matrix provided by the .hd5 file
%   frameNumber: Frame Number to be plotted;    Integer
%
% Output:
%  frameInfo: Struct with fields 
%               NIdentifiedInstances: Number of instances in frame
%               instanceIndex: Index of the instance in trackMatrix
%               positions: position of instances 
%               headAngle: angle of head-body line wrt to x-axis 
% 
% Vijay Singh wrote this Nov 21 2020
%
%%
frameInfo = struct();
frameInfo.NIdentifiedInstances = [];
frameInfo.instanceIndex = [];
frameInfo.headAngle = [];
% Collect the data for the frame to be plotted.
frameData = squeeze(trackMatrix(frameNumber, :,:,:));

nInstance = size(frameData,3);      % Number of possible instances
nIdentifiedInstances = zeros(1,nInstance);% Number of instances on the given frame

for ii = 1:nInstance
	if ~isnan(max(max(squeeze(frameData(:,:,ii)))))
        nIdentifiedInstances(1,ii) = 1;
    end
end

frameInfo.NIdentifiedInstances = sum(nIdentifiedInstances);
frameInfo.instanceIndex = find(nIdentifiedInstances);
%% Find positions of each instance
frameInfo.positions = frameData(:,:,frameInfo.instanceIndex);
xx = squeeze(frameData(1,1,frameInfo.instanceIndex) - frameData(2,1,frameInfo.instanceIndex));
yy = squeeze(frameData(1,2,frameInfo.instanceIndex) - frameData(2,2,frameInfo.instanceIndex));
zz = xx +1i* yy;
frameInfo.headAngle = angle(zz);
    

% %% Test the positions and angles
% figure; hold on;
% for ii = 1:frameInfo.NIdentifiedInstances
%     plot(squeeze(frameData(:,1,frameInfo.instanceIndex(ii))), ...
%         squeeze(frameData(:,2,frameInfo.instanceIndex(ii))),'.--', 'MarkerSize',20, 'LineWidth',2);
%     plot([squeeze(frameData(2,1,frameInfo.instanceIndex(ii))) squeeze(frameData(2,1,frameInfo.instanceIndex(ii)))+10*cos(frameInfo.headAngle(ii))], ...
%         [squeeze(frameData(2,2,frameInfo.instanceIndex(ii))) squeeze(frameData(2,2,frameInfo.instanceIndex(ii)))+10*sin(frameInfo.headAngle(ii))],'-', 'LineWidth',2);
%     plot([squeeze(frameData(1,1,frameInfo.instanceIndex(ii)))], ...
%         [squeeze(frameData(1,2,frameInfo.instanceIndex(ii))) ],'<', 'MarkerSize',20);
% end



