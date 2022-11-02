function [y,err] = DisparityMatlab2(left,right,trueground,posee)

%global d focalLength principalPoint imageSize  reprojectionMatrix
global disparityswitch points3Dswitch ORBswitch
% a=left;
% b=right;
% left=b;
% right=a;
maxRange =30;  
persistent map3D2
if isempty(map3D2)
map3D2 = occupancyMap3D(10);
end
quaternion = angle2quat(0,0,0);
pose=[posee(2) -posee(1) -posee(3) quaternion];
if disparityswitch==1
d=0.2;   
focalLength    = [1109, 1109]; % In pixels
principalPoint = [640, 360];   % In pixels [x, y]
imageSize      = [720, 1280];  % In pixels [mrows, ncols]
baseline       = d;          % In meters

reprojectionMatrix = [1, 0, 0, -principalPoint(1); 
    0, 1, 0, -principalPoint(2);
    0, 0, 0, focalLength(1);
    0, 0, 1/baseline, 0];

intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);


stereoParams  = stereoParameters(intrinsics, intrinsics, eye(3), [-baseline, 0 0]);
[currILeft, currIRight, r] = rectifyStereoImages(left, right, stereoParams);

frameLeftGray  = rgb2gray(currILeft);
frameRightGray = rgb2gray(currIRight);

frameLeftGray  = rgb2gray(left);
frameRightGray = rgb2gray(right);
disparityMap = disparitySGM(frameLeftGray, frameRightGray);
% 
% disparityRange = [0 48];
% disparityMap = disparitySGM(frameLeftGray, frameRightGray,'DisparityRange',disparityRange,'UniquenessThreshold',20);

% imshow(disparityMap, [0, 64]);
% title('Disparity Map');
% colormap jet
% 
% J = imadjust(disparityMap,[1 60]);
% 
% cmap = jet(256);
% % a(isnan(disparityMap))=0;
% % rgbImage = cat(3, a, a, a);
% J = ind2gray(RGB2,cmap);
% J = rgb2ind(RGB2,cmap);
%  RGB2 = im2uint8(rgbImage);
% a=disparityMap;
% a(a==NaN) = 0;






% 
%  figure; 
%  imshow(stereoAnaglyph(currILeft,currIRight));



%SELECT POINTS BETWEEN  3.2 and 3.7 meters away from the camera.
% Z = xyzPoints(:,:,3);
% mask = repmat(Z > 3200 & Z < 3700,[1,1,3]);
% J1(~mask) = 0;
%  imshow(J1,'InitialMagnification',50);

depth=focalLength(1)*d./disparityMap;
depth(isinf(depth)|isnan(depth)) = 100000;       %I made zero inf or NaN take care!

e=abs(trueground-depth);

RGB2e =im2uint8(e / 350); %this value corta OJO
cmap = gray(60);            %this value corta OJO
Je = ind2gray(RGB2e,cmap);
rgbImagee = ind2rgb(Je, flipud(cmap));
% imshow(rgbImagee)

RGB2 =im2uint8(depth / 255); %this value corta OJO
cmap = hot(150);            %this 150 corta OJO

J = ind2gray(RGB2,cmap);
rgbImage = ind2rgb(J, cmap);





if points3Dswitch==1
points3D = reconstructScene(disparityMap, reprojectionMatrix);

% % ptCloud = pointCloud(points3D, 'Color', right);
% % pcshow(ptCloud)
% % Create a streaming point cloud viewer
% player3D = pcplayer([-15, 15], [-10, 10], [-0.3,50], 'VerticalAxis', 'y',  'VerticalAxisDir', 'down');
% %player3D = pcplayer([-15, 15], [-10, 10], [-20,20], 'VerticalAxis', 'y',  'VerticalAxisDir', 'down');
% % Visualize the point cloud
% view(player3D, ptCloud);
% 



ind = (-maxRange < points3D(:,:,1) & points3D(:,:,1) < maxRange ...
        & -maxRange  < points3D(:,:,2) & points3D(:,:,2) < maxRange ...
         & -maxRange  < points3D(:,:,3) & points3D(:,:,3) < maxRange);
    
    points3D = points3D.*ind;

pcl = pointCloud(points3D);

pcl_wogrd_sampled = pcdownsample(pcl,'random',0.3);
% figure
% pcshow(pcl_wogrd_sampled, 'VerticalAxis', 'y',  'VerticalAxisDir', 'down')
%toc
% 
% tic
% gridStep = 0.01;
% pcl_wogrd_sampled_2 = pcdownsample(pcl,'gridAverage',gridStep);
% % % figure
% % % pcshow(pcl_wogrd_sampled_2, 'VerticalAxis', 'y',  'VerticalAxisDir', 'down')
% stepSize = floor(pcl.Count/pcl_wogrd_sampled_2.Count);
% indices = 1:stepSize:pcl.Count;
% ptCloudB = select(pcl,indices);
% toc
% figure
% pcshow(ptCloudB, 'VerticalAxis', 'y',  'VerticalAxisDir', 'down')


%ptCloudOut = pcdenoise(pcl,'NumNeighbors',1,'Threshold',0.00001);
% ptCloudOut = pcdenoise(pcl,'Threshold',0.1);
% figure
% pcshow(ptCloudOut, 'VerticalAxis', 'y',  'VerticalAxisDir', 'down')
% tic
ptCloudOut = pcdenoise(pcl_wogrd_sampled,'NumNeighbors',8,'Threshold',0.00001);
% toc
% figure
% pcshow(ptCloudOut, 'VerticalAxis', 'y',  'VerticalAxisDir', 'down')

% 
% ptCloudOutB = pcdenoise(ptCloudB,'NumNeighbors',8,'Threshold',0.00001);
% figure
% pcshow(ptCloudOutB, 'VerticalAxis', 'y',  'VerticalAxisDir', 'down')




% pcshow(pcl_wogrd_sampled, 'VerticalAxis', 'y',  'VerticalAxisDir', 'down')
 %pcshow(points3D,rgbImage, 'VerticalAxis', 'y',  'VerticalAxisDir', 'down')
% zlim([0,2])
% ylim([-2, 2])
% xlim([-2, 2])

% tic
points=ptCloudOut.Location;
% 
 points(any(isnan(points), 2), :) = [];
% xpoint=points3D(:,:,1);
% ypoint=points3D(:,:,2);
% zpoint=points3D(:,:,3);
points = [points(:,1) -points(:,3) -points(:,2)];
insertPointCloud(map3D2,pose,points,maxRange);

% toc
%  map3D2.setOccupancy(points,1);
%hold on

h=show(map3D2);
% view(175,75);
axis equal
 %  set(h,'CameraUpVector',[0,-1,-1])


end

% 
% 
% [ptCloudB,inlierIndices,outlierIndices] = pcdenoise(ptCloud,'NumNeighbors',1,'Threshold',0.0005,'PreserveStructure',true);
% 
% 
% 
% ptCloudA = pcdownsample(ptCloudB,'gridAverage',0.5,'PreserveStructure',true);
% ptCloudB.Count
% ptCloudA.Count
% player3D = pcplayer([-15, 15], [-10, 10], [-0.3,50], 'VerticalAxis', 'y',  'VerticalAxisDir', 'down');
% view(player3D, ptCloudA);


% Deleted points in downsampling
% stepSize = floor(ptCloudB.Count/ptCloudA.Count);
% indices = 1:stepSize:ptCloudB.Count;
% ptCloudB = select(ptCloudB, indices);
% 


%look for the points in meters radius from a point
% point = [0,0,3];
% radius = 5;
% [indices,dists] = findNeighborsInRadius(ptCloud,point,radius);
% ptCloudB = select(ptCloud,indices);
% figure
% pcshow(ptCloud)
% hold on
% plot3(point(1),point(2),point(3),'*')
% pcshow(ptCloudB.Location,'r')
% legend('Point Cloud','Query Point','Radial Neighbors','Location','southoutside','Color',[0 1 1])
% xlim([-15, 15])
% ylim([-10, 10])
% zlim([-0.3,50])
% hold off


% %clustering
% distThreshold = 1; %0.1  also works
% labels = segmentLidarData(ptCloud,distThreshold);
% figure
% hold on
% title('Segmented Clusters')% 
% m = mode(labels(:))
% labels2=labels;
% labels2(labels2==m) = [] ;
% m2 = mode(labels2(:));
% labels3=labels2;
% labels3(labels3==m2) = [];
% m3 = mode(labels3(:))
% labels4=labels3;
% labels4(labels4==m3) = [];
% m4 = mode(labels4(:));
% labels5=labels4;
% labels5(labels5==m4) = [];
% m5 = mode(labels5(:));
% pc1 = select(ptCloud,find(labels == m2));
% pcshow(pc1.Location,'g')
% hold on
% pc2 = select(ptCloud,find(labels == m3));
% pcshow(pc2.Location,'b')
% pc3 = select(ptCloud,find(labels == m4));
% pcshow(pc3.Location,'r')
% pc4 = select(ptCloud,find(labels == m5));
% pcshow(pc4.Location,'y')
%  xlim([-15, 15])
%  ylim([-10, 10])
%  zlim([-0.3,50])
% hold off
% figure
% pcshow(ptCloud.Location,'r')
%  xlim([-15, 15])
%  ylim([-10, 10])
%  zlim([-0.3,50])

% 
% %clustering
%  groundPtsIdx = segmentGroundFromLidarData(ptCloud);
% ptCloudWithoutGround = select(ptCloud,~groundPtsIdx,'OutputSize','full');
% distThreshold = 1;
% [labels,numClusters] = segmentLidarData(ptCloudWithoutGround,distThreshold);
% numClusters = numClusters+1;
% labels(groundPtsIdx) = numClusters;labelColorIndex = labels+1;
% pcshow(ptCloud.Location,labelColorIndex)
% colormap([hsv(numClusters);[0 0 0]])
% title('Point Cloud Clusters')
%  xlim([-15, 15])
%  ylim([-10, 10])
%  zlim([-0.3,50])



% [indices,dists] = findNeighborsInRadius(ptCloud,[0,0,0],40);
% ptCloudC = select(ptCloud,indices);
% minDistance = 0.03;
% [labels,numClusters] = pcsegdist(ptCloudC,minDistance);
% pcshow(ptCloudC.Location,labels)
% colormap(hsv(numClusters))
% title('Point Cloud Clusters')

%% Overlay depth and rgb
%     %Overlay depth and rgb
%     imgRgb=left;
%     imgDepthAbs= double(depth);
%   % Check dims.
%   assert(ndims(imgRgb) == 3);
%   assert(ndims(imgDepthAbs) == 2);
%   % Check sizes.
%   [H, W, D] = size(imgRgb);
%   assert(D == 3);
%   assert(all(size(imgDepthAbs) == [H, W]));  
%   % Check types.
%   assert(isa(imgRgb, 'uint8'));
%   assert(isa(imgDepthAbs, 'double'));  
%   imgDepth = imgDepthAbs - min(imgDepthAbs(:));
%   imgDepth = imgDepth ./ max(imgDepth(:));
%   imgDepth = uint8(imgDepth * 255);  
%   imgOverlay = reshape(imgRgb, [H*W 3]);
%   imgOverlay(:,3) = imgOverlay(:,2);
%   imgOverlay(:,2) = imgDepth(:);  
%   imgOverlay = reshape(imgOverlay, [H, W, 3]);
% figure
% imshow(imgOverlay)
%%


err=im2uint8(rgbImagee);
y=im2uint8(rgbImage);
else
err = uint8(ones(720,1280,3));
y = uint8(ones(720,1280,3));
end