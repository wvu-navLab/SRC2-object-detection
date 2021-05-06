bag = rosbag('2021-04-26-15-40-14.bag');


bSel = select(bag,'Topic','/small_scout_1/disparity');
msgs_disparity = readMessages(bSel,'DataFormat','struct');
disparity_imgs = zeros(480,640,size(msgs_disparity,1));
t_disparity_imgs = zeros(size(msgs_disparity,1),1);
for i=1:size(msgs_disparity,1)
    t_disparity_imgs(i) = double(msgs_disparity{i}.Header.Stamp.Sec) + ...
         double(msgs_disparity{i}.Header.Stamp.Nsec)*1e-9;
    
    disparity_imgs(:,:,i) = rosReadImage(msgs_disparity{i}.Image,'Encoding','32FC1');
end

% REDUCE THE RESOLUTION 

%%
bSel = select(bag,'Topic','/small_scout_1/camera/left/image_raw');
msgs_img = readMessages(bSel,'DataFormat','struct');
imgs = zeros(480,640,3,size(msgs_disparity,1));
t_imgs = zeros(size(msgs_disparity,1),1);
counter = 1;
for i=1:size(msgs_img,1)
    time = double(msgs_img{i}.Header.Stamp.Sec) + ... 
         double(msgs_img{i}.Header.Stamp.Nsec)*1e-9;
    if any(time == t_disparity_imgs)
        t_imgs(counter) = time;
        imgs(:,:,:,counter) = rosReadImage(msgs_img{i},'Encoding','rgb8');
        counter = counter+1;
    end
end


%%
% figure
obstacle_imgs = zeros(size(disparity_imgs));
for index=1:size(disparity_imgs,3)
    
    % close all;
%     index = 350;

    di = disparity_imgs(:,:,index);

    di(di(:,:) == 0) = nan;

    base_img = zeros(480,640);
    for i=240:480
        mean_disparity_row = mean(di(i,65:end,1),'omitnan');
        base_img(i,65:end) = mean_disparity_row*ones(1,640-65+1);
    end

    % base_img = [zeros(240,320), zeros(240,320); zeros(240, 65), (linspace(4,188,240)'*ones(1,640-65))];

    segmented_img = di-base_img;
    std_segmented_img = mean(std(segmented_img,'omitnan'),'omitnan');

    obstacle_imgs(:,:,index) = segmented_img>std_segmented_img;

end

%%
figure
for index=380:410%size(disparity_imgs,3)
    if index == 1
        pause
    end

    subplot(121)
    imshow(imgs(:,:,:,index)/256)

    subplot(122)
    imagesc(obstacle_imgs(:,:,index))
    drawnow
    pause(0.1)
end
%% Making robust to roll of robot and pitch of camera
close all;

for index=400
    
    di = disparity_imgs(:,:,index);

%     di(di(:,:) == 0) = nan;

    figure
    plot(disparity_imgs(:,:,index))
    
    figure
    subplot(121)
    imshow(imgs(:,:,:,index)/256)

    subplot(122)
    imagesc(di)    
    
%     base_img = zeros(480,640);
%     for i=240:480
%         mean_disparity_row = mean(di(i,65:end,1),'omitnan');
%         base_img(i,65:end) = mean_disparity_row*ones(1,640-65+1);
%     end
% 
%     % base_img = [zeros(240,320), zeros(240,320); zeros(240, 65), (linspace(4,188,240)'*ones(1,640-65))];
% 
%     segmented_img = di-base_img;
%     std_segmented_img = mean(std(segmented_img,'omitnan'),'omitnan');
% 
%     obstacle_imgs(:,:,index) = segmented_img>std_segmented_img;

    figure
    surfl(disparity_imgs(:,:,index))
    shading interp

end

% MATLAB image processing of a horizon
% https://stackoverflow.com/questions/12470573/matlab-image-processing-of-a-horizon

I = di(:,66:end); % Convert to grayscale
BW = edge(I,'canny',[]); % Edge detection using Canny
imshow(BW);
[H,T,R] = hough(BW);

figure, imshow(H,[], 'XData', T, 'YData', R, 'InitialMagnification', 'fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
P  = houghpeaks(H,50,'threshold',ceil(0.5*max(H(:))));

% Set houghpeaks parameters, threshold unsure
x = T(P(:,2));
y = R(P(:,1));
plot(x,y,'s','color','white');

% Apply median filtering
I = medfilt2(I);

% Find lines and plot them
lines = houghlines(BW,T,R,P,'FillGap',45,'MinLength',50);
figure, imshow(I),imagesc(I), hold on
max_len = 0;
for k = 1:length(lines)
    xy = [lines(k).point1; lines(k).point2];
    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

    % plot beginnings and ends of lines
    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
end

showlines = struct(lines);
cellData = struct2cell(showlines);

% X-coordinates are for width
% Y-coordinates are for height
%point1(x y) etc
for i = 1:size(cellData,3)
    % 'A' stores all 'x' coordinates of point 1
    A([i,i+1])= [cellData{1,1,i}];
    % 'B' stores all 'x' coordinates of point 2
    B([i,i+1])= [cellData{2,1,i}];
    % 'C' stores all 'y' coordinates of point 1
    C([i,i])= [cellData{1,1,i}];
    % 'D' stores all 'y' coordinates of point 2
    D([i,i])= [cellData{2,1,i}];
end


%%
DI = di(:,66:end);
figure
imagesc(DI) 

m = (xy(1,2) - xy(2,2))/ (xy(1,1) - xy(2,1));
DI2 = DI
for i = 1:575
    y_line = xy(1,2) + m * (i - xy(1,1));
    for j = 1:480
        if j<y_line
            DI2(j,i) = NaN;
        end
    end
    
end
    
figure
imagesc(DI2) 

DI2(DI2==min(min(DI2))) = NaN

figure
imagesc(DI2) 

x = [];
y = [];
z = [];
for i=1:575
    for j=1:480
        if ~isnan(DI2(j,i))
            x = [x; i];
            y = [y; j];
            z = [z; DI2(j,i)];
        end
    end
end

params = [length(x), sum(x), sum(y); ...
    sum(x), sum(x.*x), sum(y.*x); ...
    sum(y), sum(x.*y), sum(y.*y);]...
    \[sum(z); sum(z.*x); sum(z.*y)];

DI_ground = zeros(480,575);
for i=1:575
    y_line = xy(1,2) + m * (i - xy(1,1));
    for j=1:480
        if j>y_line
           DI_ground(j,i) = params(1) + i*params(2) + j*params(3);
        end
    end
end

figure
imagesc(DI_ground) 

segmented_img = DI2-DI_ground

figure
imagesc(segmented_img) 

obstacle_imgs = false(size(segmented_img));
% for i=1:575
%     y_line = xy(1,2) + m * (i - xy(1,1));
%     for j=1:480
%         stddev = std(segmented_img(j,:),'omitnan');
%         if j>y_line && segmented_img(j,i) > stddev
%            obstacle_imgs(j,i) = true;
%         end
%     end
% end


std_segmented_img = median(std(segmented_img','omitnan'),'omitnan');
obstacle_imgs = segmented_img>std_segmented_img;

figure
imagesc(obstacle_imgs) 

