%% clean up
clc
clear
close all
%% Init the image
img_raw = imread('IMG_0098.jpeg');
imshow(img_raw);     % display image
hold on;           % and keep it there while we plot
[h,w] = size(img_raw);

color_list = hsv(50);

%% load
i_start = 0;
try
    load('marked_line.mat')
    num_fields = length(fieldnames(line_list))/2;
    for i = 1:num_fields
        temp_ij = strcat('ij_', num2str(i));
        ij_pts = line_list.(temp_ij);
        plot(ij_pts(1,:), ij_pts(2,:), '-.o', 'LineWidth',3, ...
            'MarkerSize',12, 'Color',color_list(i,:), ...
            'DisplayName',num2str(i));
    end

    i_start = num_fields;
end
legend('-DynamicLegend', 'Location','bestoutside');

%% run
n = 10;

for i=i_start+(1:n)
    temp_xy = strcat('xy_', num2str(i));
    temp_ij = strcat('ij_', num2str(i));
    [line_list.(temp_xy), line_list.(temp_ij)] = readPoints(h, w, ...
        color_list(i,:), num2str(i));
end

save('marked_line.mat', 'line_list');