% This script is created originally on 13/09/2017, 
% Author: Xuesong(Ben) Li <benzlee08@gmail.com> 
% University: UNSW 
% All rights reserved 
%  This alogirthm is used to do clustering among point cloud, the metric to
%  cluster points is based on the Euclidean  distance


 %% Reading and showing the 3D point cloud ICP
 
function [] = mian()
    % %------------------------------------------------------------------------
    clear; dbstop error; clc; close all;
    warning off; %close all;

    %% global variable
    global h5;
    
    %% Some very important parameters
    forward_threshold       =   0.6 ;   % threshold to segment line per layer according to dept gap
    left_threshold               =  0.8;   % threshold to segment line per layer according to left gap
    dist_center_threshold = 1  ;    % the threshold to segment the centeral points
    
    threshold_ration  = 0.02;        %0.02   %threshold = distance*ration 
    threshold_min_dist = 0.2 ;
    
    longest_distance       = 70;      % the longest distance we can see
    back_threshold            =  2  ;    % threshold to remove back point
    height_threshold          =  0.6;   % threshold to remove too high points
    ground_threshold        =  -1.0;   % threshold to remove ground plane

    %% setting up data dir
    root_dir = 'H:\Dataset\datasets\kitti\object\';  %% set up your dataset dir, we downloaded the KITTI dataset
    data_set = 'training';
    cam = 2; % 2 = left color camera
    image_dir = fullfile(root_dir,[data_set '\image_' num2str(cam)]);
    velo_dir = [root_dir,data_set,'\velodyne\'];
    label_dir = fullfile(root_dir,[data_set '\label_' num2str(cam)]);
    calib_dir = fullfile(root_dir,[data_set '\calib']);

    calib = dir(fullfile(calib_dir,'*.txt'));    % calibration files
    ima   = dir(fullfile(image_dir,'*.png'));    % image files
    addpath('.\Planes\');
    
    fst_frame =1; nt_frames = length(ima);
    addpath('.\Kitti_sdk\');
    %% initializing figures
    fig = figure(1);
    img = imread(sprintf('%s/%06d.png',image_dir,0));
    set(fig,'position',[10,560,  size(img,2), 1.2*size(img,1)]);
    h1.axes = axes('position',[0,0,1, 1]);
    h2=[];
    h3=[];
    h0=visualization('init',image_dir);
    sprintf('In the point cloud figure, \n''SPACE'': Next Image  \n''-'': Previous Image  \n''x'': +1000  \n''y'': -1000 \n''q'': quit')
    %% clustering and plotting
    for frame = fst_frame: 1: nt_frames
        velo =[];
        px    =[];
        %%reading all the data
        [velo, px , Pts, ImaRGB] = initialization( image_dir , frame, ima, calib_dir , calib , velo_dir ,...
                                                 back_threshold ,  height_threshold , ground_threshold, longest_distance) ;

        sub_1 = px(2:end,1) - px(1:end-1,1);
        id_1 = find(sub_1>500);
        seg_num = length(id_1);
        
        clearvars PCs PC_layer  PC_clusters;
        tic;
        for i = 2: seg_num
            PCs(i-1).data     = velo(id_1(i-1)+1:id_1(i),1:3); % saving point cloud coordinates with different matrixs.
            PCs(i-1).label =  zeros(size(PCs(i-1).data,1),1);
        end
    %% Labeling all the central points.  
        [PC_layer clusters]= labeling_weighted_point( PCs , threshold_ration, threshold_min_dist);  %the number of clusters should be (clusters -1)
    toc;
    
    %% categorize according to different clusters
    for jj = 1:clusters
        PC_clusters(jj).data = [];
        num_layer = size(PC_layer, 2);
        for kk = 1:num_layer
            ind_3 =[];
            ind_3 = find(PC_layer(kk).label == jj);
            len_ind = size(ind_3,1);
            if len_ind>=1
                PC_clusters(jj).data = [PC_clusters(jj).data ; PC_layer(kk).data(ind_3,:)]   ;
            end
        end        
    end
    
    %% show  the color for every clustering. 
      tic;
      data_visulization(PC_clusters , clusters-1, ImaRGB,  h1, Pts, h2, h3, velo); %the number of clusters should be (clusters -1)\
    
     %% controlling image change
       show_label( image_dir, label_dir, calib_dir, frame, data_set, h0, nt_frames);
       toc
       pause(0.2) ;  
       %waitting for response
       waitforbuttonpress; 
       key = get(gcf,'CurrentCharacter');
       switch lower(key)                         
           case 'q',  break;                                          % quit
           case '-',  frame = max(frame-1,  0);          % previous frame
           case 'x',  frame = min(frame+1000,nt_frames-1); % +100 frames
           case 'y',  frame = max(frame-1000,0);         % -100 frames
           otherwise, frame = min(frame+1,  nt_frames-1);  % next frame
       end 
   end
end

function  data_visulization(PC_clusters , clusters, ImaRGB,  h1, Pts, h2, h3, velo)
     %% find the color for every clustering.
     global h5;
     numb_colors = 0.2;
     color_line = eye(3);
     delete(h5);
     h5 = figure(2);
    for i = 1: clusters
        color_bin = de2bi(i, 9);
        r_sum      = color_bin(1) + color_bin(4) + color_bin(7);
        g_sum     = color_bin(2) + color_bin(5) + color_bin(8);
        b_sum     = color_bin(3) + color_bin(6) + color_bin(9);
        color_cluster = [ r_sum*numb_colors  g_sum*numb_colors  b_sum*numb_colors ];
       if( size(PC_clusters(i).data,1)>3)
            pcshow(PC_clusters(i).data, color_cluster);
            hold on;
            %%plot the number of label and data
            pc_mean = mean(PC_clusters(i).data(:,:));
            no_mean = size(PC_clusters(i).data(:,:),1);
            text(pc_mean(1),pc_mean(2),pc_mean(3),sprintf('(L:%d, No:%d)',i, no_mean) );

            %% plot 3D boxes
            x_max = max(PC_clusters(i).data(:,1));
            x_min = min(PC_clusters(i).data(:,1));
            y_max = max(PC_clusters(i).data(:,2));
            y_min = min(PC_clusters(i).data(:,2));        
            z_max = max(PC_clusters(i).data(:,3));
            z_min = min(PC_clusters(i).data(:,3)); 
            xx = [x_min; x_min; x_min; x_min; x_min];
            yy = [y_max; y_min; y_min; y_max; y_max];
            zz = [z_max; z_max; z_min; z_min; z_max];
            plot3(xx, yy, zz,'color',color_cluster);
            hold on;
            xx(:) = x_max;
            plot3(xx, yy, zz,'color',color_cluster);
            hold on;

            xx = [x_min; x_min; x_max; x_max; x_min];
            yy = [y_min; y_min; y_min; y_min; y_min];
            zz = [z_max; z_min; z_min; z_max; z_max];
            plot3(xx, yy, zz,'color',color_cluster);
            hold on;
            yy(:) = y_max;
            plot3(xx, yy, zz,'color',color_cluster);
             hold on;
             %% calculate the eigenvalue and eigenvector, plot the normal vector for every point clusters

            data_sum = []; AA =[];
            data_sum = ( PC_clusters(i).data(:,:) - pc_mean);
            AA = cov(data_sum);
            [V D]=eig(AA);
            len_m = size(V,1);
            for i = 1:len_m
                norm_ = [pc_mean(1,:); pc_mean(1,:)+ D(i,i)*V(:,i)'];
                plot3( norm_(:,1),norm_(:,2),norm_(:,3), 'LineWidth',2.5 , 'Color', color_line(i,:));   %([pc_mean(:)] , [pc_mean(:)+D(i,i)*V(:,i)], 'Color ', [0.3*i  0.3*i   0.3*i ], 'ipLength', 10*unit::mm)); 
                hold on;
            end
            
       end
    end
        title('Clusters') ;
    
        imshow(ImaRGB,'parent',h1.axes)
        hold(h1.axes, 'on') 
        h2 = plot(h1.axes,Pts(:,1),Pts(:,2),'.r'); 
        title(h1.axes,'Point cloud in image') ;
        figure(3)
        pcshow(velo(:,1:3));
        
        title('Point cloud');

    
end

function [velo, px, Pts , ImaRGB] = initialization( image_dir , frame, ima, calib_dir, calib , velo_dir ,...
                                             back_threshold ,  height_threshold , ground_threshold, longest_distance)
                              
    fd = fopen( fullfile(image_dir,ima(frame+1).name));
    if fd < 1
        fprintf('Cound not open RGB image !!!\n');    keyboard
    else
        ImaRGB = imread( fullfile(image_dir,ima(frame+1).name) );
    end
    fclose(fd);
    T = Fun_open_calib(calib(frame+1).name,calib_dir);
    fd = fopen(sprintf('%s%06d.bin',velo_dir,frame),'rb');
    if fd < 1
        fprintf('No LIDAR files !!!\n');
        keyboard
    else
        velo = fread(fd,[4 inf],'single')';
        fclose(fd);
    end
    % remove all points behind image plane (approximation)
    idx = velo(:,1)<back_threshold;
    velo(idx,:) = [];
    idx=[];
    idx = velo(:,1)>longest_distance;
    velo(idx,:) = [];
    idx=[];
    idx = velo(:,3)> height_threshold;
    velo(idx,:) = [];
    idx=[];
    %% find the road plane
    idx = velo(:,3)< ground_threshold;
    ground_point = velo(idx,1:3);
    num_pc = size(ground_point , 1);
    rand_no = floor(num_pc/100);
    if rand_no<=3    % at least 3 points are needed to estimate a plane
         rand_no = num_pc;
    end
    iter_no = 6;
    in_dist_thre = 0.1;
    in_no = floor(num_pc*3/10);
    [ plane_norm plane_dist] = ransac_plane( ground_point, rand_no, iter_no, in_dist_thre , in_no);
    dist_pc2plane = velo(:,1:3)*plane_norm - plane_dist; % the distance of all points to plane
    idx=[];
    idx = find(dist_pc2plane<in_dist_thre*2);
    ground_point = [];
    ground_point = velo(idx,:);
    velo(idx,:) = [];
     
    % project to image plane (exclude luminance)
    px = (T.P2 * T.R0_rect * T.Tr_velo_to_cam * velo')';
    px(:,1) = px(:,1)./px(:,3);
    px(:,2) = px(:,2)./px(:,3);
    % % -----------------------------------------------------------------------
    ix = px(:,1)<2;                 px(ix,:)=[];velo(ix,:) = [];
    ix = px(:,1)>(size(ImaRGB,2)-1);    px(ix,:)=[];velo(ix,:) = [];
    ix = px(:,2)>size(ImaRGB,1);    px(ix,:)=[];velo(ix,:) = [];
    % % Ordering
    Pts = zeros(size(px,1),4);
    Pts = sortrows(px,2);
    % % segmenting the point cloud into different layers
end