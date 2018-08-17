%% Extracting the road plane within point cloud.
% written by xuesong li(xuesong.li@unsw.edu.au) on 29th July 2018
% Code is used to extract road plane and save into txt file.
% reading point cloud -> project into image --> filtering points located in the road--> extract plane -> save into txt file
%% one sentence one branch


function []=main(root_dir)
 
    clear; dbstop error; clc; close all;
    warning off;
    
    if nargin == 0
        root_dir  = '/home/ben/Dataset/KITTI/data_object_image_2/training';  %% training and testing are strictly separated
    end
    
    ground_height = - 1.3;
    longest_distance       = 70;      % the longest distance we can see
    back_threshold            =  2  ;    % threshold to remove back point
    
    result_dir = './training_data';
    
    
    velo_dir  = [root_dir, '/velodyne/'];
    assert(exist(velo_dir) == 7,'velody_dir does not exist');
    label_dir = [root_dir, '/label_2/'];
    assert(exist(label_dir) == 7,'label_dir does not exist');
    calib_dir = [root_dir, '/calib/'];
    assert(exist(calib_dir) == 7,'caliab_dir does not exist');
    image_dir = [root_dir, '/image_2/'];
    assert(exist(image_dir) == 7,'caliab_dir does not exist');
    
    road_dir = [root_dir, '/plane_image/'];
    assert(exist(road_dir) == 7,'image_plane_dir does not exist')
    
    calibs = dir(fullfile(calib_dir,'*.txt'));   %7518 files
    velos  = dir(fullfile(velo_dir,'*.bin'));    %7518 files
    images = dir(fullfile(image_dir,'*.png'));
    
    road_imgs = dir(fullfile(road_dir,'*.png'));
    %check the file later
    assert(length(road_imgs)==7481,'the number of training samples should be 7481')
    assert(length(calibs)==length(velos),'the file number should be the same among velody and calibration folder')
    assert(length(velos)==7481,'training samples should be 7481');
    
%     assert(exist('./Up_sampling/') == 7,'up_sample folder does not exist');
%     addpath('./Up_sampling/');
%     assert(exist('./Kitti_sdk/') == 7,'Kitti_sdk folder does not exist');
%     addpath('./Kitti_sdk/');
%     assert(exist('./Planes/') == 7,'Planes folder does not exist');
%     addpath('./Planes/');
%     assert(exist('./utiles/') == 7, 'Utility does not exist');
%     addpath('./utiles/');
    
    if exist(result_dir) ~=7
        mkdir(result_dir);
    end
    
    num_frame = length(velos)-1;
    fst_frame = 1;
    
    for i = fst_frame:1:num_frame
        % read velody data
        velo_file_path = [];
        cali_file_path = [];
        img_file_path = [];
        check1 =  split(velos(i).name,'.');
        n1 = str2num(cell2mat(check1(1)));
        check2 =  split(calibs(i).name,'.');
        n2 = str2num(cell2mat(check2(1)));
        check3 =  split(images(i).name,'.');
        n3 = str2num(cell2mat(check3(1)));
        check4 =  split(road_imgs(i).name,'.');
        n4 = str2num(cell2mat(check4(1)));
        assert(n1==n2,'name should be the same')
        assert(n1==n3,'name should be the same')
        assert(n1==n4,'name should be the same')
        
        velo_file_path = [velos(i).folder, '/',  velos(i).name];
        cali_file_path = [calibs(i).folder, '/', calibs(i).name];
        img_file_path  = [images(i).folder, '/', images(i).name];
        road_img_file_path = [road_imgs(i).folder,'/', road_imgs(i).name ]
        velo = filtering_point(velo_file_path, cali_file_path, img_file_path, road_img_file_path);
        % estimate the plane
        num_point = size(velo,1);
        % checking the point wheter road img will leave enough points
        if num_point > 1000   
            ground_point = velo(:,1:3);   %% keep the point cloud points filtered by road plane image
        else
            velo = filtering_point_non_road(velo_file_path, cali_file_path, img_file_path);
            idx = [];
            idx = velo(:,1)<back_threshold;
            velo(idx,:) = [];
            idx = [];      
            idx = velo(:,3)< ground_height;
            ground_point = velo(idx,1:3);
        end
        num_pc = size(ground_point , 1);
        
        dist_plane = [];
        plane_norm = [];
        if num_pc > 1000
            rand_no = floor(num_pc/10);
            if rand_no<=10    % at least 3 points are needed to estimate a plane
                rand_no = num_pc;
            end
            iter_no = 4;
            in_dist_thre = 0.1;
            in_no = floor(num_pc*3/10);      
            rand_no = floor(num_pc/10);
            [ plane_norm  dist_plane] = ransac_plane( ground_point, rand_no, iter_no, in_dist_thre , in_no);
            dist_pc2plane = velo(:,1:3)*plane_norm - dist_plane; % the distance of all points to plane
            idx=[];
            idx = find(dist_pc2plane < in_dist_thre*2);
            ground_point = [];
            ground_point = velo(idx,:);
            velo(idx,:) = [];            
        else
            %plane_norm = [-7.051739e-03 -9.997791e-01 -1.980150e-02] ;  %% setting as the default setting.
            plane_norm = [-1.980150e-02 ; 7.051739e-03 ; 9.997791e-01] ;  %% setting as the default setting.
            dist_plane   = -1.680367 ;
        end
        
%         if size(ground_point,1) <= 700
%              plane_norm = [-1.980150e-02  7.051739e-03  9.997791e-01] ;  %% setting as the default setting.
%              dist_plane   = -1.680367 ;           
%         end
        
        % save the plane paramters
        results_file = [result_dir, '/', calibs(i).name]; % result_name is similar with calibration
        fid = fopen(results_file,'w');
        fprintf(fid,'# Plane\n');
        fprintf(fid, 'Width %d\n',[4]);
        fprintf(fid, 'Height %d\n',[1]);
        fprintf(fid, '%f %f %f %f' ,[-plane_norm(2), -plane_norm(3), plane_norm(1), -dist_plane]);
        fclose(fid);
    end
    
    





end

function velo = filtering_point(velo_file_path, cali_file_path, img_file_path, road_img_file_path)
    % read and filter the input data
    fd = fopen(img_file_path);
    if fd < 1
        fprintf('Cound not open RGB image\n'); keyboard
    else
        img = imread(img_file_path);
    end
    fclose(fd);
    
    road_fp = fopen(road_img_file_path);
    if road_fp <1
        fprintf('Cound not open road png\n'); keyboard
    else
        road = imread(road_img_file_path);
    end
    fclose(road_fp);
    
    T = read_calib(cali_file_path);
    
    fd = fopen(velo_file_path,'rb');
    if fd < 1
        fprintf('No LIDAR files !!!\n');
        keyboard
    else
        velo = fread(fd,[4 inf],'single')';
        fclose(fd);
    end    
    idx = [];
    idx = velo(:,1)< 0.5;
    velo(idx,:) = [];    
    %% filtering
    px = (T.P2 * T.R0_rect * T.Tr_velo_to_cam * velo')';
    px(:,1) = px(:,1)./px(:,3);
    px(:,2) = px(:,2)./px(:,3);
    
    % round the px
    px(:,1) = round(px(:,1));
    px(:,2) = round(px(:,2));
    %
    
    % %  filter out out-of-boundary point-----------------------------------------------------------------------
    ix = px(:,1)<2;                 px(ix,:)=[];velo(ix,:) = []; ix=[];
    ix = px(:,1)>(size(img,2)-1);    px(ix,:)=[];velo(ix,:) = []; ix=[];
    ix = px(:,2)>size(img,1);    px(ix,:)=[];velo(ix,:) = []; ix=[];
    ix = px(:,2)<1;               px(ix,:)=[];velo(ix,:) = []; ix = [];
    % choose the point located in the road
    confid_road = impixel(road, px(:,1), px(:,2));
    ix = confid_road(:,1) < 140; 
    px(ix,:)=[];velo(ix,:) = [];
    
    
    % % Ordering
    Pts = zeros(size(px,1),4);
    Pts = sortrows(px,2);
    % % segmenting the point cloud into different layers    

end

function velo = filtering_point_non_road(velo_file_path, cali_file_path, img_file_path)
    % read and filter the input data
    fd = fopen(img_file_path);
    if fd < 1
        fprintf('Cound not open RGB image\n'); keyboard
    else
        img = imread(img_file_path);
    end
    fclose(fd);
    
%     road_fp = fopen(road_img_file_path);
%     if road_fp <1
%         fprintf('Cound not open road png\n'); keyboard
%     else
%         road = imread(road_img_file_path);
%     end
%     fclose(road_fp);
    
    T = read_calib(cali_file_path);
    
    fd = fopen(velo_file_path,'rb');
    if fd < 1
        fprintf('No LIDAR files !!!\n');
        keyboard
    else
        velo = fread(fd,[4 inf],'single')';
        fclose(fd);
    end    
    idx = [];
    idx = velo(:,1)< 0.5;   % remove too back points
    velo(idx,:) = [];    
    %% filtering
    px = (T.P2 * T.R0_rect * T.Tr_velo_to_cam * velo')';
    px(:,1) = px(:,1)./px(:,3);
    px(:,2) = px(:,2)./px(:,3);
    
    % round the px
    px(:,1) = round(px(:,1));
    px(:,2) = round(px(:,2));
    %
    
    % %  filter out out-of-boundary point-----------------------------------------------------------------------
    ix = px(:,1)<2;                 px(ix,:)=[];velo(ix,:) = []; ix=[];
    ix = px(:,1)>(size(img,2)-1);    px(ix,:)=[];velo(ix,:) = []; ix=[];
    ix = px(:,2)>size(img,1);    px(ix,:)=[];velo(ix,:) = []; ix=[];
    ix = px(:,2)<1;               px(ix,:)=[];velo(ix,:) = []; ix = [];
    % choose the point located in the road
    % confid_road = impixel(road, px(:,1), px(:,2));
    % ix = confid_road(:,1) < 140; 
    % px(ix,:)=[];velo(ix,:) = [];
    
    
    % % Ordering
    Pts = zeros(size(px,1),4);
    Pts = sortrows(px,2);
    % % segmenting the point cloud into different layers    

end