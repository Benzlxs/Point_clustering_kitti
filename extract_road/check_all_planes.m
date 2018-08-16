%% Extracting the road plane within point cloud.
% written by xuesong li(xuesong.li@unsw.edu.au) on 29th July 2018
% Code is used to extract road plane and save into txt file.
% reading point cloud -> extract plane -> save into txt file
%%


function []=main(root_dir)
 
    clear; dbstop error; clc; close all;
    warning off;
    
    if nargin == 0
        root_dir  = './data_testing';
    end
    
    all = dir(fullfile(root_dir,'*.txt'));
    

    
    num_frame = length(all);
    fst_frame = 1;
    
    for i = fst_frame:1:num_frame
        % read velody data
        file_path = [all(i).folder,'/',all(i).name];
        file = [];
        file = read_plane_paramters(file_path);
        
        display('checking %S',[all(i).name])
        
        assert(file.Plane(3)<-0.8,'checking file %s, plane is too slant',[all(i).name])
        assert(file.VarName4(3)>1.2,'checking file %s, distance is too small',[all(i).name])
        
    end


        % save the plane paramters
 
    





end

function velo = filtering_point(velo_file_path, cali_file_path, img_file_path)
    % read and filter the input data
    fd = fopen(img_file_path);
    if fd < 1
        fprintf('Cound not open RGB image\n'); keyboard
    else
        img = imread(img_file_path);
    end
    fclose(fd);
    
    T = read_calib(cali_file_path);
    
    fd = fopen(velo_file_path,'rb');
    if fd < 1
        fprintf('No LIDAR files !!!\n');
        keyboard
    else
        velo = fread(fd,[4 inf],'single')';
        fclose(fd);
    end    
    
    %% filtering
    px = (T.P2 * T.R0_rect * T.Tr_velo_to_cam * velo')';
    px(:,1) = px(:,1)./px(:,3);
    px(:,2) = px(:,2)./px(:,3);
    
    % % -----------------------------------------------------------------------
    ix = px(:,1)<2;                 px(ix,:)=[];velo(ix,:) = [];
    ix = px(:,1)>(size(img,2)-1);    px(ix,:)=[];velo(ix,:) = [];
    ix = px(:,2)>size(img,1);    px(ix,:)=[];velo(ix,:) = [];
    % % Ordering
    Pts = zeros(size(px,1),4);
    Pts = sortrows(px,2);
    % % segmenting the point cloud into different layers    

end