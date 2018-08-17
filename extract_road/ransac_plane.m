%% This script is created originally on 18/09/2017, 
% originDate = [2017,05,03]; revision = '1.0'; addcopyright(originDate, revision);

% Author: Xuesong(Ben) Li <benzlee08@gmail.com> 
% University: UNSW 
% All rights reserved 


% License: ***********


% This a new algorithm for fitting a plane to a set of point cloud with
% RANdom SAmple Censue
%   NOTES: <TODO>
% Iuput parameters:
%   pc: point cloud
%   rand_no: the number of randomly chosen points
%   iter_no: the number of iteration to find planes
%   in_dist_thre: the distance threshold to find whether point is beyond to a
%   plane or not
%   in_no: the number of points beyonding to the extracted plane.
% 
%Output parameters
%   plane_norm: is the plane normal vector,
%

%%
function  [plane_norm_r  dist_plane_r]  = ransac_plane( pc, rand_no, iter_no, in_dist_thre , in_no)

assert(size(pc,2)==3,' The input point must be n*3');
assert(rand_no>=3,'At least 3 points are needed to estimate a plane');
%% Setting initial value
success = 0; % to check wether the right plane has been found
error_min = inf;
while success == 0 
    iterations = 1;
    while iterations <= iter_no
        clearvars sample  in_pc in_new_pc norm_plane  dist_plane dist_pc2plane ind_1 ;%clear all the variables that may be employed in the next iteration, is critical to avoid some bugging problem
        
        %% find the plane with random points
        sample = randi( size(pc,1) , 1 , rand_no);
        in_pc = pc(sample,1:3);   % obtaining the coordinates of samples in point cloud
        [norm_plane  dist_plane]=Least_Square(in_pc) ;
        
        dist_pc2plane = pc(:,1:3)*norm_plane - dist_plane;   %calculate the distance of all point to plane
        dist_pc2plane = abs(dist_pc2plane);
        ind_1 = find( dist_pc2plane < in_dist_thre ); %find the points within a certain distance
        
        len1 = size(ind_1,1);
        norm_plane = [];
        dist_plane   = [];
        in_pc = [];
        dist_pc2plane = [];
        if len1 > in_no
            in_pc = pc(ind_1,1:3);
            [norm_plane  dist_plane]=Least_Square(in_pc) ;
            dist_pc2plane = pc(:,1:3)*norm_plane - dist_plane;   %calculate the distance of all point to plane
            dist_pc2plane = abs(dist_pc2plane);
            ind_2 = find( dist_pc2plane < in_dist_thre ); %find the points within a certain distance
            error(iterations) = sum(dist_pc2plane(ind_2))/size(ind_2,1);
            
            len2 = size(ind_1,1);
            norm_plane = [];
            dist_plane   = [];
            in_pc = [];
            dist_pc2plane = [];            
            
            if len2 > in_no
                in_pc = pc(ind_2,1:3);
                [norm_plane  dist_plane]=Least_Square(in_pc) ;
                dist_pc2plane = pc(:,1:3)*norm_plane - dist_plane;   %calculate the distance of all point to plane
                dist_pc2plane = abs(dist_pc2plane);
                ind_2 = find( dist_pc2plane < in_dist_thre ); %find the points within a certain distance    
                error(iterations) = sum(dist_pc2plane(ind_2))/size(ind_2,1);
            else
                error(iterations)  = inf;
            end
        else
            error(iterations)  = inf; 
        end
        
         
        if error(iterations) <= error_min
                error_min =error(iterations);
                plane_norm_r = norm_plane;    
                dist_plane_r = dist_plane;
                pc_in_r = in_pc;
        end
        
        if error_min ~= inf
            success = 1;
            
        end
        iterations = iterations + 1;
    end
    
    if error_min == inf
        in_no =  in_no/4;
    end
    
    if  in_no <= 700
         plane_norm_r = [-1.980150e-02 ; 7.051739e-03 ; 9.997791e-01] ;  %% setting as the default setting.
         dist_plane_r   = -1.680367 ;       
    end
    
end
end

