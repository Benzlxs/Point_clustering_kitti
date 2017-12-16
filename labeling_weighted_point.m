function [PCs  Label]= labeling_weighted_point( PCs , threshold_ration , threshold_min_dist)
%% labeling the centers according to the distance
    global w_x w_y w_z;
    w_x = 1; %Velodyne: x: forward, y: left,  z: up
    w_y = 2;
    w_z = 0;
    Label = 0;
    num_layers = size(PCs , 2);

    for L = 1:num_layers
        num_points = size(PCs(L).label , 1);     %number of points in every layer
        for S = 1:num_points
            if PCs(L).label(S)==0
                Label = Label+1;
                PCs(L).label(S)  =  Label;                     % labeling the starting points
                PCs = Label_neighbors(L , S , Label, PCs, threshold_ration , threshold_min_dist);                
            end
        end
    end

end


function PCs = Label_neighbors(L , S , Label, PCs , threshold_ration , threshold_min_dist)
    % the format is (layer_index, segment_index)
    index = 1;
    sequence(index,:) = [L,S];   % sequence.push(L,S)
    while  index>=1,
        l  = sequence(index,1);
        s = sequence(index,2);
        sequence(index,:)=[]  ;     %sequence.pop(L,S)
        index=index-1;
        neighbors = [];
        neighbors = Find_neighbors(l , s , Label , PCs , threshold_ration ,threshold_min_dist);
        
        num_neigh = size(neighbors,1);
        for ii = 1:num_neigh
            l_neigh   = neighbors(ii,1) ;
            s_neigh  = neighbors(ii,2) ;
            index = index + 1;
            sequence(index,:) =  [l_neigh s_neigh] ;
            PCs(l_neigh).label(s_neigh) = Label;       
        end            
    end

end

function dist_neigh = Three_distance( point1, point2)
    global w_x w_y w_z;   
    if (size(point1,1)~=size(point1,1))&(size(point1,1)~=1)
        error(' The input point must be 1*3')
    end
    dist_neigh = sqrt( w_x*(point1(1)-point2(1))^2 +  w_y*(point1(2)-point2(2))^2 + w_z*(point1(3)-point2(3))^2);
    
end


%% find the neighbors from four directions
function neighbors = Find_neighbors(l , s , Label , PCs, dist_center_threshold , threshold_min_dist)
    global w_x w_y w_z;   
    dist_threshold_lr = max([PCs(l).data(s,1)*dist_center_threshold , threshold_min_dist]);
    dist_threshold_bf =  max([PCs(l).data(s,1)*dist_center_threshold*2 , threshold_min_dist]);
    
    index = 1;
    neighbors = [];
    % find horizontal neighbors
    if s>1                          % find the right neighbor
        if PCs(l).label(s-1) == 0
            dist = Three_distance(PCs(l).data(s,:),PCs(l).data(s-1,:));
            if  dist < dist_threshold_lr
                neighbors(index,:) = [l s-1];
                index = index + 1;
            end
        end
    end
    max_point =  size(PCs(l).label, 1);
    if s < max_point          % find the left neighbors
        if PCs(l).label(s+1) == 0
            dist = Three_distance(PCs(l).data(s,:),PCs(l).data(s+1,:));
            if dist < dist_threshold_lr
                neighbors(index,:) = [l s+1];
                index = index + 1;
            end
        end
    end
    %find the up neighbors
    max_layer = size( PCs , 2 );
    temp_data = [];
    if l < max_layer
            ind_3 = find(PCs(l+1).label==0);
            if  size(ind_3 , 1) >=1
                temp_data = PCs(l+1).data(ind_3 , :);
                distance = sqrt(  w_x*(PCs(l).data(s,1) - temp_data(:,1) ).^2   +   w_y*( PCs(l).data(s,2) - temp_data(:,2) ).^2   +   w_y*( PCs(l).data(s,3) - temp_data(:,3) ).^2 );
                min_dist    = min(distance);
                     if  min_dist < dist_threshold_bf
                        min_ind_3     = find(distance==min_dist);
                        real_ind_3    =  ind_3(min_ind_3);
                        lens = length(real_ind_3);
                         for j = 1:lens
                            neighbors(index,:) = [l+1 real_ind_3(j)];
                            index = index + 1;
                         end
                     end
            end
    end
    
    %find the down neighbors
    temp_data = [];
    if l >1
            ind_4 = find(PCs(l - 1).label==0);
            if  size(ind_4 , 1) >=1
                temp_data = PCs(l - 1).data(ind_4 , :);
                distance = sqrt(  w_x*(PCs(l).data(s,1) - temp_data(:,1) ).^2   +   w_y*( PCs(l).data(s,2) - temp_data(:,2) ).^2   +   w_z*( PCs(l).data(s,3) - temp_data(:,3) ).^2 );
                min_dist    = min(distance);
                     if  min_dist < dist_threshold_bf
                        min_ind_4     = find(distance==min_dist);
                        real_ind_4    =  ind_4(min_ind_4);
                        lens = length(real_ind_4);
                         for j = 1:lens
                            neighbors(index,:) = [l-1 real_ind_4(j)];
                            index = index + 1;
                         end
                     end
            end
    end    
end

