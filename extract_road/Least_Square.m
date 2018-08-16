% Author: Xuesong(Ben) Li <benzlee08@gmail.com> 
% University: UNSW 
% The function use the least square to find normal vector from a point
% cloud.

function [ n_est ro_est ] = Least_Square( p )
%Calculate mean of all points
pbar=mean(p);

%Asum = (p(:,:)-pbar)'*(p(:,:)-pbar); 
Asum = (p(:,:)-pbar);
Asum = cov(Asum);
%Asum = (p(:,:))'*(p(:,:)); 
[V ~]=eig(Asum);  %%eigenvector and eigenvalue,
 
%Calculate new normal vector
n_est=V(:,1);
 
%Calculate new ro
ro_est=dot(n_est,pbar);
 
% [X,Y]=meshgrid(min(p(:,1)):max(p(:,1)),min(p(:,2)):max(p(:,2)));
% Z=(ro_est-n_est(1)*X-n_est(2).*Y)/n_est(3);
end

