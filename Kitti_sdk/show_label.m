function show_label(image_dir, label_dir, calib_dir, img_idx, data_set, h, nt_frames)
    % get sub-directories
    cam = 2; % 2 = left color camera



    % load projection matrix
   P = readCalibration(calib_dir,img_idx,cam);
  
  % load labels
  objects = readLabels(label_dir,img_idx);
  
   img = imread(sprintf('%s/%06d.png',image_dir,img_idx));
    
    % clear axes, draw image
    cla(h(1).axes); cla(h(2).axes);
    imshow(img,'parent',h(1).axes); axis(h(1).axes,'image','off'); hold(h(1).axes, 'on');
    imshow(img,'parent',h(2).axes); axis(h(2).axes,'image','off'); hold(h(2).axes, 'on');
    
    % title
    text(size(img,2)/2,3,sprintf('2D Bounding Boxes'),'parent',h(1).axes,'color','g','HorizontalAlignment','center','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(size(img,2)/2,3,sprintf('3D Bounding Boxes'),'parent',h(2).axes,'color','g','HorizontalAlignment','center','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    
    % legend
    text(0,00,'Not occluded','parent',h(1).axes,'color','g','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,30,'Partly occluded','parent',h(1).axes,'color','y','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,60,'Fully occluded','parent',h(1).axes,'color','r','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,90,'Unknown','parent',h(1).axes,'color','w','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(0,120,'Don''t care region','parent',h(1).axes,'color','c','HorizontalAlignment','left','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    text(size(img,2),0,sprintf('%s set frame %d/%d',data_set,img_idx,nt_frames-1), 'parent', h(1).axes,'color','g','HorizontalAlignment','right','VerticalAlignment','top','FontSize',14,'FontWeight','bold','BackgroundColor','black');
    %text(size(img,2)/2,size(img,1),sprintf('''SPACE'': Next Image  |  ''-'': Previous Image  |  ''x'': +1000  |  ''y'': -1000 | ''q'': quit'), 'parent', h(2).axes,'color','g','HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',14,'FontWeight','bold', 'BackgroundColor','black');

  % for all annotated objects do
  for obj_idx=1:numel(objects)
   
    % plot 2D bounding box
    drawBox2D(h,objects(obj_idx));
    
    % plot 3D bounding box
    [corners,face_idx] = computeBox3D(objects(obj_idx),P);
    orientation = computeOrientation3D(objects(obj_idx),P);
    drawBox3D(h, objects(obj_idx),corners,face_idx,orientation);
    
  end
 
end