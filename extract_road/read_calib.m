% 
% 
function T = read_calib(file_path)
% ---------------------
fd = fopen(file_path);
if fd < 1
    fprintf('No files !!!\n');
    keyboard
else
   raw_data = fscanf(fd,'%c');
   fclose(fd);
end

ii = find(raw_data == ':')+1;
ie = find(raw_data == 10 );

T.P0 = str2num( raw_data(ii(1):ie(1)) );
T.P1 = str2num( raw_data(ii(2):ie(2)) );
T.P2 = str2num( raw_data(ii(3):ie(3)) );
T.P3 = str2num( raw_data(ii(4):ie(4)) );
T.R0_rect = str2num( raw_data(ii(5):ie(5)) );
T.Tr_velo_to_cam = str2num( raw_data(ii(6):ie(6)) );
T.Tr_imu_to_velo = str2num( raw_data(ii(7):ie(7)) );
% % ------------------------------------------------
T.P0 = reshape(T.P0,4,3)'; T.P0(4,:) = [0 0 0 1];

T.P1 = reshape(T.P1,4,3)'; T.P1(4,:) = [0 0 0 1];

T.P2 = reshape(T.P2,4,3)'; T.P2(4,:) = [0 0 0 1];

T.P3 = reshape(T.P3,4,3)'; T.P3(4,:) = [0 0 0 1];

T.R0_rect = reshape(T.R0_rect,3,3)'; T.R0_rect(:,4) = 0; T.R0_rect(4,:) = [0 0 0 1];

T.Tr_velo_to_cam = reshape(T.Tr_velo_to_cam,4,3)'; T.Tr_velo_to_cam(4,:) = [0 0 0 1];

T.Tr_imu_to_velo = reshape(T.Tr_imu_to_velo,4,3)'; T.Tr_imu_to_velo(4,:) = [0 0 0 1];


end %END Function
