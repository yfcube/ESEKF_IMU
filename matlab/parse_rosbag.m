bag = rosbag('~/Desktop/20190122_01_imu_check_lxdd.bag');

imuSel = select(bag,'Topic','/imu/data');
gtSel = select(bag,'Topic','/navsat/odom');

imuCell = readMessages(imuSel);
gtCell = readMessages(gtSel);

imudata = zeros(length(imuCell),7);
gtdata = zeros(length(gtCell),11);
for i=1:length(imuCell)
    imudata(i,1) = imuCell{i}.Header.Stamp.Sec+imuCell{i}.Header.Stamp.Nsec*1e-9;
    imudata(i,2) = imuCell{i}.AngularVelocity.X;
    imudata(i,3) = imuCell{i}.AngularVelocity.Y;
    imudata(i,4) = imuCell{i}.AngularVelocity.Z;
    imudata(i,5) = imuCell{i}.LinearAcceleration.X;
    imudata(i,6) = imuCell{i}.LinearAcceleration.Y;
    imudata(i,7) = imuCell{i}.LinearAcceleration.Z;
end

for i = 1:length(gtCell)
    gtdata(i,1) = gtCell{i}.Header.Stamp.Sec+gtCell{i}.Header.Stamp.Nsec*1e-9;
    gtdata(i,2) = gtCell{i}.Pose.Pose.Position.X;
    gtdata(i,3) = gtCell{i}.Pose.Pose.Position.Y;
    gtdata(i,4) = gtCell{i}.Pose.Pose.Position.Z;
    gtdata(i,5) = gtCell{i}.Pose.Pose.Orientation.X;
    gtdata(i,6) = gtCell{i}.Pose.Pose.Orientation.Y;
    gtdata(i,7) = gtCell{i}.Pose.Pose.Orientation.Z;
    gtdata(i,8) = gtCell{i}.Pose.Pose.Orientation.W;
    gtdata(i,9) = gtCell{i}.Twist.Twist.Linear.X;
    gtdata(i,10) = gtCell{i}.Twist.Twist.Linear.Y;
    gtdata(i,11) = gtCell{i}.Twist.Twist.Linear.Z;
end

imuFormat = '%f %f %f %f %f %f %f\n';
gtFormat  = '%f %f %f %f %f %f %f %f %f %f %f\n';

imuFile = fopen('imudata.txt','w');
gtFile = fopen('gtdata.txt','w');

fprintf(imuFile,imuFormat, imudata');
fprintf(gtFile,gtFormat, gtdata');

fclose(imuFile);
fclose(gtFile);
