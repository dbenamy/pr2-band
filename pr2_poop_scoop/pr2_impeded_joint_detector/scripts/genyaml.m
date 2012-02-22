scale_factor = 5.0;  % the safety factor under which position error is allowed to deviate

[time,data] = rtpload('controller_data_left.rtp');
larm_lim(1) = max(abs(data.error.positions0))*scale_factor;
larm_lim(2) = max(abs(data.error.positions1))*scale_factor;
larm_lim(3) = max(abs(data.error.positions2))*scale_factor;
larm_lim(4) = max(abs(data.error.positions3))*scale_factor;
larm_lim(5) = max(abs(data.error.positions4))*scale_factor;
larm_lim(6) = max(abs(data.error.positions5))*scale_factor;
larm_lim(7) = max(abs(data.error.positions6))*scale_factor;


[time,data] = rtpload('controller_data_right.rtp');
rarm_lim(1) = max(abs(data.error.positions0))*scale_factor;
rarm_lim(2) = max(abs(data.error.positions1))*scale_factor;
rarm_lim(3) = max(abs(data.error.positions2))*scale_factor;
rarm_lim(4) = max(abs(data.error.positions3))*scale_factor;
rarm_lim(5) = max(abs(data.error.positions4))*scale_factor;
rarm_lim(6) = max(abs(data.error.positions5))*scale_factor;
rarm_lim(7) = max(abs(data.error.positions6))*scale_factor;

fid = fopen('arm_controller_error_limits.yaml', 'w');
for i=1:7
  fprintf(fid, '%s: %f\n', strcat('r',num2str(i)),rarm_lim(i) );
  fprintf(fid, '%s: %f\n', strcat('l',num2str(i)),larm_lim(i) );
end
fclose(fid);
