%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
%-------------------------------------------------------
% Authors:  J. Neira, J. Tardos
% Date   :  7-2002
%
% calculates the inverse of one or more transformations
%-------------------------------------------------------

function [wgs_lat,wgs_lon,wgs_yaw,wgs_ts_in_sec] = rtkraw2wgs (rtk_gps_lat, rtk_gps_lon, rtk_gps_yaw,rtk_gps_ts)

wgs_lat = floor(rtk_gps_lat/100)+(rtk_gps_lat-floor(rtk_gps_lat/100)*100)/60;
wgs_lon = floor(rtk_gps_lon/100)+(rtk_gps_lon-floor(rtk_gps_lon/100)*100)/60;

wgs_ts_s = rtk_gps_ts-floor(rtk_gps_ts/100)*100;
wgs_ts_h = floor(rtk_gps_ts/10000);
wgs_ts_m = floor((rtk_gps_ts-wgs_ts_h*10000 - wgs_ts_s)/100);
wgs_ts_in_sec = wgs_ts_h*3600+wgs_ts_m*60+wgs_ts_s;


wgs_yaw = rtk_gps_yaw*pi/360;
