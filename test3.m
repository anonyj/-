close all
clc
clear
tic
crv = nrbtestcrv; %加载测试曲线
% 绘制控制点 
plot(crv.coefs(1,:),crv.coefs(2,:),'bo') %画点
title('测试曲线度数提升1'); 
hold on; 
plot(crv.coefs(1,:),crv.coefs(2,:),'b--'); %连线

% 绘制Nurbs曲线 
nrbplot(crv,48); 
 
% 度提升的曲线 by 1 
icrv = nrbdegelev(crv,100); 
 nrbplot(icrv,50)
% 插入新的结点并绘制新的控制点 
plot(icrv.coefs(1,:),icrv.coefs(2,:),'ro') %画点
plot(icrv.coefs(1,:),icrv.coefs(2,:),'r--'); %连线
 
hold off;
toc