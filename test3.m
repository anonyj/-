close all
clc
clear
tic
crv = nrbtestcrv; %���ز�������
% ���ƿ��Ƶ� 
plot(crv.coefs(1,:),crv.coefs(2,:),'bo') %����
title('�������߶�������1'); 
hold on; 
plot(crv.coefs(1,:),crv.coefs(2,:),'b--'); %����

% ����Nurbs���� 
nrbplot(crv,48); 
 
% ������������ by 1 
icrv = nrbdegelev(crv,100); 
 nrbplot(icrv,50)
% �����µĽ�㲢�����µĿ��Ƶ� 
plot(icrv.coefs(1,:),icrv.coefs(2,:),'ro') %����
plot(icrv.coefs(1,:),icrv.coefs(2,:),'r--'); %����
 
hold off;
toc