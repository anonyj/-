close all;
clear;
clc;
tic
% d=[0,5;3,2;4,7;6,1;8,5;9,2;11,7;16,5;18,15;21,3];
d=[0,5;3,2;4,7;6,1;7,3];
R=[1,1,1,1,1,1,1];
plot(d(:,1),d(:,2));
% R=[1,1,1,1,1,1,1,1,1,1];%控制系数
s=length(d(:,1));
M=[0,0,0,0];%1/6,2/6,3/6,4/6,5/6,1,1,1];%节点控制点
% for i=1:s-1
%     a=i/(s-1);
%     M=[M,a];
% end
% M=[M,1,1];
for i=1:s-3
    a=i/(s-3);
    M=[M,a];
end
M=[M,1,1,1];
bm=[];
if s==4
     k=M(1,1):(M(1,8)-M(1,1))/200:M(1,8);
     bm=k;                                                                                                                                                                                                                                                                                                                                        
else
 for i=1:s-3
     if i==1
     k=M(1,i+2):(M(1,i+4)-M(1,i+2))/200:M(1,i+4);
     elseif i==s-3
     k=M(1,i+3):(M(1,i+5)-M(1,i+3))/200:M(1,i+5);   
     else
     k=M(1,i+3):(M(1,i+4)-M(1,i+3))/200:M(1,i+4);
     end
     bm=[bm;k];
 end
end
A=[] ;
%% 起始段插补
k1=0.01:0.01:1;%第一段插补点数
for i=1:s-3
[S N1]=getnurbs(d(i:i+3,:),R(1,i:i+3),M(1,i:i+7),bm(i,:));
hold on
plot(S(:,1),S(:,2))
 A=[A;S];
end
toc
figure(2)
plot(A(:,1),A(:,2))