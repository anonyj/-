close all
clear
clc
tic
d=[0,5;3,2;4,7;6,1;7,7;8,1,;9,9];
w=[1,1,1,1,1,1,1,];
plot(d(:,1),d(:,2))
s=length(d(:,1));
u=[0,0,0,0];%1/6,2/6,3/6,4/6,5/6,1,1,1];%节点控制点
for i=1:s-3
    a=i/(s-3);
    u=[u,a];
end
u=[u,1,1,1];
k=0.01:0.01:1;
i=0;j=1;
% k=1;
G=[];
while 1

v1=u(i+4)-u(i+3);
v2=u(i+5)-u(i+4);
v3=u(i+5)-u(i+3);
v4=u(i+6)-u(i+4);
v5=u(i+5)-u(i+2);
v6=u(i+6)-u(i+3);
v7=u(i+7)-u(i+4);
[N C CQ]=getmatrix(v1,v2,v3,v4,v5,v6,v7,w(1,i+1:i+4),d((i+1:i+4),:));
% t=getT(k(1,j),u(1,i+4),u(1,i+5))
% t=k(1,j)

% t=(k(1,j)-u(i+4))/(u(i+5)-u(i+4));
t=k(1,j);
As=(C(1,:)*t^3+C(2,:)*t^2+C(3,:)*t+C(4,:))/(CQ(1,:)*t^3+CQ(2,:)*t^2+CQ(3,:)*t+CQ(4,:));
% As=(C(1,:)*t^3+C(2,:)*t^2+C(3,:)*t+C(4,:))/(CQ(1,:)*t^3+CQ(2,:)*t+CQ(3,:)*t+CQ(4,:));
j=j+1;
% if t>u(i+5)
%     i=i+1;
%     j=1;
% end
if j>=length(k(1,:))
    i=i+1;
    if i==s-3
        break
    else
        j=1;
    end
end


hold on
% plot(As(:,1),As(:,2),"*")
G=[As;G];
end
toc
hold on
plot(G(:,1),G(:,2))