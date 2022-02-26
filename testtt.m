% ����һ�����ݵ㣬����ֵ�㣬��������ƶ��㣬������ͨ����Щ��ֵ���2D��3D nurbs���ߣ���ĩ��ֵ������ƶ����غϣ���Ϊnurbs���߲�ֵ
% ����n����ֵ���k��nurbs��ֵ���ߣ�����n+2��δ֪���ƶ��㣬��ĩ�ڵ�ȡ�ظ���r=k+1,�Ӷ�����n+2+k+1���ڵ�
% ����Nurbs���ߵ���ĩ�ڵ�ȡ�ظ���r=k+1=4��u1=u2=u3=u4=0,Un+3=Un+4=Un+5=Un+6=1
clear
close all
clc


% ab=[0.25 0.5 0;1 1 2];
% circleCenter=ab;
% r=[0.1;0.3 ];
% [x,y,z]=sphere;
% for i = 1:length(ab(:,1))
%     mesh(r(i)*x+ab(i,1),r(i)*y+ab(i,2),r(i)*z+ab(i,3));hold on;
% end 
% axis equal
load('matlab3.mat')
load('zhangai.mat')
load('banjing.mat')
load('lujing.mat')
load('shuju.mat')
load('guocheng.mat')
plot(XXX(:,1),XXX(:,2), 'r')
hold on
a=ab(:,1);
b=ab(:,2);
para = [a-r, b-r, 2*r, 2*r];
n=length(ab(:,1));
for i=1:n
rectangle('Position', para(i,:), 'Curvature', [1 1],'edgecolor','k','facecolor','k');
end
axis equal
% pt=[0,0;3,2;3,4;4,2];
% pt=newpath(:,1:3)
pt=[close2(1,:);CUN;close2(end,:)];
% gwo=[];
% for i=1:4
% gwo=[gwo;XX(randperm(length(XX(:,1)),1),:)]
% end
% pt=[pt;gwo;];
pt=sortrows(pt,1);
[row,column]=size(pt);  %ÿ�д���һ����ֵ�㣬������������ʾx,y,z���꣬�������Է��������
n=length(pt);  %��ֵ������
k=3;  %��������
U=zeros(1,n+k+3);  %�ڵ�ʸ��
%��һ��������ڵ�ʸ��********************
if(column == 2)   % 2D curve
    x=pt(:,1);
    y=pt(:,2);
else              % 3D curve
    x=pt(:,1);
    y=pt(:,2);
    z=pt(:,3);
end

%n�����ݵ㣬���ù淶�����ҳ���������ֵ�����������
temp=zeros(1,n-1);
for i=1:n-1  %n�����ݵ㣬n-1���ҳ�
   if(column == 2)    % 2D curve
   temp(i)=sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2);
   else               % 3D curve
   temp(i)=sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2+(z(i+1)-z(i))^2);
   end
end
sumtemp=sum(temp);%˳����������ֵ��֮�����ĺ�,�����ҳ�
for i=1:k+1      %ǰk+1���ڵ�Ϊ0
    U(i)=0;
end
for i=n+k:n+k+3  %��k+1���ڵ�Ϊ1
    U(i)=1;
end
 %n�����ݵ㣬�ڽڵ�Ϊn-2����U(k+1)��Ϊ��ʼֵ
for i=k+1:n+k-2
    U(i+1)=U(i)+temp(i-k)/sumtemp;  %n-1���ҳ���U��k+1����U��n+k����n���ڵ�
end

%�ڶ���������n+2�����Ƶ㣬������ʸ�߽�*****************************
%���ƶ������ĩ�˵�͸�����ֵ�����ĩ�˵��غ�
if(column == 2)
    dpt1=[0 1];%���������ݵ���ʸ
    dptn=[-1 0];%����ĩ���ݵ���ʸ
else
    dpt1=[0 0 1];%���������ݵ���ʸ
    dptn=[-1 0 0];%����ĩ���ݵ���ʸ
end
dU=zeros(1,n+k+3);  %�ڵ���������U=Ui+1-Ui
for i=k+1:n+k-1
    dU(i)=U(i+1)-U(i);
end

%������Է������ÿ��ƶ���������A*D=E,AΪϵ������Ԫ��ΪB������������ֵ��D�ǿ��ƶ�����������E��������
A=zeros(n);
if(column == 2)
    E=zeros(n,2);  %2D curve
else
    E=zeros(n,3);  %3D curve
end
A(1,1)=1;  %��ʸ����a1=1,b1=c1=0
A(n,n)=1;  %��ʸ����an=bn=0,cn=1
E(1,:)=pt(1,:)-(dU(4)/3)*dpt1;    %�׶˵�����
E(n,:)=pt(n,:)-(dU(n+2)/3)*dptn;  %ĩ�˵�����
%����ϵ������A��Ԫ��a,b,c�Լ�������E��Ԫ��e��ֵ
for i=2:n-1  
    A(i,i-1)=dU(i+3).^2/(dU(i+1)+dU(i+2)+dU(i+3));                  %a��ֵ
    A(i,i)=dU(i+3)*(dU(i+1)+dU(i+2))/(dU(i+1)+dU(i+2)+dU(i+3))+...  %b��ֵ
        dU(i+2)*(dU(i+3)+dU(i+4))/(dU(i+2)+dU(i+3)+dU(i+4));
    A(i,i+1)=dU(i+2).^2/(dU(i+2)+dU(i+3)+dU(i+4));                  %c��ֵ
    E(i,:)=(dU(i+2)+dU(i+3))*pt(i,:);                               %e��ֵ
end
D=A\E;  %�ⷽ���飬���ȥ����ĩ�˵�Ŀ��ƶ�������
D=[pt(1,:);D;pt(n,:)]; %���ƶ������������ƶ�������ݵ��������������ĩ�˵�
[s,t]=size(D);  %s�����Ƶ�����

% %�����������Nurbs����*****************************
% dt=0.001;  %Nurbs���߲�ֵ�ܶȣ�dtԽС��Խ�⻬
% P=[];      %Nurbs����
% syms dx;   %����ڵ��ĵ�������Ϊ����dx
% for i=k+1:s
%     u=U;
%     d=sym(D);
%     %ÿ�ε�����d�ָ���ֵ
%     for m=1:k
%         for j=i-k:i-m
%             alpha(j)=(dx-u(j+m))/(u(j+k+1)-u(j+m));
%             d(j,:)=(1-alpha(j))*d(j,:)+alpha(j)*d(j+1,:);
%         end
%     end
%     %�������dx������ڵ�����[ui,ui+1]��m=3�Ŀ��Ƶ�d
%     M=subs(d(i-k,:),dx,(u(i):dt:(u(i+1)-dt))');    %�ڵ������ڵĲ�ֵ���滻dx
%     P=[P;double(M)];
% end
% M=subs(d(s-k,:),dx,1);
% P=[P;double(M)];%�������һ����

%���Ĳ������Ƹ�����ֵ�㡢���Ƶ㼰��Nurbs����*****************************
% pts=[D(:,1)';D(:,2)';D(:,3)'];%nrbmak�������õ��������洢x,y,z���꣬���������ǲ����������洢x,y,z���꣬�ڴ˽���ת��
pts=[D(:,1)';D(:,2)';];
crv=nrbmak(pts,U);
nrbplot(crv,1000);  %�ڶ��������ǲ�ֵ���������ֵԽ����nurbs����Խ�⻬

%���Ĳ������Ƹ�����ֵ�㡢���Ƶ㼰��Nurbs����*****************************
if(column == 2)  %2D curve
    plot(pt(:,1),pt(:,2),'*');  %������ֵ��
    hold on;
%     plot(D(:,1),D(:,2),'b-o');   %���ƿ��Ƶ�
    hold on;
%     plot(P(:,1),P(:,2),'r');     %����nurbs����
    nrbplot(crv,1000);    %ʹ��nurbs_toolbox�⺯������nurbs����
    hold on;  
else             %3D curve
    plot3(pt(:,1),pt(:,2),pt(:,3),'*r'); %������ֵ��
    hold on;
    plot3(D(:,1),D(:,2),D(:,3),'b-o');   %���ƿ��Ƶ�
    hold on;
%     plot3(P(:,1),P(:,2),P(:,3),'r');     %����nurbs����
    nrbplot(crv,1000);    %ʹ��nurbs_toolbox�⺯������nurbs����
    hold on;
end
figure(2)
%  nrbplot(crv,1000);  
% plot(close2(:,1),close2(:,2),"*") 
hold on
 plot(XXX(:,1),XXX(:,2),"r")
 hold on
n=length(ab(:,1));
for i=1:n
rectangle('Position', para(i,:), 'Curvature', [1 1],'edgecolor','k','facecolor','k');
end
axis equal
figure(3)
 nrbplot(crv,1000); 
 hold on 
 n=length(ab(:,1));
for i=1:n
rectangle('Position', para(i,:), 'Curvature', [1 1],'edgecolor','k','facecolor','k');
end
axis equal