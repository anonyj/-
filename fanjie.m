clc
close all
clear
E=[0,0;1,5;6,3;7,7];
s=length(E(:,1));
M=[0,0,0,0];
for i=1:s-2
    a=i/(s-1);
    M=[M,a];
end
M=[M,1,1,1,1];
G=marxi(M);
MA=[1,0,0,0;
    G(1,:),0;
    0,G(2,:);
     0,0,0,1];
 p0=[0,1];
 pn=[-1,0];
 e(1,:)=E(1,:)-getdui(M,4)*p0/3;
 for i=2:s-1
 e(i,:)=(getdui(M,i+2)+getdui(M,i+3))*E(i,:)
 end
 e(end+1,:)=E(end,:)-getdui(M,6)*pn/3;
p=inv(MA)*e
p=[E(1,:);p;E(end,:)]
plot(p(2:5,1),p(2:5,2))
hold on
R=[1,1,1,1,1,1,1]
plot(E(:,1),E(:,2),"*")
A=nub(p,R,M)
