%函数功能：获得B样条插补的位置序列以及bezier插补的姿态序列
%输入参数：起点位置Qa，四元数Qd，终点位置Qc，四元数Qe，过渡点四元数Qg，
%          位置Qb
%返回值：  B样条插补过后的位置序列p，bezier插补过后的四元数集合qqk
function p=bt(Qa,Qc,Qb)
N=30;
lambda=0:1/30:1;
for i=1:N+1
    p(i,:)=Qa*(lambda(i)^2-2*lambda(i)+1)+Qb*(-2*lambda(i)^2+2*lambda(i))+Qc*lambda(i)^2;
end
end