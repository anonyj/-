%�������ܣ����B�����岹��λ�������Լ�bezier�岹����̬����
%������������λ��Qa����Ԫ��Qd���յ�λ��Qc����Ԫ��Qe�����ɵ���Ԫ��Qg��
%          λ��Qb
%����ֵ��  B�����岹�����λ������p��bezier�岹�������Ԫ������qqk
function p=bt(Qa,Qc,Qb)
N=30;
lambda=0:1/30:1;
for i=1:N+1
    p(i,:)=Qa*(lambda(i)^2-2*lambda(i)+1)+Qb*(-2*lambda(i)^2+2*lambda(i))+Qc*lambda(i)^2;
end
end