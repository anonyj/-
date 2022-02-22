function      G=marxi(M)
a2=(getdui(M,5))^2/(getdui(M,4)+getdui(M,3)+getdui(M,5));
b2=(getdui(M,5)*(getdui(M,4)+getdui(M,3)))/(getdui(M,4)+getdui(M,3)+getdui(M,5))+(getdui(M,4)*(getdui(M,6)+getdui(M,5)))/(getdui(M,4)+getdui(M,6)+getdui(M,5));
c2=(getdui(M,4))^2/(getdui(M,4)+getdui(M,6)+getdui(M,5));
a3=(getdui(M,6))^2/(getdui(M,4)+getdui(M,6)+getdui(M,5));
b3=(getdui(M,6)*(getdui(M,4)+getdui(M,5)))/(getdui(M,4)+getdui(M,6)+getdui(M,5))+(getdui(M,5)*(getdui(M,6)+getdui(M,7)))/(getdui(M,7)+getdui(M,6)+getdui(M,5));
c3=(getdui(M,5))^2/(getdui(M,7)+getdui(M,6)+getdui(M,5));
G=[a2,b2,c2;a3,b3,c3];
end