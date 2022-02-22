function t=getT(x,u1,u2)
t=(x-u1)/(u2-u1);
if t<0
    t=0;
elseif t>u2
    t=u2;
end
end