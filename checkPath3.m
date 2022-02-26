%% checkPath3.m	
function feasible=checkPath3(n,newPos,circleCenter,r)
feasible=true;
movingVec = [newPos(1)-n(1),newPos(2)-n(2)];
movingVec = movingVec/sqrt(sum(movingVec.^2)); %µ¥Î»»¯
if sqrt(sum((n-newPos).^2))>1
    lu=sqrt(sum((n-newPos).^2))/20;
else
    lu=0.1;
end
for R=0:lu:sqrt(sum((n-newPos).^2))
    posCheck=n + R .* movingVec;
    if ~(feasiblePoint3(roundn(posCheck,-1)-0.5*lu* movingVec,circleCenter,r))&&~(feasiblePoint3(roundn(posCheck,-1),circleCenter,r))
        feasible=false;break;
    end
end
if ~feasiblePoint3(newPos,circleCenter,r), feasible=false; end
end
