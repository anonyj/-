clc;
clear;
close all;
%%障碍物
n=8;
% load("map1.mat")
% ab=wg(:,1:2);
% a=wg(:,1);
% b=wg(:,2);
% r=wg(:,3);

r=(rand(n,1)+0.03);
a=8*rand(n,1);
b=5*rand(n,1);
% ab=[3,2;6,1;5,4;7,3;8,4];
ab=[a,b];
a=ab(:,1);
b=ab(:,2);
para = [a-r, b-r, 2*r, 2*r];
for i=1:n
rectangle('Position', para(i,:), 'Curvature', [1 1],'edgecolor','k','facecolor','k');
end
axis equal
%%
circleCenter=ab;
source=[1 1];
goal=[9 4];
stepsize = 0.5;
stepsize2=0.8;
threshold = 0.1;
maxFailedAttempts = 10000;
display = true;
searchSize = [10 10];      %探索空间六面体
%% 绘制起点和终点
hold on;
plot(source(1),source(2),"*g");
hold on
plot(goal(1),goal(2),"*b");
RRTree1 = double([source -1]);
RRTree2=double([goal -1]);
RRTree3=[];
failedAttempts = 0;
pathFound = false;
newPoint2=goal;
choose=0;
ki1=source;
ki2=goal;
%%
tic;
while failedAttempts <= maxFailedAttempts  % loop to grow RRTs
    %% chooses a random configuration
        if rem(choose,2)==0
    if rand<0.7
        sample = rand(1,2) .* searchSize;   % rando m sample
    else
        sample=ki2;
    end
        else
     if rand<0.7
        sample = rand(1,2) .* searchSize;   % rando m sample
    else
        sample=ki1;
    end
    end
        if sample(1,1)>10||sample(1,2)>10||sample(1,1)<0||sample(1,2)<0
            continue
        end
    %% selects the node in the RRT tree that is closest to qrand
    [A1, I1] = min( distanceCost3(RRTree1(:,1:2),sample) ,[],1); % 指distanceCost3最小

    closestNode1 = RRTree1(I1(1),1:2);
%         movingVec1 = [sample(1,1)-closestNode1(1,1),sample(1,2)-closestNode1(1,2)];
%     movingVec1 = movingVec1/sqrt(sum(movingVec1.^2));  
%       newPoint1 = closestNode1 + stepsize * movingVec1;
    newPoint1 = Steer(sample, closestNode1, stepsize);
    [A2, I2] = min( distanceCost3(RRTree2(:,1:2),newPoint1) ,[],1); 
    closestNode2 = RRTree2(I2(1),1:2);
     newPoint2 = Steer(newPoint1, closestNode2, stepsize);
    movingVec2 = [newPoint1(1,1)-closestNode2(1,1),newPoint1(1,2)-closestNode2(1,2)];
    movingVec2 = movingVec2/sqrt(sum(movingVec2.^2));  
%       newPoint2 = closestNode2 + stepsize * movingVec2;

    if ~checkPath3(closestNode1, newPoint1, circleCenter,r+0.15) % if extension of closest node in tree to the new point is feasible
       aaa=1;
        failedAttempts = failedAttempts + 1;
        continue;
    end
    %是否碰到障碍物 重新选择newpoint
    
%     if distanceCost3(newPoint,goal) < threshold, pathFound = true;
%         break; 
%     end % goal reached%跳出循环
    [A, I3] = min( distanceCost3(RRTree1(:,1:2),newPoint1) ,[],1); % check if new node is not already pre-existing in the tree
   
    if distanceCost3(newPoint1,RRTree1(I3(1),1:2)) < threshold ,failedAttempts = failedAttempts + 1; 
        
        continue; 
    end 
    RRTree1 = [RRTree1; newPoint1 I1(1)]; % add node
    ki1=newPoint1;
   
    plot([closestNode1(1);newPoint1(1,1)],[closestNode1(2);newPoint1(1,2)]); 
    while 1
       kt = min( distanceCost3(RRTree1(:,1:2),newPoint2) ); 

     if ~checkPath3(closestNode2, newPoint2, circleCenter,r+0.15) ||newPoint2(1,1)>10||newPoint2(1,2)>10||newPoint2(1,1)<0||newPoint2(1,2)<0% if extension of closest node in tree to the new point is feasible
            RRTree3=RRTree1;
            RRTree1=RRTree2;
            RRTree2=RRTree3;
            choose=choose+1;
            mn=0;
            break;
     else
%          plot([closestNode2(1);newPoint2(1)],[closestNode2(2);newPoint2(2)]);
         if kt<0.2
              [Ao, Io] = min( distanceCost3(RRTree1(:,1:2),closestNode2) ,[],1);
             if checkPath3(RRTree1(Io,1:2), closestNode2, circleCenter,r+0.15)  
          RRTree1 = [RRTree1; newPoint2 Io(1)];
          RRTree2=[RRTree2;newPoint2 I2(1)];
          mn=1;
           plot([RRTree1(Io,1);closestNode2(1)],[RRTree1(Io,2);closestNode2(2)]);
             break
             end
         end
         hold on
          plot([closestNode2(1);newPoint2(1)],[closestNode2(2);newPoint2(2)]);
         closestNode2=newPoint2;
          RRTree2=[RRTree2;newPoint2 I2(1)];
         [A2, I2] = min( distanceCost3(RRTree2(:,1:2),newPoint1) ,[],1);
         newPoint2 = closestNode2 + stepsize2 * movingVec2;
         ki2=newPoint2;
         failedAttempts = 0;
         
     end
  
    end
%        distanceCost3(RRTree2(:,1:2),RRTree1(:,1:2))
     if kt < 0.2&&mn==1 
         pathFound = true;
         
        break; 
    end % goal reached%跳出循环
        
        
    
    
    

%     plot([closestNode2(1);newPoint2(1)],[closestNode2(2);newPoint2(2)]);
 
end
toc

% if display && pathFound, plot([closestNode(1);goal(1)],[closestNode(2);goal(2)]); end

% if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end
%% retrieve path from parent information
path =[];
prev1 = max(find(RRTree2(:,3)==I2(1)));
while prev1 > 0
    path = [RRTree2(prev1,1:2); path];
    prev1 = RRTree2(prev1,3);
end
prev2 = max(find(RRTree1(:,3)==Io(1)));
prev2 = RRTree1(prev2,3);
while prev2 > 0
    path = [path;RRTree1(prev2,1:2 )];
    prev2 = RRTree1(prev2,3);
end

plot(path(:,1),path(:,2),'b', 'Linewidth', 3)
pathLength = 0;
for i=1:length(path(:,1))-1, 
    pathLength = pathLength + distanceCost3(path(i,1:2),path(i+1,1:2)); 
end % calculate path length
disp(['路径长度1=',num2str(pathLength)]);
% figure(2)
% for i = 1:length(circleCenter(:,1))
%     mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
% % end
% axis equal
% hold on;
% plot(path(:,1),path(:,2));
% i=1;
% j=2;
% close2=path(1,:);
% while 1
%     if j==length(path(:,1))+1
%         j=j-1;
%         close2=[close2;path(j,:)];
%         break
%     end
%       if  checkPath3(path(i,:),path(j,:),circleCenter,r)
%            j=j+1;
%       else
%           j=j-1;
%           close2=[close2;path(j,:)];
%           i=j;
%           j=j+1;
%       end
%    
% end


i=1;
j=(length(path(:,1)));
xia=1;shang=length(path(:,1));close2=path(1,:);
m=0;
while 1
    k= checkPath3(path(i,:),path(j,:),circleCenter,r+0.1);
   m=m+1;
   if k==1
       xia=j;
       if shang-j==1
          i=j;
          close2=[close2;path(i,:)];
          j=length(path(:,1));
          xia=i;
          shang=j;
          continue
       end
        if j==length(path(:,1))
            close2=[close2;path(j,:)];
              break
          end
       j=ceil((shang+j)/2);
   else
       shang=j;
    
      if j-xia==1
          i=j-1;
           close2=[close2;path(i,:)];
          j=length(path(:,1));
          xia=i;
          shang=j;
          continue
      end
     j=ceil((xia+j)/2);
   end
end

toc
pathLength=0;
for i=1:length(close2(:,1))-1
    pathLength = pathLength + distanceCost3(close2(i,1:2),close2(i+1,1:2)); 
end % calculate path length
disp(['路径长度2=',num2str(pathLength)]);

figure(1)
hold on
 plot(close2(:,1),close2(:,2),'r', 'Linewidth', 2)

 for i=1:length(close2(:,1))-2
 mq=0;
 ma=norm(close2(i+1,:)-close2(i,:))+norm(close2(i+1,:)-close2(i+2,:));
 while 1
     if mq==50
         break
     end
     sample=close2(i+1,:)+2*(rand(1,2)-0.5);
  if feasiblePoint3(sample,circleCenter,r+0.05)&&checkPath3(sample,close2(i,:),circleCenter,r+0.05)&&checkPath3(sample,close2(i+2,:) ,circleCenter,r+0.05)
     mb=norm(sample-close2(i,:))+norm(sample-close2(i+2,:));
     if ma>mb
         ma=mb;
         revise=sample;
         close2(i+1,:)=sample;
         mq=mq+1;
     end
  else
      mq=mq+1;
      continue
  end
 end

 end
   mup=close2;
   hold on
 plot(mup(:,1),mup(:,2), 'c', 'Linewidth', 2)
 toc
 pathLength=0;
 for i=1:length(close2(:,1))-1
    pathLength = pathLength + distanceCost3(close2(i,1:2),close2(i+1,1:2)); 
end % calculate path length
 disp(['路径长度3=',num2str(pathLength)]);
 
 cha=[];XX=[];
 for i=1:length(close2(:,1))-1
 Q=close2(i,:);
 W=close2(i+1,:);
w1=(Q(1):(W(1)-Q(1))/30:W(1))';
w3=(Q(2):(W(2)-Q(2))/30:W(2))' ;   
gx=[w1,w3] ;  
gx(end,:)=[];
XX=[XX;gx];
cha=[cha;length(XX(:,1))+1];
 end
XX(end+1,:)=close2(end,:);
  save('shuju','XX')   
XXX=XX;
     
  first=1   ;
  j=1;GGg=[];CUN=[];tf=[];
 for i=1:length(cha(:,1))-1 
  while 1
pw =bt(XX(cha(i)-(14-j),1:2),XX(cha(i)+(14-j),1:2),XX(cha(i),1:2));
chabw=pw ;
XXX=[XX(first:cha(i)-(15-j),:);chabw;];
% XXX=[XXX;XX(first:cha(i)-(10-j),:);chabw;];%XX(cha(i)+4:cha(i+1),:)
% shu=[first,cha(i)-4,cha(i)-3,cha(i),cha(i)+3,cha(i)+4,cha(i+1);shu]
for js=1:length(chabw(:,1))
 if feasiblePoint3(chabw(js,:),circleCenter,r+0.05)
  BA=0;

 else
     BA=1;
     break
 end
end
     if BA==1
       j=j+1;
       XXX=[];
       continue
     else
         xw1=norm([XX(cha(i)-(14-j),1:2)-XX(cha(i),1:2)]);
         xw2=norm([XX(cha(i)+(14-j),1:2)-XX(cha(i),1:2)]);
         gw=min(xw1,xw2);
         tf=[tf;XX(cha(i)-(14-j),1:2);XX(cha(i)+(14-j),1:2);XX(cha(i),1:2)]
%          CUN=[CUN;XX(cha(i)-(14-j),1:2);XX(cha(i)+(14-j),1:2)]
         CUN=[CUN;XX(cha(i),1:2)-gw*(XX(cha(i),1:2)-XX(cha(i)+(14-j),1:2))/norm([XX(cha(i)+(14-j),1:2)-XX(cha(i),1:2)]);XX(cha(i),1:2)+gw*(XX(cha(i)-(14-j),1:2)-XX(cha(i),1:2))/norm([XX(cha(i)-(14-j),1:2)-XX(cha(i),1:2)])]
         first=cha(i)+(15-j);
         j=1;
         break
     end
  end
GGg=[GGg;XXX];

 if i==length(cha(:,1))-1
XXX=[GGg;XX(cha(i)+(15-j):end,:)];
break
end
   
end
toc

  hold on 
  if length(cha(:,1))==2
      WG=XX;
  else
    WG=XXX;
  end
 
plot (WG(:,1),WG(:,2), 'y', 'Linewidth', 2)
 pathLength=0;
 for i=1:length(WG(:,1))-1
    pathLength = pathLength + distanceCost3(WG(i,1:2),WG(i+1,1:2)); 
end % calculate path length
 disp(['路径长度4=',num2str(pathLength)]);
 
 figure(2)
plot (source(:,1),source(:,2), '*')
hold on
plot (goal(:,1),goal(:,2), '*')
 hold on 
plot (XXX(:,1),XXX(:,2), 'r')
 for i=1:n
rectangle('Position', para(i,:), 'Curvature', [1 1],'edgecolor','k','facecolor','k');
end
axis equal
% r=r-0.03
blog=0;
if CUN(:,1)==0
else
for i=1:length(CUN(:,1))-1
   for j=i+1:length(CUN(:,1))
   if norm([CUN(i,:)-CUN(j,:)])<0.08
       blog=j;
   end
   end
end
end
if blog==0
else
CUN(blog,:)=[];
end
hold on
plot(tf(:,1),tf(:,2),"s")
hold on 
plot(CUN(:,1),CUN(:,2),"*")
save('matlab3','close2')
save('zhangai','ab')
save('banjing','r')
save('lujing','XXX')
save('guocheng','CUN')