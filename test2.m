line = nrbmak([0.0 1.5; 0.0 3.0],[0.0 0.0 1.0 1.0]); 
nrbplot(line, 2);
 
coefs = cat(3,[0 0; 0 1],[1 1; 0 1]); 
knots = {[0 0 1 1]  [0 0 1 1]} 
plane = nrbmak(coefs,knots); 
nrbplot(plane, [2 2]); 
