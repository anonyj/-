function  x_new = Steer(x_rand, x_near, StepSize)
% �����������x_rand����Ľڵ�x_near��x_rand������ƽ��StepSize�ľ��룬�����½ڵ�x_new
    dis = distance(x_near, x_rand);
    if dis==0
        dis=0.1;
    x_new(1) = ((dis-StepSize)*x_near(1) + StepSize*x_rand(1)) / dis;
    x_new(2) = ((dis-StepSize)*x_near(2) + StepSize*x_rand(2)) / dis;
    else
        
    % ǿ��֢�������½ڵ�����Ϊ������fix ����ȡ��(Ҳ�ɲ�ȡ����)
    x_new(1) = ((dis-StepSize)*x_near(1) + StepSize*x_rand(1)) / dis;
     x_new(2) = ((dis-StepSize)*x_near(2) + StepSize*x_rand(2)) / dis;
    end
    end