% 切换到放 URDF 及网格文件的文件夹
robot = importrobot("youshou_urdf.urdf");
robot.DataFormat = "row";         % 关节配置用行向量
show(robot);                      % 可视化手模型
%%设置关节限位
bodies = robot.Bodies;
for i = 1:numel(bodies)
  if strcmp(bodies{i}.Joint.Type,"revolute")
    bodies{i}.Joint.PositionLimits = deg2rad([-90,90]);  
  end
end
% %% 碰撞检测
% [isColliding, sepDist] = checkCollision(robot, homeConfiguration(robot), ...
%                                          "ExcludeSelfCollision", false);
% fprintf("当前是否碰撞：%d，最小分离距离：%.3f m\n", isColliding, min(sepDist));