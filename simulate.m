%% simulate

x0 = [0,0,0,0]';

dt = .01;
time = 25;
x(:,1) = x0;
t = 0:dt:time-dt;

for i = 2:(time/dt)
    u(i) = 1*sin(t(i)/.25);
%     if(abs(x(2,i-1)-pi) < 1)
%         u(i) = K*([0,pi,0,0]'-x(:,i-1));
%     end
    x(:,i) = x(:,i-1)+cartPoleDynamics(x(:,i-1),u(i))*dt;

end

%%

plot(t,x(3,:))

%% animate trajectory
figure(2)
for i = 1:5:length(x)
    quiver(x(1,i),0, -1*cos(x(2,i)+pi/2),-1*sin(x(2,i)+pi/2),'-', 'LineWidth', 2, 'ShowArrowHead', 'off')
    hold on
    plot(-5:5, zeros(1, 11),'LineWidth', 2)
    plot(x(1,i)+-1*cos(x(2,i)+pi/2), -1*sin(x(2,i)+pi/2), 'o','MarkerSize',20,'LineWidth', 3)
    rectangle('Position',[x(1,i)-.15, -.05, .3, .1]','Curvature',0.2, 'LineWidth',2)
    quiver(x(1,i)-u(i)/2,0, u(i)/2, 0, '-', 'LineWidth', 4, 'MaxHeadSize',10)

    axis([-3 3 -1.5 1.5])
    hold off
    drawnow
end

%%

% Check Jacobian and implement LQR

% [A,B] = cartPoleJacobian([0,pi,0,0]', 0,[1,1,9.81,1]');
A = [0,0, 1, 0; 0,0, 0, 1;0, 981/100, 0, 0; 0,  981/50, 0, 0];
B = [0,0,1,1]';
% Try stabilizing with LQR
Q = diag([10,10,1,1]);
R = 1;
[K,S,e] = lqr(A,B, Q, R);

%% Simulate with LQR Also add very simple swing up controller

x0 = [0,0+.1,0,0]';
% K = [0,50, 0,0];
dt = .01;
time = 15;
x(:,1) = x0;
t = 0:dt:time-dt;

for i = 2:(time/dt)
    if(abs(x(2,i-1)-pi) < .5)
       u(i) = K*([0,pi,0,0]'-x(:,i-1));
    else
       u(i) = .15*(cartPoleEnergy(x(:,i-1))-cartPoleEnergy([0 pi 0 0]))*x(4,i-1)*cos(x(2,i-1));
    end
    u(i) = min(5, max(-5, u(i)));
%     u(i) = K*([0,pi,0,0]'-x(:,i-1));
    x(:,i) = x(:,i-1)+cartPoleDynamics(x(:,i-1),u(i))*dt;
end
