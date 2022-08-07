%% Using Casadi for optimization

addpath('C:\Users\Aiden\Documents\MATLAB\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*
x = MX.sym('x')
disp(jacobian(sin(x),x))

%% MPC setup
clear 
N = 100; % number of control intervals
% T = 10;

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory
% posx   = X(1,:);
% posy = X(2,:);
% velx = X(4,:);
% vely = X(5,:);
% pos = X(1:2,:);
U = opti.variable(1,N);   % control trajectory (throttle)
T = opti.variable();      % final time

% ---- objective          ---------
opti.minimize(T); % raise cart pole in minimal time

% ---- dynamic constraints --------

% f1 = [xdot; ydot; thetadot; 0; -g1; 0];
% g1 = [0,0;0,0;0,0;-sin(theta)/m, 0;cos(theta)/m,0;0,1/ixx];
ixx = 1;
m = 1;
grav = 9.81;
% f = @(x,u) [x(4);x(5);x(6);0;-grav;0] + ...
%     [0,0;0,0;0,0;-sin(x(3))/m, 0;cos(x(3))/m,0;0,ixx]*u; %[x(2);u-x(2)]; % dx/dt = f(x,u)

f = @(x,u) cartPoleDynamics(x,u);

% dt = T/N; % length of a control interval
dt = T/N;
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% % ---- path constraints -----------
% limit = @(pos) 1-sin(2*pi*pos)/2;
% opti.subject_to(speed<=limit(pos)); % track speed limit
% opti.subject_to(0<=U<=1);           % control is limited

% circPos = [4, 4];
% circlRad = 3;
% obstacle = @(pos) sqrt((pos(1,:)-circPos(1)).^2+(pos(2,:)-circPos(2)).^2);
% opti.subject_to(obstacle(pos)>=circlRad);
% opti.subject_to(-5<=U(2,:)<=5);           % control is limited

opti.subject_to(-10<=U(1,:)<=10);           % control is limited



% ---- boundary conditions --------
% opti.subject_to(posx(1)==0);   % start at position 0 ...
% opti.subject_to(posy(1)==0);   % start at position 0 ...
% opti.subject_to(velx(1)==0); % ... from stand-still 
% opti.subject_to(vely(1)==0); % ... from stand-still 
opti.subject_to(X(1:4,1)==zeros(4,1));
opti.subject_to(X(2,N+1) == pi);

% opti.subject_to(posx(N+1)==10); % finish line at position 1
% opti.subject_to(posy(N+1)==10);

% ---- misc. constraints  ----------
opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---
opti.set_initial(X(3:4), .1*ones(2,1));
% opti.set_initial(vely, 1);
opti.set_initial(T, 1);

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

%%

t = linspace(0,sol.value(T),N+1);
x = sol.value(X);
x = x(:,1:end-1);
u = sol.value(U);

%%
%try to simulate with MPC
%% simulate
clear
x0 = [0,0,0,0]';

dtt = .01;
time = 25;
x(:,1) = x0;
t = 0:dtt:time-dtt;

for i = 2:(time/dtt)
%     u(i) = 1*sin(t(i)/.25);
%     if(abs(x(2,i-1)-pi) < 1)
%         u(i) = K*([0,pi,0,0]'-x(:,i-1));
%     end


    N = 15; % number of control intervals

    opti = casadi.Opti(); % Optimization problem
    X = opti.variable(4,N+1); % state trajectory
    U = opti.variable(1,N);   % control trajectory (throttle)
    T = opti.variable();      % final time-----
    opti.minimize(T); % raise cart pole in minimal time

    f = @(x,u) cartPoleDynamics(x,u);

    dt = T/N;
    for k=1:N % loop over control intervals
       % Runge-Kutta 4 integration
       k1 = f(X(:,k),         U(:,k));
       k2 = f(X(:,k)+dt/2*k1, U(:,k));
       k3 = f(X(:,k)+dt/2*k2, U(:,k));
       k4 = f(X(:,k)+dt*k3,   U(:,k));
       x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
       opti.subject_to(X(:,k+1)==x_next); % close the gaps
    end

    opti.subject_to(-15<=U(1,:)<=15);           % control is limited

    del = 0.0;
    opti.subject_to(X(1:4,1)==x(:,i-1));
    opti.subject_to(X(2,N+1)==pi);
    
    % ---- misc. constraints  ----------
    opti.subject_to(T>=0); % Time must be positive

    % ---- initial values for solver ---
%     opti.set_initial(X(3:4), .1*ones(2,1));
    opti.set_initial(T, 1);

    % ---- solve NLP              ------
    opti.solver('ipopt'); % set numerical backend
    if(i~=2)
%     opti.set_initial(sol.value_variables())
    end
    sol = opti.solve();   % actual solve
    u1 = sol.value(U);
    u(i) = u1(1);
    x(:,i) = x(:,i-1)+cartPoleDynamics(x(:,i-1),u(i))*dtt;

end



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


