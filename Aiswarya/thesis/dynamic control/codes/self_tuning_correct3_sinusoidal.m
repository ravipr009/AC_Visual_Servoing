%Optimal control with system identification - S. Jagannathan
clear all; close all; clc;
tic;

% alpha=0.1;  v=200; K=10;
% Q=1*[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
% R=1;

alpha=1.6;  v=100; K=200;
% EE=20;
Q=1*[1 0 0 0;0 1 0 0;0 0 0 0;0 0 0 0];
R=1;

% x0=[-0.5;1.5];
% xr0=[0.5;0.5];
% X=[-1;1;0.5;0.5];
% X_hat=rand(4,1);
% x0=[1;-1];
% xr0=[0.1;0.1];
% X=[0.9;-1.1;0.1;0.1];
% X_hat=X;
x0=[1;-1];
xr0=[0.5;0.5];
X=[0.5;-1.5;0.5;0.5];
X_hat=X;

T=500;
dt=0.01;
k=1;
nodes=1;
lam=10;
w=zeros(nodes,1);

for t=0:dt:T
w11=X(1,k)*ones(1,nodes);
w12=X(2,k)*ones(1,nodes);
w1=[w11;w12;w11;w12];
    %neural network
    lay=X(:,k)'*w1;
    phi=1./(1+exp(-lam*lay));
    
     for nm=1:4
            for nn=1:nodes
                del_phi(nn,nm)=w1(nm,nn)*lam*phi(1,nn)*(1-phi(1,nn));
            end
        end
%  del_phi = [2*X(1,k) X(2,k) X(3,k) X(4,k) 0 0 0 0 0 0;
%            0 X(1,k) 0 0 2*X(2,k) X(3,k) X(4,k) 0 0 0;
%            0 0 X(1,k) 0 0 X(2,k) 0 2*X(3,k) X(4,k) 0;
%            0 0 0 X(1,k) 0 0 X(2,k) 0 X(3,k) 2*X(4,k)]';
%  phi = [X(1,k)^2; X(1,k)*X(2,k); X(1,k)*X(3,k); X(1,k)*X(4,k); X(2,k)^2; X(2,k)*X(3,k); X(2,k)*X(4,k); X(3,k)^2; X(3,k)*X(4,k); X(4,k)^2];
 V(:,k)=phi*w(:,k);
  del_V=del_phi'*w(:,k);

  %Actual system dynamics
%   F=[-X(1,k)+(X(1,k)+X(3,k))*(X(2,k)+X(4,k))+(X(2,k)+X(4,k))^2-X(4,k)*cos(X(4,k));
%       -sin((X(1,k)+X(3,k))^3)-2*X(2,k)+(sin(X(3,k)))^3-1.2*X(4,k);
%       -X(3,k)+X(4,k)*cos(X(4,k));
%       (-sin(X(3,k)))^3-0.8*X(4,k);];
%   G=[-1;1;0;0];
%  F=[X(2,k);
%      -5*(X(3,k)+X(1,k))^3-0.5*(X(2,k)+X(4,k))+5*X(3,k);
%  X(4,k);
%  -5*X(3,k);];
%  G=[0;1;0;0];

 F=[0 1 0 0;
     -5 -0.5 0 -0.5;
     0 0 0 .1; 
     0 0 -.5 0;]*X(:,k);
G=[0;1;0;0]; 

%control law
u(:,k)= -0.5*inv(R)*G'*(del_phi'*w(:,k));
    
X(:,k+1)=X(:,k)+dt*(F+G*u(:,k));

%estimating F and X
F_hat=-(Q-G*inv(R)*G'/4)*del_phi'*w(:,k);
% F_hat(3:4)=F(3:4);
X_hat(:,k+1)=X_hat(:,k)+dt*(F_hat+G*u(:,k)+K*(X(:,k)-X_hat(:,k)));

f(:,k)=F;
f_hat(:,k)=F_hat;

%Weight Updation
J_X=X_hat(:,k);
wdot=0.5*alpha*del_phi*G*inv(R)*G'*J_X-alpha*v*del_phi*(Q-G*inv(R)*G'/4)*(X(:,k)-X_hat(:,k));
w(:,k+1)=w(:,k)+dt*(wdot);
cost(k)=del_V'*Q*del_V+u(:,k)'*R*u(:,k);
k=k+1;
end

disp('final weights')
w(:,end)
tim=0:dt:T;
toc;

%plots
 plot(tim,X(1,2:end))
hold on;
plot(tim,X(2,2:end))
title('Error Trajectories of state 1 and 2')
xlabel('Time (sec)')
ylabel('Error Trajectories')
legend('e1-Error1' ,'e2-Error2')
grid on;

figure;
plot(tim,X(1,2:end)+X(3,2:end))
hold on;
plot(tim,X(3,2:end))
title('Desired and Actual Trajectories of State 1')
xlabel('Time (sec)')
ylabel('Desired trajectory and Actual trajectory')
legend('Xd1-Actual' ,'X1-Desired')
grid on;

figure;
 plot(tim,X(2,2:end)+X(4,2:end))
hold on;
plot(tim,X(4,2:end))
title('Desired and Actual Trajectories of State 2')
xlabel('Time (sec)')
ylabel('Desired trajectory and Actual trajectory')
legend('Xd2-Actual' ,'X2-Desired')
grid on;

figure;
plot(tim,f(1,:)-f_hat(1,:))
hold on;
plot(tim,f(2,:)-f_hat(2,:))
title('Augmented System dynamics erros - F-Fhat')
xlabel('Time (sec)')
ylabel('F-Fhat, error of System Dynamics (F)')
legend('error1_F' ,'error2_F')
grid on;

figure;
plot(tim,cost)
title('Optimal cost of the system')
xlabel('Time (sec)')
ylabel('Optimal cost, C')
legend('V')
grid on;



figure;
plot(f(2,:))
hold on;
plot(f_hat(2,:))
title('Augmented System dynamics (F) of state 2')
xlabel('Time (sec)')
ylabel('Actual and Estimated F')
legend('F2-Actual' ,'F_h2-Estimated')
grid on;

figure;
plot(tim,w(1,2:end))
hold on;
% plot(tim,w(2,2:end))
% plot(tim,w(3,2:end))
% plot(tim,w(4,2:end))
% plot(tim,w(5,2:end))
% plot(tim,w(6,2:end))
% plot(tim,w(7,2:end))
title('Evolution of Weights')
% legend('w1','w2','w3','w4','w5','w6','w7')
xlabel('Time(sec)')
ylabel('Weights')