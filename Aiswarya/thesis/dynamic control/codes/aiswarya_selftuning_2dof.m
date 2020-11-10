 

%Optimal control with system identification - S. Jagannathan
%UR10

clear all; close all; clc;
tic;



alpha=0.08;  v=0.5; K=0.1;
Q=eye(8);
R=eye(2);

%2 dof. total 4 states - 2 joint position angles, 2 joint angle velocities.
th0 = [0;0];
dth0 = [0;0];

x0=[th0;dth0]; %initial states
i=1;
% T=500;
% dt=0.01;

%desired states
dt=.1;
T=20;
time=0:dt:T;
thd1= zeros(1,length(time));
 thd2= sin(time);
% thd2= ones(1,length(time));

dthd1= zeros(1,length(time));
dthd2= cos(time);
% dthd2= zeros(1,length(time));


xr=[thd1;thd2;dthd1;dthd2];
xr0=xr(:,1);
X=[x0-xr0;xr0];
X_hat=X;
states=8;


nodes=2;
w1=ones(states,nodes);
w=ones(nodes,1);
lam=0.1;
 
 k=1;


for t=0:dt:T

    %neural network
  
    %TRIAL_ELM
%    w11=zeta(1,k)*ones(1,nodes);
%   w12=zeta(2,k)*ones(1,nodes);
%   w13=zeta(3,k)*ones(1,nodes);
%   w14=zeta(1,k)*ones(1,nodes);
%   w15=zeta(2,k)*ones(1,nodes);
%   w16=zeta(3,k)*ones(1,nodes);
  w11=[X(1:4,k)];
   w1=[w11 w11;w11 w11]; %since number of nodes=2, we need only 8*1 weight vector   
 
         lay=X(:,k)'*w1;
      phi=1./(1+exp(-lam*lay))
  
    
        for nm=1:states
            for nn=1:nodes
                del_phi(nn,nm)=w1(nm,nn)*lam*phi(1,nn)*(1-phi(1,nn));
            end
        end
       
        
        V(:,k)=w(:,k)'*phi';
  del_V=del_phi'*w(:,k);

  %Actual system dynamics

      F_d=[0 0 0 0;
           0 0 0 1;
           0 0 0 0;
           0 -1 0 0];
      F_desired=F_d*xr(:,k);
% F_desired=xr(:,k);
%F_desired=zeros(4,1);
     
  th1=X(1,k)+xr(1,k);     
th2=X(2,k)+xr(2,k);
dth1=X(3,k)+xr(3,k);
dth2=X(4,k)+xr(4,k);
gr=-9.8;
 
M =[ 2*cos((th2)) + 3 cos((th2)) + 1;  cos((th2)) + 1 1];

f=[- sin((th2))*(dth2)^2 - 2*sin((th2))*(dth1)*(dth2);
                                            sin((th2))*(dth1)^2];
g=[gr*(2*cos((th1)) + cos((th1) + (th2)));gr*cos((th1) + (th2))];

 
 
 F_system_pos=[0 0 1 0; 0 0 0 1]*(X(1:4,k)+xr(:,k));
 F_system_vel=-inv(M)*(g+f);
 
 F_system=[F_system_pos;F_system_vel];
 
 F=[F_system-F_desired; F_desired];
 G_system=[0 0;0 0;inv(M)];
 G=[G_system;zeros(4,2)];
     
%control law
u(:,k)= -0.5*inv(R)*G'*(del_phi'*w(:,k)); 
X(:,k+1)=X(:,k)+dt*(F+G*u(:,k));

%estimating F and X
  F_hat=-(Q-G*inv(R)*G'/4)*del_phi'*w(:,k);
%F_hat=F;
X_hat(:,k+1)=X_hat(:,k)+dt*(F_hat+G*u(:,k)+K*(X(:,k)-X_hat(:,k)));

fff(:,k)=F;
fff_hat(:,k)=F_hat;

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
plot(tim,X(1,2:end)+X(5,2:end))
hold on;
plot(tim,X(5,2:end))
title('Desired and Actual Trajectories of State 1')
xlabel('Time (sec)')
ylabel('Desired trajectory and Actual trajectory')
legend('Xd1-Actual' ,'X1-Desired')
grid on;

figure;
plot(tim,X(2,2:end)+X(6,2:end))
hold on;
plot(tim,X(6,2:end))
title('Desired and Actual Trajectories of State 2')
xlabel('Time (sec)')
ylabel('Desired trajectory and Actual trajectory')
legend('Xd1-Actual' ,'X1-Desired')
grid on;

figure;
 plot(tim,X(4,2:end)+X(8,2:end))
hold on;
plot(tim,X(8,2:end))
title('Desired and Actual Trajectories of State 4')
xlabel('Time (sec)')
ylabel('Desired trajectory and Actual trajectory')
legend('Xd2-Actual' ,'X2-Desired')
grid on;

figure;
plot(tim,fff(1,:)-fff_hat(1,:))
hold on;
plot(tim,fff(2,:)-fff_hat(2,:))
title('Augmented System dynamics erros - F-Fhat')
xlabel('Time (sec)')
ylabel('F-Fhat, error of System Dynamics (F)')
legend('error1_F' ,'error2_F')
grid on;

figure;
plot(tim,cost)
title('Optimal cost of the system')
xlabel('Time (sec)')
ylabel('Optimal cost, V')
legend('V')
grid on;



figure;
plot(fff(2,:))
hold on;
plot(fff_hat(2,:))
title('Augmented System dynamics (F) of state 2')
xlabel('Time (sec)')
ylabel('Actual and Estimated F')
legend('F2-Actual' ,'F_h2-Estimated')
grid on;

figure;
plot(tim,w(1,2:end))
hold on;
plot(tim,w(2,2:end))
plot(tim,w(3,2:end))
plot(tim,w(4,2:end))
plot(tim,w(5,2:end))
plot(tim,w(6,2:end))
plot(tim,w(7,2:end))
title('Evolution of Weights')
legend('w1','w2','w3','w4','w5','w6','w7')
xlabel('Time(sec)')
ylabel('Weights')
