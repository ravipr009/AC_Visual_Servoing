 

%Optimal control with system identification - S. Jagannathan
%UR10

clear all; close all; clc;
tic;



alpha=.6;  v=50; K=200;
Q=eye(8);
R=0.1*eye(2);

%2 dof. total 4 states - 2 joint position angles, 2 joint angle velocities.
th0 = [0;0];
dth0 = [0;0];

x0=[th0;dth0]; %initial states
i=1;

%desired states
dt=.001;
T=10;
time=0:dt:T;
thd1= zeros(1,length(time));
 thd2= zeros(1,length(time));
dthd1= zeros(1,length(time));
dthd2= zeros(1,length(time));
ddthd1= zeros(1,length(time));
ddthd2= zeros(1,length(time));



xr=[thd1;thd2;dthd1;dthd2];
xr0=xr(:,1);
X=[x0-xr0;xr0];
X_hat=X;
states=8;
X_prev=[0;0;0;0;0;0;0;0];

nodes=1;
w1=rand(states,nodes);
w=rand(nodes,1);
lam=0.1;
 m1=1;m2=1;l1=1;l2=1;gr=-9.8;
 k=1;


for t=0:dt:T

  w11=[X(1:4,k)];
%    w1=[w11;w11]; %since number of nodes=1, we need only 8*1 weight vector   
  %w1=[w11 w11;w11 w11];
         lay=X(:,k)'*w1;
      phi=1./(1+exp(-lam*lay));
  
    
        for nm=1:states
            for nn=1:nodes
                del_phi(nn,nm)=w1(nm,nn)*lam*phi(1,nn)*(1-phi(1,nn));
            end
        end
       
        
        V(:,k)=w(:,k)'*phi';
  del_V=del_phi'*w(:,k);

  %Actual system dynamics
       
th1=X(1,k);     
th2=X(2,k);
dth1=X(3,k);
dth2=X(4,k);
th2dot=dth2; th1dot=dth1;
 
%M =[ 2*cos(conj(th2)) + 3 cos(conj(th2)) + 1;  cos(conj(th2)) + 1 1];

%f=[- sin(conj(th2))*conj(dth2)^2 - 2*sin(conj(th2))*conj(dth1)*conj(dth2);sin(conj(th2))*conj(dth1)^2];
%g=[gr*(2*cos(conj(th1)) + cos(conj(th1) + conj(th2)));gr*cos(conj(th1) + conj(th2))];

 
%% Mass matrix terms
m11=(l2)^2*m2+2*l1*l2*m2*cos(th2)+(l1)^2*(m1+m2);
m12=(l2)^2*m2+l1*l2*m2*cos(th2);
m21=(l2)^2*m2+l1*l2*m2*cos(th2);
m22=(l2)^2*m2;


% mass matrix
M = [m11,m12;
     m21,m22];
%% gravity vector
g1=m2*l2*cos(th1+th2)+(m1+m2)*l1*cos(th1);
g2=m2*l2*cos(th1+th2);

g = gr * [g1;g2];

%% vector of other effects
n1=-m2*l1*l2*sin(th2)*(th2dot)^2-2*m2*l1*l2*sin(th2);
n2=m2*l1*l2*sin(th2)*(th1dot)^2;
f = [n1;n2];
 
 % F=[X(3,k)+X(7,k); X(4,k);-inv(M)*(f+g)-[0;xr(2,k)];0;X(8,k);0;-0.5*X(6,k)];
  F=[X(3,k); X(4,k);-inv(M)*(f+g);dthd1(k); dthd2(k);ddthd1(k);ddthd2(k)];
 G=[0 0; 0 0; inv(M);0 0; 0 0; 0 0; 0 0];
 
%control law
% up = -0.5*inv(R)*G'*del_V;
% u(:,k)=up;
% 
% A=del_phi*G*inv(R)*G'*del_phi';
% e_h=X(:,k)'*Q*X(:,k)+w(:,k)'*del_phi*F-0.25*w(:,k)'*A*w(:,k);
% de_h=del_phi*F-0.5*A*w(:,k);
% X_dot=(X(:,k)-X_prev)/dt;
%alpha=(mu*norm(e_h,2)/norm(de_h*e_h,2))*(1+X(:,k)'*X_dot/(norm(X(:,k),2)));


u(:,k)= -0.5*inv(R)*G'*(del_phi'*w(:,k)); 
X(:,k+1)=X(:,k)+dt*(F+G*u(:,k));



%estimating F and X
F_hat=-(Q-G*inv(R)*G'/4)*del_phi'*w(:,k);
X_hat(:,k+1)=X_hat(:,k)+dt*(F_hat+G*u(:,k)+K*(X(:,k)-X_hat(:,k)));

fff(:,k)=F;
fff_hat(:,k)=F_hat;

%Weight Updation
J_X=X_hat(:,k);
wdot=0.5*alpha*del_phi*G*inv(R)*G'*J_X-alpha*v*del_phi*(Q-G*inv(R)*G'/4)*(X(:,k)-X_hat(:,k));
%J_X=X(:,k);
%wdot=0.5*alpha*del_phi*G*inv(R)*G'*J_X;
w(:,k+1)=w(:,k)+dt*(wdot);
X_prev=X(:,k);
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
plot(tim,X(2,2:end));
title('Error Trajectories of state 1 and 2')
xlabel('Time (sec)')
ylabel('Error Trajectories')
legend('e1-Error1' ,'e2-Error2')
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

% figure;
% plot(tim,fff(1,:)-fff_hat(1,:))
% hold on;
% plot(tim,fff(2,:)-fff_hat(2,:))
% title('Augmented System dynamics erros - F-Fhat')
% xlabel('Time (sec)')
% ylabel('F-Fhat, error of System Dynamics (F)')
% legend('error1_F' ,'error2_F')
% grid on;

figure;
plot(tim,cost)
title('Optimal cost of the system')
xlabel('Time (sec)')
ylabel('Optimal cost, c')
legend('C')
grid on;



% figure;
% plot(fff(2,:))
% hold on;
% plot(fff_hat(2,:))
% title('Augmented System dynamics (F) of state 2')
% xlabel('Time (sec)')
% ylabel('Actual and Estimated F')
% legend('F2-Actual' ,'F_h2-Estimated')
% grid on;
% 
figure;
plot(tim,w(1,2:end))
% hold on;
% plot(tim,w(2,2:end))
% plot(tim,w(3,2:end))
% plot(tim,w(4,2:end))
% plot(tim,w(5,2:end))
% plot(tim,w(6,2:end))
% plot(tim,w(7,2:end))
title('Evolution of Weights')
%legend('w1','w2','w3','w4','w5','w6','w7')
xlabel('Time(sec)')
ylabel('Weights')
