

%Optimal control with system identification - S. Jagannathan
%UR10

clear all; close all; clc;
tic;



alpha=0.5;  v=0.3; K=0.1;
Q=eye(16);
R=eye(2);

%4 dof. total 8 states - 4 joint position angles, 4 joint angle velocities.
th0 = [0;0;0;0];
dth0 = [0;0;0;0];

x0=[th0;dth0]; %initial states
i=1;
% T=500;
% dt=0.01;

%desired states
dt=.1;
T=10;
time=0:dt:10;
thd1= zeros(1,length(time));
% thd2= sin(pi/4*time);
% thd3= sin(pi/4*time);
thd2= sin(time);
thd3= sin(time);
thd4= zeros(1,length(time));


dthd1= zeros(1,length(time));
% dthd2= pi/4*cos(pi/4*time);
% dthd3= pi/4*cos(pi/4*time);
dthd2= cos(time);
dthd3= cos(time);
dthd4= zeros(1,length(time));

xr=[thd1;thd2;thd3;thd4;dthd1;dthd2;dthd3;dthd4];
xr0=xr(:,1);
X=[x0-xr0;xr0];
X_hat=X;
states=16;


nodes=1;
w1=rand(states,nodes);
w=ones(nodes,1);
lam=1;
 
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
  w11=[X(1:8,k)];
  w1=[w11;w11];   
 
         lay=X(:,k)'*w1;
      phi=1./(1+exp(-lam*lay))
  
    
        for nm=1:states
            for nn=1:nodes
                del_phi(nn,nm)=w1(nm,nn)*lam*phi(1,nn)*(1-phi(1,nn));
            end
        end
       
        
        V(:,k)=w(:,k)'*phi;
  del_V=del_phi'*w(:,k);

  %Actual system dynamics

%   F_desired_mat=[0 0 0 0 0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0 0 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 1 0 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0 1 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0 0 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0 0 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0 0 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0 0 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 -(pi/4)^2 0 0 0 0 0 0 0 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 0 -(pi/4)^2 0 0 0 0 0 0 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0 0 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0 0 0 0 0;
%                  0 0 0 0 0 0 0 0 0 0 0 0   0 0 0 0 0 0 0 0 0 0 0 0;];
%              
% F_desired=F_desired_mat*X(:,k);

% F_desired=[0; X(20,k); X(21,k); 0 ; 0 ; 0; 0 ; -(pi/4)^2*X(14,k); -(pi/4)^2*X(15,k); 0; 0; 0];
F_desired=[0; X(14,k); X(15,k); 0 ;0 ; -X(10,k); -X(11,k); 0];



%  F_system_pos_mat=[0 0 0 0 0 0 1 0 0 0 0 0  0 0 0 0 0 0 0 0 0 0 0 0 ;
%                    0 0 0 0 0 0 0 1 0 0 0 0  0 0 0 0 0 0 0 0 0 0 0 0 ;
%                    0 0 0 0 0 0 0 0 1 0 0 0  0 0 0 0 0 0 0 0 0 0 0 0 ;
%                    0 0 0 0 0 0 0 0 0 1 0 0  0 0 0 0 0 0 0 0 0 0 0 0 ;
%                    0 0 0 0 0 0 0 0 0 0 1 0  0 0 0 0 0 0 0 0 0 0 0 0 ;
%                    0 0 0 0 0 0 0 0 0 0 0 1  0 0 0 0 0 0 0 0 0 0 0 0 ;];
                   
%  F_system_pos=F_system_pos_mat*X(:,k);
F_system_pos=X(5:8,k)+X(13:16,k);
 
  M=M_full_4dof(X(1:4,k)+X(9:12,k));   
% M=M_matrix6D(X(1:4,k)+X(9:12,k));
%    f=C_full_dof(X(1:4,k)+X(9:12,k),X(5:8,k)+X(13:14,k));                                    %to pass joint angles
 GG=G_Vector_4dof(X(1:4,k)+X(9:12,k));      
%      g=G_Vecotr_4dof(X(1:4,k)+X(9:12,k));                                                           %to pass joint angles
  C=C_full_4dof(X(1:8,k)+X(9:16,k));
  F_system_vel=-inv(M)*(C*(X(5:8,k)+X(13:16,k))+GG);
%   F_system_vel=-inv(M)*(f+g);
 
 G_system=inv(M);
 
 F_system=[F_system_pos;F_system_vel];
 F=[F_system-F_desired;F_desired];
 G=[0 0 0 0 ;0 0 0 0; 0 0 0 0;  0 0 0 0;G_system(1,:);G_system(1,:);G_system(1,:);G_system(4,:);G_system(5,:);G_system(6,:);
    0 0 0 0 0 0 ;0 0 0 0 0 0;0 0 0 0 0 0; 0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0;0 0 0 0 0 0; 0 0 0 0 0 0;0 0 0 0 0 0 ];
     
%control law
u(:,k)= -0.5*inv(R)*G'*(del_phi'*w(:,k));
    
X(:,k+1)=X(:,k)+dt*(F+G*u(:,k));

%estimating F and X
% F_hat=-(Q-G*inv(R)*G'/4)*del_phi2'*w(:,k);
% F_hat(3:4)=F(3:4);
F_hat=F;
X_hat(:,k+1)=X_hat(:,k)+dt*(F_hat+G*u(:,k)+K*(X(:,k)-X_hat(:,k)));

fff(:,k)=F;
fff_hat(:,k)=F_hat;

%Weight Updation
J_X=X_hat(:,k);
wdot=0.5*alpha*del_phi*G*inv(R)*G'*J_X-alpha*v*del_phi*(Q-G*inv(R)*G'/4)*(X(:,k)-X_hat(:,k));
w(:,k+1)=w(:,k)+dt*(wdot);
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
plot(tim,fff(1,:)-fff_hat(1,:))
hold on;
plot(tim,fff(2,:)-fff_hat(2,:))
title('Augmented System dynamics erros - F-Fhat')
xlabel('Time (sec)')
ylabel('F-Fhat, error of System Dynamics (F)')
legend('error1_F' ,'error2_F')
grid on;

figure;
plot(tim,V)
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
