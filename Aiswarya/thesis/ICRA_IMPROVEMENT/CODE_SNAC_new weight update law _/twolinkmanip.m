

%regulation trial
clear all; close all; clc;

%desired point
xd=[-0.42; 0.92];

% UR5 details
umin=-2; umax=2;

%initial point
theta=rand(2,1);
x0= cos(theta(1,1))+ cos(theta(1,1)+theta(2,1));
y0=sin(theta(1,1))+ sin(theta(1,1)+theta(2,1));

theta(:,1)=theta;
x=[x0;y0];

%initial error
e(:,1)= x-xd;

dt=0.01;
T=50;

R=eye(2); Q=1*eye(2);

%%NN
m=2;
w=ones(m,1);
e_prev=[0;0];


%TRIAL
nodes=1;
w1=0.01*rand(3,nodes);
w=ones(nodes,1);

lam=5;
mu=0.6;
k=1;

for t=0:dt:T
    w11=e(1,k)*ones(1,15);
    w12=e(2,k)*ones(1,15);
    w1=[w11;w12];
    %TRIAL_ELM
    lay_1=e(:,k)'*w1;
    phi=1./(1+lam*(lay_1));
    
        for nm=1:2
            for nn=1:nodes
                del_phi(nn,nm)=w1(nm,nn)*lam*phi(1,nn)*(1-phi(1,nn));
            end
        end




%Jacobian
    J11 = -sin(theta(1,k))-sin(theta(1,k)+theta(2,k));

    J12 = -sin(theta(1,k)+theta(2,k));


  J21= cos(theta(1,k)) + cos(theta(1,k)+theta(2,k));
  
  J22=cos(theta(1,k)+theta(2,k));
  
    
    
   J=[J11 J12; J21 J22];
    
    %control law
    u(:,k)= -0.5*inv(R)*J'*(del_phi'*w(:,k));
    
    %updation od theta and error
    theta(:,k+1)=theta(:,k)+dt*u(:,k);
    e(:,k+1)= e(:,k)+dt*J*u(:,k);
    
    %weight updation
    A=del_phi*J*inv(R)*J'*del_phi';
    ec=-0.25*w(:,k)'*A*w(:,k)+e(:,k)'*Q*e(:,k);
    d_ec=-0.5*A*w(:,k);
    e_dot=(e(:,k)-e_prev)/dt;
    %alpha=1+mu*e(:,k)'*e_dot/norm(zeta(:,k));
    alpha=(mu*norm(ec,2)/norm(d_ec*ec,2))*(1+e(:,k)'*e_dot/(norm(e(:,k),2)));
    %alpha=1+mu*norm(ec,2)*e(:,k)'*e_dot/(norm(d_ec*ec,2)*norm(e(:,k),2));
    %alpha=1+mu*e(:,k)'*e_dot/norm(e(:,k),2);
    
    %wdot=-alpha*ec*(-0.5*A*w(:,k)) +0.5*(alpha_s)*del_phi*J*inv(R)*J'*e(:,k);
    
   
    wdot=alpha*del_phi*J*inv(R)*J'*e(:,k);
    
    w(:,k+1)=w(:,k)+dt*wdot;
    
    e_prev=e(:,k);
    k=k+1;
end

%plots
tim=0:dt:T;
   figure;
plot(tim,e(1,2:end)+xd(1))
hold on;
plot(tim,e(2,2:end)+xd(2))

plot(xd(1),'r*')
plot(xd(2),'r*')
title('Desired points and Actual trajectory')
xlabel('Time')
ylabel('Trajectory')

figure;
plot(tim,e(1,2:end))
hold on;
plot(tim,e(2,2:end))

title('Error Trajectories')
xlabel('time')
ylabel('Error')


figure;
plot(tim,u(1,:))
hold on;
plot(tim,u(2,:))
title('Control inputs (joint velocites)')
xlabel('time')
ylabel('U')

figure;
plot(tim,theta(1,2:end))
hold on;
plot(tim,theta(2,2:end))
title('Joint Angles')
xlabel('time')
ylabel('Theta')
