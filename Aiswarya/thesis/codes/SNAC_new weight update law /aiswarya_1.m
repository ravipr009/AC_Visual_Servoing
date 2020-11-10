

%regulation trial
clear all; close all; clc;

%desired point
xd=0.5*rand(3,1);

% UR5 details
umin=-2; umax=2;
a2=-0.425; a3=-0.39243; d1=0.0892; d3=0.10915; d4=0.109 ;d6=0.0823; d5=0.093;

%initial point
theta=rand(6,1);
x0=(sin(theta(1,1))*cos(theta(5,1))-((-cos(theta(1,1))*cos(theta(2,1))*sin(theta(3,1))-cos(theta(1,1))*sin(theta(2,1))*cos(theta(3,1)))*sin(theta(4,1))+(cos(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))-cos(theta(1,1))*sin(theta(2,1))*sin(theta(3,1)))*cos(theta(4,1)))*sin(theta(5,1)))*d6+((cos(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))-cos(theta(1,1))*sin(theta(2,1))*sin(theta(3,1)))*sin(theta(4,1))-(-cos(theta(1,1))*cos(theta(2,1))*sin(theta(3,1))-cos(theta(1,1))*sin(theta(2,1))*cos(theta(3,1)))*cos(theta(4,1)))*d5+sin(theta(1,1))*d4-cos(theta(1,1))*sin(theta(2,1))*sin(theta(3,1))*a3+cos(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))*a3+cos(theta(1,1))*cos(theta(2,1))*a2;

y0=-((-sin(theta(1,1))*cos(theta(2,1))*sin(theta(3,1))-sin(theta(1,1))*sin(theta(2,1))*cos(theta(3,1)))*sin(theta(4,1))+(sin(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))-sin(theta(1,1))*sin(theta(2,1))*sin(theta(3,1)))*cos(theta(4,1)))*sin(theta(5,1))-cos(theta(1,1))*cos(theta(5,1))*d6+((sin(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))-sin(theta(1,1))*sin(theta(2,1))*sin(theta(3,1)))*sin(theta(4,1))-(-sin(theta(1,1))*cos(theta(2,1))*sin(theta(3,1))-sin(theta(1,1))*sin(theta(2,1))*cos(theta(3,1)))*cos(theta(4,1)))*d5-cos(theta(1,1))*d4-sin(theta(1,1))*sin(theta(2,1))*sin(theta(3,1))*a3+sin(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))*a3+sin(theta(1,1))*cos(theta(2,1))*a2;

z0=((cos(theta(2,1))*cos(theta(3,1))-sin(theta(2,1))*sin(theta(3,1)))*sin(theta(4,1))+(cos(theta(2,1))*sin(theta(3,1))+sin(theta(2,1))*cos(theta(3,1)))*cos(theta(4,1)))*sin(theta(5,1))*d6+((cos(theta(2,1))*sin(theta(3,1))+sin(theta(2,1))*cos(theta(3,1)))*sin(theta(4,1))-(cos(theta(2,1))*cos(theta(3,1))-sin(theta(2,1))*sin(theta(3,1)))*cos(theta(4,1)))*d5+d1+cos(theta(2,1))*sin(theta(3,1))*a3+sin(theta(2,1))*cos(theta(3,1))*a3+sin(theta(2,1))*a2;


theta(:,1)=theta;
x=[x0;y0;z0];

%initial error
e(:,1)= x-xd;

dt=0.1;
T=1000;

R=eye(6); Q=1*eye(3);

%%NN
m=6;
alpha=1.8;
alpha_s=0.01;
w=ones(m,1);
e_prev=[0;0;0];


%TRIAL
nodes=1;
w1=0.01*rand(3,nodes);
w=ones(nodes,1);

lam=50;
mu=1.6;
k=1;

for t=0:dt:T
    w11=e(1,k)*ones(1,15);
    w12=e(2,k)*ones(1,15);
    w13=e(3,k)*ones(1,15);
    w1=[w11;w12;w13];
    %TRIAL_ELM
    lay_1=e(:,k)'*w1;
    phi=1./(1+lam*(lay_1));
    
        for nm=1:3
            for nn=1:nodes
                del_phi(nn,nm)=w1(nm,nn)*lam*phi(1,nn)*(1-phi(1,nn));
            end
        end
       
%     phi = [e(1,k)^2; e(1,k)*e(2,k); e(1,k)*e(3,k); e(2,k)^2; e(2,k)*e(3,k);e(3,k)^2];
%     del_phi = [2*e(1,k) e(2,k) e(3,k) 0 0 0; 0 e(1,k) 0 2*e(2,k) e(3,k) 0; 0 0 e(1,k) 0 e(2,k) 2*e(3,k)]';
% 
%     j=w(:,k)'*phi;
%     del_j=del_phi'*w(:,k);





%Jacobian
    J11 = (cos(theta(1,k))*cos(theta(5,k))-((sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))+sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*sin(theta(5,k)))*d6+...
            ((sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))-(sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))+sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5+cos(theta(1,k))*d4+...
            sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))*a3-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))*a3-sin(theta(1,k))*cos(theta(2,k))*a2;



    J12 = -((cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*...
            sin(theta(5,k))*d6+((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))-(cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-...
            cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))*a3-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k))*a3-cos(theta(1,k))*sin(theta(2,k))*a2;



   J13 = -((cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*...
            sin(theta(5,k))*d6+((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))-(cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-...
            cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))*a3-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k))*a3;


    J14 = ((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*d5-...
            ((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k))-(cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k)))*sin(theta(5,k))*d6;

   J15 = (-sin(theta(1,k))*sin(theta(5,k))-...
            ((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*cos(theta(5,k)))*d6;

   J16 = 0.0;



    J21 = (sin(theta(1,k))*cos(theta(5,k))-((-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*sin(theta(5,k)))*d6+...
            ((cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k))-(-cos(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-cos(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5+sin(theta(1,k))*d4-...
            cos(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))*a3+cos(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))*a3+cos(theta(1,k))*cos(theta(2,k))*a2;

    J22= -((sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*...
            sin(theta(5,k))*d6+((-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))-(sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-...
            sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))*a3-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k))*a3-sin(theta(1,k))*sin(theta(2,k))*a2;

    J23 =  -((sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*...
            sin(theta(5,k))*d6+((-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))-(sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-...
            sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))*a3-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k))*a3;

   J24= ((-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*d5-...
            ((-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k))-(sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k)))*sin(theta(5,k))*d6;


    J25= (cos(theta(1,k))*sin(theta(5,k))-((-sin(theta(1,k))*cos(theta(2,k))*sin(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(sin(theta(1,k))*cos(theta(2,k))*cos(theta(3,k))-sin(theta(1,k))*sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*cos(theta(5,k)))*d6;

    J26 = 0;



    J31 = 0.0;

    J32 =  -((-cos(theta(2,k))*sin(theta(3,k))-sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*sin(theta(5,k))*d6+...
            ((cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k))-(-cos(theta(2,k))*sin(theta(3,k))-sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-sin(theta(2,k))*sin(theta(3,k))*a3+cos(theta(2,k))*cos(theta(3,k))*a3+...
            cos(theta(2,k))*a2;

    J33 = -((-cos(theta(2,k))*sin(theta(3,k))-sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k))+(cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k)))*sin(theta(5,k))*d6+...
            ((cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k))-(-cos(theta(2,k))*sin(theta(3,k))-sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-sin(theta(2,k))*sin(theta(3,k))*a3+cos(theta(2,k))*cos(theta(3,k))*a3;

    J34 = ((cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k))+(cos(theta(2,k))*sin(theta(3,k))+sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*d5-...
            ((cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*cos(theta(4,k))-(cos(theta(2,k))*sin(theta(3,k))+sin(theta(2,k))*cos(theta(3,k)))*sin(theta(4,k)))*sin(theta(5,k))*d6;

   J35 = -((cos(theta(2,k))*cos(theta(3,k))-sin(theta(2,k))*sin(theta(3,k)))*sin(theta(4,k))+(cos(theta(2,k))*sin(theta(3,k))+sin(theta(2,k))*cos(theta(3,k)))*cos(theta(4,k)))*cos(theta(5,k))*d6;

    J36 = 0.0;
    
    
            
    J=[J11 J12 J13 J14 J15 J16; J21 J22 J23 J24 J25 J26 ; J31 J32 J33 J34 J35 J36 ];
    
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
    alpha=1+mu*norm(ec,2)*e(:,k)'*e_dot/(norm(d_ec*ec,2)*norm(e(:,k),2));

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
plot(tim,e(3,2:end)+xd(3))
plot(xd(1),'r*')
plot(xd(2),'r*')
plot(xd(3),'r*')
title('Desired points and Actual trajectory')
xlabel('Time')
ylabel('Trajectory')

figure;
plot(tim,e(1,2:end))
hold on;
plot(tim,e(2,2:end))
plot(tim,e(3,2:end))
title('Error Trajectories')
xlabel('time')
ylabel('Error')


figure;
plot(tim,u(1,:))
hold on;
plot(tim,u(2,:))
plot(tim,u(3,:))
plot(tim,u(4,:))
plot(tim,u(5,:))
plot(tim,u(6,:))
title('Control inputs (joint velocites)')
xlabel('time')
ylabel('U')

figure;
plot(tim,theta(1,2:end))
hold on;
plot(tim,theta(2,2:end))
plot(tim,theta(3,2:end))
plot(tim,theta(4,2:end))
plot(tim,theta(5,2:end))
plot(tim,theta(6,2:end))
title('Joint Angles')
xlabel('time')
ylabel('Theta')
