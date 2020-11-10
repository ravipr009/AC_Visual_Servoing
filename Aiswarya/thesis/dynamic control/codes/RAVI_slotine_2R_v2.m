%% Simulation model of a Barrett 4DOF Manipulator arm

clear all; close all; clc;

% simulation parameters
dt = 0.001; % step size;
ts = 20; % total simulation time
t = 0:dt:ts; % time span

% Geometrical and Physical parameters

m1=1;
m2=1;
l1=1;
l2=1;
g=0;

% initial condidtions

q0 = rand(2,1); % q(t=0) = [th1(t=0);th2(t=0);th3(t=0);th4(t=0);]; % initial joint angles
qdot = [0;0]; % qdot(t=0) = [th1dot(t=0);th2dot(t=0);th3dot(t=0);th4dot(t=0);]; % initial joint velocities

th1=q0(1); th2=q0(2); 

x0= l1*cos(th1)+l2*cos(th1+th2);
y0 = l1*sin(th1)+l2*sin(th1+th2);


q(:,1)=q0;

alpha=0.01;
rd=[x0;y0]+[0.1;-0.3];


for i = 1:length(t)
    
th1=q(1,i); th2=q(2,i);
th1dot = qdot(1,i); th2dot = qdot(2,i); 

xc= l1*cos(th1)+l2*cos(th1+th2);
yc = l1*sin(th1)+l2*sin(th1+th2);

r(:,i) = [xc;yc];
%% Mass matrix terms
m11=(l2)^2*m2+2*l1*l2*m2*cos(th2)+(l1)^2*(m1+m2);
m12=(l2)^2*m2+l1*l2*m2*cos(th2);
m21=(l2)^2*m2+l1*l2*m2*cos(th2);
m22=(l2)^2*m2;


% mass matrix
M = [m11,m12;
     m21,m22];
 
%% vector of other effects
n1=-m2*l1*l2*sin(th2)*(th2dot)^2-2*m2*l1*l2*sin(th2);
n2=m2*l1*l2*sin(th2)*(th1dot)^2;
n_v = [n1;n2];

%% gravity vector
g1=m2*l2*cos(th1+th2)+(m1+m2)*l1*cos(th1);
g2=m2*l2*cos(th1+th2);

g_v = g * [g1;g2];


%% jacobian
J= [-l1*sin(th1)-l2*sin(th1+th2) -l2*sin(th1+th2);
    l1*cos(th1)+l2*cos(th1+th2)   l2*cos(th1+th2)]; 
det(J*J');

%% Control inputs
tau(:,i) = [0;0];
kp = 50*eye(2); 
kv = 15*eye(2); 

e(:,i) = (r(:,i)-rd);
tau(:,i)= -J'*kp*e(:,i) - kv*qdot(:,i) + g_v;

%% acceleration and states evolution
q_double_dot = inv(M)*(tau(:,i)-n_v-g_v);

qdot(:,i+1) = qdot(:,i) + dt * q_double_dot;
q(:,i+1) = q(:,i) + dt*qdot(:,i);
       
err(i,1)=sqrt(sum(e(:,i).^2)/3);
end

figure;
plot(t,r(1,:),'b-')
hold on;
plot(t,r(2,:),'g-.')
plot(t,ones(length(t),1)*rd(1),'r--')
plot(t,ones(length(t),1)*rd(2),'m--')
title('Desired point and Trajectory')
xlabel('time')
ylabel('trajectory')

figure;
plot(t,q(:,1:i))
title('Time history of Joint Angle')
xlabel('time')
ylabel('joint angles')

figure;
plot(t,tau(1,:))
hold on;
plot(t,tau(2,:))

title('time history of Control Input')
xlabel('time')
ylabel('u')

figure;
plot(t,e(1,:))
hold on;
plot(t,e(2,:))
title('error trajectories')
xlabel('time')
ylabel('error')

figure;
plot(t,err);
