



%tracking trial
clear all; clc; close all;
tic;

% UR5 details
% umin=-3; umax=3;
% a2=-0.425; a3=-0.39243; d1=0.0892; d3=0.10915; d4=0.109 ;d6=0.0823; d5=0.093;

%UR10
umin=-3; umax=3;
d1=0.1273; d4=0.163941; d5=0.1157; d6=0.0922; a2=-0.612; a3=-0.5723;

%initial point
  theta(:,1)=[-0.51;-1.05;1.49;-1.818;-1.452;-1.5937];

 x0=(sin(theta(1,1))*cos(theta(5,1))-((-cos(theta(1,1))*cos(theta(2,1))*sin(theta(3,1))-cos(theta(1,1))*sin(theta(2,1))*cos(theta(3,1)))*sin(theta(4,1))+(cos(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))-cos(theta(1,1))*sin(theta(2,1))*sin(theta(3,1)))*cos(theta(4,1)))*sin(theta(5,1)))*d6+((cos(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))-cos(theta(1,1))*sin(theta(2,1))*sin(theta(3,1)))*sin(theta(4,1))-(-cos(theta(1,1))*cos(theta(2,1))*sin(theta(3,1))-cos(theta(1,1))*sin(theta(2,1))*cos(theta(3,1)))*cos(theta(4,1)))*d5+sin(theta(1,1))*d4-cos(theta(1,1))*sin(theta(2,1))*sin(theta(3,1))*a3+cos(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))*a3+cos(theta(1,1))*cos(theta(2,1))*a2;

y0=-((-sin(theta(1,1))*cos(theta(2,1))*sin(theta(3,1))-sin(theta(1,1))*sin(theta(2,1))*cos(theta(3,1)))*sin(theta(4,1))+(sin(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))-sin(theta(1,1))*sin(theta(2,1))*sin(theta(3,1)))*cos(theta(4,1)))*sin(theta(5,1))-cos(theta(1,1))*cos(theta(5,1))*d6+((sin(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))-sin(theta(1,1))*sin(theta(2,1))*sin(theta(3,1)))*sin(theta(4,1))-(-sin(theta(1,1))*cos(theta(2,1))*sin(theta(3,1))-sin(theta(1,1))*sin(theta(2,1))*cos(theta(3,1)))*cos(theta(4,1)))*d5-cos(theta(1,1))*d4-sin(theta(1,1))*sin(theta(2,1))*sin(theta(3,1))*a3+sin(theta(1,1))*cos(theta(2,1))*cos(theta(3,1))*a3+sin(theta(1,1))*cos(theta(2,1))*a2;

z0=((cos(theta(2,1))*cos(theta(3,1))-sin(theta(2,1))*sin(theta(3,1)))*sin(theta(4,1))+(cos(theta(2,1))*sin(theta(3,1))+sin(theta(2,1))*cos(theta(3,1)))*cos(theta(4,1)))*sin(theta(5,1))*d6+((cos(theta(2,1))*sin(theta(3,1))+sin(theta(2,1))*cos(theta(3,1)))*sin(theta(4,1))-(cos(theta(2,1))*cos(theta(3,1))-sin(theta(2,1))*sin(theta(3,1)))*cos(theta(4,1)))*d5+d1+cos(theta(2,1))*sin(theta(3,1))*a3+sin(theta(2,1))*cos(theta(3,1))*a3+sin(theta(2,1))*a2;
theta(:,1)=theta;
x=[x0;y0;z0];

R=eye(6); Q=1*eye(3);
k1=zeros(3,3);
Qbar = [Q k1;k1 k1];

 m=21;
w=rand(m,1);

nodes=1;
w1=rand(6,nodes);
w=ones(nodes,1);
%lam=15; % FOR conventional weight update law
lam=10;    % FOR my weight update law


T=68; 
dt=0.01;

k=1;
th=0;v=0;
t=0:dt:T;
n=length(t);

Xmid=(-1.07-0.69)/2;
Ymid=(-0.27+0.345)/2;
Zmid=(0.1+0.05)/2;

u_prev=[0;0;0;0;0;0];
zeta_prev=[0;0;0;0;0;0];
 xd_p=[0;0  ;0];

 mu=2.3;
                                                                                                                                                                                                                                                                                                                                                                                                                    

for t=0:dt:T;


    th=th+0.1*dt;
    
 %case 1
xdd=0.2*cos(th)+Xmid;
yd=0.2*sin(th)+Ymid;
zd=Zmid;
% %case 2
% xdd=0.4*cos(th)+Xmid;
% yd=0.4*sin(th)+Ymid;
% zd=Zmid;
% %case 3
% xdd=0.25*cos(th)+Xmid;
% yd=0.25*sin(th)+Ymid;
% zd=Zmid*cos(pi/6);
% %case 4
% xdd=0.15*cos(th)+Xmid;
% yd=0.15*sin(th)+Ymid;
% zd=Zmid*xdd;
% %case 5
% xdd=Xmid+0.32*cos(th);
%     yd=Ymid+0.32*sin(th);
%     zd=Zmid*xdd*cos(pi/6);
% % case 6
% xdd=Xmid+0.3*cos(th);
%     yd=Ymid+0.3*sin(th);
%     zd=Zmid*yd;
% % case 7
% xdd=Xmid+0.276*cos(th);
%     yd=Ymid+0.276*sin(th);
%     zd=Zmid*cos(-pi/12);
% % case 8
% xdd=Xmid+0.33*cos(th);
%     yd=Ymid+0.33*sin(th);
%     zd=Zmid*(xdd+yd);
% % case 9
% xdd=Xmid+0.18*cos(th);
%     yd=Ymid+0.18*sin(th);
%     zd=Zmid*cos(-pi/12);
% % case 10
% xdd=Xmid+0.22*cos(th);
%     yd=Ymid+0.22*sin(th);
%     zd=Zmid^2;
%     
     
 xd(:,k)=[xdd;yd;zd];
 e(:,k)=x(:,k)-xd(:,k);
  zeta(:,k)=[e(:,k);xd(:,k)];
   
  
  %TRIAL
   w11=zeta(1,k)*ones(1,nodes);
  w12=zeta(2,k)*ones(1,nodes);
  w13=zeta(3,k)*ones(1,nodes);
  w14=zeta(1,k)*ones(1,nodes);
  w15=zeta(2,k)*ones(1,nodes);
  w16=zeta(3,k)*ones(1,nodes);
  w1=[w11;w12;w13;w14;w15;w16];   

    lay=zeta(:,k)'*w1;
      phi=1./(1+exp(-lam*lay));
  
    
        for nm=1:6
            for nn=1:nodes
                del_phi(nn,nm)=w1(nm,nn)*lam*phi(1,nn)*(1-phi(1,nn));
            end
        end
       
        
  del_V =   del_phi'*w(:,k);
 


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
    
    xddot = (xd(:,k)-xd_p)/dt;
    
    F = [0-xddot(1);
     0-xddot(2);
     0-xddot(3);
     xddot(1);
     xddot(2);
     xddot(3)];
    G = [J;
     zeros(3,6)];
    
up = -0.5*inv(R)*G'*del_V;
omega(:,k) = up'*up;
u(:,k)=up;

A=del_phi*G*inv(R)*G'*del_phi';
e_h=zeta(:,k)'*Qbar*zeta(:,k)+w(:,k)'*del_phi*F-0.25*w(:,k)'*A*w(:,k);
de_h=del_phi*F-0.5*A*w(:,k);
zeta_dot=(zeta(:,k)-zeta_prev)/dt;
alpha=(mu*norm(e_h,2)/norm(de_h*e_h,2))*(1+zeta(:,k)'*zeta_dot/(norm(zeta(:,k),2)));
%alpha=1+mu*zeta(:,k)'*zeta_dot/(norm(zeta(:,k),2));

zeta(:,k+1)=zeta(:,k)+dt*(F+G*u(:,k));% [~,xk]=ode45(@(tk,xk) SysDyn(tk, xk,u(:,k),xddot,J), [t t+dt], zeta(:,k));
    
theta(:,k+1)=theta(:,k)+dt*u(:,k);
x(:,k+1)=zeta(1:3,k+1)+zeta(4:6,k+1);
delJ_zeta=zeta(:,k);

wdot=alpha*del_phi*G*inv(R)*G'*delJ_zeta;
w(:,k+1) = w(:,k) + wdot*dt;
V1(:,k)=zeta(:,k)'*Qbar*zeta(:,k)+u(:,k)'*R*u(:,k);

u_dot(:,k)=(u(:,k)-u_prev)/dt;
V2(:,k)=zeta(:,k)'*Qbar*zeta(:,k)+u(:,k)'*R*u(:,k)+u_dot(:,k)'*u_dot(:,k);
u_prev=u(:,k);
err(k,1)=sqrt(sum(e(:,k).^2)/3);
xd_p=xd(:,k);
         
%     m(k)=zeta(:,k)'*Qbar*zeta(:,k)+w(:,k)'*del_phi*F-0.25*w(:,k)'*A*w(:,k);
zeta_prev=zeta(:,k);
    k=k+1;
end
toc;
wc=w(:,end);
tim=0:dt:T;
disp('Max Cost')
max(V1)
disp('metric to compare')
sum(V2)/length(V2)
disp('RMS max input')
sqrt((max(abs(u(1,:)))^2+max(abs(u(2,:)))^2+max(abs(u(3,:)))^2+max(abs(u(4,:)))^2+max(abs(u(5,:)))^2+max(abs(u(6,:)))^2)/6)


figure;
plot3(zeta(4,2:end),zeta(5,2:end),zeta(6,2:end))
hold on;
plot3(zeta(1,2:end)+zeta(4,2:end),zeta(2,2:end)+zeta(5,2:end),zeta(3,2:end)+zeta(6,2:end))
% tt = ((0:1:length(x)-1)*dt)';
% title('Actual and Desired trajectories')

figure;
plot(tim(1:1000),zeta(1,1:1000),tim(1:1000),zeta(2,1:1000),tim(1:1000),zeta(3,1:1000))
% hold on;
% plot(tim,zeta(2,2:end))
% plot(tim,zeta(3,2:end))
legend('z1','z2','z3')
xlabel('Time Instant')
ylabel('Error Trajectories')

figure;
plot(tim,u(1,:));
hold on;
plot(tim,u(2,:));
plot(tim,u(3,:));
plot(tim,u(4,:));
plot(tim,u(5,:));
plot(tim,u(6,:));
xlabel('Time Instant')
ylabel('Control Input Velocites')
legend('u1','u2','u3','u4','u5','u6')

figure;
plot(tim,theta(1,2:end));
hold on;
plot(tim,theta(2,2:end));
plot(tim,theta(3,2:end));
plot(tim,theta(4,2:end));
plot(tim,theta(5,2:end));
plot(tim,theta(6,2:end));
xlabel('Time Instant')
ylabel('Joint Angles')
legend('Theta1','Theta2','Theta3','Theta4','Theta5','Theta6')

figure;
plot(tim,V1)
xlabel('Time Instant')
ylabel('Cost Function')
legend('V')

figure;
plot(tim,err)
xlabel('Time Instant')
ylabel('RMS error')
legend('RMS')

figure;
plot(tim,w(1,2:end))
%hold on;
% plot(tim,w(2,2:end))
% plot(tim,w(3,2:end))
% plot(tim,w(4,2:end))
% plot(tim,w(6,2:end))

xlabel('Time Instances')
ylabel('Weight Vector')
%legend('W_{1}''W_{2}''W_{3}''W_{4}''W_{5}''W_{6}''W_{7}''W_{8}''W_{9}'' W_{10}''W_{11}''W_{12}''W_{13}''W_{14}''W_{15}''W_{16}''W_{17}''W_{18}''W_{19}'' W_{20}''W_{21}')

err(end)