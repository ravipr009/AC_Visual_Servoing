function C = C_full(x) %#codegen
q1=x(1);
q2=x(2);
q3=x(3);
q4=x(4);

vq1=x(5);
vq2=x(6);
vq3=x(7);
vq4=x(8);

%CORIOLIS & Centrifugal Matrix
C=[0.5*vq2*sin(2*q2) 0 0 0;  - (vq1*sin(2*q2))/5    -(vq4*sin(q4))/10 0 0; 0 0 0 0; 0 0 0 0];


 