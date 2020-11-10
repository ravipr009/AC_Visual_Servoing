clc;
close all;
clear all;



% D-H Table and Transformation Matrices

syms L1 L2 th1 th2 dth1 dth2 ddth1 ddth2 g m1 m2

m1=1; m2=1; L1=1; L2=1;

R01 = [cos(th1) -sin(th1) 0 ; sin(th1)*cos(0) cos(th1)*cos(0) -sin(0) ; sin(th1)*sin(0) cos(th1)*sin(0) cos(0)]
R12 = [cos(th2) -sin(th2) 0; sin(th2)*cos(0) cos(th2)*cos(0) -sin(0); sin(th2)*sin(0) cos(th2)*sin(0) cos(0)]
R23 = [cos(0) -sin(0) 0; sin(0)*cos(0) cos(0)*cos(0) -sin(0); sin(0)*sin(0) cos(0)*sin(0) cos(0)]
R10 = simplify(inv(R01))
R21 = simplify(inv(R12))
R32 = inv(R23)


% Initialization
X11_cap = [1;0;0]; X22_cap = [1;0;0]; X33_cap = [1;0;0];
Y11_cap = [0;1;0]; Y22_cap = [0;1;0]; Y33_cap = [0;1;0];
Z11_cap = [0;0;1]; Z22_cap = [0;0;1]; Z33_cap = [0;0;1];

P1c1 = L1*X11_cap
P2c2 = L2*X22_cap
P01 = 0*X11_cap
P12 = L1*X11_cap
P23 = L2*X22_cap
Ic11 = zeros(3,3)
Ic22 = zeros(3,3)
f33 = [0;0;0]
n33 = [0;0;0]
w00 = [0;0;0]
dw00 = [0;0;0]
dv00 = [0;g;0]


% Outward iterations for link 1 
w11 = R10*w00 + dth1*Z11_cap
dw11 = R10*dw00 + cross(R10*w00,dth1*Z11_cap) + ddth1*Z11_cap
dv11 = R10*(cross(dw00,P01) + cross(w00,cross(w00,P01)) + dv00)
dv1c1 = cross(dw11,P1c1) + cross(w11,cross(w11,P1c1)) + dv11
F11 = m1*dv1c1
N11 = Ic11*dw11 + cross(w11,Ic11*w11)

% Outward iterations for link 2 
w22 = R21*w11 + dth2*Z22_cap
dw22 = R21*dw11 + cross(R21*w11,dth2*Z22_cap) + ddth2*Z22_cap
dv22 = simplify(R21*(cross(dw11,P12) + cross(w11,cross(w11,P12)) + dv11))
dv2c2 = simplify(cross(dw22,P2c2) + cross(w22,cross(w22,P2c2)) + dv22)
F22 = m2*dv2c2
N22 = Ic22*dw22 + cross(w22,Ic22*w22)


% Inward iterations for link 2
f22 = R23*f33 + F22
n22 = N22 + R23*n33 + cross(P2c2,F22) + cross(P23,R23*f33)
Tau2 = simplify((n22)'*Z22_cap)

% Inward iterations for link 1
f11 = R12*f22 + F11
n11 = N11 + R12*n22 + cross(P1c1,F11) + cross(P12,R12*f22)
Tau1 = simplify((n11)'*Z11_cap)

 
% Collection of M matrix

ddth1 = 1; ddth2 = 0; dth1 = 0; dth2 = 0; g = 0;
m11 = subs(Tau1)
m21 = subs(Tau2)

ddth1 = 0; ddth2 = 1; dth1 = 0; dth2 = 0; g = 0;
m12 = subs(Tau1)
m22 = subs(Tau2)

ddth1 = 0; ddth2 = 0; dth1 = sym('dth1'); dth2 = sym('dth2'); g=0;
f1 = subs(Tau1)
f2 = subs(Tau2)

ddth1 = 0; ddth2 = 0; dth1 = 0; dth2 = 0; g = 1;
g1 = subs(Tau1)*sym('g')
g2 = subs(Tau2)*sym('g')

 M =([m11 m12;m21 m22])
 f = ([f1;f2])
 g = ([g1;g2])






