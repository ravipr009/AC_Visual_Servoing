
%% This function takes as input the position  states and returns the G matrix (Euler-Lagrange C).
function G = G_Vector(X)
q1=X(1);
q2=X(2);
q3=X(3);
q4=X(4);
g=10;
 %Gravity parameters
G1 = g * sin((-2 * q4 + 2 * q2)) * 0.423516473627150170e-21 + g * sin((2 * q4)) * 0.423516473627150170e-21 + g * sin((-2 * q4 + 2 * q3 + 2 * q2)) * 0.211758236813575085e-21 + g * sin((2 * q4 + 2 * q3 + 2 * q2)) * (-0.211758236813575085e-21) + g * sin((2 * q4 + 2 * q3)) * 0.423516473627150170e-21 + g * sin((-2 * q4 - 2 * q3 + 2 * q2)) * 0.211758236813575085e-21 + g * sin((2 * q2)) * 0.563785129692462306e-17 + g * sin((2 * q4 - 2 * q3 + 2 * q2)) * 0.211758236813575085e-21 + g * sin((-2 * q3 + 2 * q2)) * 0.121430643318376497e-16 + g * sin((2 * q3)) * 0.121430643318376497e-16 + g * sin((2 * q4 + q3 + 2 * q2)) * (-0.423516473627150170e-21) + g * sin((2 * q4 + q3)) * 0.423516473627150170e-21 + g * sin((2 * q4 - q3 + 2 * q2)) * (-0.423516473627150170e-21) + g * sin((-2 * q4 + q3)) * (-0.423516473627150170e-21);
G2 = g * sin(q2) * (-0.123790555620000009e1) + g * sin(-(2 * q4) + (2 * q3) + q2) * (-0.423516473627150170e-21) + g * sin((2 * q4) + (2 * q3) + q2) * 0.423516473627150170e-21 + g * sin(-(2 * q4) - (2 * q3) + q2) * 0.423516473627150170e-21 + g * sin((2 * q4) - (2 * q3) + q2) * 0.423516473627150170e-21 + g * sin(-q3 + q2) * 0.378432600000000081e-4 + g * sin((2 * q4) + q2) * (-0.847032947254300339e-21) + g * sin((2 * q4) + q3 + q2) * 0.847032947254300339e-21 + g * sin(q3 + q2) * (-0.378432600000000081e-4) + g * sin((2 * q4) - q3 + q2) * (-0.847032947254300339e-21) + cos(q2) * g * 0.108126010000000015e-2 + g * cos(-q4 + q3 + q2) * 0.906967248000000066e-2 + g * cos(q4 + q3 + q2) * 0.906967248000000066e-2 + g * cos(-q4 - q3 + q2) * 0.906967248000000066e-2 + g * cos(q4 - q3 + q2) * 0.906967248000000066e-2 + g * cos(-q4 + q2) * (-0.181393449600000013e-1) + g * cos(q4 + q2) * 0.181393449600000013e-1 + g * cos(-q3 + q2) * (-0.299373309250000053e-1) + g * cos(q3 + q2) * (-0.299373309250000053e-1) + g * sin(q4 + q3 + q2) * (-0.370774483199999977e-1) + g * sin(-q4 + q3 + q2) * 0.370774483199999977e-1 + g * sin(q4 - q3 + q2) * (-0.370774483199999977e-1) + g * sin(-q4 - q3 + q2) * 0.370774483199999977e-1 + g * sin(q4 + q2) * (-0.741548966399999954e-1) + g * sin(-q4 + q2) * (-0.741548966399999954e-1);
G3 = g * cos(q4 - q3 + q2) * (-0.906967248000000066e-2) + g * sin(-q3 + q2) * (-0.378432600000000081e-4) + g * cos(-q4 + q3 + q2) * 0.906967248000000066e-2 + g * cos(q4 + q3 + q2) * 0.906967248000000066e-2 + g * cos(-q4 - q3 + q2) * (-0.906967248000000066e-2) + g * sin(-q4 - q3 + q2) * (-0.370774483199999977e-1) + g * sin(q4 + q3 + q2) * (-0.370774483199999977e-1) + g * sin(-q4 + q3 + q2) * 0.370774483199999977e-1 + g * sin(q4 - q3 + q2) * 0.370774483199999977e-1 + g * cos(-q3 + q2) * 0.299373309250000053e-1 + g * cos(q3 + q2) * (-0.299373309250000053e-1) + g * sin(q3 + q2) * (-0.378432600000000081e-4);
G4 = g * sin(-q4 + q2) * 0.741548966399999954e-1 + g * cos(q4 - q3 + q2) * 0.906967248000000066e-2 + g * sin(-q4 - q3 + q2) * (-0.370774483199999977e-1) + g * sin(q4 + q2) * (-0.741548966399999954e-1) + g * sin(q4 - q3 + q2) * (-0.370774483199999977e-1) + g * cos(-q4 - q3 + q2) * (-0.906967248000000066e-2) + g * cos(-q4 + q2) * 0.181393449600000013e-1 + g * cos(q4 + q2) * 0.181393449600000013e-1 + g * cos(q4 + q3 + q2) * 0.906967248000000066e-2 + g * cos(-q4 + q3 + q2) * (-0.906967248000000066e-2) + g * sin(q4 + q3 + q2) * (-0.370774483199999977e-1) + g * sin(-q4 + q3 + q2) * (-0.370774483199999977e-1);

G=[G1; G2; G3; G4];
