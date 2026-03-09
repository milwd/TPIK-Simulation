function [bTe] = RobustEndEffectorTransform(q)
bTe = zeros(4,4);

cos_q1 = cos(q(1));
sin_q1 = sin(q(1));
cos_q2 = cos(q(2));
sin_q2 = sin(q(2));
cos_q3 = cos(q(3));
sin_q3 = sin(q(3));
cos_q4 = cos(q(4));
sin_q4 = sin(q(4));
cos_q5 = cos(q(5));
sin_q5 = sin(q(5));
cos_q6 = cos(q(6));
sin_q6 = sin(q(6));
cos_q7 = cos(q(7));
sin_q7 = sin(q(7));

bTe(1,1) = cos_q7*(sin_q6*(sin_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3) - cos_q1*cos_q4*sin_q2) - cos_q6*(cos_q5*(cos_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3) + cos_q1*sin_q2*sin_q4) + sin_q5*(cos_q3*sin_q1 + cos_q1*cos_q2*sin_q3))) + sin_q7*(sin_q5*(cos_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3) + cos_q1*sin_q2*sin_q4) - cos_q5*(cos_q3*sin_q1 + cos_q1*cos_q2*sin_q3));
bTe(1,2) = cos_q7*(sin_q5*(cos_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3) + cos_q1*sin_q2*sin_q4) - cos_q5*(cos_q3*sin_q1 + cos_q1*cos_q2*sin_q3)) - sin_q7*(sin_q6*(sin_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3) - cos_q1*cos_q4*sin_q2) - cos_q6*(cos_q5*(cos_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3) + cos_q1*sin_q2*sin_q4) + sin_q5*(cos_q3*sin_q1 + cos_q1*cos_q2*sin_q3)));
bTe(1,3) = - cos_q6*(sin_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3) - cos_q1*cos_q4*sin_q2) - sin_q6*(cos_q5*(cos_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3) + cos_q1*sin_q2*sin_q4) + sin_q5*(cos_q3*sin_q1 + cos_q1*cos_q2*sin_q3));
bTe(1,4) = (863*cos_q1*sin_q2)/2000 + (291*sin_q1*sin_q3)/2000 - (33*cos_q6*(sin_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3) - cos_q1*cos_q4*sin_q2))/250 - (33*sin_q6*(cos_q5*(cos_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3) + cos_q1*sin_q2*sin_q4) + sin_q5*(cos_q3*sin_q1 + cos_q1*cos_q2*sin_q3)))/250 - (21*sin_q4*(sin_q1*sin_q3 - cos_q1*cos_q2*cos_q3))/50 - (291*cos_q1*cos_q2*cos_q3)/2000 + (21*cos_q1*cos_q4*sin_q2)/50;
bTe(2,1) = - cos_q7*(sin_q6*(sin_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1) + cos_q4*sin_q1*sin_q2) - cos_q6*(cos_q5*(cos_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1) - sin_q1*sin_q2*sin_q4) + sin_q5*(cos_q1*cos_q3 - cos_q2*sin_q1*sin_q3))) - sin_q7*(sin_q5*(cos_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1) - sin_q1*sin_q2*sin_q4) - cos_q5*(cos_q1*cos_q3 - cos_q2*sin_q1*sin_q3));
bTe(2,2) = sin_q7*(sin_q6*(sin_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1) + cos_q4*sin_q1*sin_q2) - cos_q6*(cos_q5*(cos_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1) - sin_q1*sin_q2*sin_q4) + sin_q5*(cos_q1*cos_q3 - cos_q2*sin_q1*sin_q3))) - cos_q7*(sin_q5*(cos_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1) - sin_q1*sin_q2*sin_q4) - cos_q5*(cos_q1*cos_q3 - cos_q2*sin_q1*sin_q3));
bTe(2,3) = cos_q6*(sin_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1) + cos_q4*sin_q1*sin_q2) + sin_q6*(cos_q5*(cos_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1) - sin_q1*sin_q2*sin_q4) + sin_q5*(cos_q1*cos_q3 - cos_q2*sin_q1*sin_q3));
bTe(2,4) = (863*sin_q1*sin_q2)/2000 - (291*cos_q1*sin_q3)/2000 + (33*cos_q6*(sin_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1) + cos_q4*sin_q1*sin_q2))/250 + (33*sin_q6*(cos_q5*(cos_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1) - sin_q1*sin_q2*sin_q4) + sin_q5*(cos_q1*cos_q3 - cos_q2*sin_q1*sin_q3)))/250 + (21*sin_q4*(cos_q1*sin_q3 + cos_q2*cos_q3*sin_q1))/50 - (291*cos_q2*cos_q3*sin_q1)/2000 + (21*cos_q4*sin_q1*sin_q2)/50;
bTe(3,1) = sin_q7*(sin_q5*(cos_q2*sin_q4 + cos_q3*cos_q4*sin_q2) + cos_q5*sin_q2*sin_q3) - cos_q7*(cos_q6*(cos_q5*(cos_q2*sin_q4 + cos_q3*cos_q4*sin_q2) - sin_q2*sin_q3*sin_q5) + sin_q6*(cos_q2*cos_q4 - cos_q3*sin_q2*sin_q4));
bTe(3,2) = cos_q7*(sin_q5*(cos_q2*sin_q4 + cos_q3*cos_q4*sin_q2) + cos_q5*sin_q2*sin_q3) + sin_q7*(cos_q6*(cos_q5*(cos_q2*sin_q4 + cos_q3*cos_q4*sin_q2) - sin_q2*sin_q3*sin_q5) + sin_q6*(cos_q2*cos_q4 - cos_q3*sin_q2*sin_q4));
bTe(3,3) = cos_q6*(cos_q2*cos_q4 - cos_q3*sin_q2*sin_q4) - sin_q6*(cos_q5*(cos_q2*sin_q4 + cos_q3*cos_q4*sin_q2) - sin_q2*sin_q3*sin_q5);
bTe(3,4) = (863*cos_q2)/2000 + (21*cos_q2*cos_q4)/50 + (291*cos_q3*sin_q2)/2000 - (33*sin_q6*(cos_q5*(cos_q2*sin_q4 + cos_q3*cos_q4*sin_q2) - sin_q2*sin_q3*sin_q5))/250 + (33*cos_q6*(cos_q2*cos_q4 - cos_q3*sin_q2*sin_q4))/250 - (21*cos_q3*sin_q2*sin_q4)/50 + 283/1000;
bTe(4,1) = 0;
bTe(4,2) = 0;
bTe(4,3) = 0;
bTe(4,4) = 1;
end 