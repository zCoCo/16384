% Tests if the Gradient Descent Function can Successfully Solve IK for the
% Given Goal Positions %goals% from the Given Start Configurations %th0s%
% on a Serial Revolute Robot with the Given Numbers of Joints %ns% for a
% goal with the given number of dimensions %dims%.
% Both th0s and goals are lists (row vectors) of column vectors.
% Test Case:
% grad_test([2,3,3],[2,2,3],[[pi/4;pi/2;0],[0;0;0],[0;0;0]], [[-sqrt(2);sqrt(2);0],[1.6;1.6;0],[1.6;1.6;pi/2]])
function works = grad_test(ns, dims, th0s, goals)
    works = 1;
    i = 1;
    while(i <= size(th0s,2))
        n = ns(i);
        d = dims(i);
        robot = Robot(ones(n,1), ones(n,1), ones(n,1),1);
        ths = robot.ik(th0s(1:n,i), goals(1:d,i));
        pe = robot.ee(ths);
        
        e = cost(pe(1:d), goals(1:d,i))
        works = works & e < 0.02;
    i = i+1;
    end
end % #grad_test

function e = cost(a,b)
    e = (a-b);
    e = e'*e;
end % #ferr