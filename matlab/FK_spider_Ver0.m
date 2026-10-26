function [P0,P1,P2,P3] = FK_spider_Ver0(P0,l,q)
% FK 3R de doi chieu doc lap. Model da co ham cuc bo ngay trong khoi FK.
% q1 la phuong vi the gioi do IK tra ve.
e=[cos(q(1)),sin(q(1))];
P1=[P0(1:2)+l(1)*e,P0(3),q(1)];
P2=[P1(1:2)+l(2)*cos(q(2))*e,P1(3)+l(2)*sin(q(2)),q(1)];
P3=[P2(1:2)+l(3)*cos(q(2)+q(3))*e,P2(3)+l(3)*sin(q(2)+q(3)),q(1)];
end
