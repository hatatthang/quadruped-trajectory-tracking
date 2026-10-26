function q = IK_spider_Ver0(Phip,Pfoot,l)
% IK 3R de doi chieu doc lap. Model da co ham cuc bo ngay trong khoi IK.
% q1 la goc phuong vi THE GIOI, q3 chon nhanh am.
% Khong cong goc lap hong lan nua; FK dung cung quy uoc.
d=Pfoot(1:3)-Phip(1:3);
r=hypot(d(1),d(2));a=d(3);b=r-l(1);
c=(a*a+b*b-l(2)^2-l(3)^2)/(2*l(2)*l(3));
assert(all(isfinite(d)) && r>l(1) && c>=-1-1e-10 && c<=1+1e-10, ...
    'Ver0: IK target outside workspace. Check geometry/stance/lift/shifts.');
c=max(-1,min(1,c)); % Chi bu sai so so hoc tai bien mien voi.
q3=atan2(-sqrt(max(0,1-c*c)),c);
q2=atan2(a,b)-atan2(l(3)*sin(q3),l(2)+l(3)*cos(q3));
q=[atan2(d(2),d(1)),q2,q3];
end
