function p = mirrorPoint(x, y)
%MIRRORPOINT reflects first point about the two others.
midpoint=[(x(3)+x(2))/2 , (y(3)+y(2))/2];
d_up = abs((y(3)-y(2))*x(1)-(x(3)-x(2))*y(1) + x(3)*y(2) -y(3)*x(2));
d_down = sqrt(double((y(3)-y(2))^2 + (x(3)-x(2))^2));
d = d_up/d_down;
vec=[x(1)-midpoint(1),y(1)-midpoint(2)];
vec=vec/norm(vec);
p = midpoint-d*vec;
end

