axis = 3;

waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]
n = size(waypoints,1)-1; % number of segments P1..n
A = zeros(8*n, 8*n);
b = zeros(1,8*n);
for i=1:n
    b(1,i) = waypoints(i,axis);        % b����Ϊ���ֵ
end
for i=2:n+1
    b(1,n+i-1) = waypoints(i,axis);
end

row = 1;
for i=1:n
    A(row,8*(i-1)+1:8*i) = polyT(8,0,0);   % 8�׶���ʽ��0�׵���tֵ����0
    row = row + 1;
end
for i=1:n
    A(row,8*(i-1)+1:8*i) = polyT(8,0,1);   % 8�׶���ʽ��0�׵���tֵ����1
    row = row + 1;
end
for i=1:3
    A(row,1:8) = polyT(8,i,0);   % 8�׶���ʽ��0�׵���tֵ����0
    row = row + 1;
end
for i=1:3
    A(row,25:32) = polyT(8,i,1);   % 8�׶���ʽ��i�׵���tֵ����1
    row = row + 1;
end
for i=1:n-1
    A(row,8*(i-1)+1:8*(i)) = polyT(8,1,1); % 8�׶���ʽ��1�׵���tֵ����1
    A(row,8*(i)+1:8*(i+1)) = - polyT(8,1,0); % 8�׶���ʽ��1�׵���tֵ����0
    row = row + 1
end
for i=1:n-1
    A(row,8*(i-1)+1:8*(i)) = polyT(8,2,1); % 8�׶���ʽ��2�׵���tֵ����1
    A(row,8*(i)+1:8*(i+1)) = - polyT(8,2,0); % 8�׶���ʽ��2�׵���tֵ����0
    row = row + 1
end
for i=1:n-1
    A(row,8*(i-1)+1:8*(i)) = polyT(8,3,1); % 8�׶���ʽ��3�׵���tֵ����1
    A(row,8*(i)+1:8*(i+1)) = - polyT(8,3,0); % 8�׶���ʽ��3�׵���tֵ����0
    row = row + 1
end
for i=1:n-1
    A(row,8*(i-1)+1:8*(i)) = polyT(8,4,1); % 8�׶���ʽ��3�׵���tֵ����1
    A(row,8*(i)+1:8*(i+1)) = - polyT(8,4,0); % 8�׶���ʽ��3�׵���tֵ����0
    row = row + 1
end
for i=1:n-1
    A(row,8*(i-1)+1:8*(i)) = polyT(8,5,1); % 8�׶���ʽ��3�׵���tֵ����1
    A(row,8*(i)+1:8*(i+1)) = - polyT(8,5,0); % 8�׶���ʽ��3�׵���tֵ����0
    row = row + 1
end
for i=1:n-1
    A(row,8*(i-1)+1:8*(i)) = polyT(8,6,1); % 8�׶���ʽ��3�׵���tֵ����1
    A(row,8*(i)+1:8*(i+1)) = - polyT(8,6,0); % 8�׶���ʽ��3�׵���tֵ����0
    row = row + 1
end
    
coff = inv(A)*b'



