waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]
coffx = getCoff(waypoints,1)
coffy = getCoff(waypoints,2)
coffz = getCoff(waypoints,3)
coffx(1:8,1)
positionx = zeros(1,44);
positiony = zeros(1,44);
positionz = zeros(1,44);
for t = 0:1:10
     positionx(t+1) = caculate_poly(coffx(1:8,1),t/11,0);
     positiony(t+1) = caculate_poly(coffy(1:8,1),t/11,0);
     positionz(t+1) = caculate_poly(coffz(1:8,1),t/11,0) ;   
end
for t = 11:1:21
    coffx(9:16,1)
    positionx(t+1) = caculate_poly(coffx(9:16,1),(t-11)/11,0)
    positiony(t+1) = caculate_poly(coffy(9:16,1),(t-11)/11,0);
    positionz(t+1) = caculate_poly(coffz(9:16,1),(t-11)/11,0) ;
end
for t = 22:1:32
    positionx(t+1) = caculate_poly(coffx(17:24,1),(t-22)/11,0)
    positiony(t+1) = caculate_poly(coffy(17:24,1),(t-22)/11,0)
    positionz(t+1) = caculate_poly(coffz(17:24,1),(t-22)/11,0) 
end
for t = 33:1:43
    positionx(t+1) = caculate_poly(coffx(25:32,1),(t-33)/11,0)
    positiony(t+1) = caculate_poly(coffy(25:32,1),(t-33)/11,0)
    positionz(t+1) = caculate_poly(coffz(25:32,1),(t-33)/11,0) 
end
t = 0:1:43;
% plot(t, positionx)
% hold on
% plot(t, positiony)
% hold on
% plot(t, positionz)
% hold on
% plot([0,11,22,33,44],waypoints(:,1),'+')
% hold on
% plot([0,11,22,33,44],waypoints(:,2),'+')
% hold on
% plot([0,11,22,33,44],waypoints(:,3),'+')
plot3(positionx, positiony, positionz)

