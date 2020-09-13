a = csvread("BatchTestResults2.csv");

x1 = a(:,2);
y1 = a(:,3);
z1 = a(:,4);

x2 = a(:,9);
y2 = a(:,10);
z2 = a(:,11);

plot3(x1,y1,z1,'r')
hold on;
plot3(x2,y2,z2,'k')
