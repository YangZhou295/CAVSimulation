clc;
clear;
Indi=[0,1,1,1,1,0,1,0,1,0];%%% 1 is the linear controller; 0 is the MPC;
Trajectory=csvread('C:\×÷Òµ\Research\Finished\Finished papers\Distributed Robust Connected Automated Car Following Strategy to Stabilize Mixed Traffic\traj0.csv');
sss=Trajectory.*0.3408;%%%%%%%1st vehicle Position (from ft to meter)£»
s=sss(1,:);
v=diff(s).*10;
a=diff(v).*10;
% v=v';
% s=s';
leada1=a;
tao=1;% unit sec
TT=0.3;
N=10;
TTT=476;%%%%%% TTT is the analysis period
accr=zeros((N+1),TTT+1);
position=zeros((N+1),TTT+1);
speed=zeros((N+1),TTT+1);
deltavall=zeros(N,TTT+1);
deltaeall=zeros(N,TTT+1);
u=zeros(N,TTT+1);
position(1,:)=s(1,1:(TTT+1));
speed(1,:)=v(1,1:(TTT+1));
accr(1,:)=a(1,1:(TTT+1));
for kkk=1:1:10
    kkk
    if kkk<=1
        if Indi(1,kkk)
            linearnew;
            position(kkk+1,:)=pposition(2,:);
            speed(kkk+1,:)=sspeed(2,:);
            accr(kkk+1,:)=ACC(2,:);
            deltavall(kkk,:)=Speeddiff(1,:);
            deltaeall(kkk,:)=deltae(1,:);
        else
            linearnewnew;
            position(kkk+1,:)=pposition(2,:);
            speed(kkk+1,:)=sspeed(2,:);
            accr(kkk+1,:)=ACC(2,:);
            deltavall(kkk,:)=Speeddiff(1,:);
            deltaeall(kkk,:)=deltae(1,:);
        end
    else
        if Indi(1,kkk)
            v=speed(kkk,:);
            s=position(kkk,:);
            leada1=accr(kkk,:);
            linearnew;
            position(kkk+1,:)=pposition(2,:);
            speed(kkk+1,:)=sspeed(2,:);
            accr(kkk+1,:)=ACC(2,:);
            deltavall(kkk,:)=Speeddiff(1,:);
            deltaeall(kkk,:)=deltae(1,:);
        else
            v=speed(kkk,:);
            s=position(kkk,:);
            leada1=accr(kkk,:);
            linearnewnew;
            position(kkk+1,:)=pposition(2,:);
            speed(kkk+1,:)=sspeed(2,:);
            accr(kkk+1,:)=ACC(2,:);
            deltavall(kkk,:)=Speeddiff(1,:);
            deltaeall(kkk,:)=deltae(1,:);
        end
    end
end
%%%% Write a plot function
figure;
t=0:0.1:TTT/10;
for kkk=1:1:11
    plot(t,position(kkk,:));
    hold on;
end
legend('Vehicle 1845','CAV 1','CAV 2','CAV 3', 'CAV 4', 'CAV 5','CAV 6','CAV 7','CAV 8', 'CAV 9', 'CAV 10');
hold off;
figure;
for kkk=1:1:11
    plot(t,speed(kkk,:));
    hold on;
end
legend('Vehicle 1845','CAV 1','CAV 2','CAV 3', 'CAV 4', 'CAV 5','CAV 6','CAV 7','CAV 8', 'CAV 9', 'CAV 10');
figure;
for kkk=1:1:10
    plot(t,deltavall(kkk,:));
    hold on;
end
legend('CAV 1','CAV 2','CAV 3', 'CAV 4', 'CAV 5','CAV 6','CAV 7','CAV 8', 'CAV 9', 'CAV 10');
figure;
for kkk=1:1:10
    plot(t,deltaeall(kkk,:));
    hold on;
end
legend('CAV 1','CAV 2','CAV 3', 'CAV 4', 'CAV 5','CAV 6','CAV 7','CAV 8', 'CAV 9', 'CAV 10');
%speed is for speed, the first speed is for the first vehicle speed, which
%is from the data, similar to position;
