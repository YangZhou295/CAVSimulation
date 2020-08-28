% Mixed Platoon Trajectory plot and l2 norm calculation:
% Define the initial Condition:
% Define the communication delay:
theta=0;%%%% U can specify it as zero
% Define the discretization step
ts=0.1;
% Set the delay steps;
theta1=theta/ts;
% Set the constant time gap
C=eye(3);
% Define the total number of vehicles including the first leading vehicle
N=2;
% Define the total running time
T=TTT+1;% 30.1 sec data length;
% set the k coefficients:
%%%%%%% Set the gains manually
ks=1.5;
kfi=0 ;
kv=1.5;
ka= -0.8;
k=[ks,kv,ka];
%%%%%%%% Calculate the gains by extended LQR;
% Q=3.*eye(3);
% R=1;
% [X,L,G] = care(A,B,Q,R,[],[]);
% k=-G;%%%%%%%% Feedback gain
% kfi=-1/R*B'/((A-1/R*B*B'*X)')*X*D;%%%%%%%%feedforward gain
% Discretize the gains according to discretization step
Ai=[0,1,-tao;0,0,-1;0,0,-1/TT];
Bi=[0;0;1/TT];
Aci=Ai+Bi*k;
Di=[0;1;0];
DDi=Di;
Acid=expm(Aci*ts);
DDid=eye(3)/Aci*(Acid-eye(3))*DDi;
DDDid=(Acid-eye(3))/Aci*DDi;
Bki=eye(3)/Aci*(Acid-eye(3))*(Bi*kfi);
Bkid=(Acid-eye(3))/Aci*(Bi*kfi);
%%%%%%% Set the first vehicle trajectory and acceleration;
Tra=zeros(N,T);
Tra(1,:)=leada1(1,1:T);% initialize the leading vehicle trajectory
Speeddiff=zeros(N-1,T);
SSpeed=zeros(N,T);
deltae=zeros(N-1,T);
ACC=zeros(N-1,T);% ACC is the 
acc=leada1(1,1:T);% initialize the leading vehicle acceleration portofolio;
MACC=zeros(N,T);
x=zeros(3*(N-1),T-1);
% Delay Embedding for the first vehicle
for t=1:1:T-1
         if (t-theta1<=0)
             MACC(1,t)=0;
         else
            MACC(1,t)=acc(1,t-theta1);
         end
end
%%%%%%% Set the state matrix for all vehicles
for i=1:1:N-1
    if i==1
        for t=1:1:T-1
            x(3*i-2:3*i,t+1)=Acid*x(3*i-2:3*i,t)+Bki*MACC(i,t)+DDid*acc(1,t);
        end
    else
        for t=1:1:T-1
            x(3*i-2:3*i,t+1)=Acid*x(3*i-2:3*i,t)+Bki*MACC(i,t)+DDid*ACC(i-1,t);
        end
    end
    %%%%% Save the information
    ACC(i,:)=x(3*i,:);
    Speeddiff(i,:)=x(3*i-1,:);
    deltae(i,:)=x(3*i-2,:);
    %Communication Delay Embedding
    for t=1:1:T-1
         if (t-theta1<=0)
             MACC(i+1,t)=0;
         else
            MACC(i+1,t)=ACC(i,t-theta1);
         end
    end
end
%%%%%% Reconstruct the Trajcetory and speed
ACC=[acc;ACC];
%%%%%% Calculate back speed and position
sspeed=zeros(N,T-1);
es=zeros(N-1,T-1);
ps=zeros(N-1,T-1);
sspeed(1,1:T)=v(1,1:T);
stand=5;%%%%Standstill spacing
%%%Position
pposition=zeros(N,T);
pposition(1,1:T)=s(1,1:T);
for j=1:1:N-1
    for i=1:1:T
        sspeed(j+1,i)=sspeed(j,i)-Speeddiff(j,i);
        es(j,i)=sspeed(j+1,i)*tao+stand;
        ps(j,i)=deltae(j,i)+es(j,i);
        pposition(j+1,i)=pposition(j,i)-ps(j,i);
    end
end
%%%% Result plot:
t=0:0.1:(T-1)/10;
% figure(1);
% plot(t,deltae(1,:),t,deltae(2,:),t,deltae(3,:),t,deltae(4,:),t,deltae(5,:),'LineWidth',2.5);
% legend('CAV 1','CAV 2','CAV 3', 'CAV 4', 'CAV 5');
% xlabel('Time (sec)');
% ylabel({'\Delta d (m)'});
% ylim([-4,4])
% set(gca,'FontSize',26);
% figure(2);
% plot(t,Speeddiff(1,:),t,Speeddiff(2,:),t,Speeddiff(3,:),t,Speeddiff(4,:),t,Speeddiff(5,:),'LineWidth',2.5);
% legend('CAV 1','CAV 2','CAV 3', 'CAV 4', 'CAV 5');
% xlabel('Time (sec)');
% ylabel({'\Delta v (m/s)'});
% ylim([-3,3])
% set(gca,'FontSize',26);
% figure(3);
% plot(t,ACC(1,:),t,ACC(2,:),t,ACC(3,:),t,ACC(4,:),t,ACC(5,:),t,ACC(6,:),'LineWidth',2.5);
% legend('Vehicle 1845','CAV 1','CAV 2','CAV 3', 'CAV 4', 'CAV 5');
% xlabel('Time (sec)');
% ylabel({'a (m)/s2'});
% ylim([-4,4])
% set(gca,'FontSize',26);
% figure(4);
% plot(t,position(1,:),t,position(2,:),t,position(3,:),t,position(4,:),t,position(5,:),t,position(6,:),'LineWidth',2.5);
% legend('Vehicle 1845','CAV 1','CAV 2','CAV 3', 'CAV 4', 'CAV 5');
% xlabel('Time (sec)');
% ylabel({'position (m)'});
% set(gca,'FontSize',26);
% figure;
% plot(t,sspeed(1,:),t,sspeed(2,:),'LineWidth',2.5);
% legend('Vehicle 1845','CAV 1');

% Calculate the energy disturbance ratio
% Energy=zeros(1,6);
% for i=1:1:6
%     Energy(1,i)=(ACC(i,:)*(ACC(i,:)'))^0.5;
% end
% Ratio=zeros(1,6);
% for i=1:1:6
%     Ratio(1,i)=Energy(1,i)/Energy(1,1);%%%%%%%%%%Disturbance damping ratio
% end
