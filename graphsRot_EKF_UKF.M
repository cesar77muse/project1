%close all;

tv=(0:length(Vision(:,1))-1)*Tv;
ti=(0:length(Inertial(:,1))-1)*Ti;
tref=(0:length(q_ref)-1)'*Ti;

f4=figure;
subplot(4,1,1);
hold on;
subplot(4,1,2);
hold on;
subplot(4,1,3);
hold on;
subplot(4,1,4);
hold on;
subplot(4,1,1);
plot(TIME_EKF,X_EKF(:,10),'r');
plot(TIME_UKF,X_UKF(:,10),'k');
plot(tref,q_ref(:,1),'--','Color',[0 0.5 0]);
v=axis;v(2)=tv(length(tv));axis(v);ylabel('q_0');
subplot(4,1,2);
plot(TIME_EKF,X_EKF(:,11),'r');
plot(TIME_UKF,X_UKF(:,11),'k');
plot(tref,q_ref(:,2),'--','Color',[0 0.5 0]);
v=axis;v(2)=tv(length(tv));axis(v);ylabel('q_1');
subplot(4,1,3);
plot(TIME_EKF,X_EKF(:,12),'r');
plot(TIME_UKF,X_UKF(:,12),'k');
plot(tref,q_ref(:,3),'--','Color',[0 0.5 0]);
v=axis;v(2)=tv(length(tv));axis(v);ylabel('q_2');
subplot(4,1,4);
plot(TIME_EKF,X_EKF(:,13),'r');
plot(TIME_UKF,X_UKF(:,13),'k');
plot(tref,q_ref(:,4),'--','Color',[0 0.5 0]);
v=axis;v(2)=tv(length(tv));axis(v);ylabel('q_3');xlabel('Time [sec]');
for i=1:length(Vision(:,1))
	if (Vision(i,1)==1)
		subplot(4,1,1);plot(tv(i),Vision(i,8),'.');
		subplot(4,1,2);plot(tv(i),Vision(i,9),'.');
		subplot(4,1,3);plot(tv(i),Vision(i,10),'.');
        subplot(4,1,4);plot(tv(i),Vision(i,11),'.');
	end
end





