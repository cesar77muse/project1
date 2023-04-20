%close all;

tv=(0:length(Vision(:,1))-1)*Tv;
ti=(0:length(Inertial(:,1))-1)*Ti;
tref=(0:length(q_ref)-1)'*Ti;

f3=figure;
subplot(2,1,1);
hold on;
subplot(2,1,2);
hold on;
subplot(2,1,1);
plot(TIME_EKF,X_EKF(:,1),'r');
plot(TIME_UKF,X_UKF(:,1),'k');
%plot(tref,p_ref(:,1),'--','Color',[0 0.5 0]);
v=axis;v(2)=tv(length(tv));axis(v);ylabel('p_x');
subplot(2,1,2);
plot(TIME_EKF,X_EKF(:,2),'r');
plot(TIME_UKF,X_UKF(:,2),'k');
%plot(tref,p_ref(:,2),'--','Color',[0 0.5 0]);
v=axis;v(2)=tv(length(tv));axis(v);ylabel('p_y');;xlabel('Time [sec]');
for i=1:length(Vision(:,1))
	if (Vision(i,1)==1)
		subplot(2,1,1);plot(tv(i),Vision(i,2),'.');
		subplot(2,1,2);plot(tv(i),Vision(i,3),'.');
	end
end





