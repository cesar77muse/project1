%Plots all graphs
tv=(0:length(Vision(:,1))-1)*Tv;
ti=(0:length(Inertial(:,1))-1)*Ti;
tref=(0:length(p_ref)-1)'*Ti;

f1=figure;
subplot(3,1,1);
hold on;
subplot(3,1,2);
hold on;
subplot(3,1,3);
hold on;
for i=1:length(Vision(:,1))
	if (Vision(i,1)==1)
		subplot(3,1,1);plot(tv(i),Vision(i,2),'.');
		subplot(3,1,2);plot(tv(i),Vision(i,3),'.');
		subplot(3,1,3);plot(tv(i),Vision(i,4),'.');
	end
end
subplot(3,1,1);
plot(TIME,X(:,1),'r');
plot(tref,p_ref(:,1),'k');
v=axis;v(2)=tv(length(tv));axis(v);ylabel('p_x [m]');hold off;
subplot(3,1,2);
plot(TIME,X(:,2),'r');
plot(tref,p_ref(:,2),'k');
v=axis;v(2)=tv(length(tv));axis(v);ylabel('p_y [m]');hold off;
subplot(3,1,3);
plot(TIME,X(:,3),'r');
plot(tref,p_ref(:,3),'k');
v=axis;v(2)=tv(length(tv));axis(v);ylabel('p_z [m]');xlabel('Time [sec]');hold off;

if length(AINew>0)
f2=figure;
subplot(3,1,1);
plot(ti,AINew(:,1)-mean(AINew(:,1)));hold on;
subplot(3,1,2);
plot(ti,AINew(:,2)-mean(AINew(:,2)));hold on;
subplot(3,1,3);
plot(ti,AINew(:,3)-mean(AINew(:,3)));hold on;
for i=1:length(Inertial(:,1))
	if (Inertial(i,1)==0)
		subplot(3,1,1);plot(ti(i),AINew(i,1)-mean(AINew(:,1)),'ko');
		subplot(3,1,2);plot(ti(i),AINew(i,2)-mean(AINew(:,2)),'ko');
		subplot(3,1,3);plot(ti(i),AINew(i,3)-mean(AINew(:,3)),'ko');
	end
end
subplot(3,1,1);
plot(TIME,X(:,7),'r');
v=axis;v(2)=ti(length(ti));axis(v);ylabel('a_x [m/s^2]');hold off;
subplot(3,1,2);
plot(TIME,X(:,8),'r');
v=axis;v(2)=ti(length(ti));axis(v);ylabel('a_y [m/s^2]');hold off;
subplot(3,1,3);
plot(TIME,X(:,9),'r');
v=axis;v(2)=ti(length(ti));axis(v);ylabel('a_z [m/s^2]');xlabel('Time [sec]');hold off;

f3=figure;
subplot(3,1,1);
plot(ti,WINew(:,1));hold on;
subplot(3,1,2);
plot(ti,WINew(:,2));hold on;
subplot(3,1,3);
plot(ti,WINew(:,3));hold on;
for i=1:length(Inertial(:,1))
	if (Inertial(i,1)==0)
		subplot(3,1,1);plot(ti(i),WINew(i,1),'ro');
		subplot(3,1,2);plot(ti(i),WINew(i,2),'ro');
		subplot(3,1,3);plot(ti(i),WINew(i,3),'ro');
	end
end
subplot(3,1,1);
plot(TIME,X(:,14),'r');
v=axis;v(2)=ti(length(ti));axis(v);ylabel('w_x [rad/s]');hold off;
subplot(3,1,2);
plot(TIME,X(:,15),'r');
v=axis;v(2)=ti(length(ti));axis(v);ylabel('w_y [rad/s]');hold off;
subplot(3,1,3);
plot(TIME,X(:,16),'r');
v=axis;v(2)=ti(length(ti));axis(v);ylabel('w_z [rad/s]');xlabel('Time [sec]');hold off;
end

f4=figure;
subplot(4,1,1);
hold on;
subplot(4,1,2);
hold on;
subplot(4,1,3);
hold on;
subplot(4,1,4);
hold on;
for i=1:length(Vision(:,1))
	if (Vision(i,1)==1)
		subplot(4,1,1);plot(tv(i),Vision(i,8),'.');
		subplot(4,1,2);plot(tv(i),Vision(i,9),'.');
		subplot(4,1,3);plot(tv(i),Vision(i,10),'.');
        subplot(4,1,4);plot(tv(i),Vision(i,11),'.');
	end
end
subplot(4,1,1);
plot(TIME,X(:,10),'r');
plot(tref,q_ref(:,1),'k');
v=axis;v(2)=tv(length(tv));axis(v);ylabel('q_0');hold off;
subplot(4,1,2);
plot(TIME,X(:,11),'r');
plot(tref,q_ref(:,2),'k');
v=axis;v(2)=tv(length(tv));axis(v);ylabel('q_1');hold off;
subplot(4,1,3);
plot(TIME,X(:,12),'r');
plot(tref,q_ref(:,3),'k');
v=axis;v(2)=tv(length(tv));axis(v);ylabel('q_2');hold off;
subplot(4,1,4);
plot(TIME,X(:,13),'r');
plot(tref,q_ref(:,4),'k');
v=axis;v(2)=tv(length(tv));axis(v);ylabel('q_3');xlabel('Time [sec]');hold off;



%save Vision.txt Vision -ascii;
%save Inertial.txt Inertial -ascii;





