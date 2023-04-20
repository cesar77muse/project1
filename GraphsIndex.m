%

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Rotational_TimedWhileLoop_10';
load JRot10d;
figure
f=subplot(1,1,1);
mesh(qw,qq,J);
set(f,'XScale','Log');
set(f,'YScale','Log');
axis([1e-6 1e-2 1e-11 1e-4 5e-4 1.5e-3]);
set(f,'YTick',[1e-10 1e-7 1e-4]);
xlabel('q_w');
ylabel('q_q');
zlabel('J');


cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Rotational_TimedWhileLoop_30';
load JRot30d;
figure
f=subplot(1,1,1);
mesh(qw,qq,J);
set(f,'XScale','Log');
set(f,'YScale','Log');
axis([1e-6 1e-2 1e-11 1e-4 5e-4 1.5e-3]);
set(f,'YTick',[1e-10 1e-7 1e-4]);
xlabel('q_w');
ylabel('q_q');
zlabel('J');

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Rotational_TimedWhileLoop_50';
load JRot50d;;
figure
f=subplot(1,1,1);
mesh(qw,qq,J);
set(f,'XScale','Log');
set(f,'YScale','Log');
axis([1e-6 1e-2 1e-11 1e-4 5e-4 1.5e-3]);
set(f,'YTick',[1e-10 1e-7 1e-4]);
xlabel('q_w');
ylabel('q_q');
zlabel('J');

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Rotational_TimedWhileLoop_75';
load JRot75d;
figure
f=subplot(1,1,1);
mesh(qw,qq,J);
set(f,'XScale','Log');
set(f,'YScale','Log');
axis([1e-6 1e-2 1e-11 1e-4 5e-4 1.5e-3]);
set(f,'YTick',[1e-10 1e-7 1e-4]);
xlabel('q_w');
ylabel('q_q');
zlabel('J');

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Translational_TimedWhileLoop_10';
load JTras10d;
figure
f=subplot(1,1,1);
mesh(qa,qp,J);
set(f,'XScale','Log');
set(f,'YScale','Log');
axis([1e-2 1e4 1e-6 1e0 0.001 0.07]);
set(f,'XTick',[1e-2 1e1 1e4]);
set(f,'YTick',[1e-6 1e-3 1e0]);
xlabel('q_a');
ylabel('q_p');
zlabel('J');

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Translational_TimedWhileLoop_30';
load JTras30d;
figure
f=subplot(1,1,1);
mesh(qa,qp,J);
set(f,'XScale','Log');
set(f,'YScale','Log');
axis([1e-2 1e4 1e-6 1e0 0.001 0.07]);
set(f,'XTick',[1e-2 1e1 1e4]);
set(f,'YTick',[1e-6 1e-3 1e0]);
xlabel('q_a');
ylabel('q_p');
zlabel('J');

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Translational_TimedWhileLoop_50';
load JTras50d;
figure
f=subplot(1,1,1);
mesh(qa,qp,J);
set(f,'XScale','Log');
set(f,'YScale','Log');
axis([1e-2 1e4 1e-6 1e0 0.001 0.07]);
set(f,'XTick',[1e-2 1e1 1e4]);
set(f,'YTick',[1e-6 1e-3 1e0]);
xlabel('q_a');
ylabel('q_p');
zlabel('J');

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Translational_TimedWhileLoop_75';
load JTras75d;
figure
f=subplot(1,1,1);
mesh(qa,qp,J);
set(f,'XScale','Log');
set(f,'YScale','Log');
axis([1e-2 1e4 1e-6 1e0 0.001 0.07]);
set(f,'XTick',[1e-2 1e1 1e4]);
set(f,'YTick',[1e-6 1e-3 1e0]);
xlabel('q_a');
ylabel('q_p');
zlabel('J');
