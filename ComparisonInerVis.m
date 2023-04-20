%
cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Rotational_TimedWhileLoop_10';
load InertialVisionData;

[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,0);
[JRotInerVis(1)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,1);
[JRotVis(1)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,2);
[JRotIner(1)]=CalculateIndex(p_ref,q_ref,X);

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Rotational_TimedWhileLoop_30';
load InertialVisionData;

[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,0);
[JRotInerVis(2)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,1);
[JRotVis(2)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,2);
[JRotIner(2)]=CalculateIndex(p_ref,q_ref,X);

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Rotational_TimedWhileLoop_50';
load InertialVisionData;

[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,0);
[JRotInerVis(3)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,1);
[JRotVis(3)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,2);
[JRotIner(3)]=CalculateIndex(p_ref,q_ref,X);

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Rotational_TimedWhileLoop_75';
load InertialVisionData;

[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,0);
[JRotInerVis(4)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,1);
[JRotVis(4)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,2);
[JRotIner(4)]=CalculateIndex(p_ref,q_ref,X);

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Translational_TimedWhileLoop_10';
load InertialVisionData;

[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,0);
[JTrasInerVis(1)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,1);
[JTrasVis(1)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,2);
[JTrasIner(1)]=CalculateIndex(p_ref,q_ref,X);

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Translational_TimedWhileLoop_30';
load InertialVisionData;

[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,0);
[JTrasInerVis(2)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,1);
[JTrasVis(2)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,2);
[JTrasIner(2)]=CalculateIndex(p_ref,q_ref,X);

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Translational_TimedWhileLoop_50';
load InertialVisionData;

[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,0);
[JTrasInerVis(3)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,1);
[JTrasVis(3)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,2);
[JTrasIner(3)]=CalculateIndex(p_ref,q_ref,X);

cd '\\leoaran2\Leo\Investigacion\Robotica_Movil\VisionInertial\ICRA Workshop\Experimental Data\Translational_TimedWhileLoop_75';
load InertialVisionData;

[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,0);
[JTrasInerVis(4)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,1);
[JTrasVis(4)]=CalculateIndex(p_ref,q_ref,X);
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R,2);
[JTrasIner(4)]=CalculateIndex(p_ref,q_ref,X);

figure;
subplot(2,1,1);
plot(dB([JRotInerVis' JRotIner' JRotVis']))
subplot(2,1,2);
plot(dB([JTrasInerVis' JTrasIner' JTrasVis']))