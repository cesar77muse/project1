%Simplex method to calculate the noise covariance related with a rotational
%movement

load InertialVisionData

%System noise
Q=[74.47*eye(3) zeros(3,6);zeros(3) 38*eye(3) zeros(3);zeros(3,6) 1.9074e-5*eye(3)];


%Measurement noise	 
p0=[1;1];
p1=[5;1];
p2=[3;sqrt(12)];

R0 = [p0(1)*1e-2*eye(3,3) zeros(3,10)
     zeros(3,3) 1.8149e-3*eye(3,3) zeros(3,7)
     zeros(3,6) p0(2)*1e-5*eye(3,3) zeros(3,4)
     zeros(4,9) 9.9441e-5*eye(4)];
R1 = [p1(1)*1e-2*eye(3,3) zeros(3,10)
     zeros(3,3) 1.8149e-3*eye(3,3) zeros(3,7)
     zeros(3,6) p1(2)*1e-5*eye(3,3) zeros(3,4)
     zeros(4,9) 9.9441e-5*eye(4)];
R2 = [p2(1)*1e-2*eye(3,3) zeros(3,10)
     zeros(3,3) 1.8149e-3*eye(3,3) zeros(3,7)
     zeros(3,6) p2(2)*1e-5*eye(3,3) zeros(3,4)
     zeros(4,9) 9.9441e-5*eye(4)];

[X0,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R0);
J0=CalculateIndexTras(p_ref,X0);
[X1,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R1);
J1=CalculateIndexTras(p_ref,X1);
[X2,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R2);
J2=CalculateIndexTras(p_ref,X2);

J=[J0;J1;J2];
P=[p0 p1 p2];
[J,I]=sort(J);

J=flipud(J);
I=flipud(I);
P=P(:,I);


l=sqrt((p0(1)-p1(1))^2+(p0(2)-p1(2))^2);
prec=1e-1;
Pprev=[0;0];

%Simplex method

close all;

figure(1);
plot([P(1,:) P(1,1)],[P(2,:) P(2,1)],'-*');
hold on;

while(l>prec)
    %Worst point projection
    p0=P(:,2)+P(:,3)-P(:,1);
    if (p0(1)<=0 || p0(2)<=0) || ((Pprev(1)==p0(1))&&(Pprev(2)==p0(2)))
        %Triangle reduction
        Pprev=P(:,3);
        P(:,1)=(P(:,1)+P(:,3))/2;
        P(:,2)=(P(:,2)+P(:,3))/2;
        
        R0 = [P(1,1)*1e-2*eye(3,3) zeros(3,10)
              zeros(3,3) 1.8149e-3*eye(3,3) zeros(3,7)
              zeros(3,6) P(2,1)*1e-5*eye(3,3) zeros(3,4)
              zeros(4,9) 9.9441e-5*eye(4)];
        R1 = [P(1,2)*1e-2*eye(3,3) zeros(3,10)
              zeros(3,3) 1.8149e-3*eye(3,3) zeros(3,7)
              zeros(3,6) P(2,2)*1e-5*eye(3,3) zeros(3,4)
              zeros(4,9) 9.9441e-5*eye(4)];
        [X0,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R0);
        J0=CalculateIndexTras(p_ref,X0);
        [X1,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R1);
        J1=CalculateIndexTras(p_ref,X1);
        J=[J0;J1;J(3)];
        [J,I]=sort(J);
        J=flipud(J);
        I=flipud(I);
        P=P(:,I);
        l=sqrt((P(1,1)-P(1,2))^2+(P(2,1)-P(2,2))^2);
    else
        Pprev=P(:,1);
        P(:,1)=p0;
        R0 = [P(1,1)*1e-2*eye(3,3) zeros(3,10)
              zeros(3,3) 1.8149e-3*eye(3,3) zeros(3,7)
              zeros(3,6) P(2,1)*1e-5*eye(3,3) zeros(3,4)
              zeros(4,9) 9.9441e-5*eye(4)];
        [X0,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R0);
        J0=CalculateIndexTras(p_ref,X0);
        J(1)=J0;
        [J,I]=sort(J);
        J=flipud(J);
        I=flipud(I);
        P=P(:,I);
        l=sqrt((P(1,1)-P(1,2))^2+(P(2,1)-P(2,2))^2);
    end
    P(:,3)
    J(3)
    plot([P(1,:) P(1,1)],[P(2,:) P(2,1)]);
    pause(1);
end

p=(P(:,3)-(P(:,1)+P(:,2))/2)/3+(P(:,1)+P(:,2))/2;
R = [p(1)*1e-2*eye(3,3) zeros(3,10)
              zeros(3,3) 1.8149e-3*eye(3,3) zeros(3,7)
              zeros(3,6) p(2)*1e-5*eye(3,3) zeros(3,4)
              zeros(4,9) 9.9441e-5*eye(4)];
[X,TIME,AINew,WINew]=MR_EKF(Inertial,Vision,Ti,Tv,Q,R);

          
          
        
    
        
    
    
    
    
    
    
    
