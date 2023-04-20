function [J]=CalculateIndexRot(q_ref,X)

l1=length(q_ref);
l2=length(X);
l=min([l1 l2]);
q_ref=q_ref(1:l,:);
X=X(1:l,:);
Jq=sum((q_ref(:,1)-X(:,10)).^2+(q_ref(:,2)-X(:,11)).^2+(q_ref(:,3)-X(:,12)).^2+(q_ref(:,4)-X(:,13)).^2);
J=Jq/l;