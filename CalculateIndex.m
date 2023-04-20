function [J_pq,J_p,J_q]=CalculateIndex(p_ref,q_ref,X)

l1=length(q_ref);
l2=length(X);
l=min([l1 l2]);
q_ref=q_ref(1:l,:);
p_ref=p_ref(1:l,:);
X=X(1:l,:);
J_q=(1/2)*sum((q_ref(:,1)-X(:,10)).^2+(q_ref(:,2)-X(:,11)).^2+(q_ref(:,3)-X(:,12)).^2+(q_ref(:,4)-X(:,13)).^2)/l;
J_p=(1/2)*sum((p_ref(:,1)-X(:,1)).^2+(p_ref(:,2)-X(:,2)).^2+(p_ref(:,3)-X(:,3)).^2)/l;
J_pq=(J_q+J_p)/2;
%J=Jq;