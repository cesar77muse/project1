function [NoiseW, NoiseA, NoiseP, NoiseQ] = NoiseInit(NoiseFak,dim);

if nargin == 1
	dim = 5001;
end

NoiseW = normrnd(0,0.05,3,dim)*NoiseFak;
NoiseA = normrnd(0,0.05,3,dim)*NoiseFak;

NoiseP = normrnd(0,0.025,3,dim)*NoiseFak;
NoiseQ = normrnd(0,0.01,4,dim)*NoiseFak; 