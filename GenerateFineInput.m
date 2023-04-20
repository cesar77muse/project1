function [aDt, tsim] = GenerateFineInput(aInArray,dt,tend);

[AnzA dum] = size(aInArray);
clear dum;

tsim = 0:dt:tend;
aDt = zeros(size(tsim));

for i=1:AnzA
	if ( (aInArray(i,2) == aInArray(i,3)) & (aInArray(i,2) == aInArray(i,4)) )
		StartInd = round(aInArray(i,1)/dt+1);
		if (i<AnzA)
			EndInd = round(aInArray(i,2)/dt);
		else
			EndArray = dim;
		end
		aDt(StartInd:EndInd) = aInArray(i,3);
	elseif (aInArray(i,3) == aInArray(i,4))
		StartInd = round(aInArray(i,1)/dt+1);
		EndInd = round(aInArray(i,2)/dt);
		aDt(StartInd:EndInd) = aInArray(i,3);
	else
		StartInd = round(aInArray(i,1)/dt+1);
		EndInd = round(aInArray(i,2)/dt);
		d = aInArray(i,3);
		k = (aInArray(i,4) - aInArray(i,3)) / (aInArray(i,2) - aInArray(i,1));
		for j=StartInd:EndInd
			aDt(j) = d + k*(tsim(j)-aInArray(i,1));
		end
	end
end

for i=2:AnzA
	if ( (i>1) & (aInArray(i,1) ~= aInArray(i-1,2)) )
		StartInd = round(aInArray(i-1,2)/dt+1);
		EndInd = round(aInArray(i,1)/dt);
		aDt(StartInd:EndInd) = aInArray(i-1,4);
	end
end