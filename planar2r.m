function och = planar2r()

och.icf = @planar2r_icf; 
och.tcf = @planar2r_tcf; 
och.fcf = @planar2r_fcf; 

och.nlicf = @planar2r_nlicf; 
och.nltcf = @planar2r_nltcf; 
och.nlfcf = @planar2r_nlfcf; 
och.nlgcf = @planar2r_nlgcf; 



function [f,df] = planar2r_icf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 


end


function [f,df] = planar2r_tcf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 
	 f(1,:) = a1d.^2+a2d.^2+a4d.^2+a5d.^2;

	 df(1,:)= zeros(size(a1)); 
	 df(2,:)= 2.*a1d; 
	 df(3,:)= zeros(size(a2)); 
	 df(4,:)= 2.*a2d; 
	 df(5,:)= zeros(size(a3)); 
	 df(6,:)= zeros(size(a3d)); 
	 df(7,:)= zeros(size(a4)); 
	 df(8,:)= 2.*a4d; 
	 df(9,:)= zeros(size(a5)); 
	 df(10,:)= 2.*a5d; 
	 df(11,:)= zeros(size(a6)); 
	 df(12,:)= zeros(size(a6d)); 

end


function [f,df] = planar2r_fcf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 


end


function [f,df] = planar2r_nlicf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 
	 f(1,:) = 3.625.*sin(a2 + a3) + 3.625.*sin(a3) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5);
	 f(2,:) = 3.625.*cos(a2 + a3) + 3.625.*cos(a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5);

	 df(1,1,:) = - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(2,1,:) = zeros(size(a1d)); 
	 df(3,1,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(4,1,:) = zeros(size(a2d)); 
	 df(5,1,:) = 3.625.*cos(a2 + a3) + 3.625.*cos(a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(6,1,:) = zeros(size(a3d)); 
	 df(7,1,:) = 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) + 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(8,1,:) = zeros(size(a4d)); 
	 df(9,1,:) = 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(10,1,:) = zeros(size(a5d)); 
	 df(11,1,:) = zeros(size(a6)); 
	 df(12,1,:) = zeros(size(a6d)); 
	 df(1,2,:) = 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) + 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(2,2,:) = zeros(size(a1d)); 
	 df(3,2,:) = 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) - 3.625.*sin(a2 + a3) + 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(4,2,:) = zeros(size(a2d)); 
	 df(5,2,:) = 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) - 3.625.*sin(a3) - 3.625.*sin(a2 + a3) + 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(6,2,:) = zeros(size(a3d)); 
	 df(7,2,:) = - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(8,2,:) = zeros(size(a4d)); 
	 df(9,2,:) = -3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(10,2,:) = zeros(size(a5d)); 
	 df(11,2,:) = zeros(size(a6)); 
	 df(12,2,:) = zeros(size(a6d)); 

end


function [f,df] = planar2r_nltcf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 
	 f(1,:) = (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 - (277.*sin(a1 + a2 + a3 - a4 - a5))./400 + (1173.*sin(a2 + a3))./400 - (389.*sin(a1 + a2 + a3 - a4))./300 + (1061.*sin(a3))./300 + (42809.^(1./2).*cos(a1 + a2 + a3 - 46153121068169./35184372088832))./600;
	 f(2,:) = (1173.*cos(a2 + a3))./400 - (277.*cos(a1 + a2 + a3 - a4 - a5))./400 - (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 - (389.*cos(a1 + a2 + a3 - a4))./300 + (1061.*cos(a3))./300 + (42809.^(1./2).*cos(a1 + a2 + a3 + 145829781912851./562949953421312))./600;

	 df(1,1,:) = - (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 - (277.*cos(a1 + a2 + a3 - a4 - a5))./400 - (389.*cos(a1 + a2 + a3 - a4))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 - 46153121068169./35184372088832))./600; 
	 df(2,1,:) = zeros(size(a1d)); 
	 df(3,1,:) = (1173.*cos(a2 + a3))./400 - (277.*cos(a1 + a2 + a3 - a4 - a5))./400 - (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 - (389.*cos(a1 + a2 + a3 - a4))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 - 46153121068169./35184372088832))./600; 
	 df(4,1,:) = zeros(size(a2d)); 
	 df(5,1,:) = (1173.*cos(a2 + a3))./400 - (277.*cos(a1 + a2 + a3 - a4 - a5))./400 - (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 - (389.*cos(a1 + a2 + a3 - a4))./300 + (1061.*cos(a3))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 - 46153121068169./35184372088832))./600; 
	 df(6,1,:) = zeros(size(a3d)); 
	 df(7,1,:) = (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 + (277.*cos(a1 + a2 + a3 - a4 - a5))./400 + (389.*cos(a1 + a2 + a3 - a4))./300; 
	 df(8,1,:) = zeros(size(a4d)); 
	 df(9,1,:) = (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 + (277.*cos(a1 + a2 + a3 - a4 - a5))./400; 
	 df(10,1,:) = zeros(size(a5d)); 
	 df(11,1,:) = (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600; 
	 df(12,1,:) = zeros(size(a6d)); 
	 df(1,2,:) = (277.*sin(a1 + a2 + a3 - a4 - a5))./400 - (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 + (389.*sin(a1 + a2 + a3 - a4))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 + 145829781912851./562949953421312))./600; 
	 df(2,2,:) = zeros(size(a1d)); 
	 df(3,2,:) = (277.*sin(a1 + a2 + a3 - a4 - a5))./400 - (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 - (1173.*sin(a2 + a3))./400 + (389.*sin(a1 + a2 + a3 - a4))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 + 145829781912851./562949953421312))./600; 
	 df(4,2,:) = zeros(size(a2d)); 
	 df(5,2,:) = (277.*sin(a1 + a2 + a3 - a4 - a5))./400 - (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 - (1173.*sin(a2 + a3))./400 + (389.*sin(a1 + a2 + a3 - a4))./300 - (1061.*sin(a3))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 + 145829781912851./562949953421312))./600; 
	 df(6,2,:) = zeros(size(a3d)); 
	 df(7,2,:) = (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 - (277.*sin(a1 + a2 + a3 - a4 - a5))./400 - (389.*sin(a1 + a2 + a3 - a4))./300; 
	 df(8,2,:) = zeros(size(a4d)); 
	 df(9,2,:) = (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 - (277.*sin(a1 + a2 + a3 - a4 - a5))./400; 
	 df(10,2,:) = zeros(size(a5d)); 
	 df(11,2,:) = (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600; 
	 df(12,2,:) = zeros(size(a6d)); 

end


function [f,df] = planar2r_nlfcf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 
	 f(1,:) = 3.625.*sin(a2 + a3) + 3.625.*sin(a3) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5);
	 f(2,:) = 3.625.*cos(a2 + a3) + 3.625.*cos(a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5);

	 df(1,1,:) = - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(2,1,:) = zeros(size(a1d)); 
	 df(3,1,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(4,1,:) = zeros(size(a2d)); 
	 df(5,1,:) = 3.625.*cos(a2 + a3) + 3.625.*cos(a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(6,1,:) = zeros(size(a3d)); 
	 df(7,1,:) = 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) + 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(8,1,:) = zeros(size(a4d)); 
	 df(9,1,:) = 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(10,1,:) = zeros(size(a5d)); 
	 df(11,1,:) = zeros(size(a6)); 
	 df(12,1,:) = zeros(size(a6d)); 
	 df(1,2,:) = 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) + 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(2,2,:) = zeros(size(a1d)); 
	 df(3,2,:) = 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) - 3.625.*sin(a2 + a3) + 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(4,2,:) = zeros(size(a2d)); 
	 df(5,2,:) = 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) - 3.625.*sin(a3) - 3.625.*sin(a2 + a3) + 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(6,2,:) = zeros(size(a3d)); 
	 df(7,2,:) = - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(8,2,:) = zeros(size(a4d)); 
	 df(9,2,:) = -3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5); 
	 df(10,2,:) = zeros(size(a5d)); 
	 df(11,2,:) = zeros(size(a6)); 
	 df(12,2,:) = zeros(size(a6d)); 

end


function [f,df] = planar2r_nlgcf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 


end


end