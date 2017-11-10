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
 
	 f(1,:) = a1d.^2+a1d+a2d.^2+a4d.^2+a4d+a5d.^2;

	 df(1,:)= zeros(size(a1)); 
	 df(2,:)= 2.*a1d + 1; 
	 df(3,:)= zeros(size(a2)); 
	 df(4,:)= 2.*a2d; 
	 df(5,:)= zeros(size(a3)); 
	 df(6,:)= zeros(size(a3d)); 
	 df(7,:)= zeros(size(a4)); 
	 df(8,:)= 2.*a4d + 1; 
	 df(9,:)= zeros(size(a5)); 
	 df(10,:)= 2.*a5d; 
	 df(11,:)= zeros(size(a6)); 
	 df(12,:)= zeros(size(a6d)); 

end


function [f,df] = planar2r_fcf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 


end


function [f,df] = planar2r_nlicf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 
	 f(1,:) = 3.625.*sin(a2 + a3) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) + 3.625.*sin(a3) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5);
	 f(2,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) + 3.625.*cos(a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5);

	 df(1,1,:) = - 3.625.*cos(1.0.*a4 - a2 - a3 - a1) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(2,1,:) = zeros(size(a1d)); 
	 df(3,1,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1); 
	 df(4,1,:) = zeros(size(a2d)); 
	 df(5,1,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1) + 3.625.*cos(a3); 
	 df(6,1,:) = zeros(size(a3d)); 
	 df(7,1,:) = 3.625.*cos(1.0.*a4 - a2 - a3 - a1) + 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(8,1,:) = zeros(size(a4d)); 
	 df(9,1,:) = 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(10,1,:) = zeros(size(a5d)); 
	 df(11,1,:) = zeros(size(a6)); 
	 df(12,1,:) = zeros(size(a6d)); 
	 df(1,2,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(2,2,:) = zeros(size(a1d)); 
	 df(3,2,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*sin(a2 + a3); 
	 df(4,2,:) = zeros(size(a2d)); 
	 df(5,2,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*sin(a2 + a3) - 3.625.*sin(a3); 
	 df(6,2,:) = zeros(size(a3d)); 
	 df(7,2,:) = 3.625.*sin(1.0.*a4 - a2 - a3 - a1) + 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(8,2,:) = zeros(size(a4d)); 
	 df(9,2,:) = 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(10,2,:) = zeros(size(a5d)); 
	 df(11,2,:) = zeros(size(a6)); 
	 df(12,2,:) = zeros(size(a6d)); 

end


function [f,df] = planar2r_nltcf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 
	 f(1,:) = 3.625.*sin(a2 + a3) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) + 3.625.*sin(a3) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5);
	 f(2,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) + 3.625.*cos(a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5);
	 f(3,:) = (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 - (277.*sin(a1 + a2 + a3 - a4 - a5))./400 + (1173.*sin(a2 + a3))./400 - (389.*sin(a1 + a2 + a3 - a4))./300 + (1061.*sin(a3))./300 + (42809.^(1./2).*cos(a1 + a2 + a3 - 46153121068169./35184372088832))./600;
	 f(4,:) = (1173.*cos(a2 + a3))./400 - (277.*cos(a1 + a2 + a3 - a4 - a5))./400 - (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 - (389.*cos(a1 + a2 + a3 - a4))./300 + (1061.*cos(a3))./300 + (42809.^(1./2).*cos(a1 + a2 + a3 + 145829781912851./562949953421312))./600;
	 f(5,:) = (29.0.*sin(a3))./(8.0.*cos(a3).^2 + 8.0.*sin(a3).^2);
	 f(6,:) = (29.0.*cos(a3))./(8.0.*cos(a3).^2 + 8.0.*sin(a3).^2);
	 f(7,:) = 3.625.*sin(a2 + a3) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) + 3.625.*sin(a3);
	 f(8,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) + 3.625.*cos(a3);

	 df(1,1,:) = - 3.625.*cos(1.0.*a4 - a2 - a3 - a1) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(2,1,:) = zeros(size(a1d)); 
	 df(3,1,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1); 
	 df(4,1,:) = zeros(size(a2d)); 
	 df(5,1,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1) + 3.625.*cos(a3); 
	 df(6,1,:) = zeros(size(a3d)); 
	 df(7,1,:) = 3.625.*cos(1.0.*a4 - a2 - a3 - a1) + 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(8,1,:) = zeros(size(a4d)); 
	 df(9,1,:) = 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(10,1,:) = zeros(size(a5d)); 
	 df(11,1,:) = zeros(size(a6)); 
	 df(12,1,:) = zeros(size(a6d)); 
	 df(1,2,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(2,2,:) = zeros(size(a1d)); 
	 df(3,2,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*sin(a2 + a3); 
	 df(4,2,:) = zeros(size(a2d)); 
	 df(5,2,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*sin(a2 + a3) - 3.625.*sin(a3); 
	 df(6,2,:) = zeros(size(a3d)); 
	 df(7,2,:) = 3.625.*sin(1.0.*a4 - a2 - a3 - a1) + 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(8,2,:) = zeros(size(a4d)); 
	 df(9,2,:) = 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(10,2,:) = zeros(size(a5d)); 
	 df(11,2,:) = zeros(size(a6)); 
	 df(12,2,:) = zeros(size(a6d)); 
	 df(1,3,:) = - (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 - (277.*cos(a1 + a2 + a3 - a4 - a5))./400 - (389.*cos(a1 + a2 + a3 - a4))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 - 46153121068169./35184372088832))./600; 
	 df(2,3,:) = zeros(size(a1d)); 
	 df(3,3,:) = (1173.*cos(a2 + a3))./400 - (277.*cos(a1 + a2 + a3 - a4 - a5))./400 - (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 - (389.*cos(a1 + a2 + a3 - a4))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 - 46153121068169./35184372088832))./600; 
	 df(4,3,:) = zeros(size(a2d)); 
	 df(5,3,:) = (1173.*cos(a2 + a3))./400 - (277.*cos(a1 + a2 + a3 - a4 - a5))./400 - (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 - (389.*cos(a1 + a2 + a3 - a4))./300 + (1061.*cos(a3))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 - 46153121068169./35184372088832))./600; 
	 df(6,3,:) = zeros(size(a3d)); 
	 df(7,3,:) = (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 + (277.*cos(a1 + a2 + a3 - a4 - a5))./400 + (389.*cos(a1 + a2 + a3 - a4))./300; 
	 df(8,3,:) = zeros(size(a4d)); 
	 df(9,3,:) = (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600 + (277.*cos(a1 + a2 + a3 - a4 - a5))./400; 
	 df(10,3,:) = zeros(size(a5d)); 
	 df(11,3,:) = (53.*sin(a1 + a2 + a3 - a4 - a5 - a6))./600; 
	 df(12,3,:) = zeros(size(a6d)); 
	 df(1,4,:) = (277.*sin(a1 + a2 + a3 - a4 - a5))./400 - (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 + (389.*sin(a1 + a2 + a3 - a4))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 + 145829781912851./562949953421312))./600; 
	 df(2,4,:) = zeros(size(a1d)); 
	 df(3,4,:) = (277.*sin(a1 + a2 + a3 - a4 - a5))./400 - (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 - (1173.*sin(a2 + a3))./400 + (389.*sin(a1 + a2 + a3 - a4))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 + 145829781912851./562949953421312))./600; 
	 df(4,4,:) = zeros(size(a2d)); 
	 df(5,4,:) = (277.*sin(a1 + a2 + a3 - a4 - a5))./400 - (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 - (1173.*sin(a2 + a3))./400 + (389.*sin(a1 + a2 + a3 - a4))./300 - (1061.*sin(a3))./300 - (42809.^(1./2).*sin(a1 + a2 + a3 + 145829781912851./562949953421312))./600; 
	 df(6,4,:) = zeros(size(a3d)); 
	 df(7,4,:) = (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 - (277.*sin(a1 + a2 + a3 - a4 - a5))./400 - (389.*sin(a1 + a2 + a3 - a4))./300; 
	 df(8,4,:) = zeros(size(a4d)); 
	 df(9,4,:) = (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600 - (277.*sin(a1 + a2 + a3 - a4 - a5))./400; 
	 df(10,4,:) = zeros(size(a5d)); 
	 df(11,4,:) = (53.*cos(a1 + a2 + a3 - a4 - a5 - a6))./600; 
	 df(12,4,:) = zeros(size(a6d)); 
	 df(1,5,:) = zeros(size(a1)); 
	 df(2,5,:) = zeros(size(a1d)); 
	 df(3,5,:) = zeros(size(a2)); 
	 df(4,5,:) = zeros(size(a2d)); 
	 df(5,5,:) = (29.0.*cos(a3))./(8.0.*cos(a3).^2 + 8.0.*sin(a3).^2); 
	 df(6,5,:) = zeros(size(a3d)); 
	 df(7,5,:) = zeros(size(a4)); 
	 df(8,5,:) = zeros(size(a4d)); 
	 df(9,5,:) = zeros(size(a5)); 
	 df(10,5,:) = zeros(size(a5d)); 
	 df(11,5,:) = zeros(size(a6)); 
	 df(12,5,:) = zeros(size(a6d)); 
	 df(1,6,:) = zeros(size(a1)); 
	 df(2,6,:) = zeros(size(a1d)); 
	 df(3,6,:) = zeros(size(a2)); 
	 df(4,6,:) = zeros(size(a2d)); 
	 df(5,6,:) = -(29.0.*sin(a3))./(8.0.*cos(a3).^2 + 8.0.*sin(a3).^2); 
	 df(6,6,:) = zeros(size(a3d)); 
	 df(7,6,:) = zeros(size(a4)); 
	 df(8,6,:) = zeros(size(a4d)); 
	 df(9,6,:) = zeros(size(a5)); 
	 df(10,6,:) = zeros(size(a5d)); 
	 df(11,6,:) = zeros(size(a6)); 
	 df(12,6,:) = zeros(size(a6d)); 
	 df(1,7,:) = -3.625.*cos(1.0.*a4 - a2 - a3 - a1); 
	 df(2,7,:) = zeros(size(a1d)); 
	 df(3,7,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1); 
	 df(4,7,:) = zeros(size(a2d)); 
	 df(5,7,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1) + 3.625.*cos(a3); 
	 df(6,7,:) = zeros(size(a3d)); 
	 df(7,7,:) = 3.625.*cos(1.0.*a4 - a2 - a3 - a1); 
	 df(8,7,:) = zeros(size(a4d)); 
	 df(9,7,:) = zeros(size(a5)); 
	 df(10,7,:) = zeros(size(a5d)); 
	 df(11,7,:) = zeros(size(a6)); 
	 df(12,7,:) = zeros(size(a6d)); 
	 df(1,8,:) = -3.625.*sin(1.0.*a4 - a2 - a3 - a1); 
	 df(2,8,:) = zeros(size(a1d)); 
	 df(3,8,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(a2 + a3); 
	 df(4,8,:) = zeros(size(a2d)); 
	 df(5,8,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(a2 + a3) - 3.625.*sin(a3); 
	 df(6,8,:) = zeros(size(a3d)); 
	 df(7,8,:) = 3.625.*sin(1.0.*a4 - a2 - a3 - a1); 
	 df(8,8,:) = zeros(size(a4d)); 
	 df(9,8,:) = zeros(size(a5)); 
	 df(10,8,:) = zeros(size(a5d)); 
	 df(11,8,:) = zeros(size(a6)); 
	 df(12,8,:) = zeros(size(a6d)); 

end


function [f,df] = planar2r_nlfcf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 
	 f(1,:) = 3.625.*sin(a2 + a3) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4) + 3.625.*sin(a3) - 3.625.*sin(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5);
	 f(2,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4) + 3.625.*cos(a3) - 3.625.*cos(a1 + a2 + a3 - 1.0.*a4 - 1.0.*a5);

	 df(1,1,:) = - 3.625.*cos(1.0.*a4 - a2 - a3 - a1) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(2,1,:) = zeros(size(a1d)); 
	 df(3,1,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1); 
	 df(4,1,:) = zeros(size(a2d)); 
	 df(5,1,:) = 3.625.*cos(a2 + a3) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*cos(1.0.*a4 - a2 - a3 - a1) + 3.625.*cos(a3); 
	 df(6,1,:) = zeros(size(a3d)); 
	 df(7,1,:) = 3.625.*cos(1.0.*a4 - a2 - a3 - a1) + 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(8,1,:) = zeros(size(a4d)); 
	 df(9,1,:) = 3.625.*cos(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(10,1,:) = zeros(size(a5d)); 
	 df(11,1,:) = zeros(size(a6)); 
	 df(12,1,:) = zeros(size(a6d)); 
	 df(1,2,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(2,2,:) = zeros(size(a1d)); 
	 df(3,2,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*sin(a2 + a3); 
	 df(4,2,:) = zeros(size(a2d)); 
	 df(5,2,:) = - 3.625.*sin(1.0.*a4 - a2 - a3 - a1) - 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5) - 3.625.*sin(a2 + a3) - 3.625.*sin(a3); 
	 df(6,2,:) = zeros(size(a3d)); 
	 df(7,2,:) = 3.625.*sin(1.0.*a4 - a2 - a3 - a1) + 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(8,2,:) = zeros(size(a4d)); 
	 df(9,2,:) = 3.625.*sin(1.0.*a4 - a2 - a3 - a1 + 1.0.*a5); 
	 df(10,2,:) = zeros(size(a5d)); 
	 df(11,2,:) = zeros(size(a6)); 
	 df(12,2,:) = zeros(size(a6d)); 

end


function [f,df] = planar2r_nlgcf(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d)
 


end


end