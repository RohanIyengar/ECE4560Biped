function och = planar2r()

och.icf = @planar2r_icf; 
och.tcf = @planar2r_tcf; 
och.fcf = @planar2r_fcf; 

och.nlicf = @planar2r_nlicf; 
och.nltcf = @planar2r_nltcf; 
och.nlfcf = @planar2r_nlfcf; 
och.nlgcf = @planar2r_nlgcf; 



function [f,df] = planar2r_icf(a1,a1d,a2,a2d,a3,a3d)
 


end


function [f,df] = planar2r_tcf(a1,a1d,a2,a2d,a3,a3d)
 
	 f(1,:) = a1d.^2+a2d.^2;

	 df(1,:)= zeros(size(a1)); 
	 df(2,:)= 2.*a1d; 
	 df(3,:)= zeros(size(a2)); 
	 df(4,:)= 2.*a2d; 
	 df(5,:)= zeros(size(a3)); 
	 df(6,:)= zeros(size(a3d)); 

end


function [f,df] = planar2r_fcf(a1,a1d,a2,a2d,a3,a3d)
 


end


function [f,df] = planar2r_nlicf(a1,a1d,a2,a2d,a3,a3d)
 
	 f(1,:) = 3.625.*sin(a1) + 3.625.*cos(a1).*sin(a2) + 3.625.*cos(a2).*sin(a1);
	 f(2,:) = 3.625.*sin(a1).*sin(a2) - 3.625.*cos(a1) - 3.625.*cos(a1).*cos(a2) - 2.0;

	 df(1,1,:) = 3.625.*cos(a1) - 3.625.*sin(a1).*sin(a2) + 3.625.*cos(a1).*cos(a2); 
	 df(2,1,:) = zeros(size(a1d)); 
	 df(3,1,:) = 3.625.*cos(a1).*cos(a2) - 3.625.*sin(a1).*sin(a2); 
	 df(4,1,:) = zeros(size(a2d)); 
	 df(5,1,:) = zeros(size(a3)); 
	 df(6,1,:) = zeros(size(a3d)); 
	 df(1,2,:) = 3.625.*sin(a1) + 3.625.*cos(a1).*sin(a2) + 3.625.*cos(a2).*sin(a1); 
	 df(2,2,:) = zeros(size(a1d)); 
	 df(3,2,:) = 3.625.*cos(a1).*sin(a2) + 3.625.*cos(a2).*sin(a1); 
	 df(4,2,:) = zeros(size(a2d)); 
	 df(5,2,:) = zeros(size(a3)); 
	 df(6,2,:) = zeros(size(a3d)); 

end


function [f,df] = planar2r_nltcf(a1,a1d,a2,a2d,a3,a3d)
 


end


function [f,df] = planar2r_nlfcf(a1,a1d,a2,a2d,a3,a3d)
 
	 f(1,:) = 3.625.*sin(a1) + 3.625.*cos(a1).*sin(a2) + 3.625.*cos(a2).*sin(a1);
	 f(2,:) = 3.625.*sin(a1).*sin(a2) - 3.625.*cos(a1) - 3.625.*cos(a1).*cos(a2) - 2.0;

	 df(1,1,:) = 3.625.*cos(a1) - 3.625.*sin(a1).*sin(a2) + 3.625.*cos(a1).*cos(a2); 
	 df(2,1,:) = zeros(size(a1d)); 
	 df(3,1,:) = 3.625.*cos(a1).*cos(a2) - 3.625.*sin(a1).*sin(a2); 
	 df(4,1,:) = zeros(size(a2d)); 
	 df(5,1,:) = zeros(size(a3)); 
	 df(6,1,:) = zeros(size(a3d)); 
	 df(1,2,:) = 3.625.*sin(a1) + 3.625.*cos(a1).*sin(a2) + 3.625.*cos(a2).*sin(a1); 
	 df(2,2,:) = zeros(size(a1d)); 
	 df(3,2,:) = 3.625.*cos(a1).*sin(a2) + 3.625.*cos(a2).*sin(a1); 
	 df(4,2,:) = zeros(size(a2d)); 
	 df(5,2,:) = zeros(size(a3)); 
	 df(6,2,:) = zeros(size(a3d)); 

end


function [f,df] = planar2r_nlgcf(a1,a1d,a2,a2d,a3,a3d)
 


end


end