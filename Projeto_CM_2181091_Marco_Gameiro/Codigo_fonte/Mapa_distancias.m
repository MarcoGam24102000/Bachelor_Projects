
clear all, 
close all
smithchart
f= 1e9; %Hz
c=3e8 %m/s
lambda=c/f;%m

lim = 500;
     
Gtx=0 , Grx=0 % dB
Ptx=0 %dBm

for i=-lim:1:lim
    for j=-lim:1:lim
        distance_map(i+lim+1,j+lim+1)=sqrt(i^2+j^2);
    end
end

L=20*log10(4*pi.*distance_map/lambda);

Prx_dBm=Ptx+Gtx+Grx-L;
Prx_dBm(lim+1,lim+1)=0; % at the location of the base station

%plot(d,Prx_dBm)
meshc(-lim:1:lim,-lim:1:lim,Prx_dBm)
xlabel('Distance x')
ylabel('Distance y')
zlabel('P_r_x [dBm]')
colorbar

