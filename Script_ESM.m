clear
close all
% load('lat.mat')
% load('lon.mat')


Paux=220; % kW

m = 500; % t
beta=0.04; 

me=m*(1+beta); % equivalent mass

a_max = 0.7; % starting acceleration

f_ad_max=0.33; % dry rail considered

Nwmot=16; % motorized wheelsets
Ntot=32; % overall wheelsets

Ft_start=me*a_max; % kN


nt=0.9; %Tractive effiency 
nb=0.9;% Braking efficiency


Dtot=4870; % m
time=4000; % s


Bel=zeros(1,time); %Percentage of regenerative braking

a1=0.65;
b=55; % parameters for curvature resistance calculation

t = linspace(1,time,time); %time array
s=zeros(1,time); %space
v=zeros(1,time); % speed in m/s
vkm=zeros(1,time); % speed in km/h
a=zeros(1,time); %acceleration
vmax=zeros(1,time); %max speed allowed
vmax(1)=90/3.6;
limit=0; % parameter used to control speed

Ri=zeros(1,time); % grade resistance
Ro=zeros(1,time); % resistance to forward motion
Rc=zeros(1,time); % curvature resistance
R=zeros(1,time); % total resistance
Ft=ones(1,time); % tractive effort

dbr=zeros(1,time); % braking distance
Brake=zeros(1,time); % array to evidence braking points and save them
fr=zeros(1,time); % array to evidence free running points and save them
Fad=zeros(1,time); % adhesion force
Slip=zeros(1,time); % array to evidence slip condition and save it
B=zeros(1,time); % Dtot-s-dbr, Dtot is the target distance we want to stop
Braking_request=zeros(1,time); % quantity used to calculate Braking Percentage 

Vline=3000*ones(1,time); % Voltage line
Pel=zeros(1,time); % Electric power supply
I=zeros(1,time); % Current
Eel=zeros(1,time); % Energy consumed

curv=zeros(1,time); % curvature array
pend=zeros(1,time); % slope array

% Starting quantities
Ft(1)=Ft_start; % kN
R(1)=(1+0.0125*(0/10)^2)*m*9.81; % N
a(1)=(Ft(1)*1000-R(1))/(me*1000); % m/s^2
Fad(1)=(f_ad_max/(1+0.011*vkm(1)))*m*9.81*Nwmot/Ntot; %kN
Pel(1)=Paux; %kW
I(1)=(Paux*1000)/Vline(1); % A
Eel(1)=Paux/3600; % kWh

D=[4870, 133035, 147878]; % distances corresponding to stops (Torino Porta Susa, Milano Rho Fiera and Milano Centrale)

Tunnel=false; % variable to say if we are in a tunnel or not (see function below)
i=2; % second at which train leaves
A=zeros(3,1); % matrix to save distances between theoretical stop points are obtained one

x = [10,70,100,14, 20, 40];
y=[60,20,14,62, 56, 35];

pp=spline(x,y); % curve for braking energy breaking as a function of braking request percentage

for j=1:3
    
    while true
        
        
        v(i)=a(i-1)*(t(i)-t(i-1))+v(i-1);
        s(i)=(v(i)+v(i-1))/2*(t(i)-t(i-1))+s(i-1);
        vkm(i)=v(i)*3.6;
        
        
        pend(i)=Pendenza(s(i));
        Ri(i)=m*9.81*pend(i); % N     
        
        [vmax(i), limit]=SpeedLimit(s(i));
        
        
        curv(i)=Curvatura(s(i));
        
        if curv(i)<350
            
            b=65;
            
        end
        
        Rc(i)=(a1/(curv(i)-b))*m*9.81*1000; % N
        
        
        Tunnel = Tn(s(i));
        
        if Tunnel == false
            
            Ro(i)=(1+0.0125*(vkm(i)/10)^2)*m*9.81; % N
        else
            
            Ro(i)=(1+0.0207*(vkm(i)/10)^2)*m*9.81; % N
        end
        
        
        R(i)=Ro(i)+0.001*m*9.81 + Ri(i) + Rc(i);
        brs=Brake(i-1); % save the variable Brake(i-1)
      
        if vmax(i)==300/3.6 % dbr to set speed at 100 km/h 
            

             dbr(i)=((100/3.6)^2-v(i)^2)/(2*-0.405);  % -0.45 m/s^2 is considered to be an average acceleceration fro the train between a speed of 300  km/h and 100 km/h
             Dtot=130000; % end of 300 km/h speed limit: new limit 100 km/h


        
        else % dbr to stop at next station

            dbr(i)=v(i)^2/(2*0.5); % in this case -0.5 m/s^2 is considered as average acceleration
            Dtot=D(j);
            Brake(i-1)=false; % set in order to avoid the control on braking, in this case only the variable B below is necessary to control the variable Brake 

            
        end
        
        B(i)=Dtot-s(i)-dbr(i);
        
        if Brake(i-1)==false && B(i)>=0 
            %Brake(i-1)==false is another condition emplyed in the control of braking at the end of high speed sector when speed limit become 100 km/h.
            %Due to the not constant value of acceleration during braking phase and the use of an average value of acceleration for the computation of dbr, 
            %B can oscillate from positive to negative when close to zero in the high speed sector when approaching the speed limit of 100 km/h. 
            %To avoid this oscillation even for the Brake variable, the condtion Brake(i-1)==false is added in order to set brake to true even if B become for 
            %a certain instant positive  before reaching the target point and the train at the instant before was braking.
 
             Brake(i)=false;
       
        else
            
            Brake(i)=true; 
            
            if dbr(i)>2100
                
            Braking_request(i)=20e+04/dbr(i); % computation of braking request percentage from dbr.
            
            else
                
            Braking_request(i)=100;
            
            end
           
        end
                       
        Vline(i) = Alimentazione(s(i));
             
        [Pmax, abr]=Potenza(Vline(i),Brake(i));
        Brake(i-1)=brs; % set Brake(i-1) to its original value
             
        if Brake(i) == false 
        
            vb = Pmax/Ft_start;
            
            if v(i)<vb && v(i)<vmax(i)-limit*vmax(i)
                
                Ft(i)=Ft_start; %kN
                
            elseif v(i)>vb && v(i)<vmax(i)-limit*vmax(i)
                
                Ft(i)=Pmax/v(i); %kN
                
            elseif v(i)>vmax(i)-limit*vmax(i) && v(i)<vmax(i) 
                
                Ft(i)=R(i)/1000;
                fr(i)=true;
                
            else
                
                Brake(i)=true;
                [Pmax, abr]=Potenza(Vline(i),Brake(i));
                vb=Pmax/(me*abs(abr));

            if v(i)>vb
                
                Ft(i)=-Pmax/v(i);
            
            else
                
                Ft(i)=abr*me;
            
            end
                
            end      
           
        else
            
            vb=Pmax/(me*abs(abr));

            if v(i)>vb
                
                Ft(i)=-Pmax/v(i);
            
            else
                
                Ft(i)=abr*me;
            
            end
                      
        end
        
        if Vline(i)==0 && Ft(i)>0 % EBPs
            
            Ft(i)=0;
        end
        
        a(i)=(Ft(i)*1000-R(i))/(me*1000); %m/s^2
        
        if Ft(i)>=0
            
            Fad(i)=(f_ad_max/(1+0.011*vkm(i)))*m*9.81*Nwmot/Ntot; %kN
        else
            
             Fad(i)=(f_ad_max/(1+0.011*vkm(i)))*m*9.81; %kN
        end
        
        if abs(Ft(i))>Fad(i)
            
            Slip(i)=true;
            
        end       
        
        if Ft(i)>=0 
            
            Pel(i)=(Ft(i)*v(i))/nt + Paux; %kW
            
        elseif Ft(i)<0 
            
            Bel(i)=ppval(pp,Braking_request(i)); %computation of breaking energy percentage from braking request
            
            Pel(i)=Ft(i)*v(i)*nb*Bel(i)/100 + Paux; %kW
        else
            
            Pel(i)=0; % for EBPs
            
        end
        
        if Vline(i)~=0
            
            I(i)=Pel(i)*1000/Vline(i);
        else
            
            I(i)=0; %for EBPs
        end
        
        if Vline(i)==0
            
            Eel(i)=Eel(i-1);  %for EBPs
        else
            
            Eel(i)=(Pel(i-1)*(t(i)-t(i-1)))/3600+Eel(i-1);
        
        end
        
        
        if v(i)<0.1
            
            A(j)=s(i)-D(j);
            fprintf('Train stopped at %d at %d:%d minutes.\n',s(i),floor(t(i)/60), mod(t(i),60));
            break
                 
        end
        
        i=i+1;
        
    end

if j == 3
    break
end

i=i+1;
v(i:i+120)=0;
a(i:i+120)=0;
s(i:i+120)=s(i-1);
R(i:i+120)=R(1);
Fad(i:i+120)=Fad(1);
Eel(i:i+120)=Eel(i-1);
Pel(i:i+120)=Paux;
I(i:i+120)=(Paux*1000)/Vline(i);
Ft(i:i+120)=0;

i=i+121;
Ft(i)=Ft_start;
v(i)=0;
a(i)=a(1);
s(i)=s(i-1);
R(i)=R(1);
Fad(i)=Fad(1);
Eel(i)=Eel(i-1);
Pel(i)=Paux;
I(i)=(Paux*1000)/Vline(i);

i = i+1; 

end

t=t(1:i);
s=s(1:i);
v=v(1:i);
vkm=vkm(1:i);
a=a(1:i);
Ft=Ft(1:i);
Fad=Fad(1:i);
pend=pend(1:i);
Pel=Pel(1:i);
Eel=Eel(1:i);
I=I(1:i);
Brake=Brake(1:i);
Slip=Slip(1:i);
Rc=Rc(1:i);
Ro=Ro(1:i);
Ri=Ri(1:i);
R=R(1:i);
vmax=vmax(1:i);
B=B(1:i);
Vline=Vline(1:i);



s=s./1000; % conversion from m to km
t=t./60; %conversion seconds to minutes

b=find(Brake); % array for braking phase
f=find(fr); % array for free running phase
tr=find(~Vline); % array for EBPs

% PS=[45.07169875,7.665903998];
% RHO=[45.52126027, 9.088462783];

% h=figure;
% geoplot(lat,lon)
% hold on
% geoplot(lat(1),lon(1), '.', 'Markersize', 10)
% hold on
% geoplot(PS(1),PS(2), '.', 'Markersize', 10)
% hold on
% geoplot(RHO(1),RHO(2), '.', 'Markersize', 10)
% hold on
% geoplot(lat(end),lon(end), '.', 'Markersize', 10)
% ax = gca;  ax.FontSize = 13;%saveas(h,'Coordinates.png')

figure;
plot(t,s,'LineWidth', 1.1)
xlabel('time [minutes]')
ylabel('distance [km]')
hold on
plot(t(b),s(b), '.','Markersize', 5)
hold on
plot(t(tr),s(tr), '.','Markersize', 5)
legend('Traction','Braking', 'EBP')
title('Space Profile')
ax = gca;  ax.FontSize = 13;

figure;
plot(t,vkm,'LineWidth', 1.1)
xlabel('time [minutes]')
ylabel('v [km/h]')
hold on
plot(t(b),vkm(b), '.','Markersize', 5)
hold on
plot(t(tr),vkm(tr), '.','Markersize', 5)
legend('Traction','Braking', 'EBP')
legend show
title('Speed Profile')
ax = gca;  ax.FontSize = 13;

figure;
plot(t,a, 'LineWidth', 1.1, 'DisplayName', 'Traction')
hold on
plot(t(b),a(b), '.','Markersize', 5, 'DisplayName', 'Braking');
hold on
plot(t(tr),a(tr), '.','Markersize', 5, 'DisplayName', 'EBP')
xlabel('time [minutes]')
ylabel('a [m/s^2]')
legend show
title('Acceleration')
ax = gca;  ax.FontSize = 13;


figure;
plot(t,Pel, 'LineWidth', 1.1, 'DisplayName', 'Traction')
hold on
plot(t(b),Pel(b), '.','Markersize', 5, 'DisplayName', 'Braking');
hold on
plot(t(tr),Pel(tr), '.','Markersize', 5,'LineWidth', 1.1, 'DisplayName', 'EBP')
xlabel('time [minutes]')
ylabel('P_e [kW]')
legend show
title('Supply System Electric Power')
ax = gca;  ax.FontSize = 13;

figure;
plot(t,Eel,'LineWidth', 1.1, 'DisplayName', 'Energy')
hold on
plot(t(b),Eel(b), '.','Markersize', 5, 'DisplayName', 'Braking')
hold on
plot(t(tr),Eel(tr), '.','Markersize', 5, 'color', [0.9290 0.6940 0.1250], 'DisplayName', 'EBP')
xlabel('time [minutes]')
ylabel('E_e [kWh]')
title('Energy')
legend show
ax = gca;  ax.FontSize = 13;

figure;
plot(t,I,'LineWidth', 1.1, 'DisplayName', 'Current')
hold on
plot(t(b),I(b), '.','Markersize', 5, 'DisplayName', 'Braking');
hold on
plot(t(tr),I(tr), '.','Markersize', 5, 'DisplayName', 'EBP')
xlabel('time [minutes]')
ylabel('I [A]')
legend show
title('Current')
ax = gca;  ax.FontSize = 13;

sp=find(Ft>0);
br=find(Ft<0);

figure;
plot(vkm(sp),Fad(sp),  '.','Markersize', 5)
hold on
plot(vkm(sp),abs(Ft(sp)), '.','Markersize', 5)
xlabel('v [km/h]')
ylabel('[kN]')
legend('F_a','F_t')
title('Tractive Effort')
ax = gca;  ax.FontSize = 13;

figure;
plot(vkm(br),Fad(br),  '.','Markersize', 5)
hold on
plot(vkm(br),abs(Ft(br)), '.','Markersize', 5)
xlabel('v [km/h]')
ylabel('[kN]')
legend('F_a','|F_t|')
title('Braking Effort')
ax = gca;  ax.FontSize = 13;

figure;
plot(Braking_request(b),  dbr(b), '.', 'Markersize', 5)
xlabel('Braking request percentage [-]')
ylabel('Braking distance [m]')
ax = gca;  ax.FontSize = 13;

h=figure;
plot(Braking_request(b), Bel(b), '.', 'Markersize', 5)
xlabel('Braking request percentage[-]')
ylabel('Braking energy percentage [-]')
ax = gca;  ax.FontSize = 13;

figure;
xlabel('distance [km]', 'Fontsize', 13)
xline(0, '-',{'90 km/h'}, 'LabelHorizontalAlignment','left', 'Fontsize', 13)
hold on
xline(0, '-',{'Torino P. N.'}, 'LabelHorizontalAlignment','left', 'LabelVerticalAlignment', 'bottom', 'Fontsize', 13)
hold on
xline(4.870, '-',{'120 km/h'},'LabelHorizontalAlignment', 'left', 'Fontsize', 13)
hold on
xline(15.325, '-',{'300 km/h'},'LabelHorizontalAlignment', 'left', 'Fontsize', 13)
hold on
xline(130.000, '-',{'100 km/h'},'LabelHorizontalAlignment', 'left', 'Fontsize', 13)
hold on
xline(133.035, '-',{'70 km/h'},'LabelHorizontalAlignment', 'left', 'Fontsize', 13)
hold on
xline(133.035, '-',{'Milano Rho Fiera'},'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom', 'Fontsize', 13)
hold on
xline(4.870, '-',{'Torino P.S.'},'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom', 'Fontsize', 13)
hold on
xline(147.878, '-',{'Milano Centrale'},'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom', 'Fontsize', 13)
set(gca,'YTickLabel',[])
xticks(0:10:150)
ax = gca;  ax.FontSize = 13;

figure
plot(t,Ro/1000,'LineWidth', 1.1, 'DisplayName', 'Resistance to forward motion')
hold on
plot(t,Ri/1000, 'LineWidth', 1.1, 'DisplayName', 'Grade Resistance')
hold on
plot(t,Rc/1000,  'LineWidth', 1.1,'DisplayName', 'Curvature Resistance')
hold on
plot(t,R/1000, 'LineWidth', 1.1, 'DisplayName', 'Total Resistance')
legend show
xlabel('time [minutes]')
ylabel('[kN]')
title('Resistances')
ax.FontSize = 13;
ax = gca;  ax.FontSize = 13;

figure
plot(t,Ft,'LineWidth', 1.1, 'DisplayName', 'Force')
hold on
plot(t,R/1000, 'LineWidth', 1.1, 'DisplayName', 'Total Resistance')
xlabel('time [minutes]')
ylabel('[kN]')
title('Force and overall resistance')
legend show
ax.FontSize = 13;
ax = gca;  ax.FontSize = 13;


function [Pmax, abr]=Potenza(Vline,Brake)

if Vline == 3000
    
    abr=-0.48;
    if Brake == false
        
        Pmax=6900;
    else
        
        Pmax=5650;
        
    end
else
    
    abr=-0.58;
    if Brake == false
        
        Pmax=9800;
    else
        
        Pmax=10500;
    end
end
end
    
function Vline = Alimentazione(s)

if s < 13240-1600 || s > 130000+1600
   
    Vline=3000;
    
elseif s > 13240 && s < 130000
    

    Vline=25000;
else

    Vline=0;
end

end

function Tunnel = Tn(s)

if s < 1570 || s > 9030
    
    Tunnel=false;
else
    Tunnel=true;
end
end

function [vmax, limit] = SpeedLimit(s)

if s > 15240 && s < 130000
    
    vmax=300/3.6;
    limit=0.01;
    
elseif s < 4870
    
    vmax = 90/3.6;
    limit=0.5;

elseif s>=4870 && s<=15240
    
    vmax=120/3.6;
    limit=0.1;
    
elseif s >= 130000 && s < 133035
    
    vmax = 100/3.6;
    limit=0.15;

else
    
    vmax=70/3.6;
    limit=0.1;
    

end
end

function pend=Pendenza(s)

if s < 10e+3
    
    pend=-0.9;
    
elseif s>=10e+3 && s<20e+03
    
    pend=-2;
    
elseif s>=20e+3 && s<30e+03
    
   pend=-0.3;

elseif s>=30e+3 && s<40e+03
    
   pend=0.6;
   
elseif s>=40e+3 && s<50e+03
    
   pend=0.1;
   
elseif s>=50e+3 && s<60e+03
    
   pend=-0.3;
   
elseif s>=60e+3 && s<70e+03
    
   pend=-3.8;
   
elseif s>=70e+3 && s<80e+03
    
   pend=-0.4;
   
elseif s>=80e+3 && s<90e+03
    
   pend=-0.2;
   
elseif s>=90e+3 && s<100e+03
    
   pend=-1.2;
   
elseif s>=100e+3 && s<110e+03
    
   pend=-3.2;
   
elseif s>=110e+3 && s<120e+03
    
   pend=3.4;

elseif s>=120e+3 && s<130e+03
    
   pend=-0.3;
   
elseif s>=130e+3 && s<140e+03
    
   pend=-1.5;
   
elseif s>=140e+3 
    
   pend=-1.3;
   
end
end

function curv=Curvatura(s)

if  s>1070 && s < 2860 
    
    curv=770;
    
elseif s>9930 && s<10800
    
    curv=830;
    
elseif s>13300 && s<14600
    
    curv=1330;
    
elseif s>35000 && s<41000
    
    curv=8530;
    
elseif s>59400 && s<67200
    
    curv=9040;
    
elseif s>93200 && s<95700
    
    curv=5750;
    
elseif s>111000 && s<114000
    
    curv=4240;
    
elseif s>130000 && s<133000
    
    curv=1260;
    
elseif s>=133000 && s<135000
    
    curv=1340;
    
elseif s>140000 && s<142000
    
    curv=540;
    
elseif s>=141000 && s<142000
    
    curv=541;

elseif s>145000 && s<147000
    
    curv=510;
else
    
   curv=inf;
end
end