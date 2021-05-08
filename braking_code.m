%%Before running the code import data of uddscol.txt
%% Output type coloumn Vector, Select Column and import. The speed column and name the varaible as SpeedMph

%%torque cap and speed cap left , cant keep speed cap as it is related to
%%distance of the breake pedal travelled. Only max braking force cap.
%%Convert SpeedMph to m/s
SpeedMPH = 1.6*SpeedMPH; %%miles perhour to kmph
SpeedMph=SpeedMPH;
%%SpeedMph=0.409*SpeedMph;
SpeedMph=0.2778*SpeedMph; 
%Vehicle constants
wheel_radius=0.318; %%Tesla model S https://www.errolstyres.co.za/content/tyre-overall-rolling-diameter
grade_deg=0; %%maybe grade change
roll_coef=0.01;  %%Car tyre on smooth tarmac road
area=2.4;  %%from tesla S
aero_coeff=0.24;  %%from tesla S
inertia_coeff=0.04; %% assumption
vehicle_mass=2200;  %%from tesla S mass(car+driver)
drive_eff=0.88;   %% from tesla S 
wheel_base=2.959;
la=1.4795;
lb=1.4795; 
cg_height=0.4572; %%http://www.roperld.com/science/TeslaModelS.htm


%Initialization for variables 
torque=zeros(length(SpeedMph),1);
roll_ress=zeros(length(SpeedMph),1);
aero_ress=zeros(length(SpeedMph),1);
acc_ress=zeros(length(SpeedMph),1);
grade_ress=zeros(length(SpeedMph),1);
braking_force=zeros(length(SpeedMph),2);

for i = 2 : length(SpeedMph)
    if SpeedMph(i)-SpeedMph(i-1) >= 0
        braking=1;
    else 
        braking=-1;
    end 
    roll_ress(i)= calc_rollingres(vehicle_mass,roll_coef,grade_deg);
    aero_ress(i)= calc_aerodrag(SpeedMph(i),area,aero_coeff);
    acc_ress(i)=calc_accress(SpeedMph(i),SpeedMph(i-1),vehicle_mass,inertia_coeff);
    grade_ress(i)= calc_grade(vehicle_mass,grade_deg);
    if braking ==1 
        torque(i)=calc_torque(roll_ress(i),aero_ress(i),acc_ress(i),grade_ress(i),wheel_radius,drive_eff);
    else 
        braking_force(i,1)=calc_fbraking(vehicle_mass,SpeedMph(i),SpeedMph(i-1),wheel_base,lb,cg_height);
        braking_force(i,2)=calc_rbraking(vehicle_mass,SpeedMph(i),SpeedMph(i-1),wheel_base,la,cg_height);
        torque(i)=wheel_radius*braking_force(i,1);
    end 
    
end

%%subplot(2,1,1)
%%plot(SpeedMPH,braking_force(:,1))
%%subplot(2,1,2)
%%plot(SpeedMPH,braking_force(:,2))
plot(torque)

function roll_ress= calc_rollingres(mass,roll_coef, grade_deg)
roll_ress= mass*9.81*roll_coef*cosd(grade_deg);
end 

function aero_ress= calc_aerodrag(next_speed,area,aero_coeff)
vehiclespeed= next_speed^2;
aero_ress = 0.5*1.27*area*aero_coeff*vehiclespeed;
end

function acc_ress = calc_accress(next_speed, prev_speed, mass, inertia_coeff)
 acc = (next_speed-prev_speed)/1;
 acc_ress = (mass + mass*inertia_coeff)*acc;
end

function grade_ress = calc_grade(mass,grade_deg)
grade_ress = mass*9.81*sind(grade_deg);
end 

function torque = calc_torque(roll_ress,aero_ress,acc_ress,grade_ress,wheel_radius,drive_eff)
   torque = (wheel_radius*(roll_ress+aero_ress+acc_ress+grade_ress))/9.73;
   torque = torque/drive_eff;
end 

function rbrakingf =calc_rbraking(mass,next_speed,prev_speed,wheel_base,la,cg_height)
decel=next_speed-prev_speed/1;
rbrakingf=(decel*mass/wheel_base)*(la-(cg_height*decel/9.81))/9.73;
end

function fbrakingf =calc_fbraking(mass,next_speed,prev_speed,wheel_base,lb,cg_height)
decel=next_speed-prev_speed/1;
fbrakingf=(decel*mass/wheel_base)*(lb+(cg_height*decel/9.81))/9.73;
end

