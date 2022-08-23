clear
clc
%setting ramp parameters such as length,angle,and length of horizontal
%startup
w=1;
while w==1 %make a decison on whether to set values repeats until acceptable answer accepted
    decision=input('Would you like to set values yourself? yes or no','s');
    if strcmp(decision,'yes')==1 || strcmp(decision,' yes')==1 || strcmp(decision,'no')==1 || strcmp(decision,' no')==1
        w=0;
    else %answers not accpetable
        fprintf('\n Please put an expected response')
    end
end
[rampleth,ramplethc,Rampleth,rectleth,rectwith,rampangd,rampang,m,F_intial,a_p,mule]=setup(decision);
rampAng=pi/2-rampang;
%placing ramp on graph 
Rampxpoints=[0 ramplethc Rampleth];
Rampypoints=[rampleth*sin(rampang) 0 0];
corn1=[Rampleth-rectleth 0];
corn2=[corn1(1)+rectleth 0];
[corn3, corn4]=rectangle(corn1,corn2,rectwith);
%other necessary parametrs such as Time,intial speed e.t.c
T=0;
u_x1=0;
Max_height=0;
t=0.01;

avgv_x=0;
avgv_y=0;
avga_x=0;
avga_y=0;
pushtime=0.8*sqrt(2*(corn1(1)-ramplethc)/a_p);
%rand*0.4*sqrt(2*(corn1(1)-ramplethc)/a_p)+0.3*sqrt(2*(corn1(1)-ramplethc)/a_p);
g=9.81;
a=0;
Max_velocity=0;
% graph construction
figure(1)
h1=fill([corn1(1) corn2(1) corn3(1) corn4(1) corn1(1)],[corn1(2) corn2(2) corn3(2) corn4(2) corn1(2)], 'y-');
hold on
h2=plot(Rampxpoints, Rampypoints);
hold on
h3=plot(0, 0,'r*');
hold on
str=sprintf('pushtime=%.2f\nTime=%.2f\n x-averag speed=%.2f\n y-average speed=%.2f\nx-average acceleration=%.2f\n y-average acceleration=%.2f,',pushtime,T,avgv_x,avgv_y,avga_x,avga_y);
h4=text(0.6*Rampleth,0.8*Rampleth,str);%display parameters
axis([0 Rampleth 0 Rampleth])
axis off
hold off
%loop of operation
while a==0
    b=0;
    T=T+t;
    if   T<pushtime || (T-t<pushtime && T>pushtime) %movement when object pushed
        if T>=pushtime %ensure not over push time
         T=T-t;
         t=pushtime-T;
         if t<=0.0001  
             T=pushtime;
             t=0;
         end 
         T=T+t;
        end
        %movement parameters ofr corner1
        if mule*g>=a_p || abs(abs(mule*g)-abs(a_p))<=0.0001
            a_x1=0;  
        else
            a_x1=-a_p+mule*g;
        end
        a_y1=0;
        s_x1=u_x1*t+0.5*a_x1*t^2;
        s_y1=0;
        u_y1=0;
        %corner two parametrs
        [s_y2, s_x2, u_y2, u_x2, a_y2, a_x2]=parallelmove(s_y1, s_x1, u_y1, u_x1,a_y1, a_x1);
    elseif corn1(1)>ramplethc || (corn1(1)>=ramplethc && u_x1>0)%horizontal movement  withou being pushed
        if u_x1~=0%ensuring friction opposes motion
        a_x1=-sign(u_x1)*mule*m*g;
        elseif u_x1==0
            a_x1=0;
        end
     
          if sign(u_x1)~=sign(u_x1+a_x1*t) && u_x1~=0%ensuring to record movement in one direction
            t=abs(u_x1/a_x1);
          end
        [s_x1,Time]=meetingpointmovement(t,u_x1,a_x1,corn1,ramplethc,1);%seeting distance and time
        t=Time;
      
        %setting other parameters
        u_y1=0;
        a_y1=0;
        s_y1=0;
        [s_y2, s_x2, u_y2, u_x2, a_y2, a_x2]=parallelmove(s_y1, s_x1, u_y1, u_x1,a_y1, a_x1);
    elseif corn1(1)==ramplethc && u_x2<=0
        if u_x2<0%setting friction force
            a_x2=m*g*mule;
        elseif u_x2==0
            a_x2=0;
        end
        if sign(u_x2)~=sign(u_x2+a_x2*t) && u_x2~=0%ensuring to record movement in one direction
            t=abs(u_x2/a_x2);
        end
        %setting second corner distance
        [s_x2,Time]=meetingpointmovement(t,u_x2,a_x2,corn2,ramplethc,1);
        t=Time;
        a_y2=0;
        [s_x1,s_y1]=intialtransit(s_x2,corn1,corn2,rampang);%distance up ramp
        %setting neccesary movement parameters
        s=u_x2*t;
        [Distancex, Distancey]=intialtransit(s,corn1,corn2,rampang);
        u_x1=Distancex/t;
        u_y1=Distancey/t;
        s=0.5*a_x2*t^2;
        [Distancex, Distancey]=intialtransit(s,corn1,corn2,rampang);
        a_x1=2*Distancex/t^2;
        a_y1=2*Distancey/t^2;
    elseif (corn2(1)>ramplethc && corn1(1)<ramplethc) || (corn2(1)==ramplethc && u_x1>0)%transition stage(one corner on ramp other on horizontal face)
       [a_x2]=baseacceleration(corn1, corn2,cenpos,m,rampAng,u_x2,mule);%forces at bottom corner
        s_y2=0;
           if sign(u_x2)~=sign(u_x2+a_x2*t) && u_x2~=0%ensure friction opposes motion in coreect direction
            t=abs(u_x2/a_x2);
           end        
        
        if u_x2>0 || (u_x2==0 &&a_x2>0)%displacement calculated security system different for sign of velocity
            rampdistance=ramplethc+rectleth;
            [s_x2,Time]=meetingpointmovement(t,u_x2,a_x2,corn2,rampdistance,2);
            if corn2(1)+s_x2==ramplethc+rectleth || abs(corn2(1)+s_x2-(ramplethc+rectleth))<=0.0001
                s_x1=ramplethc-corn1(1);
                s_y1=-corn1(2);
                t=Time;
            else
                [s_x1,s_y1]=transitvector(s_x2,corn1,corn2,rampang);
            end
        elseif u_x2<0 || (u_x2==0 &&a_x2<0) 
            [s_x2,Time]=meetingpointmovement(t,u_x2,a_x2,corn2,ramplethc,1);
            [s_x1,s_y1]=transitvector(s_x2,corn1,corn2,rampang);
            t=Time;
        end
        %other neccessary parameters
        s=u_x2*t;
        [distancexu,distanceyu]=transitvector(s, corn1, corn2,rampang);
        s=0.5*a_x2*t^2;
        [distancexa,distanceya]=transitvector(s,corn1,corn2,rampang);
        u_y2=0;
        u_x1=distancexu/t;
        a_x1=2*distancexa/t^2;
        u_y1=distanceyu/t;
        a_y1=2*distanceya/t^2;  
    else %up or down ramp
        a_x1=m*g*sin(rampang)*cos(rampang);%acceleration components
        a_y1=-m*g*sin(rampang)*sin(rampang);    
        if sign(u_x1)~=sign(u_x1+a_x1*t) && u_x1~=0%ensure one direction for velocity
            t=abs(u_x1/a_x1);
        end
        %distance with security
      [s_x1,Time]=meetingpointmovement(t,u_x1,a_x1,corn2,ramplethc,2);
      t=Time;
      [s_y1,Time]=meetingpointmovement(t,u_y1,a_y1,corn2,0,1);
      [s_y2, s_x2, u_y2, u_x2, a_y2, a_x2]=parallelmove(s_y1, s_x1, u_y1, u_x1,a_y1, a_x1);   
    end
    %average velocities and acceleration for two corners
    avgv_x=0.25*(2*(u_x1+u_x2)+(a_x1+a_x2)*t);
    avgv_y=0.25*(2*(u_y1+u_y2)+(a_y1+a_y2)*t);
    avga_x=0.5*(a_x1+a_x2);
    avga_y=0.5*(a_y1+a_y2);
    %update velocities
    [Speeds]=vectors(u_x1,u_y1,u_x2,u_y2,a_x1,a_y1,a_x2,a_y2,t);
    u_x1=Speeds(1);
    u_y1=Speeds(2);
    u_x2=Speeds(3);
    u_y2=Speeds(4);
    %update corner positions
    corn1(1)=corn1(1)+s_x1;
    corn1(2)=corn1(2)+s_y1;
    corn2(1)=corn2(1)+s_x2;
    corn2(2)=corn2(2)+s_y2;
        %required max height and velocity reached
    if Max_height<corn1(2)
        Max_height=corn1(2);
    end

    if Max_velocity<sqrt((avgv_x)^2+(avgv_y)^2)
        Max_velocity=sqrt((avgv_x)^2+(avgv_y)^2);
    end
    %update graph
    [corn3, corn4]=rectangle(corn1,corn2,rectwith);
    cenpos=[(corn1(1)+corn2(1)+corn3(1)+corn4(1))/4 (corn1(2)+corn2(2)+corn3(2)+corn4(2))/4];
    str=sprintf('pushtime=%f\nTime=%f\n Max x-averag speed=%f\n Max y-average speed=%f\nMax x-average acceleration=%f\n Max y-average acceleration=%f,',pushtime,T,avgv_x,avgv_y,avga_x,avga_y);
    set(h3,'Xdata',cenpos(1),'Ydata',cenpos(2));
    set(h1, 'Xdata',[corn1(1) corn2(1) corn3(1) corn4(1) corn1(1)] ,'Ydata',[corn1(2) corn2(2) corn3(2) corn4(2) corn1(2)])
    set(h4,'String',str)
    pause(0.01)
    %stop condition
    if isnan(u_x1) || isnan(u_y1) || isnan(u_x2) || isnan(u_y2)
        a=1;
    elseif isnan(a_x1) || isnan(a_y1) || isnan(a_x2) || isnan(a_y2)
        a=1;
    elseif u_x1==Inf || u_y1==Inf || u_x2==Inf || u_y2==Inf 
        a=1;
    elseif a_x1==Inf || a_y1==Inf || a_x2==Inf || a_y2==Inf 
        a=1;
    elseif u_x1==0 && a_x1==0
        a=1; %no momemtum
    elseif corn2(1)>=Rampleth
        a=1;%beyond horizontal
    elseif corn1(2)>=Rampypoints(1)
        a=1;%corner over ramp
    elseif T>20
        a=1;
    end
    t=0.01;%update t if changed

end
str=sprintf('pushtime=%.2fsecs\nTime=%.2fsecs\nintial push Force %.2fN\nMass %.2fkg\nCoefficient of friction %.2f\nMax velocity=%.2fm/s\nMax height=%.2fm\nRamp height=%.2fm\nRamp angle=%.2f',pushtime,T,F_intial,m,mule,Max_velocity,Max_height,Rampypoints(1),rampangd);
set(h4,'String',str)%final parameters displayed
function [ramplength,ramplengthhorizontal,Ramplength,rectanglelength,rectanglewidth,rampangledegrees,rampangleradians,mass,Force,acceleration_push,coefficient_friction]=setup(decision)
if strcmp(decision,'yes')==1 || strcmp(decision,' yes')==1 %allow users to set parameters
    Continue=1;
    while Continue==1
        error1=0;
        error2=0; 
        ramplength=abs(input('\n Ramplength (m)='));
        rampangledegrees=abs(input('\n Ramp angle='));
        if  ramplength==Inf || isnan(ramplength) || ramplength<0.5% in the event an unacceptable value chosen
            fprintf('\n Ramp length undefined or too short')
            error1=1;
        end
        if rampangledegrees>=90 || rampangledegrees<10 || isnan(rampangledegrees) || rampangledegrees==Inf
            fprintf('\n Ramp angle undefienied or outside angle range (89 to 10)')
            error2=1;
        end
        if error1==0 && error2==0%loop continues until acceptable value given
            Continue=0;
        end
    end
    rampangleradians=rampangledegrees*pi/180;% other ramp parameters set based on user choices
    ramplengthhorizontal=ramplength*cos(rampangleradians);
    Ramplength=ramplength+ramplengthhorizontal;
    %rectangle parameters such as corner postions length and width
    while Continue==0 %rectangle parameters set with security
        String=sprintf('\n Rectangle length(Max=%.2fm)=',0.495*ramplengthhorizontal);
        rectanglelength=abs(input(String));
        String=sprintf('\n Rectangle width (Max=%.2fm)=',0.441*ramplengthhorizontal);
        rectanglewidth=abs(input(String));
        error1=0;
        error2=0;
        if rectanglelength<=0.05*ramplengthhorizontal || rectanglelength==Inf || isnan(rectanglelength) || rectanglelength>=0.5*ramplengthhorizontal
            fprintf('\n Rectangle lengthundefined or too large')
            error1=1;
        end
        if rectanglewidth==Inf || isnan(rectanglewidth) || rectanglewidth>=0.9*rectanglelength || rectanglewidth<=0.1*rectanglelength
            fprintf('\n Rectangle width undefined or too large')
            error2=1;
        end
        if error1==0 && error2==0
            Continue=1;
        end
    end
    while Continue==1
        error1=0;
        error2=0;
        mass=abs(input('\n Mass (kg)='));
        Force=abs(input('\n Force(N)='));
        if mass==0 || mass==Inf || isnan(mass) || mass<0.1
            fprintf('\n Mass is not defined')
            error1=1;
        end
        if Force==0 || Force==Inf || isnan(Force) || Force<10
            fprintf('\nForce not defined or too small')
            error2=1;
        end
        if error1==0 && error2==0
            Continue=0;
        end 
    end
    acceleration_push=Force/mass;
    while Continue==0
        error1=0;
        coefficient_friction=abs(input('\nCoefficient of friction='));
        if coefficient_friction>=1 || coefficient_friction<=0.01 || isnan(coefficient_friction) 
            fprintf('\n Coefficient of friction out of range')
            error1=1;
        end
        if error1==0
            Continue=1;
        end
    end
elseif strcmp(decision,'no')==1 || strcmp(decision,' no')==1
    ramplength=15*rand+45;
    rampangledegrees=round(70*rand+10);
    rampangleradians=rampangledegrees*pi/180;
    ramplengthhorizontal=ramplength*cos(rampangleradians);
    Ramplength=ramplength+ramplengthhorizontal;
    %rectangle parameters such as corner postions length and width
    rectanglelength=0.15*rand*(ramplength)+0.1*(ramplength);
    rectanglewidth=0.4*rand*rectanglelength+0.1*rectanglelength;
    mass=round(rand*(99)+0.5);
    Force=round(rand*99000+50);
    acceleration_push=Force/mass;
    coefficient_friction=0.9*rand+0.01;
end
end
function [point3, point4]=rectangle(point1,point2,rectanglewidth)
rectm=(point2(2)-point1(2))/(point2(1)-point1(1));
rectangleangle=atan(rectm);
point3=[point2(1)-rectanglewidth*cos(pi/2-rectangleangle) point2(2)+rectanglewidth*sin(pi/2-rectangleangle)];
point4=[point1(1)-rectanglewidth*cos(pi/2-rectangleangle) point1(2)+rectanglewidth*sin(pi/2-rectangleangle)];
end
function [Distance,Time]=meetingpointmovement(time,speed,acceleration,point1,rampdis,version)
Distance=speed*time+0.5*acceleration*time^2;
      if point1(1)+Distance<=rampdis && acceleration~=0 && version==1
            Distance=-point1(1)+rampdis;
            Time=(-speed-sqrt(speed^2-4*0.5*acceleration*(-Distance)))/(2*0.5*acceleration);
      elseif point1(1)+Distance>=rampdis && acceleration~=0 && version==2
          Distance=-point1(1)+rampdis;
          Time=(-speed+sqrt(speed^2-4*0.5*acceleration*(-Distance)))/(2*0.5*acceleration);
      else
          Time=time;
      end
end
function [distancey2, distancex2, speedy2, speedx2, accelerationy2, accelerationx2]=parallelmove(distancey1, distancex1, speedy1, speedx1,accelerationy1, accelerationx1)
%all corresponding vectors same vaule
distancey2=distancey1;
distancex2=distancex1;
speedy2=speedy1;
speedx2=speedx1;
accelerationy2=accelerationy1;
accelerationx2=accelerationx1;
end
function [acceleration]=baseacceleration(point1, point2, centre,mass,angle,speed,coeeficient_friction)
%neccessary parameters obtained from freebody analysis
displacement_point1_to_centre=[(centre(1)-point1(1)) (centre(2)-point1(2))];
displacement_point1_to_point2=[(point2(1)-point1(1)) (point2(2)-point1(2))];
Normalreactionforcecomponents_1=[cos(angle) sin(angle)];
Fg=[0 -mass*9.81];
Noramlreactionforce_1=Fg(2)*(displacement_point1_to_point2(1)-displacement_point1_to_centre(1))/(Normalreactionforcecomponents_1(1)*displacement_point1_to_point2(2)-Normalreactionforcecomponents_1(2)*displacement_point1_to_point2(1));
Slideforce_2=-Noramlreactionforce_1*Normalreactionforcecomponents_1(1);
Normalreactionforce_2=-Noramlreactionforce_1*Normalreactionforcecomponents_1(2)-Fg(2);
%caluculate acceleration
if abs(speed)<=0.0001 %initial speed considered zero
    if abs(abs(coeeficient_friction*Normalreactionforce_2/mass)-abs(Slideforce_2/mass))<=0.0001 %frcition force and center force equal
        acceleration=0;
    elseif abs(coeeficient_friction*Normalreactionforce_2/mass)<abs(Slideforce_2/mass) %postion unstable resulting in slide
        acceleration=-Slideforce_2/mass-coeeficient_friction*Normalreactionforce_2/mass;
    else %friction higher
        acceleration=0;
    end
elseif speed>=0 %friction opposes motion
    acceleration=-Slideforce_2/mass-coeeficient_friction*Normalreactionforce_2/mass; 
elseif speed<=0
    acceleration=-Slideforce_2/mass+coeeficient_friction*Normalreactionforce_2/mass;
end
     
end
function [distancex, distancey] = transitvector(s, point1, point2, angle)%gets movement when one corner at base and one at ramp
%neccessary parameters obtained from trig analysis
y=point1(2)/sin(angle);
d=point2(1)-point1(1);
W=d-y*cos(angle)+s;
distancex=-(sqrt(y^2*(sin(angle))^2+d^2-W^2*(sin(angle))^2)-W*cos(angle)-y)*cos(angle);
distancey=(sqrt(y^2*(sin(angle))^2+d^2-W^2*(sin(angle))^2)-W*cos(angle)-y)*sin(angle);
end
function [Distancex, Distancey]=intialtransit(s,point1,point2,angle)%from two corners at base to one on ramp
h=point2(1)- point1(1);
Distancex=-(sqrt(h^2+(cos(angle)*(h+s))^2-(h+s)^2)-(h+s)*cos(angle))*cos(angle);
Distancey=(sqrt(h^2+(cos(angle)*(h+s))^2-(h+s)^2)-(h+s)*cos(angle))*sin(angle);
end
function [Vector]=vectors(speedx1,speedy1,speedx2,speedy2,acclerationx1,accelerationy1,accelerationx2,accelerationy2,time)%update intial speed
Vector=zeros(1,4);
speedmatrix=[speedx1,speedy1,speedx2,speedy2];
accelerationmatrix=[acclerationx1,accelerationy1,accelerationx2,accelerationy2];
for k=1:1:4 %updates each speed
   Vector(k)=speedmatrix(k)+accelerationmatrix(k)*time;
   if abs(Vector(k))<=0.0001%errror margin
       Vector(k)=0;
   end
end
end
    


