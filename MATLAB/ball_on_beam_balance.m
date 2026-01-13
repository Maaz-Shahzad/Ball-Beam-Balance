clc
clear all
close all

pkg load control
pkg load signal
pkg load symbolic

###################################################################################
## SYSTEM MODELLING AND PROPERTIES
###################################################################################
A = [0 1;0 0];
B = [0; 9.81];
C = [1 0];
D = 0;

sys_ss = ss(A,B,C,D);
[num,den] = ss2tf(A,B,C,D);
sys_tf = tf(num,den);

%% Controllability
P = ctrb(sys_ss);
if (rank(P) == size(A,1))
  disp("System is controllable.");
else
  disp("System is NOT controllable.");
endif

%% Observability
Q = obsv(sys_ss);
if (rank(P) == size(A,1))
  disp("System is observable.");
else
  disp("System is NOT observable.");
endif

%% Stability
eigvals = eig(A);
[eigvecs,lambda_mat] = eig(A);
# The '0' eigenvalues are repeated (AM=2) and produce only one independent eigenvector (GM=1).
# As AM is not equal to GM, the system is unstable.
disp("System is UNstable");

###################################################################################
## CONTROLLER DESIGN: LINEAR STATE FEEDBACK CONTROL
###################################################################################
# u = -K*x + G*r

## Controller design criteria
perc_OS = 0.1;
ts = 2;

zeta = sqrt((log(perc_OS/100))^2/(pi^2 + (log(perc_OS/100))^2));
wn = 4/(zeta*ts);
s1 = -zeta*wn + 1i*wn*sqrt(1-zeta^2);
s2 = conj(s1);
p_des = [s1 s2];  # Desired poles array

K = place(A,B,p_des);
G = -1/(C/(A-B*K)*B);

###################################################################################
## OBSERVER DESIGN FOR LINEAR STATE FEEDBACK CONTROL
###################################################################################
# The observer poles are typically placed at 5-10 times the controller poles

p_des = [10*s1 4.5*s2];  # Desired poles array
L = place(A', C', p_des)';
###################################################################################
## SYSTEM SIMULATION WITH LINEAR STATE FEEDBACK CONTROL
###################################################################################
% Specify Time array
t = 0:0.01:10;
n = length(t);
dt = t(2) - t(1);
% Initialize x,y,u arrays
x = zeros(2,n);
y = zeros(1,n);
u = zeros(1,n);
% Initial state
x_i = 10;  % cm
v_i = 0; % meters per sec
% Final State
r = 2; % cm

x(:,1) = [x_i/100;v_i];
r = r / 100; % cm to m

x_hat = zeros(2, n);
x_hat(:,1) = [0;0];

for i = 1:n-1
    u(i) = -K * x(:,i) + G * r;   # Linear state feedback control withOUT observer
##    u(i) = -K * x_hat(:,i) + G * r;   # Linear state feedback control with observer
    x_dot = A*x(:,i) + B*u(i);
    x(:,i+1) = x(:,i) + dt*x_dot;
    y(i) = C*x(:,i) + D*u(i);

    ##  Observer:
    xh_i = x_hat(:,i);
    xh_dot = A*xh_i + B*u(i) + L*(y(i) - C*xh_i);
    x_hat(:,i+1) = xh_i + dt*xh_dot;
end


##
figure 1
plot(t(1:end),y(1:end)*100,'Linewidth',3,'Color',"b");
grid on;
xlabel('Time','Fontsize',22)
ylabel('Position (cm)','Fontsize',22)
title('Response of Closed-loop system','Fontsize',22)
##
figure 2
plot(t(1:end),u(1:end)*57.3,'Linewidth',3,'Color',"b");
grid on;
xlabel('Time','Fontsize',22)
ylabel('Input angle (deg)','Fontsize',22)
title('Input angle history','Fontsize',22)
##
##figure 3
##plot(t(1:end),x(1,:),'Linewidth',3,'Color',"b");
##hold on
##grid on;
##xlabel('Time','Fontsize',22)
##ylabel('Position (cm)','Fontsize',22)
##title('Actual and Observed POSITION history','Fontsize',22)
##plot(t(1:end),x_hat(1,:),'--','Linewidth',3,'Color',"r");
##hold off
##
##figure 4
##plot(t(1:end),x(2,:),'Linewidth',3,'Color',"b");
##hold on
##grid on;
##xlabel('Time','Fontsize',22)
##ylabel('x_dot (meters per sec)','Fontsize',22)
##title('Actual and Observed VELOCITY history','Fontsize',22)
##plot(t(1:end),x_hat(2,:),'--','Linewidth',3,'Color',"r");
##hold off
##

###################################################################################
## CONTROLLER DESIGN: PID CONTROL
###################################################################################
# u = kp*(x-xref) + kd*(vel) + ki*error_sum

## Controller design criteria
perc_OS = 0.1;
ts = 2;

zeta = sqrt((log(perc_OS/100))^2/(pi^2 + (log(perc_OS/100))^2));
wn = 4/(zeta*ts);
s1 = -zeta*wn + 1i*wn*sqrt(1-zeta^2);
s2 = conj(s1);
desired_poles = [s1 s2];

## TUNING BY COMPARISON OF DESIRED AND PID CHARACTERISTIC POLYNOMIALS
## PID chr. poly. : Obtained using den of the TF (GC/1+GC) where C is PID-TF and G is Plant-TF
## => s^3 + 9.81*kd*s^2 + 9.81*kp*s + 9.81*ki
## DESIRED chr. poly. : Obtained by (s^2+2*zeta*wn*s+wn^2)(s+a) where (s+a) was added
## to make the polynomial third order and is a dominated pole (a ~= 10*Re(desired_poles))
## => s^3 + 24*s^2 + 84.83*s + 96.55

## The calculated values are at the edge of requirements
## kp = 8.647; ki = 9.842; kd = 2.446;
kp = 0.03*100; ki = 0; kd = 0.0125*100;
s = tf('s');
C_pid = kp + kd * s + ki / s;

#####################################################################################
#### SYSTEM SIMULATION WITH PID CONTROL
#####################################################################################
s = tf('s');
C_pid = kp + ki/s + kd*s;
T = feedback(C_pid*sys_tf, 1);
figure 10
step(T,10)
grid on

function plot_pid(kp,ki,kd,G,color)
  s = tf('s');
  C_pid = kp + ki/s + kd*s;
  T = feedback(C_pid*G, 1);
  step(T,10,color)
  grid on
endfunction


