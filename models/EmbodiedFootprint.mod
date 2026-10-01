# Chapter 11 / TITS 2024: Eqs. (29)-(41).
param model_data{i in {1..17}};
param Nobs := model_data[16];
param obstacle_vertices{i in {1..Nobs}, j in {1..4}, k in {1..2}};
param obstacle_areas{i in {1..Nobs}};
param Nfe := model_data[7];

var tf >= 0;
var x{i in {1..Nfe}};
var y{i in {1..Nfe}};
var theta{i in {1..Nfe}};
var v{i in {1..Nfe}};
var a{i in {1..Nfe}};
var phi{i in {1..Nfe}};
var k{i in {1..Nfe}};
var w{i in {1..Nfe}};

var AX{i in {1..Nfe}};
var AY{i in {1..Nfe}};
var BX{i in {1..Nfe}};
var BY{i in {1..Nfe}};
var CX{i in {1..Nfe}};
var CY{i in {1..Nfe}};
var DX{i in {1..Nfe}};
var DY{i in {1..Nfe}};

var dt{i in {1..Nfe}} >= 0;
var s{i in {1..Nfe}} >= 0;
var up{i in {1..Nfe}};
var down{i in {1..Nfe}};
var left{i in {1..Nfe}};
var right{i in {1..Nfe}};

param a_max := model_data[10];
param v_max := model_data[8];
param w_max := model_data[11];
param phi_max := model_data[9];

param lw := model_data[12];
param lr := model_data[14];
param hlb := model_data[15];
param LF := model_data[13];
param lb := hlb * 2;
param max_dt := model_data[17];

minimize objective_:
sum{i in {1..(Nfe-1)}}(dt[i]^2);#


s.t. time_bound1{i in {1..(Nfe-1)}}:
dt[i] <= max_dt;



s.t. DIFF_dxdt{i in {2..Nfe}}:
x[i] = x[i-1] + v[i-1] * dt[i-1] * cos(theta[i-1]);
s.t. DIFF_dydt {i in {2..Nfe}}:
y[i] = y[i-1] + v[i-1] * dt[i-1] * sin(theta[i-1]);
s.t. DIFF_dvdt {i in {2..Nfe}}:
v[i] = v[i-1] + dt[i-1] * a[i-1];
s.t. DIFF_dthetadt {i in {2..Nfe}}:
theta[i] = theta[i-1] + dt[i-1] * v[i-1] * tan(phi[i-1]) / lw;
s.t. DIFF_dphidt {i in {2..Nfe}}:
phi[i] = phi[i-1] + dt[i-1] * w[i-1];

s.t. define_kappa{i in {1..(Nfe-1)}}:
k[i] = tan(phi[i]) / lw;

s.t. define_s{i in {1..(Nfe-1)}}:
s[i] = v[i] * dt[i];

s.t. define_up{i in {1..(Nfe-1)}}:
up[i] = s[i] + hlb * s[i] * abs(k[i]);

s.t. define_down{i in {1..(Nfe-1)}}:
down[i] = 0;

s.t. define_left{i in {1..(Nfe-1)}}:
left[i] = max((LF+0.5*s[i])*k[i]*s[i], -lr*k[i]*s[i]);

s.t. define_right{i in {1..(Nfe-1)}}:
right[i] = max(-(LF+0.5*s[i])*k[i]*s[i], lr*k[i]*s[i]);

s.t. Init_X:
x[1] = model_data[1];
s.t. Init_Y:
y[1] = model_data[2];
s.t. Init_Theta:
theta[1] = model_data[3];
s.t. End_X:
x[Nfe] = model_data[4];
s.t. End_Y:
y[Nfe] = model_data[5];
s.t. End_Theta:
theta[Nfe] = model_data[6];

s.t. Init_W:
w[1] = 0;
s.t. Init_A:
a[1] = 0;
s.t. Init_Phy:
phi[1] = 0;
s.t. Init_v:
v[1] = 0;
s.t. End_W:
w[Nfe] = 0;
s.t. End_A:
a[Nfe] = 0;
s.t. End_Phy:
phi[Nfe] = 0;
s.t. End_v:
v[Nfe] = 0;

s.t. Bounds_v {i in {1..Nfe}}:
0 <= v[i] <= v_max;
s.t. Bounds_a {i in {1..Nfe}}:
-a_max <= a[i] <= a_max;
s.t. Bounds_phi {i in {1..Nfe}}:
-phi_max <= phi[i] <= phi_max;
s.t. Bounds_w {i in {1..Nfe}}:
-w_max <= w[i] <= w_max;


s.t. EmbodiedPrerequisite_bound1{i in {1..(Nfe-1)}}:
-1.5707963267948966 <= s[i] * k[i] <= 1.5707963267948966;

s.t. EmbodiedPrerequisite_bound2{i in {1..(Nfe-1)}}:
(1 + hlb * abs(k[i])) * tan(abs(k[i]) * s[i]) <= lr * abs(k[i]);

s.t. EmbodiedPrerequisite_bound3{i in {1..(Nfe-1)}}:
abs(k[i]) * LF * tan(abs(k[i]) * s[i]) <= 1 + hlb * abs(k[i]);


s.t. final_dt: dt[Nfe] = 0;
s.t. final_s: s[Nfe] = 0;
s.t. final_k: k[Nfe] = 0;
s.t. final_up: up[Nfe] = 0;
s.t. final_down: down[Nfe] = 0;
s.t. final_left: left[Nfe] = 0;
s.t. final_right: right[Nfe] = 0;
s.t. total_time: tf = sum{i in 1..Nfe-1} dt[i];
## Geometrics ##
s.t. RELATIONSHIP_AX{i in {1..(Nfe-1)}}:
AX[i] = x[i] + (LF + up[i]) * cos(theta[i]) - (hlb + left[i]) * sin(theta[i]);

s.t. RELATIONSHIP_BX{i in {1..(Nfe-1)}}:
BX[i] = x[i] + (LF + up[i]) * cos(theta[i]) + (hlb + right[i]) * sin(theta[i]);

s.t. RELATIONSHIP_CX{i in {1..(Nfe-1)}}:
CX[i] = x[i] - (lr) * cos(theta[i]) + (hlb + right[i]) * sin(theta[i]);

s.t. RELATIONSHIP_DX{i in {1..(Nfe-1)}}:
DX[i] = x[i] - (lr) * cos(theta[i]) - (hlb + left[i]) * sin(theta[i]);

s.t. RELATIONSHIP_AY{i in {1..(Nfe-1)}}:
AY[i] = y[i] + (LF + up[i]) * sin(theta[i]) + (hlb + left[i]) * cos(theta[i]);

s.t. RELATIONSHIP_BY{i in {1..(Nfe-1)}}:
BY[i] = y[i] + (LF + up[i]) * sin(theta[i]) - (hlb + right[i]) * cos(theta[i]);

s.t. RELATIONSHIP_CY{i in {1..(Nfe-1)}}:
CY[i] = y[i] - (lr) * sin(theta[i]) - (hlb + right[i]) * cos(theta[i]);

s.t. RELATIONSHIP_DY{i in {1..(Nfe-1)}}:
DY[i] = y[i] - (lr) * sin(theta[i]) + (hlb + left[i]) * cos(theta[i]);



## Collison avoidance ##
s.t. eq_PPPoutsideABCD {i in {2..(Nfe-1)}, nn in {1..Nobs}, jj in {1..4}}:
(abs((AX[i] - obstacle_vertices[nn,jj,1])*(BY[i] - obstacle_vertices[nn,jj,2]) - (AY[i] - obstacle_vertices[nn,jj,2])*(BX[i] - obstacle_vertices[nn,jj,1])) * 0.5 + abs((BX[i] - obstacle_vertices[nn,jj,1])*(CY[i] - obstacle_vertices[nn,jj,2]) - (BY[i] - obstacle_vertices[nn,jj,2])*(CX[i] - obstacle_vertices[nn,jj,1])) * 0.5 + abs((CX[i] - obstacle_vertices[nn,jj,1])*(DY[i] - obstacle_vertices[nn,jj,2]) - (CY[i] - obstacle_vertices[nn,jj,2])*(DX[i] - obstacle_vertices[nn,jj,1])) * 0.5 + abs((DX[i] - obstacle_vertices[nn,jj,1])*(AY[i] - obstacle_vertices[nn,jj,2]) - (DY[i] - obstacle_vertices[nn,jj,2])*(AX[i] - obstacle_vertices[nn,jj,1])) * 0.5) >= (LF + lr + up[i]) * (lb + right[i] + left[i]) + 0.1;

s.t. eq_AoutsidePRECTANGLEPPP {i in {2..(Nfe-1)}, nn in {1..Nobs}}:
(abs((obstacle_vertices[nn,1,1] - AX[i])*(obstacle_vertices[nn,2,2] - AY[i]) - (obstacle_vertices[nn,1,2] - AY[i])*(obstacle_vertices[nn,2,1] - AX[i])) * 0.5 + abs((obstacle_vertices[nn,2,1] - AX[i])*(obstacle_vertices[nn,3,2] - AY[i]) - (obstacle_vertices[nn,2,2] - AY[i])*( obstacle_vertices[nn,3,1] - AX[i])) * 0.5 + abs((obstacle_vertices[nn,3,1] - AX[i])*( obstacle_vertices[nn,4,2] - AY[i]) - (obstacle_vertices[nn,3,2] - AY[i])*( obstacle_vertices[nn,4,1] - AX[i])) * 0.5 + abs((obstacle_vertices[nn,4,1] - AX[i])*( obstacle_vertices[nn,1,2] - AY[i]) - (obstacle_vertices[nn,4,2] - AY[i])*( obstacle_vertices[nn,1,1] - AX[i])) * 0.5) >= obstacle_areas[nn];

s.t. eq_BoutsidePRECTANGLEPPP {i in {2..(Nfe-1)}, nn in {1..Nobs}}:
(abs((obstacle_vertices[nn,1,1] - BX[i])*(obstacle_vertices[nn,2,2] - BY[i]) - (obstacle_vertices[nn,1,2] - BY[i])*(obstacle_vertices[nn,2,1] - BX[i])) * 0.5 + abs((obstacle_vertices[nn,2,1] - BX[i])*(obstacle_vertices[nn,3,2] - BY[i]) - (obstacle_vertices[nn,2,2] - BY[i])*( obstacle_vertices[nn,3,1] - BX[i])) * 0.5 + abs((obstacle_vertices[nn,3,1] - BX[i])*( obstacle_vertices[nn,4,2] - BY[i]) - (obstacle_vertices[nn,3,2] - BY[i])*( obstacle_vertices[nn,4,1] - BX[i])) * 0.5 + abs((obstacle_vertices[nn,4,1] - BX[i])*( obstacle_vertices[nn,1,2] - BY[i]) - (obstacle_vertices[nn,4,2] - BY[i])*( obstacle_vertices[nn,1,1] - BX[i])) * 0.5) >= obstacle_areas[nn];

s.t. eq_CoutsidePRECTANGLEPPP {i in {2..(Nfe-1)}, nn in {1..Nobs}}:
(abs((obstacle_vertices[nn,1,1] - CX[i])*(obstacle_vertices[nn,2,2] - CY[i]) - (obstacle_vertices[nn,1,2] - CY[i])*(obstacle_vertices[nn,2,1] - CX[i])) * 0.5 + abs((obstacle_vertices[nn,2,1] - CX[i])*(obstacle_vertices[nn,3,2] - CY[i]) - (obstacle_vertices[nn,2,2] - CY[i])*( obstacle_vertices[nn,3,1] - CX[i])) * 0.5 + abs((obstacle_vertices[nn,3,1] - CX[i])*( obstacle_vertices[nn,4,2] - CY[i]) - (obstacle_vertices[nn,3,2] - CY[i])*( obstacle_vertices[nn,4,1] - CX[i])) * 0.5 + abs((obstacle_vertices[nn,4,1] - CX[i])*( obstacle_vertices[nn,1,2] - CY[i]) - (obstacle_vertices[nn,4,2] - CY[i])*( obstacle_vertices[nn,1,1] - CX[i])) * 0.5) >= obstacle_areas[nn];

s.t. eq_DoutsidePRECTANGLEPPP {i in {2..(Nfe-1)}, nn in {1..Nobs}}:
(abs((obstacle_vertices[nn,1,1] - DX[i])*(obstacle_vertices[nn,2,2] - DY[i]) - (obstacle_vertices[nn,1,2] - DY[i])*(obstacle_vertices[nn,2,1] - DX[i])) * 0.5 + abs((obstacle_vertices[nn,2,1] - DX[i])*(obstacle_vertices[nn,3,2] - DY[i]) - (obstacle_vertices[nn,2,2] - DY[i])*( obstacle_vertices[nn,3,1] - DX[i])) * 0.5 + abs((obstacle_vertices[nn,3,1] - DX[i])*( obstacle_vertices[nn,4,2] - DY[i]) - (obstacle_vertices[nn,3,2] - DY[i])*( obstacle_vertices[nn,4,1] - DX[i])) * 0.5 + abs((obstacle_vertices[nn,4,1] - DX[i])*( obstacle_vertices[nn,1,2] - DY[i]) - (obstacle_vertices[nn,4,2] - DY[i])*( obstacle_vertices[nn,1,1] - DX[i])) * 0.5) >= obstacle_areas[nn];


data;
param model_data := include parameters.dat;
param obstacle_vertices := include obstacles.dat;
param obstacle_areas := include obstacle_areas.dat;