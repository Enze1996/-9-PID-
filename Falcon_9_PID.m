%% PID Control Design

% Constants & Design Parameters

h3 = 10; % distance between the center of gravity of the rocket & the gimbaled Merlin 1D engine in consideration (meters) 
T = 845.22 * 10^3; % thrust of a Falcon 9 FT stage 1 Merlin 1D engine (Newtons)
J = 37576837; % moment of inertia of the Falcon 9 about the vertical axis (assumption: cylindrical body)
C = (h3*T)/J; % constant

s = tf('s');

% Plant TF, 'Gp'
Gp = zpk(minreal(C/s^2))

% These values were obtained by auto-tuning the PID controller in Simulink
P = 0.825493884458449;
I = 0.0468147782797737;
D = 5.24517904929436;
N = 5.45397025421619;
Gc = zpk(minreal(pid(P,I,D,N),1e-05))

%消除系统中的无关紧要的极点-零点对来简化传递函数
L = zpk(minreal((Gc*Gp),1e-05))
T = zpk(minreal((L/(1 + L)),1e-05))
%灵敏度函数可以理解为系统对输入扰动的放大率或抑制率。
% 理想情况下，灵敏度函数的值应接近于零，这意味着系统能够有效地抑制扰动对输出的影响。
S = zpk(minreal((1 - T),1e-05))
Y = zpk(minreal((T*Gp),1e-05))
GpS = zpk(minreal((Gp*S),1e-05))

% Internal stability check
Y_stability = isstable(Y)
T_stability = isstable(T) 
% isstable() 通过检查传递函数的极点来判断系统的稳定性。
% 对于连续系统，所有极点必须在左半平面；对于离散系统，所有极点必须在单位圆内
S_stability = isstable(S)
GpS_stability = isstable(GpS)

%getPeakGain(S) 在控制系统分析中用于确定传递函数的最大增益。
% 这个指标对于评估系统的性能和鲁棒性至关重要，尤其是在灵敏度分析和控制器设计中
M2 = 1/getPeakGain(S) % M2-margin
BW = bandwidth(T) % bandwidth of the closed-loop
AE = getPeakGain(Y) % maximum actuator effort

figure(1)
bodemag(Y, S, T);
legend('Y','S','T');
print -depsc PID.eps;
