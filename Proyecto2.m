clear
clc
close all
format short
%% Linearized model Matrices
A=[-0.4555 0.0  0.0438  0.0;
    0.0058 -0.3285 0.0437 0.2875;
   -2.1564 10.4769 -9.9054 0;
   6.0408 2.3730 3.1573 -12.0195];
B=[0.1109;0.0131;0.0;0.7157];
C =[1 0 0 0];
D=[0];
rank(ctrb(A,B)); %Controlability check
%% Pole placement definition
pole_1=-10+4i;
pole_2=-10-4i;
pole_3=-7+5i;
pole_4=-7-5i;
pole_5=-0.1548;
POLES = [pole_1 pole_2 pole_3 pole_4 pole_5];
%% Augmented matrices for full state feedback control plus integral
A_hat=[A;-C];
A_hat = [A_hat zeros(5,1)];
B_hat = [B;0];
PP=[A B;C 0];
rank(PP);
%% Finding proportional K with given Augmented matrices and poles.
K_5 = place(A_hat,B_hat,POLES)   % The result is about the same as in Malkhede et al. (2013) 
Kpp=K_5(1:4);
KI = -K_5(5);

%% Defining Q and R for LQR Optimal control
Q=diag([100000 100000 1 1 100000000])%%Best
R=1; 
[X,Klqr,L] = icare(A_hat,B_hat,Q,R,[],[],[]);
Klqr
K_lqr=Klqr(1:4);            %%Feedback state Constants
KI_lqr = -Klqr(5);          %%Integral constant   
%% LQR Algorithm by solving Ricatti Eq. (iterative method) % Not working :( lloremos
% n=5;             %%Number of states
% m=1;             %%Number of inputs
% %K=zeros(m,n);
% S=zeros(n,n);
% MaxNumbIter = 10;

% %%%%%% Not working :( lloremos
%     for i=1:MaxNumbIter
%         M1 = (((B_hat')*S*B_hat) + R )\1;
%         M2 = S - (S*B_hat*M1*(B_hat')*S);
%         S = ((A_hat')*M2*A_hat) + Q;
%         Ktemp = M1*(B_hat')*S*A_hat;
%             if i<=1
%             K = Ktemp;
%             else
%             K=[K;Ktemp];
%             end
%     end

%% Simulate
simdata=sim('LQROptimazition');
%% Plotting
plot(simdata.tout,simdata.PP)
hold on
title('System output')
plot(simdata.tout,simdata.LQR)
plot(simdata.tout,simdata.input)
grid on
axis([0 1.5 0 1.2])
plot(simdata.input)
legend('PP','LQR','Input')
hold off

figure
plot(simdata.tout,simdata.PP_states)
title('States comparision Pole placemente(PP) vs Linear quadratic regulator(LQR)')
hold on
plot(simdata.tout,simdata.PP_dot_eta)
plot(simdata.tout,simdata.LQR_states)
plot(simdata.tout,simdata.LQR_dot_eta)
grid on
legend('PP x1','PP x2','PP x3','PP x4','PP x5','LQR x1','LQR x2','LQR x3','LQR x4','LQR x5')

figure
plot(simdata.tout,simdata.PP_u)
title('Comparision U')
hold on
plot(simdata.tout,simdata.LQR_u)
axis([0 5 0 130])
grid on
legend('PP U','LQR U')

figure
plot(real(POLES),imag(POLES),'Xr')
title('Poles Comparision')
hold on
plot(real(L),imag(L),'xb')
legend('Initial Proposal','LQR Methos result')
grid

