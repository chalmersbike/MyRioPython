% LQR Gain Coefficients calculation based on RightHand Rule 
close all

n=3; %order of the fit

velocity_K = linspace(1.5,4,2500);  % [m/s]
K_full_dis_eig_plot=zeros(size(velocity_K,2),3);          %to be used in real bike
eigenvalues_dis_eig=zeros(3,size(velocity_K,2));
abs_eigenvalues_dis=zeros(3,size(velocity_K,2));


Ts=0.04;           % [sec]


h = 0.586;          % height of center of mass [m]
b = 1.095;          % length between wheel centers [m]
c = 0.06;           % length between front wheel contact point and the extention of the fork axis
lambda=deg2rad(70); % angle of the fork axis
a = b-0.77;         % distance from rear wheel to center of mass [m]

% Q = diag([10000,10,100]); % cost of states in LQR controller // [100,100,10]
% R = 30;                 % cost of inputs in LQR controller // 10 
% Q = diag([10000,10,100]); %ORIGINAL WEIGHTS BY UMUR cost of states in LQR controller // [100,100,10]
% R = 30;                 %ORIGINAL WEIGHTS BY UMUR cost of inputs in LQR controller // 10 

g = 9.81;   % gravity

C = eye(3);      %does not depent on velocity
D = zeros(3,1);  %does not depent on velocity

 for i=1:size(velocity_K,2)
        v=velocity_K(i);
% For RightHand Rule frame, consistent with Astrom model
% NOTE THAT, this also applys with Umur's previous coordinate setting,
% BECAUSE both \phi and \delta are INVERTED compared with the Astrom Model
% setting
        A = [0      0            1;
             0      0            0;
             g/h    sin(lambda)*(h*v^2-g*a*c)/(h^2*b)   0];
        B = [0; 1; (a*v*sin(lambda))/(h*b)];


        sys_con = ss(A,B,C,D);
        sys_dis = c2d(sys_con,Ts);

        K_dis_plot = lqrd(A,B,Q,R,Ts);     
        K_full_dis_eig_plot(i,:)= K_dis_plot;

        eigenvalues_dis=eig(sys_dis.A-sys_dis.B*K_dis_plot);
        abs_eigenvalues_dis(:,i)=[norm(eigenvalues_dis(1));norm(eigenvalues_dis(2));norm(eigenvalues_dis(3))];  
        
 end
    
   
    
p1 = polyfit(velocity_K.',K_full_dis_eig_plot(:,1),n)   
p2 = polyfit(velocity_K.',K_full_dis_eig_plot(:,2),n)  
p3 = polyfit(velocity_K.',K_full_dis_eig_plot(:,3),n)
 
y1 = polyval(p1,velocity_K);
y2 = polyval(p2,velocity_K);
y3 = polyval(p3,velocity_K);

figure()
    subplot(3,1,1)
    plot(velocity_K,K_full_dis_eig_plot(:,1))
    hold on
    plot(velocity_K,y1)
    ylabel('$K_\varphi$','interpreter','latex')
    grid on
    subplot(3,1,2)
    plot(velocity_K,K_full_dis_eig_plot(:,2))
    hold on
    plot(velocity_K,y2)
    ylabel('$K_\delta$','interpreter','latex')
    grid on
    subplot(3,1,3)
    plot(velocity_K,K_full_dis_eig_plot(:,3))
    hold on
    plot(velocity_K,y3)
    xlabel('Velocity [m/s]')
    ylabel('$K_{\dot{\varphi}}$','interpreter','latex')
    grid on  

figure()    
    plot(velocity_K,abs_eigenvalues_dis(1,:))
    hold on
    plot(velocity_K,abs_eigenvalues_dis(2,:))
    plot(velocity_K,abs_eigenvalues_dis(3,:))
    grid on

    