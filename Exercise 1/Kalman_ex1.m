% Exercise 1 - Kalman Filter Implementation
%% section 1

t=[1,2]; 
% time vector with only two entries as requested
z=[2,3];            % these are the measurments at time t(1) and t(2)

% Let us present the model
A=1; R=1; Q=2;
% Let us initialize the state vector and state uncertainty
mu=0;              % this is the state estimation at each time step, it is initialized to 
                   % Zero as this is given
                   
State_cov=10;      % This is the state Covariance matrix, initialized to 10 as given
                   % For the first step. as this is a one dimensional
                   % model, the Cov. matrix reduces to scalar form.

% and the process
for i=1:2
end


%% Section 2
 Z=csvread('car_measument_data.dat');

dt=0.1; A=[1 dt;0 1]; B=[dt^2/2; dt]; R = [dt^4/4 dt^3/2; dt^3/2 dt^2];
u=1.5; process_std=0.5; meas_std=10; C = [1 0];

% initialize 
t=0:dt:10;
xx=-100:0.1:200;

X_prediction(:,1)=[0;0];
X_correction(:,1)=[0;0];
X_true(:,1)=[0;0];
sigma_process_prediction(:,:,1)=zeros(2);
sigma_process_correction(:,:,1)=zeros(2);

% Kalman
for i=(1:length(t)-1)
    X_true(:,i+1)=A*X_true(:,i)+B*u+[dt^2/2 ; dt]*process_std*randn;
    % prediction
    X_prediction(:,i+1)=A*X_correction(:,i)+B*u;
    sigma_process_prediction(:,:,i+1)=A*sigma_process_prediction(:,:,i)*A'+R*process_std^2;
    %kalman gain
    K=sigma_process_prediction(:,:,i+1)*C'*(C*sigma_process_prediction(:,:,i+1)*C'+meas_std^2);
    %correction
    X_correction(:,i+1)=X_prediction(:,i+1)+K*(Z(i+1,2)-C*X_prediction(:,i+1));
    sigma_process_correction(:,:,i+1)=(eye(2)-K*C)*sigma_process_prediction(:,:,i+1);   
    
    
    %pdf printing
    if ((i+1)*dt)==0.2
        prediction_pdf_values=normpdf(xx,X_prediction(1,i+1),sigma_process_prediction(1,1,i+1));
        correction_pdf_values=normpdf(xx,X_correction(1,i+1),sigma_process_correction(1,1,i+1));
        measurment_pdf_values=normpdf(xx,Z(i+1,2),meas_std);
        figure(1);clf;
        plot(xx,prediction_pdf_values)
        hold on;
        plot(xx,correction_pdf_values)
        hold on;
        plot(xx,measurment_pdf_values)
        hold off;
    end

    
end


