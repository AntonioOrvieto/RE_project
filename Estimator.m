function [posEst,oriEst,radiusEst, posVar,oriVar,radiusVar,estState] = Estimator(estState,actuate,sense,tm,estConst)
% [posEst,oriEst,posVar,oriVar,baseEst,baseVar,estState] =
% 	Estimator(estState,actuate,sense,tm,knownConst,designPart)
%
% The estimator.
%
% The function will be called in two different modes:
% If tm==0, the estimator is initialized; otherwise the estimator does an
% iteration step (compute estimates for the time step k).
%
%   estState        previous estimator state (time step k-1)
% Inputs:
%                   May be defined by the user (for example as a struct).
%   actuate         control input u(k), [1x2]-vector
%                   actuate(1): u_V, drive wheel angular velocity
%                   actuate(2): u_R, drive wheel angle
%   sense           sensor measurements z(k), [1x2]-vector, INF if no
%                   measurement
%                   sense(1): z_d, distance measurement
%                   sense(2): z_r, orientation measurement
%   tm              time, scalar
%                   If tm==0 initialization, otherwise estimator
%                   iteration step.
%   estConst        estimator constants (as in EstimatorConstants.m)
%
% Outputs:
%   posEst          position estimate (time step k), [1x2]-vector
%                   posEst(1): x position estimate
%                   posEst(2): y position estimate
%   oriEst          orientation estimate (time step k), scalar
%   radiusEst       estimate of wheel radius W (time step k), scalar
%   posVar          variance of position estimate (time step k), [1x2]-vector
%                   posVar(1): x position variance
%                   posVar(2): y position variance
%   oriVar          variance of orientation estimate (time step k), scalar
%   radiusVar       variance of wheel radius estimate (time step k), scalar
%   estState        current estimator state (time step k)
%                   Will be input to this function at the next call.
%
%
% Class:
% Recursive Estimation
% Spring 2016
% Programming Exercise 1
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Raffaello D'Andrea, Michael Muehlebach
% michaemu@ethz.ch
%
% --
% Revision history
% [19.04.11, ST]    first version by Sebastian Trimpe
% [30.04.12, PR]    adapted version for spring 2012, added unknown wheel
%                   radius
% [06.05.13, MH]    2013 version
% [23.04.15, MM]    2015 version
% [14.04.16, MM]    2016 version


%% Mode 1: Initialization
if (tm == 0)
    %{
    The robot starts at (x(0), y(0)) = (x_0 , y_0 ) with orientation 
    r(0) = r_0 . The initial position is uniformly distributed with
    x_0 , y_0 ∈ [-p_bar, p_bar], and the probability density function of
    r 0 uniformly distributed with r 0 ∈ [−r_bar, r_bar].
    
    x_m is ste state variable, a 4D vector that jointly tracks x,y,z,W as
    output of previous the measument update step (x_m(k-1) is where we 
    linearize and do the predicion step)
    %}
    
    %State of the filter
    state=struct();
    
    %Initialization step at time 0
    state.t_prev=0;
    
    %Initialization of x_m 
    state.x_m(1)=0; %x(0)
    state.x_m(2)=0; %y(0)
    state.x_m(3)=0; %r(0)
    state.x_m(4)=estConst.NominalWheelRadius; %W(0)
    
    %Initialization of the variance of x_m
    state.P_m=[estConst.TranslationStartBound^2/3,0,0,0; %Var(x(0))
        0,estConst.TranslationStartBound^2/3,0,0; %Var(x(0)) 
        0,0,estConst.RotationStartBound^2/3,0; %Var(r(0))
        0,0,0,estConst.WheelRadiusError^2/3]; %Var(W(0))
    
    %As this is the first step, we just return the estimates we have just
    %initialized, this is exactly our prior knowledge and is the best we
    %can do as we have no measurements
    
    posEst = [state.x_m(1) state.x_m(2)];
    oriEst = state.x_m(3);
    radiusEst = state.x_m(4);

    posVar = [state.P_m(1,1) state.P_m(2,2)];
    oriVar = state.P_m(3,3);
    radiusVar = state.P_m(4,4);
    estState=state;
    return;
end


%% Mode 2: Estimator iteration.
% If we get this far tm is not equal to zero, and we are no longer
% initializing.  Run the estimator.

%% PRIOR UPDATE STEP

%we store the previous state and time
t_prev = estState.t_prev;
x_m_prev = estState.x_m;
P_m_prev = estState.P_m;

%control variables, as input. constant in time interval from previous to
%current measurement (where the integrate, starting at x_m)
uv=actuate(1);
ur=actuate(2);

%Noise matrices
Q=[estConst.VelocityInputPSD,0;0,estConst.AngleInputPSD];
R=[estConst.CompassNoise,0;0,estConst.DistNoise];

%B imported
B=estConst.WheelBase;

%0 mean noise, we linearize around 0 (we could directy insert them
vr=0;
vv=0;

%Everything is loaded now, we can start the actual coding

%PRIOR UPDATE FOR THE MEAN
q =@(t,x) [x(4)*uv*cos(ur)*cos(x(3)); x(4)*uv*cos(ur)*sin(x(3));(x(4)*uv*sin(ur))/B; 0];
[ti,x_t] = ode45(q,[t_prev tm],x_m_prev);
%x_t is a dim(t) x 3 matrix
x_p = x_t(end,:);

%PRIOR UPDATE for variance

%Calculation of matrix A (4x4) and L(4x2)
%each of those matrix are the results in the linearization at a particular
%time t

AA=zeros(4,4,length(ti));
LL=zeros(4,2,length(ti));

for i=1:length(ti)
    AA(:,:,i)=[0,0,-x_t(i,4)*uv*cos(ur)*sin(x_t(i,3)),cos(x_t(i,3))*(x_t(i,4)*uv*cos(ur))/x_t(i,4);
        0,0,x_t(i,4)*uv*cos(ur)*cos(x_t(i,3)),sin(x_t(i,3))*(x_t(i,4)*uv*cos(ur))/x_t(i,4);
        0,0,0,uv*sin(ur)/B;
        0,0,0,0];
end

for t=1:length(ti)
    LL(:,:,i)=[x_t(i,4)*uv*cos(ur)*cos(x_t(i,3)),-x_t(i,4)*uv*sin(ur)*cos(x_t(i,3));
         x_t(i,4)*uv*cos(ur)*sin(x_t(i,3)),-x_t(i,4)*uv*sin(ur)*sin(x_t(i,3));
         -(x_t(i,4)*uv*sin(ur))/B,-(x_t(i,4)*uv*cos(ur))/B;
         0,0];
end

%Define A and L as functions of times in ti vector, to solve the matrix
%differential equation later in an agile way
A = @(t) AA(:,:,2);
L = @(t) LL(:,:,2);

%Update function
up_p = @(t,p) reshape(A(t)*reshape(p,[4,4])+reshape(p,[4,4])*transpose(A(t)) + L(t)*Q*transpose(L(t)), [16,1]);

%Solve differential equation..
[ti,p_sol] = ode45(up_p,ti,reshape(P_m_prev,[1,16]));

P_p = reshape(p_sol(end,:),[4,4]);

%% MEASUREMENT UPDATE STEP

%Measurement
zd=sense(1);
zr=sense(2);
z_m=[zr,zd]; %IS THE ORDER CORRECT?! (I THINK SO)

M=eye(2);
H=[0,0,1,0;
    x_p(1)/sqrt(x_p(1)*x_p(1)+x_p(2)*x_p(2)),x_p(2)/sqrt(x_p(1)*x_p(1)+x_p(2)*x_p(2)),0,0];

K= P_p*transpose(H)*inv(H*P_p*transpose(H)+M*R*transpose(M));
x_m = x_p.' + K*(z_m.'-[x_p(3),sqrt(x_p(1)^2+x_p(2)^2)].');
P_m= (eye(4)-K*H)*P_p;


%% OUTPUT OF THE FILTER
posEst = [x_m(1) x_m(2)];
oriEst = x_m(3);
posVar = [P_m(1,1) P_m(2,2)];
oriVar = P_m(3,3);
radiusEst = x_m(4);
radiusVar = P_m(4,4);

end