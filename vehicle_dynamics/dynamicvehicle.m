function dydt = dynamicvehicle( t, y, control ,mu)
if nargin<4
    mu=1.0;
end
if nargin<5
    uw=0;
end

m = 1400; 
f = 0.01;
Cd=0.3;
A=2.5;
rho=1.225;
Iz = 2420; 
bw = 0; 
fw = 0;
Nw = 2;
rw = 0.31;
J = 2.65;
a = 1.14;
L = 2.54;
b = L - a;


% we make some assumptions about how the varying the road friction affects
% the magic formula. Neither of these assumptions are exact, but the
% behavior is close to what we see, and it allows us to simulate
% different road surfaces without wiriting out a full set of parameters for
% each mu

%the peak of our tire force is scaled proportionally with road friction
Dx = mu;
Dy = mu;

%we assume the stiffness (where we get the nice linear behavior at low slip) is
%is not affected by the reduction in friction.
if mu==0
    Bx=0;
    By=0;
else
Bx = 25*1/mu;
By = 0.27*1/mu;
end


Cx = 1.35;
Shx  = 0;
Svx = 0;
Ex = -2.9;

Cy = 1.2;
Ey = -1.6;
Shy = 0;
Svy = 0;

g=9.86055;

%normal forces with steady acceleration state assumption
%normal force for 2 front wheels
Fzf = m*g*b/L;
%normal force fr 2 real wheels
Fzr = m*g*a/L;
%normal force per rear wheel
Fz=Fzr/2;

dydt = zeros( 7, 1 );

%get control inputs from function
outputcontrol = control( t );

%wheel angle
c1 = outputcontrol( 1 );

%wheel torque
c2 = outputcontrol( 2 );

lambda = ( y( 3 ) * rw - y( 2 ) )./( y( 3 ) * rw );

alphaf = rad2deg( c1 - atan( ( y( 5 ) + a * y( 7 ) )/y( 2 ) ) ); %% convert from rad to degrees
alphar = rad2deg( atan( ( y( 5 ) - b * y( 7 ) )/y( 2 ) ) );  %% convert from rad to degrees

phix = ( 1 - Ex ) * ( lambda + Shx ) + Ex/Bx * atan( Bx * ( lambda + Shx ) );
phiyf = ( 1 - Ey ) * ( alphaf + Shy ) + Ey/By * atan( By * ( alphaf + Shy ) );
phiyr = ( 1 - Ey ) * ( alphar + Shy ) + Ey/By * atan( By * ( alphar + Shy ) );

%driving force per driven wheel
Fx =  Dx*Fz* sin( Cx * atan( Bx * phix ) ) + Svx;

%front lateral force per 2 front wheels
Fyf = Dy*Fzf * sin( Cy * atan( By * phiyf ) ) + Svy;

%the rear lateral force is negated because of how we calculate the slip angle in
%the SAE coordinate frame this is also per 2 rear wheels
Fyr = -Dy*Fzr * sin( Cy * atan( By * phiyr ) ) + Svy;

%check for tire saturation
Frmax=mu*Fzr;
Frtotal=sqrt((2*Fx)^2+Fyr^2);
if Frtotal>Frmax   
    Fx=Frmax/Frtotal*(Fx);
    Fyr=Frmax/Frtotal*(Fyr);
end



dydt( 1 ) = y( 2 ) .* cos( y( 6 ) ) - y( 5 ) .* sin( y( 6 ) );
dydt( 2 ) = (-0.5*rho*Cd*A*(y(2)+uw^2) -f * m * g + Nw * Fx )/m;
dydt( 3 ) = ( c2 - rw * Fx - fw * Fz - bw * y( 3 ) )/J;%wheel rpm
dydt( 4 ) = y( 2 ) .* sin( y( 6 ) ) - y( 5) .* cos( y( 6 ) );
dydt( 5 ) = ( Fyf + Fyr )/m - y( 2 ) .* y( 7 );
dydt( 6 ) = y( 7 );
dydt( 7 ) = ( Fyf * a - Fyr * b )/Iz;

end
