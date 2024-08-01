function [ control ] = lanechangecontrol( t )
%output of wheel angle (c1) is in radians
endtime=3;
if ( t < endtime )
   % control( 1 ) = polyval( [ -0.9744 0.5974 0.1 ], t );
   control(1)=1/5*sin(2*pi/endtime*t);
    control( 2 ) =50;
else
    control( 1 ) = 0.005;
    control( 2 ) = 50;
end