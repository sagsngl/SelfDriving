clear all;
close all;
clc;
% initial condition
z0 = [ 0 10 32.5839 -5 0 0 0 ];
t0 = 0;
tf = 6.0114;

friction_factors=[0.25,0.5,0.75,1.0];

figure(1)
subplot(2,2,[1,2])
fill([40,50,50,40,40],[-10,-10,0,0,-10],'r')
hold on; plot( linspace( 0, 150, 101 ), -10 * ones( 1, 101 ), 'k', 'LineWidth', 5 );
hold on; plot( linspace( 0, 150, 101 ), 10 * ones( 1, 101 ), 'k', 'Linewidth', 5 );

axis( [ 0 150 -12 12 ] );
xlabel( 'Longitudinal Position (m)','FontSize', 12 );
ylabel( 'Lateral Position (m)','FontSize', 12 );
subplot(2,2,3)
hold on
xlabel( 'Longitudinal Position (m)','FontSize', 12 );
ylabel( 'Wheel Angle (deg)','FontSize', 12 );
subplot(2,2,4)
hold on
xlabel( 'Longitudinal Position (m)','FontSize', 12 );
ylabel( 'Longitudinal Velocity (m/s)','FontSize', 12 );
ylim([0,40])
%% sharp lane change
for j=1:length(friction_factors)
    
[ t2, y2 ] = ode45( @(tau, x ) dynamicvehicle( tau, x, @( tau ) lanechangecontrol( tau ) ,friction_factors(j)), [ t0 tf ], z0 );

figure(1)
subplot(2,2,[1,2])
plot( y2( :, 1 ), y2( :, 4 ),'LineWidth',1.5); 

times = t2;
c1 = zeros( size( times ) );
for i = 1:length(times)
    controller = lanechangecontrol( times( i ) );
    c1( i ) = rad2deg( controller( 1 ) );
end


subplot(2,2,3)
 plot(  y2( :, 1 ),c1, 'LineWidth',1.5 );
 
 subplot(2,2,4);
plot( y2( :, 1), y2( :, 2),'LineWidth',1.5);

end





