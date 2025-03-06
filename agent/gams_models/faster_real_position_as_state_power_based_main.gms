*option nlp=SCIP;
option nlp=IPOPT;
parameter
         model_status,
         solver_status;
sets
         i 'number of vehicles',
         r 'number of prediction steps';

parameter
         h 'length of the time interval',
         tau 'tau in the car-following model',
         d_0 'd_0 in the car-following model',
         r_dynamics(r) 'dynamic prediction steps used to calculate the differences',
         r_critical_points(r) 'the position where there should be critical points',
         s_init(i) 'initial position of vehicle i at lane k',
         s_critical(i,r) 'critical position points of vehicle i at time r',
         v_init(i) 'initial speed of vehicle i at lane k',
         veh_dynamics(i) 'indicator of vehicles to calculate the car-following distances',
         veh_ice(i) 'indicator of whether the vehicle is an ice or not';
*$gdxin _gams_py_gdb3
$if not set gdxincname $abort 'no include file name for data file provided'
$gdxin %gdxincname%
$load i r h tau d_0 r_dynamics r_critical_points s_init s_critical v_init veh_dynamics veh_ice
$gdxin

variable
         f 'cost function',
         f_inst_ice(i,r) 'the instantanuous fuel consumption rate',
         f_inst_ev(i,r) 'the instantanuous fuel consumption rate',
         f_position(i,r) 'position penalty',
         a(i,r) 'acceleration of vehicle i',
         v(i,r) 'speed of vehicle i on lane k at time step r',
         s(i,r) 'position of vehicle i on lane k at time step r';

equations
         cost 'cost function',
         fuel_consumption_ice(i,r) 'calculate fuel consumption rate',
         fuel_consumption_ice_2(i,r) 'calculate fuel consumption rate',
         fuel_consumption_ev(i,r) 'calculate ev energy consumption rate',
         fuel_consumption_ev_2(i,r) 'calculate ev energy consumption rate',
         dynamics_speed(i,r) 'dynamics of the speed',
         dynamics_position(i,r) 'lower bound of a vehicle position',
         car_following(i,r) 'car-following model',
         position_penalty(i,r) 'car-following model';

*cost..                                                                           f =e= sum(r, sum(i, f_inst_ice(i,r))) + sum(r$(r_critical_points(r) ne 0), sum(i, f_position(i,r)));
cost..                                                                           f =e= sum(r, sum(i, f_inst_ice(i,r))) + sum(r, sum(i, f_inst_ev(i,r))) + sum(r$(r_critical_points(r) ne 0), sum(i, power(s(i,r) - s_critical(i,r), 2)));
dynamics_speed(i,r)$(r_dynamics(r) ne 0)..                                       v(i,r) - v(i,r-1) =e= h*a(i,r-1);
dynamics_position(i,r)$(r_dynamics(r) ne 0)..                                    s(i,r) - s(i,r-1) =e= h*v(i,r-1) + a(i,r-1)*h*h/2;
*car_following(i,r)$(veh_dynamics(i) ne 0 and r_dynamics(r) ne 0)..               s(i,r) - s(i+1,r) =g= tau*v(i,r) + d_0;
car_following(i,r)$(veh_dynamics(i) ne 0 and r_dynamics(r) ne 0)..               s(i,r) - s(i+1,r) =g= tau*v(i,r);
*position_penalty(i,r)$(r_critical_points(r) ne 0)..                              f_position(i,r) =e= power(s(i,r) - s_critical(i,r), 2);
*car_following_2(i,r)$(veh_dynamics(i) ne 0)..                                    s(i,r) - s(i+1,r) =g= 5;
fuel_consumption_ev(i,r)$(veh_ice(i) ne 1)..                                     f_inst_ev(i,r) =e= (1266*a(i,r)*v(i,r) + 1266*9.8*0.006*v(i,r) + 1.3*power(v(i,r),3))/10000;
fuel_consumption_ev_2(i,r)$(veh_ice(i) ne 0)..                                   f_inst_ev(i,r) =e= 0;
fuel_consumption_ice(i,r)$(veh_ice(i) ne 0)..                                    f_inst_ice(i,r) =e= 0.2736 + 0.0599*v(i,r) + 0.3547*a(i,r) - 0.0058*power(v(i,r),2) + 0.0179*v(i,r)*a(i,r) + 0.0663*power(a(i,r),2) + 0.0002*power(v(i,r),3) + 0.002*power(v(i,r),2)*a(i,r) + 0.0245*v(i,r)*power(a(i,r),2) - 0.0489*power(a(i,r),3);
fuel_consumption_ice_2(i,r)$(veh_ice(i) ne 1)..                                  f_inst_ice(i,r) =e= 0;

a.up(i,r) = 4;
a.lo(i,r) = -5;
*a.l(i,r) = 0;
v.up(i,r) = 13;
v.lo(i,r) = -1;
*v.l(i,r) = v_init(i);
s.up(i,r) = 700;
s.lo(i,r) = -200;
v.fx(i,'1') = v_init(i);
s.fx(i,'1') = s_init(i);

model m /cost, dynamics_speed, dynamics_position, car_following, fuel_consumption_ice, fuel_consumption_ice_2, fuel_consumption_ev, fuel_consumption_ev_2/;

*option optcr=0;
*option minlp=BONMIN;
*m.optfile = 1;
solve m using nlp minimizing f;
model_status = m.Modelstat;
solver_status = m.solvestat;
