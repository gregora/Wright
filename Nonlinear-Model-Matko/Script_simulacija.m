%% Script_simulacija

out = sim(Model_name);
figure
plot(out.t,atan2(out.Result_uvw(:,3),out.Result_uvw(:,1))*180/pi)
xlabel('t [s]'), ylabel('\alpha [deg]')
%%
figure
plot(out.t,asin(out.Result_uvw(:,2).'./(sqrt(sum((out.Result_uvw.^2).'))))*180/pi)
xlabel('t [s]'), ylabel('\beta [deg]')
%%
figure
plot(out.t,out.Result_Euler(:,1))
xlabel('t [s]'), ylabel('\phi [deg]')
%%
figure
plot(out.t,out.Result_Euler(:,2))
xlabel('t [s]'), ylabel('\theta [deg]')
%%
figure
plot(out.t,-out.Result_position(:,3))
xlabel('t [s]'), ylabel('h [m]')
%%
figure
plot(out.t,(sqrt(sum((out.Result_uvw.^2).'))))
xlabel('t [s]'), ylabel('TAS [m/s]')
