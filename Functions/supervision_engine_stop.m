function stop = supervision_engine_stop(input)

T_EM = input(1);
T_MGB = input(2);
state_CE = input(3);

stop = ~((~((T_MGB>0)&(state_CE==0))|(T_EM==T_MGB))&(~(T_MGB<=0)|(T_EM>=(T_MGB))));
stop = double(stop);