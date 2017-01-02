function [ u_tot , u_fast , u_slow ] = controller_out(  Ais, Bis, Cis, Dis, ...
                                            Aif, Bif, Cif, Dif, ...
                                            Ajs, Bjs, Cjs, Djs, ...
                                            Ajf, Bjf, Cjf, Djf, ...
                                            r , y , u_min , new_slow_state , new_fast_state)

    u_slow  = Cjs*new_slow_state + Djs*(r-y);
    u_fast  = Cjf*new_fast_state + Djf*(y);
    u_tot   = u_slow + u_fast;
                                    


end

