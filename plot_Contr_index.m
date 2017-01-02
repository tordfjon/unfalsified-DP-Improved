%%   Revised 2016 by Vahid to make it work with new Matlab 2015
%%(Still needs further notification)
% This m-file opens all the simulink scope & changes the color & attributes of the scope that plots online all the 
% unfalsified controller & current controller indices. This scope is identified by its tag, when flag is 'change'.

% Tord: Function is called from StartFcn* callback function of the model.
% This is found in Model Properties -> Callbacks!! 
% Now commented out with flag 'open' and 'change'

function plot_Contr_index(flag)
switch flag
    
case ('open')  %% when count == 0 in unfalsification.m, all scopes are opened
    close all;  
    scop = find_system(bdroot,'BlockType','Scope');
    for( i = 1: length(scop))
        set_param(scop{i},'Open','on')
    end
    
case('change')        %% when count == 1 in unfalsification algorithm, attributes of lines of plots are changed
    disp('Vahid! modify this part (function named plot_Contr_index) later!')
    h = find_system(get_param(bdroot,'Handle'),'LookUnderMasks','all','Tag','cont_plot');
    set_param(h,'Open','on') %%% making Controller index scope the current scope
    set(0,'ShowHiddenHandles','On'); %%Making handle of simulink scopes on
    h=findobj(gca,'Type','Line');
%     for (b=1:length(h))
%          set(h(b),'Color','y');
%     end
%     set(h(size(h)),'Color','r','Linewidth',3);
%     set(gca,'YLim',[0 length(h)]) 
%     set(0,'ShowHiddenHandles','Off') ;
end


