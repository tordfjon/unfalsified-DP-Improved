%%   Revised 2016 by Vahid to make it work with new Matlab 2015
% This m-file is called from the initilization command of the masked subsystem "Unfalsification Algorithm". It sets the port 
% size of the mux & demux blocks inside the subsystem to the number of initial candidate controller whenever the user 
% changes the set of initial condidate controller from the parameter entry block of the subsystem.

% It determines the total number of Candidate Controllers (whenever the user changes the set of initial condidate controller). 
% Then it sets the number of input & outputs of the mux & demux blocks inside the 'Unfalsification Algorithm' sub system.
% The MUX & DEMUX blocks are identified by the tag set in the block properties of the respective blocks

% Demux & mux blocks are identified by their tags
surge_demux_1 = char(find_system(gcb,'LookUnderMasks','all','Tag','surge_demux_1'));
surge_demux_2 = char(find_system(gcb,'LookUnderMasks','all','Tag','surge_demux_2'));
surge_mux_1 = char(find_system(gcb,'LookUnderMasks','all','Tag','surge_mux_1'));
surge_mux_2 = char(find_system(gcb,'LookUnderMasks','all','Tag','surge_mux_2'));

% Number of initial Candidate Controllers are determined:
[k,total_new] = form_K(C_p, C_i, C_d); % initial number of elements of the candidate controllers
total_old = str2num(get_param(surge_demux_1,'Outputs'));


if (total_old ~= total_new)
    surge_unfalsification  = char(find_system(gcb,'LookUnderMasks','all','Tag','surge_unfalsification'));  
    surge_per_monitor =  char(find_system(gcb,'LookUnderMasks','all','Tag','surge_per_monitor'));
      %% deleting existing lines between mux & demux blocks      
      for(i=1:total_old)      
          d_1 = sprintf('%s/%d',get_param(surge_demux_1,'Name'),i);
          d_2 = sprintf('%s/%d',get_param(surge_demux_2,'Name'),i);
          m_1 = sprintf('%s/%d',get_param(surge_mux_1,'Name'),i);
          m_2 = sprintf('%s/%d',get_param(surge_mux_2,'Name'),i);
          delete_line(surge_unfalsification,d_1,m_1);
          delete_line(surge_per_monitor,d_2,m_2);
      end

      %% Setting the port size of mux & demux blocks to the number of initial candidate controllers
      total_new_char = num2str(total_new);
      set_param(surge_demux_1,'Outputs',total_new_char); %% setting new port dimensions
      set_param(surge_demux_2,'Outputs',total_new_char);
      set_param(surge_mux_1,'Inputs',total_new_char);
      set_param(surge_mux_2,'Inputs',total_new_char);

      %% adding new lines between the mux & demux blocks
      for(i=1:total_new)
          d_1 = sprintf('%s/%d',get_param(surge_demux_1,'Name'),i);
          d_2 = sprintf('%s/%d',get_param(surge_demux_2,'Name'),i);
          m_1 = sprintf('%s/%d',get_param(surge_mux_1,'Name'),i);
          m_2 = sprintf('%s/%d',get_param(surge_mux_2,'Name'),i);
          add_line(gcb,d_1,m_1);
          add_line(surge_per_monitor,d_2,m_2);
      end
end