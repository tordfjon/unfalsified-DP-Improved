global surge_u_cost sway_u_cost yaw_u_cost delta rho

motion = 'surge';

switch motion
    case 'surge'
        time_hor = (surge_u_cost.count-1)*delta;
        time = 0:delta:time_hor;
        line = ones(1,surge_u_cost.count);
        for cont=1:4;
            fg1 = figure;
            
            % cont =4;
            
            subplot(4,1,1);
            plot(time,surge_u_cost.myj1(cont,:),time,surge_u_cost.temp1(cont,:));
            title(sprintf('Error contribution, controller: %3.0i \n Kp=%4.1f, Ki=%6.3f, Kd=%5.2f', ...
                cont,surge_u_cost.K(cont,1),surge_u_cost.K(cont,2),surge_u_cost.K(cont,3)))
            
            grid on
            
            subplot(4,1,2);
            plot(time,surge_u_cost.myj2(cont,:),time,surge_u_cost.temp2(cont,:));
            title(sprintf('Control contribution, controller: %3.0i',cont))
            grid on
            
            subplot(4,1,3);
            plot(time,surge_u_cost.myj3(cont,:),time,surge_u_cost.temp3(cont,:));
            title(sprintf('Reference contribution, controller: %3.0i',cont))
            grid on
            
            subplot(4,1,4);
            plot(time,surge_u_cost.myj(cont,:),time,rho*line);
            title(sprintf('Total cost, controller: %2i',cont))
            grid on
        end

    case 'sway'
        time_hor = (sway_u_cost.count-1)*delta;
        time = 0:delta:time_hor;
        line = ones(1,yaw_u_cost.count);
        for cont=1:4;
            fg1 = figure;
            
            % cont =4;
            
            subplot(4,1,1);
            plot(time,sway_u_cost.myj1(cont,:),time,sway_u_cost.temp1(cont,:));
            title(sprintf('Error contribution, controller: %3.0i \n Kp=%4.1f, Ki=%4.1f, Kd=%4.1f', ...
                cont,sway_u_cost.K(cont,1),sway_u_cost.K(cont,2),sway_u_cost.K(cont,3)))
            
            grid on
            
            subplot(4,1,2);
            plot(time,sway_u_cost.myj2(cont,:),time,sway_u_cost.temp2(cont,:));
            title(sprintf('Control contribution, controller: %3.0i',cont))
            grid on
            
            subplot(4,1,3);
            plot(time,sway_u_cost.myj3(cont,:),time,sway_u_cost.temp3(cont,:));
            title(sprintf('Reference contribution, controller: %3.0i',cont))
            grid on
            
            subplot(4,1,4);
            plot(time,sway_u_cost.myj(cont,:),time,rho*line);
            title(sprintf('Total cost, controller: %2i',cont))
            grid on
        end
        
        
        
    case 'yaw'
        time_hor = (yaw_u_cost.count-1)*delta;
        time = 0:delta:time_hor;
        line = ones(1,yaw_u_cost.count);
        for cont=1:4;
            fg1 = figure;
            
            % cont =4;
            
            subplot(4,1,1);
            plot(time,yaw_u_cost.myj1(cont,:),time,yaw_u_cost.temp1(cont,:));
            title(sprintf('Error contribution, controller: %3.0i \n Kp=%4.1f, Ki=%4.1f, Kd=%4.1f', ...
                cont,yaw_u_cost.K(cont,1),yaw_u_cost.K(cont,2),yaw_u_cost.K(cont,3)))
            
            grid on
            
            subplot(4,1,2);
            plot(time,yaw_u_cost.myj2(cont,:),time,yaw_u_cost.temp2(cont,:));
            title(sprintf('Control contribution, controller: %3.0i',cont))
            grid on
            
            subplot(4,1,3);
            plot(time,yaw_u_cost.myj3(cont,:),time,yaw_u_cost.temp3(cont,:));
            title(sprintf('Reference contribution, controller: %3.0i',cont))
            grid on
            
            subplot(4,1,4);
            plot(time,yaw_u_cost.myj(cont,:),time,rho*line);
            title(sprintf('Total cost, controller: %2i',cont))
            grid on
        end
        
    otherwise 
        %Do nothing
end