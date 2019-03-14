classdef cWorld
    properties (Constant)  
        world_size = 100;
        landmarks = [20.0, 20.0; 80.0, 80.0; 20.0, 80.0; 80.0, 20.0;]% 50.0 20.0; 50.0 80.0]; %uncommenst to add two more Landmarks
    end 
    methods     
        function plot(obj)  
            figure
            plot(obj.landmarks(:,1),obj.landmarks(:,2),'ko','MarkerFaceColor','k')
            xlim([0,obj.world_size]);
            ylim([0,obj.world_size]);
            set(gca,'Xtick',[0:10:obj.world_size]);
            set(gca,'Ytick',[0:10:obj.world_size]);           
            set(gca,'Xticklabels',{'0' '10' '20' '30' '40' '50' '60' '70' '80' '90' '100'});
            set(gca,'Yticklabels',{'0' '10' '20' '30' '40' '50' '60' '70' '80' '90' '100'});            
            set(gca,'FontSize',16);           
            xlabel('x');
            ylabel('y');
            title('robot world')         
        end      
    end  
end