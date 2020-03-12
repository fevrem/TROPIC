function [ax,lh] = set_axis( x0 , rbm , anim_options , ax , lh )


    if isfield(anim_options,'axis')

        x = anim_options.axis.x;
        y = anim_options.axis.y;
        z = anim_options.axis.z;
        

        max_axis = max([range(x),range(y),range(z)]);

        ax.XLim = [mean(x) - max_axis/2, mean(x) + max_axis/2];
        ax.YLim = [mean(y) - max_axis/2, mean(y) + max_axis/2];
        ax.ZLim = [0, mean(z) + max_axis/2];
       

        
    else

        [self(1:rbm.model.NB).val] = numToCell(x0(1:rbm.model.NB));
        Pcom = p_com(self.val);

        xl = [-1 1] + Pcom(1);
        yl = [-1 1] + Pcom(2);
        zl = [-1 2];% + Pcom(3);
        
        if strcmpi(rbm.dynamics,'hybrid')
            zl(1) = 0;
        end
        
        % 3D
        ax.XLim = xl;
        ax.YLim = yl;
        ax.ZLim = zl;

    end
    
    
    if ~isempty(lh)
        lh.Position = [ ax.XLim(2)+1 , ax.YLim(1)+0*range(ax.YLim) , ax.ZLim(1)+0.7*range(ax.ZLim) ];
        %[ax.XLim(2)+1,ax.YLim(1)+0*range(ax.YLim) , ax.ZLim(1)+0.7*range(ax.ZLim)]
        
        %lh.Position(1) = sin(pi/180*ax.View(1))*lh.Position(1)
    end
        
        
 end
