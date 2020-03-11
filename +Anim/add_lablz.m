function [ax,fig] = add_lablz( ax , fig , axtitle , anim_options )

    [interpreter,linewidth,fontname,fontsize] = Anim.plt_settings();

    ax.FontSize = fontsize;
    ax.FontName = fontname;
    %ax.Interpreter = interpreter;

    ax.XLabel.String = 'x (m)';
    ax.XLabel.FontSize = fontsize;
    ax.XLabel.Interpreter = interpreter;
    ax.XLabel.FontName = fontname;

    ax.YLabel.String = 'y (m)';
    ax.YLabel.FontSize = fontsize;
    ax.YLabel.Interpreter = interpreter; 
    ax.YLabel.FontName = fontname;

    ax.ZLabel.String = 'z (m)';
    ax.ZLabel.FontSize = fontsize;
    ax.ZLabel.Interpreter = interpreter;
    ax.ZLabel.FontName = fontname;

    if length(anim_options.views) == 1
        %title should be in fig name
        
        fig.Name = axtitle;

    else

        ax.Title.String = axtitle;
        %ax.Title.Interpreter = interpreter;
        ax.Title.FontSize = fontsize;
        ax.Title.FontWeight = 'bold';
        ax.Title.FontName = fontname;
    end
    
    %ax.XGrid = 'on';
    %ax.YGrid = 'on';
    %ax.ZGrid = 'on';






end