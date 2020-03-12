function add_coordinate_sys(rbm)


% calculate size of arrows for coordinate system
maxAx = max( [ range(xlim) , range(ylim) , range(zlim) ] );

percent_of_fig = 0.10;

Larrow = percent_of_fig*maxAx;

origin_arrow = [0,0,0];

xLi = xlim;
yLi = ylim;


Anim.mArrow3(origin_arrow,origin_arrow+[Larrow,0,0],'color','b','stemWidth',Larrow/30,'tipWidth',Larrow/15);
Anim.mArrow3(origin_arrow,origin_arrow+[0,Larrow,0],'color','g','stemWidth',Larrow/30,'tipWidth',Larrow/15);
Anim.mArrow3(origin_arrow,origin_arrow+[0,0,Larrow],'color','r','stemWidth',Larrow/30,'tipWidth',Larrow/15);



        

end


