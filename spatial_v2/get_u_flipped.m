function u_flipped = get_u_flipped( u )


% R_swap & R_offset can be obtained by inspection
R_swap = [ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 ;
           0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 ;
           0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,-1 , 0 , 0 ;
           0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ,-1 , 0 , 0 , 0 ;
           0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 ;
           0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 ] ;

Ru_swap = R_swap(end-5:end,end-5:end);

u_flipped = Ru_swap*u;

end