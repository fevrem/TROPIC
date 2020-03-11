function [options] = IPOPToptions(obj)
% I suggest changing the memory allocation based on the 
% complexity/dimension of the mechanical system

arguments
    obj (1,1) NLP
end

options = struct;


options.ipopt.mu_strategy = 'adaptive';
options.ipopt.max_iter = 1000;
options.ipopt.tol = 1E-3;
options.ipopt.linear_solver = obj.Settings.LinearSolver;

options.ipopt.ma57_automatic_scaling = 'yes';
options.ipopt.linear_scaling_on_demand = 'no';
options.ipopt.ma57_pre_alloc = 2;

options.ipopt.recalc_y = 'yes';
options.ipopt.recalc_y_feas_tol = 1E-3;

options.ipopt.hessian_approximation = 'limited-memory';
options.ipopt.limited_memory_update_type = 'bfgs';
options.ipopt.limited_memory_max_history = 10;



%{


    ### Linear Solver ###

Linear solver used for step computations.
      Determines which linear algebra package is to be used for the solution of
      the augmented linear system (for obtaining the search directions). N




    COIN-OR documention 
    https://www.coin-or.org/Bonmin/option_pages/options_list_ipopt.html



    
    ### MA27 ###

    * ma27_liw_init_factor     1 <= (5) <  +inf
        Integer workspace memory for MA27.
        The initial integer workspace memory = liw_init_factor * memory required
        by unfactored system. Ipopt will increase the workspace size by
        meminc_factor if required.  This option is only available if Ipopt has
        been compiled with MA27.

    * ma27_la_init_factor       1 <= (5) <  +inf
        Real workspace memory for MA27.
        The initial real workspace memory = la_init_factor * memory required by
        unfactored system. Ipopt will increase the workspace size by
        meminc_factor if required.  This option is only available if  Ipopt has
        been compiled with MA27.

    --> I do not recommend changing other options

    MA27 is written in Fortran 77 -> doesn't have dynamic memory allocation.  
    Ipopt guesses beforehand how much memory MA27 requires (based on the number of nonzeros).
    The options ma27_liw_init_factor and ma27_la_init_factor determine this initial guess of 
    MA27's memory requirement. If during the factorization MA27 complains that it doesn't 
    have enough memory, the guesses are increased by some factor, and that factor is the 
    value of the option ma27_meminc_factor (default 10).


    
    ### MA57 ###

    * ma57_pre_alloc            1 <= (3) <  +inf
        Safety factor for work space memory allocation for the linear solver MA57.
        If 1 is chosen, the suggested amount of work space is used.  However,
        choosing a larger number might avoid reallocation if the suggest values
        do not suffice.  This option is only available if Ipopt has been compiled
        with MA57.

    --> I do not recommend changing other options


    ### Mumps ###

    * mumps_mem_percent         0 <= (1000) <  +inf
        Percentage increase in the estimated working space for MUMPS.
        In MUMPS when significant extra fill-in is caused by numerical pivoting,
        larger values of mumps_mem_percent may help use the workspace more
        efficiently.  On the other hand, if memory requirement are too large at
        the very beginning of the optimization, choosing a much smaller value for
        this option, such as 5, might reduce memory requirements.

    --> I do not recommend changing other options

%}






end