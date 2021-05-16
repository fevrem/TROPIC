
<img src="images/logo.png" width="300"> 

## Get Started

Follow these steps to run the 12-DOF spatial biped example:

- Run `TROPIC_add_path()` from the Matlab command line in the main TROPIC directory. 
- Go to `examples/spatial-12-dof-biped/` and open `main.m`
- Run `main.m` (this will overwrite the seed file with the newly optimized gait), or run each section and stop after the animation.


## Requested Features 

- [x] Add humanoid example


    Gait optimization on full-order dynamics of 20-DOF spatial biped robot: 
      
    <img src="examples/spatial-20-dof-biped/anim_biped_20_DOF.gif" width="500">


- [x] Add reaction wheel system (RWS) 

    *Flywheel* gait on planar five-link biped model

    <img src="examples/planar-8-dof-biped-flywheel/planar-8dof-biped-flywheel.gif" width="300">


- [ ] Add flat feet
- [ ] Add "get started" explanations


## Contributing to TROPIC

If you have improvements to TROPIC, send me a pull request! First, fork TROPIC’s repository into your own GitHub account and then push your changes into a branch on your fork. Once you believe your code is ready to be merged into TROPIC’s primary repository, open a pull request via the GitHub website. I will then review your code, which will also undergo CI tests before it is merged into TROPIC's primary repository.


## Credits

TROPIC was started by Martin Fevre in the Locomotion & Biomechanics Lab at the University of Notre Dame. Other people have since contributed and helped make TROPIC successful. Here’s a list of contributors:
* Paul Montgomery 


## Citation

To cite TROPIC, we suggest the following citation:

[[Link]](https://github.com/fevrem/TROPIC/blob/master/MF_PMW_JPS_IROS2020_TROPIC.pdf) M. Fevre, P. M. Wensing, and J. P. Schmiedeler, "Rapid Bipedal Gait Optimization in CasADi", in Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2020, pp. 3672-3678.
