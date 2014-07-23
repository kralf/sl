/*! \mainpage SL Core Libraries

    These files implement the Simulation Lab (SL) core libraries, which are the basis of 
    every SL simulation or robot controller.

*/

/*! 
    \defgroup SLcommon  
    
    A library of functions shared accross the different SL processes.

*/
/*! 
    \defgroup SLtask 

    The library that generates the process for managing user tasks.
*/
/*! 
    \defgroup SLmotor 
    
    The library that generates the process for the controller 
    and I/O to the robot/simulation.
*/
/*! 
    \defgroup SLvision 
    
    The library that generates the process managing visual input.
*/
/*! 
    \defgroup SLsimulation 

    The library that generates the process managing the physical simulation.
*/
/*! 
    \defgroup SLopenGL 
    
    The library that generates graphics visualizations with openGL.
*/
/*! 
    \defgroup SLparameterEstimation
    
    Compiles and executeable for Rigid Body Dynamics parameter estimation
*/
/*! 
    \defgroup SLskeletons
    
    Several files that require robot specific include files, usually about
    kinematics and dynamics parameters. These files have no content, but
    include a header file (with suffix *body.h), which has the actually
    programming and which includes the robot spedific header files.
*/
/*! 
    \defgroup SLros
    
    The library that generates ROS specific functionality
*/
