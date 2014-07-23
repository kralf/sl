{ // general numerical integration
double dt;

// integration runs at higher rate
dt = 1./(double)(motor_servo_rate)/n_integration;

for (i=1; i<=n_integration; ++i) {
  
  // integrate the simulation
  switch (integrate_method) {
  case INTEGRATE_RK:
    SL_IntegrateRK(joint_sim_state, &base_state, 
		   &base_orient, ucontact, endeff,dt,N_DOFS);
    break;
    
  case INTEGRATE_EULER:
    SL_IntegrateEuler(joint_sim_state, &base_state, 
		      &base_orient, ucontact, endeff,dt,N_DOFS,TRUE);
    break;
    
  default:
    printf("invalid integration method\n");
    
  }
  
}

}

