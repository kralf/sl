//-------------------------------------------------------------------------
// check for clmcplot_mode or playback_mode
if (clmcplot_mode || playback_mode || userGraphics_mode) {
  
  // check for graphics update
  if (clmcplot_mode)
    clmcplotUpdateState();
  else if (playback_mode)
    playbackUpdateState();
  else if (userGraphics_mode) {
    checkForUserGraphics();
    userGraphicsUpdateState();
  }
  
  linkInformation(joint_sim_state,&base_state,&base_orient,endeff,
		  joint_cog_mpos_sim,joint_axis_pos_sim,joint_origin_pos_sim,
		  link_pos_sim,Alink_sim,Adof_sim);


  // compute COG
  for (i=0; i<=N_DOFS; ++i)
    joint_cog_mpos[i] = joint_cog_mpos_sim[i];
  for (i=0; i<=N_LINKS; ++i)
    link_pos[i] = link_pos_sim[i];

  compute_cog();
  
  // update the graphics
  glutPostRedisplayAll();
  
}

//-------------------------------------------------------------------------
// check for pause and stand-alone mode
if (pause_flag || stand_alone_flag) {
  receiveOscilloscopeData(); // need to keep an emptying the buffer
  usleep(10000);
  return;
}


#ifdef __XENO__
  // we want to be in real-time mode here
  rt_task_set_mode(0,T_PRIMARY,NULL);
#endif
//-------------------------------------------------------------------------
// get 60Hz semaphore
if (semTake(sm_openGL_servo_sem,WAIT_FOREVER) == ERROR) {
  printf("semTake Time Out -- Servo Terminated\n");
  exit(-1);
}


//-------------------------------------------------------------------------
// advance the time (note that the servo time is copied from the time stamp
// of the receive_sim_state function)
++openGL_servo_calls;

//-------------------------------------------------------------------------
// read from shared memory
receive_sim_state();
receive_misc_sensors();
receive_contacts();
checkForUserGraphics();
checkForMessages();
#ifdef __XENO__
  // we want to be in secondary mode here
  rt_task_set_mode(T_PRIMARY,0,NULL);
#endif
updateComet();
receiveOscilloscopeData();

//-------------------------------------------------------------------------
// compute link info
linkInformation(joint_sim_state,&base_state,&base_orient,endeff,
		joint_cog_mpos_sim,joint_axis_pos_sim,joint_origin_pos_sim,
		link_pos_sim,Alink_sim,Adof_sim);

// compute COG
for (i=0; i<=N_DOFS; ++i)
  joint_cog_mpos[i] = joint_cog_mpos_sim[i];
for (i=0; i<=N_LINKS; ++i)
  link_pos[i] = link_pos_sim[i];

compute_cog();


