/*!=============================================================================
  ==============================================================================

  \ingroup SLskeletons

  \file    SL_kinematics.c

  \author  Stefan Schaal & Ludovic Righetti
  \date    2011

  ==============================================================================
  \remarks

  all kinematics functions

  ============================================================================*/

// the system headers
#include "SL_system_headers.h"

#ifndef max
#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif


/* private includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "mdefs.h"
#include "SL_kinematics.h"
#include "utility_macros.h"

/* global variables */

/* local variables */

/* global functions */

/* local functions */

/* external variables */

/*!*****************************************************************************
 *******************************************************************************
\note  init_kinematics
\date  June 1999
   
\remarks 

        initializes the kinematics variables

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
void
init_kinematics(void)

{
  int i;

  link_pos             = my_matrix(0,N_LINKS,1,3);
  link_pos_des         = my_matrix(0,N_LINKS,1,3);
  link_pos_sim         = my_matrix(0,N_LINKS,1,3);
  joint_cog_mpos       = my_matrix(0,N_DOFS,1,3);
  joint_cog_mpos_des   = my_matrix(0,N_DOFS,1,3);
  joint_cog_mpos_sim   = my_matrix(0,N_DOFS,1,3);
  joint_origin_pos     = my_matrix(0,N_DOFS,1,3);
  joint_origin_pos_des = my_matrix(0,N_DOFS,1,3);
  joint_origin_pos_sim = my_matrix(0,N_DOFS,1,3);
  joint_axis_pos       = my_matrix(0,N_DOFS,1,3);
  joint_axis_pos_des   = my_matrix(0,N_DOFS,1,3);
  joint_axis_pos_sim   = my_matrix(0,N_DOFS,1,3);
 
  J                = my_matrix(1,N_ENDEFFS*6,1,N_DOFS);
  dJdt             = my_matrix(1,N_ENDEFFS*6,1,N_DOFS);
  Jdes             = my_matrix(1,N_ENDEFFS*6,1,N_DOFS);
  Jcog             = my_matrix(1,2*N_CART,1,N_DOFS);
  Jcogdes          = my_matrix(1,2*N_CART,1,N_DOFS);

  Jbase            = my_matrix(1,N_ENDEFFS*6,1,6);
  dJbasedt         = my_matrix(1,N_ENDEFFS*6,1,6);
  Jbasedes         = my_matrix(1,N_ENDEFFS*6,1,6);
  Jbasecog         = my_matrix(1,2*N_CART,1,2*N_CART);
  Jbasecogdes      = my_matrix(1,2*N_CART,1,2*N_CART);

  for (i=0; i<=N_LINKS; ++i) {
    Alink[i]     = my_matrix(1,4,1,4);
    Alink_des[i] = my_matrix(1,4,1,4);
    Alink_sim[i] = my_matrix(1,4,1,4);
  }

  for (i=0; i<=N_DOFS; ++i) {
    Adof[i]     = my_matrix(1,4,1,4);
    Adof_des[i] = my_matrix(1,4,1,4);
    Adof_sim[i] = my_matrix(1,4,1,4);
  }

  // initialize indicators for prismatic joints
  for (i=0; i<=N_DOFS; ++i)
    prismatic_joint_flag[i] = FALSE;

#include "Prismatic_Joints.h"  

}

/*!*****************************************************************************
 *******************************************************************************
\note  linkInformation
\date  March 2005
   
\remarks 

        computes the m*cog, rotation axis, and local coord.sys. orgin for
        every link. This information can be used to compute the COG and
        COG jacobians, assuming the mass and center of mass parameters are
        properly identified.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
L
 \param[in]     state   : the state containing th, thd, thdd, and receiving the
                  appropriate u
 \param[in]     basec   : the position state of the base
 \param[in]     baseo   : the orientational state of the base
 \param[in]     endeff  : the endeffector parameters
 \param[out]    Xmcog   : array of mass*cog vectors 
 \param[out]    Xaxis   : array of rotation axes
 \param[out]    Xorigin : array of coord.sys. origin vectors
 \param[out]    Xlink   : array of link position
 \param[out]    Ahmat   : homogeneous transformation matrices of each link
 \param[out]    Ahmatdof: homogeneous transformation matrices of each dof

 ******************************************************************************/
void 
linkInformation(SL_Jstate *state,SL_Cstate *basec,
		SL_quat *baseo, SL_endeff *eff, 
		double **Xmcog, double **Xaxis, double **Xorigin, double **Xlink,
		double ***Ahmat, double ***Ahmatdof)

{

#include "LInfo_declare.h"

#include "LInfo_math.h"

}

/*!*****************************************************************************
 *******************************************************************************
\note  linkInformationDes
\date  March 2005
   
\remarks 

        computes the m*cog, rotation axis, and local coord.sys. orgin for
        every link. This information can be used to compute the COG and
        COG jacobians, assuming the mass and center of mass parameters are
        properly identified.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     state   : the state containing th, thd, thdd, and receiving the
                  appropriate u
 \param[in]     basec   : the position state of the base
 \param[in]     baseo   : the orientational state of the base
 \param[in]     endeff  : the endeffector parameters
 \param[out]    Xmcog   : array of mass*cog vectors 
 \param[out]    Xaxis   : array of rotation axes
 \param[out]    Xorigin : array of coord.sys. origin vectors
 \param[out]    Xlink   : array of link position
 \param[out]    Ahmat   : homogeneous transformation matrices of each link
 \param[out]    Ahmatdof: homogeneous transformation matrices of each dof

 ******************************************************************************/
void 
linkInformationDes(SL_DJstate *state,SL_Cstate *basec,
		   SL_quat *baseo, SL_endeff *eff, 
		   double **Xmcog, double **Xaxis, double **Xorigin, 
		   double **Xlink, double ***Ahmat, double ***Ahmatdof)

{

#include "LInfo_declare.h"
  
#include "LInfo_math.h"

}
  
/*!*****************************************************************************
 *******************************************************************************
\note  jacobian
\date  June 1999
   
\remarks 

        computes the jacobian

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     lp      : the link positions
 \param[in]     jop     : joint origin positions
 \param[in]     jap     : joint axix unit vectors
 \param[out]    Jac     : the jacobian

 ******************************************************************************/
void 
jacobian(Matrix lp, Matrix jop, Matrix jap, Matrix Jac)

{
  int i,j,r;
  double c[2*N_CART+1];
#include "GJac_declare.h"

#include "GJac_math.h"

  /* the Jlist variable generated by the math files contains the 
     the indicators which joints contribute to each endeffector of
     the jacobian. Now, this information is used to compute the
     geometric jacobian */

  for (i=1; i<=n_endeffs; ++i) {
    for (j=1; j<=n_dofs; ++j) {
      if ( Jlist[i][j] != 0 ) {
	if (prismatic_joint_flag[j]) {
	  prismaticGJacColumn( lp[link2endeffmap[i]],
			      jop[j],
			      jap[j],
			      c );
	} else {
	  revoluteGJacColumn( lp[link2endeffmap[i]],
			      jop[j],
			      jap[j],
			      c );
	}
	for (r=1; r<=2*N_CART; ++r) 
	  Jac[(i-1)*6+r][j] = c[r];
      }
    }
  }

}
  
/*!*****************************************************************************
 *******************************************************************************
\note  genJacobian
\date  August 2012
   
\remarks 

        computes the jacobian for an arbitrary point attached to a link

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     point   : position of the point
 \param[in]     link    : id of the link the point is attached to
 \param[in]     jop     : joint origin positions
 \param[in]     jap     : joint axis unit vectors
 \param[out]    Jac     : the jacobian

 ******************************************************************************/
void 
genJacobian(Vector point, int link, Matrix jop, Matrix jap, Matrix J)
{
  int i,j,r;
  double c[2*N_CART+1];

#include "Contact_GJac_declare.h"
#include "Contact_GJac_math.h"

  /* the Jlist variable generated by the math files contains the 
     the indicators which joints contribute to each link position
     in the jacobian. Now, this information is used to compute the
     geometric jacobian */

  for (j=1; j<=n_dofs; ++j) {
    if ( Jlist[link][j] != 0 ) {
      if (prismatic_joint_flag[j]) {
        prismaticGJacColumn( point,
                            jop[j],
                            jap[j],
                            c );
      } else {
        revoluteGJacColumn( point,
                            jop[j],
                            jap[j],
                            c );
      }
      for (r=1; r<=2*N_CART; ++r) 
        J[r][j] = c[r];
    }
  }
}

/*!*****************************************************************************
 *******************************************************************************
\note  baseJacobian
\date  June 1999
   
\remarks 

        computes the jacobian of the base coordinates w.r.t to all endeffectors

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     lp      : the link positions
 \param[in]     jop     : joint origin positions
 \param[in]     jap     : joint axix unit vectors
 \param[out]    Jb      : the jacobian

 ******************************************************************************/
void 
baseJacobian(Matrix lp, Matrix jop, Matrix jap, Matrix Jb)

{
  int i,j,r;
  double c[2*N_CART+1];
  
  // the base Jacobian is almost an identity matrix, except for the
  // upper right quadrant
  
  for (i=1; i<=n_endeffs; ++i) {
    for (r=1; r<=N_CART; ++r) {
      Jb[(i-1)*6+r][r]        = 1.0;
      Jb[(i-1)*6+r+3][r+3]    = 1.0;
    }
    for (j=1; j<=N_CART; ++j) {
      revoluteGJacColumn( lp[link2endeffmap[i]],
			  base_state.x,
			  Jb[j], // this is a equivalent to a unit vector
			  c );
      for (r=1; r<=N_CART; ++r) {
	Jb[(i-1)*6+r][N_CART+j] = c[r];
      }
    }
  }
  
}


/*!*****************************************************************************
 *******************************************************************************
\note  inverseKinematics
\date  Feb 2011
   
\remarks 

        computes the inverse kinematics based on the pseudo-inverse
        with optimization, using a robust SVD-based inversion, and using
	velocity reduction towards the end of the workspace. Changes are
	based on Ludovic Righetti's version of IK.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] state     : the state of the robot (given as a desired state)
 \param[in]     endeff  : the endeffector parameters
 \param[in]     rest    : the optimization posture
 \param[in]     cart    : the cartesian state (pos & orientations in a matrix)
 \param[in]     status  : which rows to use from the Jacobian
 \param[in]     dt      : the integration time step     

the function updates the state by adding the appropriate joint velocities
and by integrating the state forward with dt

The function returns the condition number of the inverted matrix. Normally,
condition number above 5000 become a bit critical. The SVD clips at a condition
number of about 5000.

 ******************************************************************************/
double
inverseKinematics(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
		  Vector cart, iVector status, double dt)
{
  return inverseKinematicsClip(state, eff, rest, cart, status, dt, 0.0, 0.0);
}

/*!*****************************************************************************
 *******************************************************************************
\note  inverseKinematicsClip
\date  Feb 2011
   
\remarks 

       the same as inverseKinematics but with threshold for max joint 
       velocities

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out] state    : the state of the robot (given as a desired state)
 \param[in]     endeff   : the endeffector parameters
 \param[in]     rest     : the optimization posture
 \param[in]     cart     : the cartesian state (pos & orientations in a matrix)
 \param[in]     status   : which rows to use from the Jacobian
 \param[in]     dt       : the integration time step     
 \param[in]     max_rev  : max velocity for revolute joints  (0.0 to ignore)
 \param[in]     max_pris : max velocity for prismatic joints (0.0 to ignore)

 for return values, see inverseKinematics()

 ******************************************************************************/
double
inverseKinematicsClip(SL_DJstate *state, SL_endeff *eff, SL_OJstate *rest,
		      Vector cart, iVector status, double dt, double max_rev,
		      double max_pris)
{
  
  int            i,j,n;
  int            count;
  MY_MATRIX(Jac,1,6*N_ENDEFFS,1,N_DOFS);
  MY_MATRIX(Jreal,1,6*N_ENDEFFS,1,N_DOFS);
  MY_IVECTOR(ind,1,6*N_ENDEFFS);
  MY_MATRIX(B,1,N_DOFS,1,6*N_ENDEFFS);
  MY_MATRIX(O,1,N_DOFS,1,N_DOFS);
  MY_MATRIX(local_link_pos_des,0,N_LINKS,1,3);
  MY_MATRIX(local_joint_cog_mpos_des,0,N_DOFS,1,3);
  MY_MATRIX(local_joint_origin_pos_des,0,N_DOFS,1,3);
  MY_MATRIX(local_joint_axis_pos_des,0,N_DOFS,1,3);
  MY_MATRIX_ARRAY(local_Alink_des,1,4,1,4,N_LINKS+1);
  MY_MATRIX_ARRAY(local_Adof_des,1,4,1,4,N_DOFS+1);
  double         ralpha = 2.0;
  double         condnr;
  double         condnr_cutoff = 70.0;  // this corresponds to condnr_cutoff^2 in invere space

  /* compute the Jacobian */
  linkInformationDes(state,&base_state,&base_orient,eff,
		     local_joint_cog_mpos_des,
		     local_joint_axis_pos_des,
		     local_joint_origin_pos_des,
		     local_link_pos_des,
		     local_Alink_des,
		     local_Adof_des);

  jacobian(local_link_pos_des,local_joint_origin_pos_des,local_joint_axis_pos_des,Jac);

  /* how many contrained cartesian DOFs do we have? */
  count = 0;
  for (i=1; i<=6*N_ENDEFFS; ++i) {
    if (status[i]) {
      ++count;
      ind[count] = i;
    }
  }

  /* build the pseudo-inverse according to the status information using SVD */

  // the Jacobian that is actually needed, i.e., only the constraint rows
  mat_zero(Jreal);
  for (i=1; i<=count; ++i)
    for (j=1; j<=N_DOFS; ++j)
      Jreal[i][j] = Jac[ind[i]][j];


  // inversion with SVD with damping
  MY_MATRIX(M,1,count,1,count);
  MY_MATRIX(P,1,count,1,count);
  MY_MATRIX(U,1,count,1,N_DOFS);
  MY_MATRIX(V,1,N_DOFS,1,N_DOFS);
  MY_VECTOR(s,1,N_DOFS);
  MY_VECTOR(si,1,N_DOFS);

  mat_equal_size(Jreal,count,N_DOFS,U);
  my_svdcmp(U,count,N_DOFS,s,V);

  //printf("count=%d\n",count);
  //print_mat("V",V);getchar();

  // regularize if the condition number gets too large -- after the cutoff, we decay
  // the inverse of the singular value in a smooth way to zero
  for (i=1; i<=count; ++i)
    if (s[1]/(s[i]+1.e-10) > condnr_cutoff) {
      double sc = s[1]/condnr_cutoff;
      si[i] = -2./(sqr(sc)*sc)*sqr(s[i])+3./sqr(sc)*s[i];
      //si[i] = sc;
    } else {
      si[i] = 1./s[i];
    }

  condnr = s[1]/(s[count]+1.e-10);

  //  P=inv(U*S*V'*V*S'*U')=U*inv(S*S')*U' is the inverse
  for (i=1; i<=count; ++i) 
    for (j=1; j<=count; ++j)
      M[i][j] = U[j][i]*sqr(si[i]);

  mat_mult_size(U,count,count,M,count,count,P);
  
  /* build the B matrix, i.e., the pseudo-inverse */
  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=count; ++j) {
      B[i][j]=0.0;
      for (n=1; n<=count; ++n) {
	B[i][j] += Jac[ind[n]][i] * P[n][j];
      }
    }
  }

  /* this provides the first part of the optimized pseudo inverse */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].thd = 0;
    for (j=1; j<=count; ++j) {
      state[i].thd += B[i][j] * cart[ind[j]];
    }
  }

  /* the optimization part: we use the unrecularized Null space such that we don't
     suddently pop up new null space dimensions when the inversion is ill conditioned */
  for (i=1; i<=N_DOFS; ++i) {
    for (j=i; j<=N_DOFS; ++j) {
      O[i][j] = 0.0;
      for (n=count+1; n<=N_DOFS; ++n)
	O[i][j] += V[i][n] * V[j][n];
      O[j][i] = O[i][j];
    }
  }

  /* add the optimization part to the velocities */
  for (i=1; i<=N_DOFS; ++i) {
    for (j=1; j<=N_DOFS; ++j) {
      state[i].thd += ralpha* rest[j].w * (rest[j].th - state[j].th) * O[i][j];
    }
  }

  /* reduce the joint velocities if we get too close to the end of range of 
     motion of a DOF */
  for (i=1; i<=N_DOFS; ++i) {
    double max_thd, min_thd;
    double max_thd_now;
    double min_thd_now;

    if (prismatic_joint_flag[i] && max_pris == 0.0)
      continue;

    if (!prismatic_joint_flag[i] && max_rev == 0.0)
      continue;

    
    if (prismatic_joint_flag[i]) {

      max_thd =  max_pris*max(0.0,tanh(-20.0*(state[i].th-joint_range[i][MAX_THETA])));
      min_thd = -max_pris*max(0.0,tanh( 20.0*(state[i].th-joint_range[i][MIN_THETA])));

    } else {

      max_thd =  max_rev*max(0.0,tanh(-20.0*(state[i].th-joint_range[i][MAX_THETA])));
      min_thd = -max_rev*max(0.0,tanh( 20.0*(state[i].th-joint_range[i][MIN_THETA])));

    }

    if (state[i].thd > max_thd)
      state[i].thd = max_thd;
    
    if (state[i].thd < min_thd)
      state[i].thd = min_thd;
  }
  
  /* integrate forward */
  for (i=1; i<=N_DOFS; ++i) {
    state[i].th += state[i].thd * dt;
  }

  return condnr;

}

/*!*****************************************************************************
 *******************************************************************************
\note  computeLinkVelocity
\date  March 2006
   
\remarks 

        Computes the velocity of a particular link in world coordinates

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     lID    : the ID of the link
 \param[in]     lp     : the link positions
 \param[in]     jop    : joint origin positions
 \param[in]     jap    : joint axix unit vectors
 \param[in]     js     : joint state
 \param[out]    v      : velocity vector

 ******************************************************************************/
void 
computeLinkVelocity(int lID, Matrix lp, Matrix jop, Matrix jap, 
		    SL_Jstate *js, double *v)
{


  // this is just a special case for computeLinkVelocityPoint() where the
  // point coincides with the link position

  computeLinkVelocityPoint(lID, lp[lID], lp, jop, jap,js,v);

}

/*!*****************************************************************************
 *******************************************************************************
\note  computeLinkVelocityPoint
\date  March 2006
   
\remarks 

        Computes the velocity of a particular point that is fixed in
        a particular llink in world coordinates.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     lID    : the ID of the link
 \param[in]     point  : the point belonging to link lID in world coordinates
 \param[in]     lp     : the link positions
 \param[in]     jop    : joint origin positions
 \param[in]     jap    : joint axix unit vectors
 \param[in]     js     : joint state
 \param[out]    v      : velocity vector

 ******************************************************************************/
void 
computeLinkVelocityPoint(int lID, double *point, Matrix lp, Matrix jop, Matrix jap, 
			 SL_Jstate *js, double *v)
{
  int i,j,r;
  double c[2*N_CART+1];
  MY_MATRIX(Jlink,1,N_CART,1,n_dofs);
  MY_MATRIX(Jlinkbase,1,N_CART,1,2*N_CART);

#include "Contact_GJac_declare.h"
#include "Contact_GJac_math.h"

  /* the Jlist variable generated by the math files contains the 
     the indicators which joints contribute to each link position
     in the jacobian. Now, this information is used to compute the
     geometric jacobian */
  
  for (j=1; j<=n_dofs; ++j) {
    if ( Jlist[lID][j] != 0 ) {
      if (prismatic_joint_flag[j]) {
	prismaticGJacColumn( point,
			     jop[j],
			     jap[j],
			     c );
      } else {
	revoluteGJacColumn( point,
			    jop[j],
			    jap[j],
			    c );
      }
      for (r=1; r<=N_CART; ++r) 
	Jlink[r][j] = c[r];
    }
  }

  // next the base Jacobian
  // the base Jacobian is almost an identity matrix, except for the
  // upper right quadrant
  
  for (r=1; r<=N_CART; ++r) {
    Jlinkbase[r][r]        = 1.0;
  }
  for (j=1; j<=N_CART; ++j) {
    revoluteGJacColumn( point,
			base_state.x,
			Jlinkbase[j], // this is a equivalent to a unit vector
			c );
    for (r=1; r<=N_CART; ++r) {
      Jlinkbase[r][N_CART+j] = c[r];
    }
  }

  // compute the link velocity from Jlink and the base jacobian
  for (i=1; i<=N_CART; ++i) {

    v[i]     = 0.0;

    /* contributations from the joints */
    for (r=1; r<=n_dofs; ++r) {
      v[i] += Jlink[i][r] * js[r].thd;
    }

    /* contributations from the base */
    for (r=1; r<=N_CART; ++r) {
      v[i] += Jlinkbase[i][r] * base_state.xd[r];
      v[i] += Jlinkbase[i][3+r] * base_orient.ad[r];
    }
    
  }

}

/*!*****************************************************************************
 *******************************************************************************
\note  computeConstraintJacobian
\date  July 2009
   
\remarks 

   from the constraint information in the endeff structure, the constraint
   Jacobian for the floating base dynamics is computed

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     state   : the joints state
 \param[in]     basec   : cartesin info of base
 \param[in]     baseo   : orientation info of base
 \param[in]     eff     : endeffector info
 \param[out]    Jc      : constraint Jacobian
 \param[out]    nr      : number of rows in Jc
 \param[out]    nc      : number of columns in Jc

     note: memory for Jc needs to be provided as N_ENDEFFS*6 x N_DOFS+6


 ******************************************************************************/
void 
computeConstraintJacobian(SL_Jstate *state,SL_Cstate *basec,
			  SL_quat *baseo, SL_endeff *eff, 
			  Matrix Jc, int *nr, int *nc)
{
  int i,j,m,n;
  int count;
  MY_MATRIX(Xmcog,0,N_DOFS,1,3);
  MY_MATRIX(Xaxis,0,N_DOFS,1,3);
  MY_MATRIX(Xorigin,0,N_DOFS,1,3);
  MY_MATRIX(Xlink,0,N_LINKS,1,3);
  MY_MATRIX_ARRAY(Ahmat,1,4,1,4,N_LINKS);
  MY_MATRIX_ARRAY(Ahmatdof,1,4,1,4,N_DOFS);
  MY_MATRIX(Ec,1,(N_CART*2),1,(N_CART*2));
  MY_MATRIX(Jac,1,(N_ENDEFFS*2*N_CART),1,N_DOFS);
  
  mat_eye(Ec);
  
  // compute the link information for this state
  linkInformation(state, basec, baseo, eff, 
		  Xmcog, Xaxis, Xorigin, Xlink, Ahmat, Ahmatdof);
  
  // compute Jacobian
  jacobian(Xlink, Xorigin, Xaxis, Jac);

  // create the Jc matrix
  count = 0;
  for (i=1; i<=N_ENDEFFS; ++i) {

    // compute the Jacobian component due to the base: these results
    // come from cross productcs of the unit vectors of the base coordinate
    // with the endeffector-base position.
    Ec[_Y_][N_CART+_X_] = -(Xlink[link2endeffmap[i]][_Z_] - basec->x[_Z_]);
    Ec[_Z_][N_CART+_X_] =   Xlink[link2endeffmap[i]][_Y_] - basec->x[_Y_];

    Ec[_X_][N_CART+_Y_] =   Xlink[link2endeffmap[i]][_Z_] - basec->x[_Z_];
    Ec[_Z_][N_CART+_Y_] = -(Xlink[link2endeffmap[i]][_X_] - basec->x[_X_]);

    Ec[_X_][N_CART+_Z_] = -(Xlink[link2endeffmap[i]][_Y_] - basec->x[_Y_]);
    Ec[_Y_][N_CART+_Z_] =   Xlink[link2endeffmap[i]][_X_] - basec->x[_X_];

    for (j=1; j<=2*N_CART; ++j) {

      if (eff[i].c[j]) {

	++count;

	for (m=1; m<=N_DOFS; ++m)
	  (Jc)[count][m] = Jac[(i-1)*2*N_CART+j][m];

	for (m=1; m<=2*N_CART; ++m)
	  (Jc)[count][N_DOFS+m] = Ec[j][m];	  

      }

    }
  }

  *nr = count;
  *nc = 2*N_CART+N_DOFS;

}
 

/*!*****************************************************************************
 *******************************************************************************
\note  computeQR
\date  Nov. 2008
\remarks 

          computes the QR decomposition of a Jacobian using SVD


 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

               nf = N_DOFS+6 : full state dim
               nu = nf - nr  : unconstraint space dim

 \param[in]     Jc : the Jacobian
 \param[in]     nr : number of rows in Jacobian
 \param[in]     nc : number of columns in Jacobian
 \param[out]    Q : Q matrix (needs to be nf x nf)
 \param[out]    Qu : the part of Q that is the basis of unconstraint space (nf x nu)
 \param[out]    R : R matrix (needs to be at least nr x nr)
 \param[out]    sv : singular values (needs to be nf)

 ******************************************************************************/
void 
computeQR(Matrix Jc, int nr, int nc, Matrix Q, Matrix Qu, Matrix R, Vector sv)

{
  int i,j,n,m;
  int nf = N_DOFS+6;          // DOFs of full floating base space
  int nu = N_DOFS+6 - nr;     // number of unconstraint DOFs
  double aux;

  MY_MATRIX(U,1,nr,1,nf);
  MY_MATRIX(Su,1,nu,1,nf);

  // compute an orthonormal basis of the constraint Jacobian  with SVD
  mat_equal_size(Jc,nr,nf,U);
  my_svdcmp(U, nr, nf, sv, Q);

  // sort the columns of U and Q accoring to w coefficients
  for (j=1; j<=nf; ++j)
    for (i=1; i<=nf-j; ++i)
      if (sv[i] < sv[i+1]) {
	for (n=1; n<=nf; ++n) {
	  aux = Q[n][i];
	  Q[n][i] = Q[n][i+1];
	  Q[n][i+1] = aux;
	}
	for (n=1; n<=nr; ++n) {
	  aux = U[n][i];
	  U[n][i] = U[n][i+1];
	  U[n][i+1] = aux;
	}
	aux = sv[i];
	sv[i]=sv[i+1];
	sv[i+1]=aux;
      }

  // determine Su (nu x nf)
  mat_zero(Su);
  for (i=1; i<=nu; ++i)
    Su[i][nr+i] = 1.0;

  // Qu = Q*SuT  (nf x nu)
  mat_mult_normal_transpose(Q,Su,Qu);

  // this is the R matrix from the QR decomposition, derived from SVD, i.e.,
  // J = U S V'; J' = Q R; thus J' = V S U', and V = Q, S U' = R
  for (i=1; i<=nr; ++i)
    for (j=1; j<=nr; ++j)
      R[i][j] = U[j][i]*sv[i];

}

/*!*****************************************************************************
 *******************************************************************************
\note  checkIKTarget
\date  March 2011
\remarks 

Performs a velocity-based IK from a Cartesian start to a Cartesian target,
but aborts when joint limits are reached. Thus, the best reachable target is
returned. Note that this does not include orientations, and that the standard
joint_opt_state is used for redundancy resolution.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in,out]     js       : the starting joint state
 \param[in]         bs       : the base state
 \param[in]         bo       : the base orientation
 \param[in]         eff      : the endeffector information
 \param[in,out]     ct       : the cartesian target state
 \param[in]         status   : status variables for which endeffector states
                               to use (array of n_endeffs*N_CART+2+1)
 \param[in]         max_iter : the maximal number of iterations

 returns TRUE if no problem occured, and FALSE if there was problem and the
 target was not reached.

 ******************************************************************************/
#define SIM_DT   (1./500.)
int
checkIKTarget(SL_DJstate *js, SL_Cstate *bs, SL_quat *bo, SL_endeff *eff, 
	      SL_Cstate *ct, int *status, int max_iter)
{
  int       i,j;
  int       count = 0;
  SL_Cstate cs[n_endeffs+1];
  SL_Cstate last_cs[n_endeffs+1];
  double    aux;
  double    dist[n_endeffs+1];
  double    c_ref[n_endeffs*2*N_CART+1];
  double    condnr;

  MY_MATRIX(local_link_pos_des,0,N_LINKS,1,3);
  MY_MATRIX(local_joint_cog_mpos_des,0,N_DOFS,1,3);
  MY_MATRIX(local_joint_origin_pos_des,0,N_DOFS,1,3);
  MY_MATRIX(local_joint_axis_pos_des,0,N_DOFS,1,3);
  MY_MATRIX_ARRAY(local_Alink_des,1,4,1,4,n_links+1);
  MY_MATRIX_ARRAY(local_Adof_des,1,4,1,4,n_dofs+1);

  // compute the current cartesian positions
  linkInformationDes(js,bs,bo,eff,
		     local_joint_cog_mpos_des,
		     local_joint_axis_pos_des,
		     local_joint_origin_pos_des,
		     local_link_pos_des,
		     local_Alink_des,
		     local_Adof_des);

  for (j=1; j<=n_endeffs; ++j)
    for (i=1; i<=N_CART; ++i)
      cs[j].x[i] = local_link_pos_des[link2endeffmap[j]][i];

  // remember the last cartesian state such that we can use it if we hit a joint limit
  for (i=1; i<=N_ENDEFFS; ++i)
    last_cs[i] = cs[i];

  // this is the IK loop --------------------------------------------------------------
  while (TRUE) { 

    if (++count > 1000) // break out if too many iterations
      break;

    // compute reference velocity, or actually reference cartesian update step
    aux = 0.0;
    for (j=1; j<=n_endeffs; ++j) {
      
      // positions
      dist[j] = 0.0;
      for (i=1; i<=N_CART; ++i) {
	
	if (status[(j-1)*2*N_CART+i]) { // if this dimensions is constraint
	  
	  c_ref[(j-1)*2*N_CART+i] = (ct[j].x[i]-cs[j].x[i]);
	  dist[j] += sqr(c_ref[(j-1)*2*N_CART+i]);
	  
	} else {
	  
	  c_ref[(j-1)*2*N_CART+i] = 0.0;
	  
	}
      }
      dist[j] = sqrt(dist[j]);
      if (dist[j] > aux)
	aux = dist[j];
      
      // orientations
      for (i=N_CART+1; i<=2*N_CART; ++i) {
	if (status[(j-1)*2*N_CART+i]) { // if this dimensions is constraint
	  printf("checkIKTarget was not programmed for orientations yet\n");
	}
	c_ref[(j-1)*2*N_CART+i] = 0.0;
      }

    }

    // have we reached the goal
    if (aux < 0.001)
      return TRUE;

    // move at 1cm in most moving endeffector
    for (i=1; i<=n_endeffs*2*N_CART; ++i) { 
      c_ref[i] /= aux;
      c_ref[i] *= 0.01/SIM_DT;
    }
    
    // inverse kinematics simulated at SIM_DT
    condnr = inverseKinematics(js, eff, joint_opt_state, c_ref, status, SIM_DT);

    // check whether we violate joint ranges
    if (!check_range(js))
      break;

    // where are we in cartesian space
    linkInformationDes(js,bs,bo,eff,
		       local_joint_cog_mpos_des,
		       local_joint_axis_pos_des,
		       local_joint_origin_pos_des,
		       local_link_pos_des,
		       local_Alink_des,
		       local_Adof_des);
    
    for (j=1; j<=n_endeffs; ++j)
      for (i=1; i<=N_CART; ++i)
	cs[j].x[i] = local_link_pos_des[link2endeffmap[j]][i];
    
    for (i=1; i<=N_ENDEFFS; ++i)
      last_cs[i] = cs[i];

    
  } // while (TRUE)


  // correct the target to the most recently reached state
  for (i=1; i<=N_ENDEFFS; ++i)
    ct[i] = last_cs[i];

  return FALSE;
}
