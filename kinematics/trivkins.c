/********************************************************************
* Description: trivkins.c
*   general trivkins for 3 axis Cartesian machine
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* License: GPL Version 2
*
* Copyright (c) 2009 All rights reserved.
*
********************************************************************/

#include "rtapi_math.h"
#include "kinematics.h"
#include "rtapi.h"
#include "rtapi_string.h"
#include "posemath.h"
#include "hal.h"




static struct haldata {
  // Example pin pointers
  hal_u32_t *in;
  hal_u32_t *out;
  // Example parameters
  hal_float_t param_rw;
  hal_float_t param_ro;
  hal_float_t *d1, *d2, *d3, *d4, *d5, *d6, *r1;
} *haldata;
// hal pin/param types:
// hal_bit_t   bit
// hal_u32_t   unsigned 32bit integer
// hal_s32_t   signed 32bit integer
// hal_float_t float (double precision)

#define D1 (*(haldata->d1))
#define D2 (*(haldata->d2))
#define D3 (*(haldata->d3))
#define D4 (*(haldata->d4))
#define D5 (*(haldata->d5))
#define D6 (*(haldata->d6))
#define R1 (*(haldata->r1))


static int mykins_setup(void) {
#define HAL_PREFIX "mykins"
    int res=0;
    int comp_id;
    // this name must be different than the component name:
    comp_id = hal_init("mykinsdata");
    if (comp_id < 0) goto error;
    haldata = hal_malloc(sizeof(struct haldata));
    if (!haldata) goto error;

    // hal pin examples:
    res += hal_pin_u32_newf(HAL_IN ,&(haldata->in) ,comp_id,"%s.in" ,HAL_PREFIX);
    res += hal_pin_u32_newf(HAL_OUT,&(haldata->out),comp_id,"%s.out",HAL_PREFIX);

    // hal parameter examples:
    res += hal_param_float_newf(HAL_RW, &haldata->param_rw,comp_id,"%s.param-rw",HAL_PREFIX);
    res += hal_param_float_newf(HAL_RO, &haldata->param_ro,comp_id,"%s.param-ro",HAL_PREFIX);

    if (res) goto error;
    hal_ready(comp_id);
    rtapi_print("*** %s setup ok\n",__FILE__);
    return 0;
error:
    rtapi_print("\n!!! %s setup failed res=%d\n\n",__FILE__,res);
    return -1;
#undef HAL_PREFIX
}

KINS_NOT_SWITCHABLE;
// see millturn.comp for example of switchable kinematics

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsInverse);
EXPORT_SYMBOL(kinematicsForward);

KINEMATICS_TYPE kinematicsType()
{
static bool is_setup=0;
    if (!is_setup) mykins_setup();
    return KINEMATICS_IDENTITY; // set as required
           // Note: If kinematics are identity, using KINEMATICS_BOTH
           //       may be used in order to allow a gui to display
           //       joint values in preview prior to homing
} // kinematicsType()

static bool is_homed=0;
int kinematicsForward(const double * joint,
                      EmcPose * world,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
double a0, a1, a3;
    double x, y, z, c;

/* convert joint angles to radians for sin() and cos() */

    a0 = joint[0] * ( PM_PI / 180 );
    a1 = joint[1] * ( PM_PI / 180 );
    a3 = joint[3] * ( PM_PI / 180 );  //Can ignore joint 3 
   
/* convert angles into world coords */

    // Edit these lines to compensate for our angle difference
    a1 = a1 - a0/R1; // compensation for co-axial shaft
    a1= a1 + a0; // Convert a1 to epsilon
	//Get X-Y from joints
    x = D2*cos(a0) + D4*cos(a1); // + D6*cos(a3); 
    y = D2*sin(a0) + D4*sin(a1); // + D6*sin(a3);
    z = D1 + D3 - joint[2] - D5;
   // c = a3;

    *iflags = 0;
    if (joint[1] < 90)
        *iflags = 1;

    world->tran.x = x;
    world->tran.y = y;
    world->tran.z = z;
    //world->c = c * 180 / PM_PI;

    //world->a = joint[4];
    //world->b = joint[5];

    return 0;
} // kinematicsForward()

int kinematicsInverse(const EmcPose * world,
                      double * joint,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags)
{
    double a3;
    double q0, q1;
    double xt, yt, rsq, cc;
    double x, y, z, c;

    x = world->tran.x;
    y = world->tran.y;
    z = world->tran.z;
    //c = world->c;

    /* convert degrees to radians */
    //a3 = c * ( PM_PI / 180 );

    /* center of end effector (correct for D6) */
    xt = x; // - D6*cos(a3);
    yt = y; // - D6*sin(a3);

    /* horizontal distance (squared) from end effector centerline
        to main column centerline */
    rsq = xt*xt + yt*yt;
    /* joint 1 angle needed to make arm length match sqrt(rsq) */
    cc = (rsq - D2*D2 - D4*D4) / (2*D2*D4);
    if(cc < -1) cc = -1;
    if(cc > 1) cc = 1;
    q1 = acos(cc);

    if (*iflags)
        q1 = -q1;

    /* angle to end effector */
    q0 = atan2(yt, xt);

    /* end effector coords in inner arm coord system */
    xt = D2 + D4*cos(q1);
    yt = D4*sin(q1);

    /* inner arm angle */
    q0 = q0 - atan2(yt, xt);

    /* q0 and q1 are still in radians. convert them to degrees */
    q0 = q0 * (180 / PM_PI);
    q1 = q1 * (180 / PM_PI);
	// Edit this to convert kinematics
    joint[0] = q0;
    joint[1] = q1 + q0/R1; //Outputting psi angle instead of phi <--------------------------
    joint[2] = D1 + D3 - D5 - z;
    //joint[3] = c - ( q0 + q1);
    //joint[4] = world->a;
    //joint[5] = world->b;

    *fflags = 0;

    return 0;
} // kinematicsInverse()


#define DEFAULT_D1 490
#define DEFAULT_D2 340
#define DEFAULT_D3  50
#define DEFAULT_D4 250
#define DEFAULT_D5  50
#define DEFAULT_D6  50

/*static int scaraKinematicsSetup(const  int   comp_id,
                                const  char* coordinates,
                                kparms*      kp)
{
    int res=0;

    haldata = hal_malloc(sizeof(*haldata));
    if (!haldata) goto error;

    res += hal_pin_float_newf(HAL_IN, &(haldata->d1), comp_id,"%s.D1",kp->halprefix);
    res += hal_pin_float_newf(HAL_IN, &(haldata->d2), comp_id,"%s.D2",kp->halprefix);
    res += hal_pin_float_newf(HAL_IN, &(haldata->d3), comp_id,"%s.D3",kp->halprefix);
    res += hal_pin_float_newf(HAL_IN, &(haldata->d4), comp_id,"%s.D4",kp->halprefix);
    res += hal_pin_float_newf(HAL_IN, &(haldata->d5), comp_id,"%s.D5",kp->halprefix);
    res += hal_pin_float_newf(HAL_IN, &(haldata->d6), comp_id,"%s.D6",kp->halprefix);
    if (res) { goto error; }

    D1 = DEFAULT_D1;
    D2 = DEFAULT_D2;
    D3 = DEFAULT_D3;
    D4 = DEFAULT_D4;
    D5 = DEFAULT_D5;
    D6 = DEFAULT_D6;

    return 0;

error:
    return -1;
} */// scaraKinematicsSetup()

/*int switchkinsSetup(kparms* kp,
                    KS* kset0, KS* kset1, KS* kset2,
                    KF* kfwd0, KF* kfwd1, KF* kfwd2,
                    KI* kinv0, KI* kinv1, KI* kinv2
                   )
{
    kp->kinsname    = "userkins"; // !!! must agree with filename
    kp->halprefix   = "userkins"; // hal pin names
    kp->required_coordinates = "xyzabc"; // ab are scaragui table tilts
    kp->allow_duplicates     = 0;
    kp->max_joints = strlen(kp->required_coordinates);

    rtapi_print("\n!!! switchkins-type 0 is %s\n",kp->kinsname);
    *kset0 = scaraKinematicsSetup;
    *kfwd0 = scaraKinematicsForward;
    *kinv0 = scaraKinematicsInverse;

    *kset1 = identityKinematicsSetup;
    *kfwd1 = identityKinematicsForward;
    *kinv1 = identityKinematicsInverse;

    *kset2 = userkKinematicsSetup;
    *kfwd2 = userkKinematicsForward;
    *kinv2 = userkKinematicsInverse;

    return 0;
} */// switchkinsSetup()

