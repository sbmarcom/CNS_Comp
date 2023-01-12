component userkins"Template for user-built kinematics";

description
"""
.if \\n[.g] .mso www.tmac

The userkins.comp file is a template for creating
kinematics that can be user-built using halcompile.

The unmodified userkins component can be used
as a kinematics file for a machine with identity
kinematics for an xyz machine employing 3 joints
(motors).

\\fBUSAGE:\\fR

  1) Copy the userkins.comp file to a user-owned
     directory (\\fBmydir\\fR).

     Note: The userkins.comp file can be downloaded from:
.URL https://github.com/LinuxCNC/linuxcnc/raw/2.8/src/hal/components/userkins.comp
     where '2.8' is the branch name (use 'master' for
     the master branch)

     For a RIP (run-in-place) build, the file is located in
     the git tree as:
       src/hal/components/userkins.comp

  2) Edit the functions kinematicsForward() and
     kinematicsInverse() as required
  3) If required, add hal pins following examples in
     the template code
  4) Build and install the component using halcompile:
     $ cd \\fBmydir\\fR
     $ [sudo] halcompile --install userkins.comp
     # Note:
     #      sudo is required when using a deb install
     #      sudo is \\fBnot\\fR required for run-in-place builds
     # $ man halcompile for more info
  5) Specify userkins in an ini file as:
     \\fB[KINS]\\fR
     \\fBKINEMATICS=userkins\\fR
     \\fBJOINTS=3\\fR
     # the number of JOINTS must agree with the
     # number of joints used in your modified userkins.comp
  6) Note: the manpage for userkins is not updated by
     halcompile --install
  7) To use a different component name, rename the file
     (example mykins.comp) and change all instances of
     'userkins' to 'mykins'

\\fBNOTES:\\fR
  1  A 'dummy' pin is required to satisfy the requirements of
     the halcompile utility but it is not accessible to
     kinematics functions.

  2  Hal pins and parameters needed in kinematics functions
     (kinematicsForward(), kinematicsInverse()) must
     be setup in a function (userkins_setup()) invoked
     by the initial motion module call to kinematicsType().
""";
// The halcompile utility requires a pin definition but
// the dummy pin is not accessible in kinematics functions.
// Use the *_setup() function for actual pins and params.
pin out bit dummy=1;
license "GPL";
;;

#include "rtapi_math.h"
#include "kinematics.h"
#include "rtapi.h"
#include "rtapi_string.h"
#include "posemath.h"
#include "hal.h"


#define D1 (*(haldata->d1))
#define D2 (*(haldata->d2))
#define D3 (*(haldata->d3))
#define D4 (*(haldata->d4))
#define D5 (*(haldata->d5))
#define D6 (*(haldata->d6))
#define R1 (*(haldata->r1))

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

static int userkins_setup(void) {
#define HAL_PREFIX "userkins"
    int res=0;
    int comp_id;
    // this name must be different than the component name:
    comp_id = hal_init("userkinsdata");
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
    if (!is_setup) userkins_setup();
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