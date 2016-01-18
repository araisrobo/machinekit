#ifndef __sync_cmd_h__
#define __sync_cmd_h__
/*
 *******************************************************************************
 * SYNC Command Format:
 *    NAME        OP_CODE[15:14]  OPERAND[13:0]     Description
 *    SYNC_JNT           2'b00    {DIR_W, POS_W}    DIR_W[13]:    Direction, (positive(1), negative(0))
 *                                                  POS_W[12:0]:  Relative Angle Distance (0 ~ 8191)
 *    NAME        OP_CODE[15:12]  OPERAND[11:0]     Description
 *    SYNC_DOUT          4'b0100  {ID, VAL}         ID[11:4]: Output PIN ID
 *                                                  VAL[0]:   ON(1), OFF(0)
 *    SYNC_DIN           4'b0101  {ID, TYPE}        ID[11:4]: Input PIN ID
 *                                                  TYPE[2:0]: LOW(000), HIGH(001), FALL(010), RISE(011)
 *                                                             TIMEOUT(100)
 *    RESERVED           4'b0110  
 *    SYNC_MOT_PARAM     4'b0111  {JOINT_ID}{ADDR}  Motion parameters for JOINT_ID, 
 *                                                  ADDR[11:4]
 *                                                    ID[ 3:0]: Joint ID
 *                                                  DATA[31:0]: from immediate data
 *    SYNC_AHC           4'b1000  ... RESERVED      auto height control
 *    SYNC_VEL           4'b1001  {VEL, VAL}        VEL: velocity in mm/s
 *                                                  VAL[0]: 1: velocity sync'd
 *                                                          0: velocity not sync'd
 *    SYNC_USB_CMD       4'b1010  
 *    SYNC_MACH_PARAM    4'b1011  ... TODO      
 *    SYNC_DATA          4'b1100  ... TODO:         {VAL} Send immediate data
 *                                                  VAL[7:0]: one byte data
 *    SYNC_EOF           4'b1101                    End of frame                                            
 *    SYNC_DAC           4'b1110  {ID, ADDR}        write into DAC register with {ID[11:8], ADDR[7:0]}
 *                                                  ADDR: 0x01 ... Data register for AD5422
 *                                                  ADDR: 0x55 ... Control register for AD5422
 *                                                  ADDR: 0x99 ... DAC_OFFSET for RISC
 *    Write 2nd byte of SYNC_CMD[] will push it into SFIFO.
 *    The WB_WRITE got stalled if SFIFO is full.
 */

//      SFIFO COMMANDS
#define SYNC_JNT            0x0000      // 0x0000 ~ 0x3FFF
#define SYNC_DOUT           0x4000
#define SYNC_DIN            0x5000
#define SYNC_MOT_POS_CMD    0x6000
#define SYNC_MOT_PARAM      0x7000
// RESERVED #define SYNC_AHC            0x8000         // auto height control
#define SYNC_VEL            0x9000
#define SYNC_USB_CMD        0xA000
#define SYNC_MACH_PARAM     0xB000
#define SYNC_DATA           0xC000
#define SYNC_EOF            0xD000
#define SYNC_DAC            0xE000      // 1st 16-bit for ID and ADDR, 2nd 32-bit for VALUE
// RESERVED  0xf000

//  timeout type
#define WAIT_LEVEL_LOWER    0x0   // wait analog input to be lower than specified value
#define WAIT_LEVEL_HIGHER   0x1   // wait analog input to be higher than specified value
// 0x2 reserved
// 0x3 reserved
#define WAIT_LOW            0x4
#define WAIT_HIGH           0x5
#define WAIT_FALL           0x6
#define WAIT_RISE           0x7
#define NO_WAIT             0xF

//      SFIFO COMMAND MASK
#define SFIFO_SYNC_JNT_MASK             0xC000
#define DIR_P                           0x2000
#define DIR_N                           0x0000
#define POS_MASK                        0x1FFF

#define SYNC_OP_CODE_MASK               0xF000
#define SYNC_DATA_MASK                  0x00FF
#define SYNC_MOT_PARAM_ID_MASK          0x0F00
#define SYNC_MOT_PARAM_ADDR_MASK        0x00FF
#define SYNC_MACH_PARAM_ADDR_MASK       0x0FFF
#define SYNC_USB_CMD_TYPE_MASK 		0x0FFF

// SYNC VEL CMD masks
#define VEL_MASK                        0x0FFE
#define VEL_SYNC_MASK                   0x0001

// PROBE mask
#define SYNC_PROBE_MASK                 0x0FFF

// SYNC_DAC masks
#define SYNC_DAC_ID_MASK                0x0F00
#define SYNC_DAC_ADDR_MASK              0x00FF
#define SYNC_DAC_VAL_MASK               0xFFFF
#define SYNC_DAC_DATA                   0x01 // DAC_DATA at ADDR field
#define SYNC_DAC_CTRL                   0x55 // DAC_CTRL at ADDR field
#define SYNC_DAC_OFFSET                 0x99 // DAC_OFFSET at ADDR field

// SFIFO DATA MACROS
// SYNC_DOUT          4'b0100  {ID, VAL}         ID[11:4]: Output PIN ID
//                                               VAL[0]:   ON(1), OFF(0)
// SYNC_DIN           4'b0101  {ID, TYPE}        ID[11:4]: Input PIN ID
//                                               TYPE[2:0]: LOW(000), HIGH(001), FALL(010), RISE(011)
//                                                          TIMEOUT(100)
#define SYNC_DI_DO_PIN_MASK             0x0FF0
#define SYNC_DOUT_VAL_MASK              0x0001
#define SYNC_DIN_TYPE_MASK              0x0007
#define DIO_PIN_OFFSET                  4
#define GET_IO_ID(i)                    (((i) & SYNC_DI_DO_PIN_MASK) >> DIO_PIN_OFFSET)
#define DO_VAL(v)                       (((v) & SYNC_DOUT_VAL_MASK))
#define DI_TYPE(t)                      (((t) & SYNC_DIN_TYPE_MASK))
#define PACK_IO_ID(i)                   (((i) << DIO_PIN_OFFSET) & SYNC_DI_DO_PIN_MASK)

#define GET_DATA_VAL(t)                 (((t) & SYNC_DATA_MASK))
#define GET_MOT_PARAM_ID(t)             (((t) & SYNC_MOT_PARAM_ID_MASK) >> 8)
#define GET_MOT_PARAM_ADDR(t)           (((t) & SYNC_MOT_PARAM_ADDR_MASK))
#define GET_MACH_PARAM_ADDR(t)          ((t) & SYNC_MACH_PARAM_ADDR_MASK)
#define GET_USB_CMD_TYPE(t)             ((t) & SYNC_USB_CMD_TYPE_MASK)
#define GET_DAC_ID(i)                   (((i) & SYNC_DAC_ID_MASK) >> 8)
#define GET_DAC_ADDR(a)                 ((a) & SYNC_DAC_ADDR_MASK)
#define GET_DAC_VAL(v)                  ((v) & SYNC_DAC_VAL_MASK)

#define PACK_SYNC_DATA(t)               ((t & 0xFF))
#define PACK_MOT_PARAM_ID(t)            (((t) << 8) & SYNC_MOT_PARAM_ID_MASK)
#define PACK_MOT_PARAM_ADDR(t)          ((t) & SYNC_MOT_PARAM_ADDR_MASK)
#define PACK_MACH_PARAM_ADDR(t)         ((t) & SYNC_MACH_PARAM_ADDR_MASK)
#define PACK_USB_CMD_TYPE(t)            ((t) & SYNC_USB_CMD_TYPE_MASK)

#define RISC_CMD_TYPE                   0x0004  // for SYNC_USB_CMD

/* bit index for machine_status[31:0] */
#define FERROR_MASK                     0x000000FF  // machine_status[7:0]
#define ALARM_MASK                      0x00000100  // machine_status[8]
#define SFIFO_IS_EMPTY_MASK             0x00000200  // machine_status[9]
#define SFIFO_IS_EMPTY_BIT              9           // set to 1 if SFIFO is EMPTY
#define TP_RUNNING_BIT                  17
#define AHC_DOING_BIT                   18

/**
 *  MACHINE_CTRL,   // [31:28]  SPINDLE_AUX_JOINT_ID
 *                  // [27:24]  SPINDLE_JOINT_ID
 *                  // [23:16]  NUM_JOINTS
 *                  // [l5: 8]  OBSOLETED: JOG_SEL
 *                              OBSOLETED:     [15]: MPG(1), CONT(0)
 *                              OBSOLETED:     [14]: RESERVED
 *                              OBSOLETED:     [13:8]: J[5:0], EN(1), DISABLE(0)
 *                  // [ 7: 4]  ACCEL_STATE, the FSM state of S-CURVE-VELOCITY profile
 *                  // [    3]  HOMING
 *                  // [ 2: 1]  MOTION_MODE: 
 *                                  FREE    (0) 
 *                                  TELEOP  (1) 
 *                                  COORD   (2)
 *                  // [    0]  MACHINE_ON
 **/
#define MCTRL_MACHINE_ON_MASK           0x00000001  // MACHINE_ON mask for MACHINE_CTRL
#define MCTRL_MOTION_MODE_MASK          0x00000006  // MOTION_MODE mask for MACHINE_CTRL
#define MCTRL_HOMING_MASK               0x00000008  // HOMING_MASK for MACHINE_CTRL
#define MCTRL_ACCEL_STATE_MASK          0x000000F0  // ACCEL_STATE mask for MACHINE_CTRL
#define MCTRL_JOG_SEL_MASK              0x0000FF00  // JOG_SEL mask for MACHINE_CTRL
#define MCTRL_NUM_JOINTS_MASK           0x00FF0000  // NUM_JOINTS mask for MACHINE_CTRL
#define MCTRL_SPINDLE_ID_MASK           0x0F000000  // SPINDLE_JOINT_ID mask for MACHINE_CTRL
#define MCTRL_SPINDLE_AUX_ID_MASK       0xF0000000  // SPINDLE_AUX_JOINT_ID mask for MACHINE_CTRL

/**
 *  GANTRY_CTRL,    // [31]     GANTRY_EN
 **/                  
#define GCTRL_EN_MASK                   0x80000000  // Gantry Enable Bit
// TODO: add GANTRY_MASTER_ID and GANTRY_SLAVE_ID to GANTRY_CTRL register

typedef enum {
    // naming: RISC_...CMD..._REQ
    RCMD_IDLE = 0,              // RCMD_FSM, set by RISC
    RCMD_ALIGNING,              // RCMD_FSM, joint is aligning
    RCMD_UPDATE_POS_REQ,        // RCMD_FSM, request HOST to update position
    RCMD_UPDATE_POS_ACK,        // RCMD set by HOST 
    RCMD_RISC_PROBE,            // Do risc probe 
    RCMD_HOST_PROBE,            // Do host probe
    RCMD_PSO,                   // PSO -- progress synced output
    RCMD_REMOTE_JOG,            // remote control 
    RCMD_SET_ENC_POS,           // set encoder position
} rsic_cmd_t;

typedef enum {
    RISC_PROBE_LOW = 0,
    RISC_PROBE_HIGH,
    RISC_PROBE_INDEX,
} rsic_probe_type_t;

typedef enum {
    OR = 0,
    AONLY,
    DONLY,
    AND,
} host_probe_type_t;

// memory map for machine config
enum machine_parameter_addr {
    AHC_JNT,
    AHC_POLARITY,
    GANTRY_POLARITY,
    ANALOG_REF_LEVEL,       // wait analog signal: M110
    AHC_MAX_OFFSET,
    AHC_ANALOG_CH,
    WAIT_TIMEOUT,
    AHC_STATE,
    AHC_LEVEL,
    GANTRY_CTRL,            // [31]     GANTRY_EN
                            
    JOINT_LSP_LSN,          // format: {JOINT[31:16], LSP_ID[15:8], LSN_ID[7:0]}
    JOINT_JOGP_JOGN,        // format jog: {JOINT[31:16], JOGP_ID[15:8], LOGN_ID[7:0]}
    ALR_OUTPUT_0,           // DOUT_0 value, dout[31:0], when ESTOP is pressed
    ALR_OUTPUT_1,           // DOUT_1 value, dout[63:32], when ESTOP is pressed
    ALR_EN_BITS,            // the bitmap of ALARM bits for all joints (DIN[6:1])
                            //             as well as ALARM_EN/ESTOP bit (DIN[0])

    MACHINE_CTRL,           // [31:28]  SPINDLE_AUX_JOINT_ID
                            // [27:24]  SPINDLE_JOINT_ID
                            // [23:16]  NUM_JOINTS
                            // [15: 8]  OBSOLETE: JOG_SEL
                            // [ 7: 4]  ACCEL_STATE
                            // [    3]  HOMING
                            // [ 2: 1]  MOTION_MODE: 
                            // [    0]  MACHINE_ON
    MACHINE_PARAM_ITEM
};

// accel_state enum for MACHINE_CTRL judgement
enum accel_state_type {
  MACH_ACCEL_S0 = (0 << 4),     // 0
  MACH_ACCEL_S1 = (1 << 4),     // 1
  MACH_ACCEL_S2 = (2 << 4),     // 2
  MACH_ACCEL_S3 = (3 << 4),     // 3
  MACH_ACCEL_S4 = (4 << 4),     // 4
  MACH_ACCEL_S5 = (5 << 4),     // 5
  MACH_ACCEL_S6 = (6 << 4)      // 6
};

enum ahc_state_enum {
    AHC_DISABLE,
    AHC_ENABLE,
};

// memory map for motion parameter for each joint
// parameters updated by SYNC_MOT_PARAM command
enum motion_parameter_addr {
    MAX_VELOCITY      ,
    MAX_ACCEL         ,
    MAX_JERK	      ,
    MAXFOLLWING_ERR   ,

    // PID section: begin
    PPG               ,     // PositionPropotionGain, Q16.16
    PIG               ,     // PositionIntegrationGain, Q16.16
    PFFG              ,     // PositionFeedForwardGain, Q16.16
    PDB               ,     // PositionDeadBand, Q32.0
    PME               ,     // PositionMaxError, Q32.0
    PMIE              ,     // PositionMaxIntegrationError, Q32.0
    PMO               ,     // PositionMaxOutput, Q16.16
    VPG               ,     // VelocityPropotionalGain, Q16.16
    VFFG              ,     // VelocityFeedForwardGain, Q16.16
    // PID section: end
    SCALE             ,     // unit_pulses/servo_period : 16.16 format, 
    ENC_SCALE         ,     // encoder scale: 16.16 format
    SSYNC_SCALE       ,     // spindle sync compensation scale: 16.16 format
    // OUTPUT: begin
    OUT_DEV           ,     // output device：AB-PHASE/STEP-DIR/PWM/DAC
                            // [31:28] TYPE: ANALOG/DIGITAL
                            // [27:24] CHANNEL: DAC channel
                            // [23: 0] RESERVED
    OUT_RANGE         ,     // [31:16] MIN_OUT [15:0] MAX_OUT
    OUT_MAX_IPULSE    ,     // Max input pulse for the output device, Q16.16
    OUT_SCALE         ,     // output scale per pulse, Q16.16
    OUT_OFFSET        ,     // output offset, Q16.16
                            // output = ipulse * OUT_SCALE + OUT_OFFSET
    OUT_SCALE_RECIP   ,     // reciprocol of OUT_SCALE, Q16.16
    // OUTPUT: end
    MAX_PARAM_ITEM
};
#define NUM_PID_PARAMS 14   // pid params: from P_GAIN to MAXOUTPUT
#define OUT_DEV_TYPE_MASK   0xF0000000  // [31:28] TYPE: ANALOG/PWM/PULSE
#define OUT_DEV_CH_MASK     0x0F000000  // [27:24] CHANNEL: DAC channel
#define OUT_RANGE_MAX_MASK  0x0000FFFF  // [15: 0] MAX_OUT
#define OUT_RANGE_MIN_MASK  0xFFFF0000  // [31:16] MIN_OUT

#define OUT_TYPE_AB_PHASE   (0)         // digital pulse type: AB-PHASE 
#define OUT_TYPE_STEP_DIR   (1)         // digital pulse type: STEP-DIR
#define OUT_TYPE_DAC        (2)         // DAC output type, +/-10V, 0~10V, 0~20mA ... etc.
#define OUT_TYPE_PWM        (3)         // PWM-DIR type: 100% PWM + DIR signal

#define ENC_TYPE_LOOPBACK   (0)
#define ENC_TYPE_AB_PHASE   (2)
#define ENC_TYPE_STEP_DIR   (3)

#endif // __sync_cmd_h__
