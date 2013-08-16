#ifndef GLOBAL_HH
#define GLOBAL_HH
#include <IRobot.h>

#define COMMANDER_PORT 254    //the actual port will be 10254
#define COMMANDER_PORT_BASE 10000   

#define NUM_LEDS                       4        //number of blinkenlight
#define NUM_IRCOMMS                    4        //number of IR Communication channels
#define NUM_DOCKS                      4
#define NUM_IRS                        8        //number of ir proximity sensor

//#define DEFAULT_FORAGING_COUNT 80 //80 seconds
//#define DEFAULT_WAITING_COUNT 20 //20 seconds
#define DEFAULT_RECRUITMENT_COUNT       150
#define DEFAULT_RECRUITMENT_WAIT_COUNT 2400 // 240 seconds, duration wating for robot to dock, no recruitment signals during this period
#define RECRUITMENT_SIGNAL_INTERVAL 20 //2 seconds
//#define DEFAULT_ASSEMBLY_COUNT          1200 //120 seconds -- 2 minutes

#define INORGANISM_WAIT_COUNT 200 //one minutues delay after organism formed, then disassembly

#define BEACON_SIGNAL_DETECTED_THRESHOLD 3
#define PROXIMITY_SIGNAL_DETECTED_THRESHOLD 10
#define BUMPED_THRESHOLD 100
#define DOCKING_CHECKING_INTERVAL 8

#define FORWARD 1
#define BACKWARD -1


//for rgb led
#define RED LED_RED
#define GREEN LED_GREEN
#define BLUE LED_BLUE
#define WHITE LED_RED|LED_GREEN|LED_BLUE
#define CYAN LED_GREEN|LED_BLUE
#define YELLOW LED_RED|LED_GREEN
#define MAGENTA LED_RED|LED_BLUE


//for docking motor
#define OPEN -1
#define CLOSE 1
#define STOP 0

//for hinge motor
#define DOWN -1
#define UP 1

//LED frequence index
#define FREQ_DOCKING    0x4  //31.25Hz  for docking sensing
#define FREQ_PROXIMITY  0x2  //62.5Hz  for proximity

#define STAGE0 0
#define STAGE1 1
#define STAGE2 2
#define STAGE3 3
#define STAGE4 4
#define STAGE5 5

#define TURN_LEFT 0
#define TURN_RIGHT 1
#define MOVE_FORWARD 2
#define MOVE_BACKWARD 3
#define CHECKING 4

enum fsm_state_t {
    CALIBRATING = 0,
    EXPLORING, 
    RESTING, 
    SEEDING, 
    FORAGING, 
    ASSEMBLY, 
    WAITING, 
    LOCATEENERGY, 
    LOCATEBEACON, 
    ALIGNMENT, 
    RECOVER, 
    DOCKING, //11
    LOCKING, 
    DISASSEMBLY,
    UNDOCKING,
    INORGANISM,  // 15
    RECRUITMENT, 
    RAISING,
    LOWERING,
    RESHAPING,
    MACROLOCOMOTION,
    FAILED,
    SUPPORT,
    LEADREPAIR,
    REPAIR,
    BROADCASTSCORE,
    DEBUGGING, //26
    CLIMBING,
    DAEMON, //28
    STATE_COUNT
};
enum robot_mode_t {
    SWARM = 0, 
    ORGANISM
};
enum ir_pos_t {
    FR=0, 
    RF, 
    RB, 
    BR, 
    BL, 
    LB, 
    LF, 
    FL
}; //F -- FRONT, R -- RIGHT, B-- BACK, L--LEFT

enum msg_type_t {
    MSG_TYPE_UNKNOWN =0,
 
    MSG_TYPE_PROPAGATED,
    MSG_TYPE_DISASSEMBLY,
    MSG_TYPE_NEWROBOT_JOINED,
    MSG_TYPE_ORGANISM_FORMED,
    MSG_TYPE_RAISING,
    MSG_TYPE_RAISING_START,
    MSG_TYPE_RAISING_STOP,
    MSG_TYPE_LOWERING,
    MSG_TYPE_RESHAPING,

    // for self-repair
    MSG_TYPE_FAILED,
    MSG_TYPE_SUB_OG_STRING,
    MSG_TYPE_SCORE_STRING,
    MSG_TYPE_SCORE,
    MSG_TYPE_RESHAPING_SCORE,

    MSG_TYPE_RETREAT,
    MSG_TYPE_STOP,

    MSG_TYPE_IP_ADDR_COLLECTION, //issued by the leaf node robot

    MSG_TYPE_ACK,              //followed by acknowledged message type

    IR_MSG_TYPE_REMOTE_DEBUG,

    //broadcast, no ack required
    IR_MSG_TYPE_RECRUITING, //followed by assembly_info
    IR_MSG_TYPE_RECRUITING_REQ, //followed by mytype 
    IR_MSG_TYPE_EXPELLING,        //no data 
    IR_MSG_TYPE_POWERSOURCE_FOUND,//no data
    IR_MSG_TYPE_GUIDEME, //no data
    IR_MSG_TYPE_DOCKING_SIGNALS_REQ,
    IR_MSG_TYPE_OBJECTTYPE, 

    //broadcast, ack required
    IR_MSG_TYPE_LOCKED,
    IR_MSG_TYPE_UNLOCKED,
    IR_MSG_TYPE_LOCKME,
    IR_MSG_TYPE_UNLOCKME,
    IR_MSG_TYPE_ORGANISM_SEQ,

    IR_MSG_TYPE_ASSEMBLY_INFO,
    IR_MSG_TYPE_ASSEMBLY_INFO_REQ,

    IR_MSG_TYPE_IP_ADDR,
    IR_MSG_TYPE_IP_ADDR_REQ,

    IPC_MSG_HINGE_3D_MOTION_REQ,
    IPC_MSG_LOCOMOTION_2D_REQ,
    IPC_MSG_DOCKING_ROTATION_REQ,
    IPC_MSG_RESET_POSE_REQ,

    IPC_MSG_IRSENSOR_DATA_REQ,

    IPC_MSG_RAISING_START,
    IPC_MSG_RAISING_STOP,
    IPC_MSG_LOWERING_START,
    IPC_MSG_LOWERING_STOP,
    IPC_MSG_CLIMBING_START,
    IPC_MSG_CLIMBING_STOP,

    IPC_MSG_OPAQUE,
    
    IPC_MSG_RESHAPING_START,
    IPC_MSG_RESHAPING_DONE,
    IPC_MSG_ORGANISM_SEQ,

    IPC_MSG_ACK,

    MSG_TYPE_COUNT
};


enum docking_motor_status_t{
    OPENED = 0x0,
    OPENING = 0x1,
    CLOSED = 0x2,
    CLOSING = 0x3
};

enum hinge_motor_status_t{
    LOWED=0x0,
    LOWING=0x1,
    LIFTED=0x2,
    LIFTING=0x3
};


enum remote_cmd_t{
    CMD_NONE = 0x0,
    CMD_LOCKING_MOTOR,
    CMD_2D_LOCOMOTION,
    CMD_3D_HINGE,

    REMOTE_CMD_COUNT
};

enum ir_data_t{
    IR_REFLECTIVE_DATA = 0,
    IR_AMBIENT_DATA,
    IR_PROXIMITY_DATA,
    IR_BEACON_DATA,
    IR_AUX_REFLECTIVE_DATA
};

//#define IR_MSG_REPEATED_DELAY 20 // 1 SECOND


extern const char* state_names[STATE_COUNT];
extern const char* message_names[MSG_TYPE_COUNT];
extern const char* remote_cmd_names[REMOTE_CMD_COUNT];
//enum led_t {LED0=0x1,LED1=0x2,LED2=0x4, IR_PULSE0=0x1, IR_PULSE1=0x2, IR_PULSE2=0x4}; //TODO: ActiveWheel has different no




#endif
