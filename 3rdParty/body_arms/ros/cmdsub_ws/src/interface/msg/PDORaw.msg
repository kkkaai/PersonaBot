#ok
#Header header
int16[] status_word
int32[] position
int16[] errcd 
int32[] speed
int16[] torque
# {0x6064, 0x00, 32}, /* Position Actual Value */
#     {0x60fd, 0x00, 32}, /* Digital inputs */
#     {0x6041, 0x00, 16}, /* Status Word */
#     {0x603f, 0x00,16},  /*actual corrent*/
#     {0x606c, 0x00,32},  /*actual speed*/
#     {0x6077, 0x00,16}, 
