/*
 * status.h
 *
 *  Created on: Oct 6, 2022
 *      Author: crazy
 */

#ifndef INC_STATUS_H_
#define INC_STATUS_H_

typedef enum
{
  IDLE = 0x00,
  RUNNING_A = 0x01,
  RUNNING_B = 0x02,
  STOPPED = 0x03,
  ERR = 0xFF,
}TDK_Status;


#endif /* INC_STATUS_H_ */
