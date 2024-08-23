/*
 * stack.h
 *
 *  Created on: 2021Äê8ÔÂ3ÈÕ
 *      Author: 95159
 */

#ifndef CODE_STACK_H_
#define CODE_STACK_H_

#define Stack_Size 3760
#include "zf_common_headfile.h"

typedef struct Stack
{
    int32 top;
    int32 MAX;
    Point *data;
}Stack;

void InitStack(Stack *s,uint32 max,Point *data);

uint8 FullStack(Stack * s);

uint8 EmptyStack(Stack *s);

uint8 PushStack(Stack * s, uint8 x, uint8 y);

Point PopStack(Stack * s);



#endif /* CODE_STACK_H_ */
