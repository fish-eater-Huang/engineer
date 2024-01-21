//
// Created by 15795 on 2024/1/19.
//

#ifndef RM_FRAME_AUTO_EXCHANGE_H
#define RM_FRAME_AUTO_EXCHANGE_H
#include "base/robotics/robotics.h"

namespace auto_exchange{
//视觉信息转换成转移矩阵 cv->T
Matrixf<4,4> cv2t(void);

//通过视觉信息控制机械臂跟随
void auto_follow();

}



//
#endif //RM_FRAME_AUTO_EXCHANGE_H
