//
// Created by 15795 on 2024/1/19.
//

#ifndef RM_FRAME_AUTO_EXCHANGE_H
#define RM_FRAME_AUTO_EXCHANGE_H
#include "base/robotics/robotics.h"
#include "app/arm.h"


class  AutoExchangeController{
public:

    AutoExchangeController();
    //视觉信息转换为传递矩阵
    Matrixf<4,4> cv2t(void);
    //自动跟随函数
    void auto_follow();

    struct AutoExchange_ref_t
    {
        float x;
        float y;
        float z;
        float yaw;
        float pitch;
        float roll;
    } ref_;
};




//
#endif //RM_FRAME_AUTO_EXCHANGE_H
