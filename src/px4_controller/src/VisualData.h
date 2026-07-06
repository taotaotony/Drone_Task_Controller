#ifndef VISUALDATA_H
#define VISUALDATA_H

// 视觉目标数据结构（独立头文件，避免循环包含）
struct VisualData
{
    int Target1_Exist;
    float Target1_PR;
    int Target1_LU_x;
    int Target1_LU_y;
    int Target1_RU_x;
    int Target1_RU_y;
    int Target1_RD_x;
    int Target1_RD_y;
    int Target1_LD_x;
    int Target1_LD_y;
    int Target2_Exist;
    float Target2_PR;
    int Target2_LU_x;
    int Target2_LU_y;
    int Target2_RU_x;
    int Target2_RU_y;
    int Target2_RD_x;
    int Target2_RD_y;
    int Target2_LD_x;
    int Target2_LD_y;
    int Target3_Exist;
    float Target3_PR;
    int Target3_LU_x;
    int Target3_LU_y;
    int Target3_RU_x;
    int Target3_RU_y;
    int Target3_RD_x;
    int Target3_RD_y;
    int Target3_LD_x;
    int Target3_LD_y;
};

#endif // VISUALDATA_H