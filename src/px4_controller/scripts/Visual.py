#! usr/bin/env/python
import rospy
from px4_controller.msg import tbag
import cv2
import torch
from ultralytics import YOLO
import time



def main():
    prev_time = 0
    fps = 0
    #ros preparation
    rospy.init_node("Visual")
    rate = rospy.Rate(30)
    pub = rospy.Publisher("IR",tbag,queue_size=10)
    # 存储概率前 3 的物体信息的二维列表
    top_3_list = [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
    # 加载自定义的 YOLOv8 模型，将 'path/to/your/custom_model.pt' 替换为自定义模型的路径
    model = YOLO('/home/ros/Desktop/CUADC_ws/best20250731.pt')
    # 打开摄像头，0 表示默认摄像头，也可以使用视频文件的路径
    cap = cv2.VideoCapture(4)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH,1200)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT,800)
    width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"当前摄像头分辨率：{width}*{height}")
    while cap.isOpened() and not rospy.is_shutdown():
        # 读取一帧
        ret, frame = cap.read()
        if not ret:
            break
        # 使用 YOLOv8 进行目标检测
        results = model(frame,verbose=False)
        # 获取第一个检测结果
        result = results[0]
        # 获取检测到的对象的边界框、类别和概率
        boxes = result.boxes
        # 重置标志位
        top_3_list = [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
        video_pack = tbag()
        if len(boxes) > 0:
            # 获取概率
            confidences = boxes.conf
            # 找出概率前三的索引
            top_3_indices = torch.argsort(confidences, descending=True)[:3]
            for i, index in enumerate(top_3_indices):
                # 获取概率前三的框
                top_3_box = boxes[index]
                # 获取边界框的坐标
                xyxy = top_3_box.xyxy[0].tolist()
                # 获取概率
                conf = top_3_box.conf.item()
                # 将四个顶点的坐标、概率和标志位存储在列表中
                target_info = [
                    (xyxy[0], xyxy[1]),  # 左上角顶点
                    (xyxy[2], xyxy[1]),  # 右上角顶点
                    (xyxy[2], xyxy[3]),  # 右下角顶点
                    (xyxy[0], xyxy[3]),  # 左下角顶点
                    conf,
                    1  # 标志位，识别到为 1
                ]
                top_3_list[i] = target_info
                # 在画面中绘制框和显示概率
                label = f'{conf:.2f}'
                cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
                cv2.putText(frame, label, (int(xyxy[0]), int(xyxy[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # 标注左上角顶点坐标
                top_left_text = f'({int(xyxy[0])}, {int(xyxy[1])})'
                cv2.putText(frame, top_left_text, (int(xyxy[0]), int(xyxy[1]) + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # 标注右下角顶点坐标
                bottom_right_text = f'({int(xyxy[2])}, {int(xyxy[3])})'
                cv2.putText(frame, bottom_right_text, (int(xyxy[2]) - 80, int(xyxy[3])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # 输出二维列表
        #print("Top 3 Detection List:", top_3_list)

        # 绘制坐标系
        height, width, _ = frame.shape
        cv2.line(frame, (0, 0), (width, 0), (255, 0, 0), 2)  # x-axis
        cv2.line(frame, (0, 0), (0, height), (0, 255, 0), 2)  # y-axis
        cv2.putText(frame, 'X', (width - 20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(frame, 'Y', (20, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        #计算帧率
        #current_time = time.time()
        #if prev_time != 0:
        #    fps=1/(current_time-prev_time)
        #prev_time = current_time    
        #cv2.putText(frame,f"FPS:{int(fps)}",(10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0))
        # 显示结果帧
        cv2.imshow("YOLOv8 Custom Model Detection", frame)

        #publish data
        video_pack.Target1_Exist=top_3_list[0][5]
        if(video_pack.Target1_Exist):
            video_pack.Target1_PR=round(top_3_list[0][4],2)
            video_pack.Target1_LU_x=int(top_3_list[0][0][0])
            video_pack.Target1_LU_y=int(top_3_list[0][0][1])
            video_pack.Target1_RU_x=int(top_3_list[0][1][0])
            video_pack.Target1_RU_y=int(top_3_list[0][1][1])
            video_pack.Target1_RD_x=int(top_3_list[0][2][0])
            video_pack.Target1_RD_y=int(top_3_list[0][2][1])
            video_pack.Target1_LD_x=int(top_3_list[0][3][0])
            video_pack.Target1_LD_y=int(top_3_list[0][3][1])
        else:
            video_pack.Target1_PR=0
            video_pack.Target1_LU_x=0
            video_pack.Target1_LU_y=0
            video_pack.Target1_RU_x=0
            video_pack.Target1_RU_y=0
            video_pack.Target1_RD_x=0
            video_pack.Target1_RD_y=0
            video_pack.Target1_LD_x=0
            video_pack.Target1_LD_y=0
        video_pack.Target2_Exist=top_3_list[1][5]
        if(video_pack.Target2_Exist):
            video_pack.Target2_PR=round(top_3_list[1][4],2)
            video_pack.Target2_LU_x=int(top_3_list[1][0][0])
            video_pack.Target2_LU_y=int(top_3_list[1][0][1])
            video_pack.Target2_RU_x=int(top_3_list[1][1][0])
            video_pack.Target2_RU_y=int(top_3_list[1][1][1])
            video_pack.Target2_RD_x=int(top_3_list[1][2][0])
            video_pack.Target2_RD_y=int(top_3_list[1][2][1])
            video_pack.Target2_LD_x=int(top_3_list[1][3][0])
            video_pack.Target2_LD_y=int(top_3_list[1][3][1])
        else:
            video_pack.Target2_PR=0
            video_pack.Target2_LU_x=0
            video_pack.Target2_LU_y=0
            video_pack.Target2_RU_x=0
            video_pack.Target2_RU_y=0
            video_pack.Target2_RD_x=0
            video_pack.Target2_RD_y=0
            video_pack.Target2_LD_x=0
            video_pack.Target2_LD_y=0
        video_pack.Target3_Exist=top_3_list[2][5]
        if(video_pack.Target3_Exist):
            video_pack.Target3_PR=round(top_3_list[2][4],2)
            video_pack.Target3_LU_x=int(top_3_list[2][0][0])
            video_pack.Target3_LU_y=int(top_3_list[2][0][1])
            video_pack.Target3_RU_x=int(top_3_list[2][1][0])
            video_pack.Target3_RU_y=int(top_3_list[2][1][1])
            video_pack.Target3_RD_x=int(top_3_list[2][2][0])
            video_pack.Target3_RD_y=int(top_3_list[2][2][1])
            video_pack.Target3_LD_x=int(top_3_list[2][3][0])
            video_pack.Target3_LD_y=int(top_3_list[2][3][1])
        else:
            video_pack.Target3_PR=0
            video_pack.Target3_LU_x=0
            video_pack.Target3_LU_y=0
            video_pack.Target3_RU_x=0
            video_pack.Target3_RU_y=0
            video_pack.Target3_RD_x=0
            video_pack.Target3_RD_y=0
            video_pack.Target3_LD_x=0
            video_pack.Target3_LD_y=0

        pub.publish(video_pack)
        # 按下 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rate.sleep()
    # 释放摄像头资源和关闭窗口
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

