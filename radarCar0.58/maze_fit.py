import cv2
import numpy as np
import matplotlib.pyplot as plt

def process_maze(image_path):
    # Load the image in grayscale
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    # Threshold the image to separate walls (black) from walkable area (white/gray)
    _, thresholded = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)
    
    # Apply a median filter to remove small noise points
    filtered = cv2.medianBlur(thresholded, 5)

    # 找到所有轮廓
    contours, _ = cv2.findContours(filtered, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 创建全白底图（迷宫外部）
    maze_processed = np.ones_like(image, dtype=np.uint8) * 255

    # 画出最大轮廓（假设为外部区域）
    if contours:
        # 找到最大轮廓
        max_contour = max(contours, key=cv2.contourArea)
        # 填充最大轮廓为0（黑色，暂时作为掩码）
        cv2.drawContours(maze_processed, [max_contour], -1, (0,), thickness=cv2.FILLED)

    # 现在maze_processed为：外部255，内部0
    # 迷宫墙体mask
    wall_mask = filtered == 255
    # 迷宫内部mask（最大轮廓内，且不是墙体）
    inside_mask = (maze_processed == 0) & (~wall_mask)
    # 迷宫外部mask
    outside_mask = maze_processed == 255

    # 记录原图中像素值为127的点，作为外部掩码
    outside_mask_127 = image == 127

    # 生成最终图像：外部255，内部浅绿色，墙体0
    final_img = np.ones((*image.shape, 3), dtype=np.uint8) * 255  # 三通道全白
    # 浅绿色 (BGR): (144, 238, 144)
    final_img[inside_mask] = (144, 238, 144)
    final_img[wall_mask] = (0, 0, 0)
    # 像素值为127的点强制设为白色
    final_img[outside_mask_127] = (255, 255, 255)

    # 显示结果
    plt.figure(figsize=(10, 10))
    plt.imshow(cv2.cvtColor(final_img, cv2.COLOR_BGR2RGB))
    plt.axis('off')
    plt.show()

    # 保存处理后的图像到当前目录
    cv2.imwrite('maze_processed.png', final_img)

    return final_img

# Process the uploaded image
image_path = 'test.png'
processed_image = process_maze(image_path)
