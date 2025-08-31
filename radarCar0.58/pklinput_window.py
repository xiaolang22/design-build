import tkinter as tk
from tkinter import messagebox, PhotoImage
import subprocess
import sys
from PIL import Image, ImageTk, ImageDraw
import os
from pathlib import Path

# 创建圆形图像函数
def create_circle_image(size, color):
    image = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    draw.ellipse((0, 0, size, size), fill=color)
    return ImageTk.PhotoImage(image)

# 创建圆角矩形函数
def create_rounded_rectangle(width, height, radius, fill_color):
    image = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    draw.rounded_rectangle([(0, 0), (width, height)], radius, fill=fill_color)
    return ImageTk.PhotoImage(image)

# 提交按钮回调函数
def submit():
    filename = entry.get().strip()
    if not filename:
        show_custom_message("提示", "请输入文件名！")
        return
    try:
        # 调用 run_dwa_path_tracking.py 并传递参数
        subprocess.Popen([sys.executable, "run_dwa_path_tracking.py", filename])
        root.destroy()  # 关闭窗口
    except Exception as e:
        show_custom_message("错误", f"无法运行脚本: {e}")

# 自定义消息框
def show_custom_message(title, message):
    msg_window = tk.Toplevel(root)
    msg_window.title(title)
    msg_window.geometry("300x150")
    msg_window.configure(bg="#f0f0f0")
    msg_window.resizable(False, False)
    msg_window.transient(root)
    msg_window.grab_set()
    
    # 创建圆形信息图标
    info_frame = tk.Frame(msg_window, bg="#f0f0f0")
    info_frame.pack(pady=(15, 5))
    
    info_icon = create_circle_image(50, "#1E90FF")
    info_label = tk.Label(info_frame, image=info_icon, bg="#f0f0f0")
    info_label.image = info_icon
    info_label.pack()
    
    # 在圆形上添加"i"
    i_label = tk.Label(info_frame, text="i", font=("Arial", 24, "bold"), fg="white", bg="#1E90FF")
    i_label.place(relx=0.5, rely=0.5, anchor="center")
    
    # 消息文本
    msg_label = tk.Label(msg_window, text=message, font=("微软雅黑", 12), bg="#f0f0f0", wraplength=250)
    msg_label.pack(pady=10)
    
    # OK按钮
    ok_button_frame = tk.Frame(msg_window, bg="#f0f0f0")
    ok_button_frame.pack(pady=10)
    
    ok_btn_bg = create_rounded_rectangle(80, 35, 15, "#FF69B4")
    ok_btn = tk.Button(
        ok_button_frame, 
        image=ok_btn_bg, 
        text="OK", 
        font=("微软雅黑", 12, "bold"),
        fg="white",
        compound="center",
        borderwidth=0,
        highlightthickness=0,
        command=msg_window.destroy,
        cursor="hand2"
    )
    ok_btn.image = ok_btn_bg
    ok_btn.pack()
    
    # 居中显示
    msg_window.update_idletasks()
    width = msg_window.winfo_width()
    height = msg_window.winfo_height()
    x = (msg_window.winfo_screenwidth() // 2) - (width // 2)
    y = (msg_window.winfo_screenheight() // 2) - (height // 2)
    msg_window.geometry(f"{width}x{height}+{x}+{y}")

# 创建主窗口
root = tk.Tk()
root.title("DWA寻路")
root.geometry("400x180")
root.configure(bg="#f0f0f0")
root.resizable(False, False)
root.lift()  # 窗口前置
root.attributes('-topmost', True)  # 置顶

# 创建图标
icon_frame = tk.Frame(root, bg="#f0f0f0")
icon_frame.pack(pady=(15, 5))

info_icon = create_circle_image(50, "#1E90FF")
icon_label = tk.Label(icon_frame, image=info_icon, bg="#f0f0f0")
icon_label.image = info_icon
icon_label.pack()

# 在圆形上添加"i"
i_label = tk.Label(icon_frame, text="i", font=("Arial", 24, "bold"), fg="white", bg="#1E90FF")
i_label.place(relx=0.5, rely=0.5, anchor="center")

# 标题文本
title_label = tk.Label(
    root, 
    text="请输入pkl点云文件名运行DWA寻路", 
    font=("微软雅黑", 14, "bold"), 
    bg="#f0f0f0"
)
title_label.pack(pady=(5, 15))

# 输入框容器
entry_frame = tk.Frame(root, bg="#f0f0f0")
entry_frame.pack(pady=5, fill="x", padx=50)

# 输入框
entry = tk.Entry(
    entry_frame, 
    width=30, 
    font=("微软雅黑", 12),
    relief="solid",
    bd=1
)
entry.pack(fill="x")
entry.focus_set()  # 自动聚焦

# 按钮容器
button_frame = tk.Frame(root, bg="#f0f0f0")
button_frame.pack(pady=15)

# 提交按钮
ok_btn_bg = create_rounded_rectangle(80, 35, 15, "#FF69B4")
submit_btn = tk.Button(
    button_frame, 
    image=ok_btn_bg, 
    text="OK", 
    font=("微软雅黑", 12, "bold"),
    fg="white",
    compound="center",
    borderwidth=0,
    highlightthickness=0,
    command=submit,
    cursor="hand2"
)
submit_btn.image = ok_btn_bg
submit_btn.pack()

# 回车键绑定提交
root.bind('<Return>', lambda event: submit())

# 居中显示
root.update_idletasks()
width = root.winfo_width()
height = root.winfo_height()
x = (root.winfo_screenwidth() // 2) - (width // 2)
y = (root.winfo_screenheight() // 2) - (height // 2)
root.geometry(f"{width}x{height}+{x}+{y}")

root.mainloop()
