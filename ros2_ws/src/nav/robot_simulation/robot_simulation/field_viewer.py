"""Read-only course presentation of actual simulation poses, paths and QR results."""
import json
import math
from pathlib import Path
import signal
import time

from .course_scene import load_scene, screen_point
from .world import corners
from robot_geometry.profile import load_profile


def main(args=None):
    import tkinter as tk
    from PIL import Image, ImageDraw, ImageFont, ImageTk
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as NavPath
    from sensor_msgs.msg import Image as RosImage
    from std_msgs.msg import Bool, String
    from cv_bridge import CvBridge
    from .planar_base import require_isolated_domain

    require_isolated_domain()
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('field_viewer')
    node.declare_parameter('scene_file', '')
    node.declare_parameter('geometry_file', '')
    node.declare_parameter('fullscreen', False)
    scene = load_scene(node.get_parameter('scene_file').value)
    profile = load_profile(node.get_parameter('geometry_file').value, allow_unconfirmed=True)
    empty_bounds = [v/1000. for v in profile['empty_bounds_mm']]
    loaded_bounds = [v/1000. for v in profile['loaded_bounds_mm']]
    root = tk.Tk()
    root.title('DASE7503 | Course field simulation')
    root.geometry('1440x900')
    root.minsize(960, 600)
    if node.get_parameter('fullscreen').value:
        root.attributes('-fullscreen', True)
    canvas = tk.Label(root, borderwidth=0, background='#f3f5f8')
    canvas.pack(fill='both', expand=True)
    font_file = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if not font_file.is_file():
        raise RuntimeError('Install fonts-noto-cjk to display the Chinese field viewer')
    fonts = {size: ImageFont.truetype(str(font_file), size=size) for size in (14, 16, 18, 20, 24, 26, 32, 38, 46)}
    state = {'phase': None, 'index': 0, 'pose': None, 'pose_at': None,
             'path': [], 'trail': [], 'loaded': False, 'codes': [], 'frame': None,
             'confirmed_frame': None, 'started': None, 'ended': None, 'closed': False}
    bridge = CvBridge()
    size = [value/1000. for value in scene['size_mm']]
    scale = min(457/size[1], 690/size[0])
    field = (435.5-size[1]*scale/2, 202, 435.5+size[1]*scale/2, 202+size[0]*scale)
    phase_names = {'WAIT_START': '等待起点扫码', 'NAV_TO_RACK': '前往货架',
                   'WAIT_RACK_QR': '识别货架二维码', 'LIFT_RACK': '模拟抬起',
                   'NAV_TO_DESTINATION': '运往目的地区域', 'VERIFY_DESTINATION': '确认终点二维码',
                   'LOWER_RACK': '模拟放下', 'COMPLETE': '四架配送流程完成', 'ERROR': '任务停止 · 检查日志'}

    def pose_callback(message):
        if message.header.frame_id != 'map':
            return
        p, q = message.pose.position, message.pose.orientation
        pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if not all(math.isfinite(value) for value in pose):
            return
        state['pose'], state['pose_at'] = pose, time.monotonic()
        trail = state['trail']
        if not trail or math.hypot(p.x-trail[-1][0], p.y-trail[-1][1]) > .025:
            trail.append((p.x, p.y))
            if len(trail) > 1800:
                del trail[0]

    def path_callback(message):
        if message.header.frame_id == 'map':
            state['path'] = [(p.pose.position.x, p.pose.position.y) for p in message.poses
                             if math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y)]

    def status_callback(message):
        try:
            status = json.loads(message.data)
            if status.get('state') not in phase_names or not isinstance(status.get('rack_index'), int):
                return
            state['phase'], state['index'] = status['state'], max(0, min(4, status['rack_index']))
            if status['state'] in ('COMPLETE', 'ERROR') and state['ended'] is None:
                state['ended'] = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            pass

    def image_callback(message):
        try:
            pixels = bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            # During travel the fixture sends blank frames. Preserve the last
            # real input image, explicitly presented as the latest scan image.
            if pixels.size and pixels.std() > 5:
                state['frame'] = Image.fromarray(pixels).copy()
        except Exception as exc:
            node.get_logger().warning(str(exc), throttle_duration_sec=5.)

    def qr_callback(message):
        state['codes'].append(message.data)
        if message.data == 'START' and state['started'] is None:
            state['started'] = time.monotonic()
        if state['frame'] is not None:
            state['confirmed_frame'] = state['frame'].copy()

    node.create_subscription(PoseStamped, '/simulation/pose', pose_callback, 10)
    node.create_subscription(NavPath, '/plan', path_callback, 10)
    node.create_subscription(String, '/mission/status', status_callback, 10)
    node.create_subscription(String, '/qr/data', qr_callback, 10)
    node.create_subscription(RosImage, '/camera/image_raw', image_callback, qos_profile_sensor_data)
    node.create_subscription(Bool, '/nav/loaded', lambda msg: state.update(loaded=msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def point(x, y):
        return screen_point(x, y, size, field)

    def rect(box_mm):
        x0, y0, x1, y1 = [value/1000. for value in box_mm]
        return (*point(x0, y0), *point(x1, y1))

    def close(*unused):
        state['closed'] = True

    root.protocol('WM_DELETE_WINDOW', close)
    root.bind('<Escape>', close)
    previous = {sig: signal.signal(sig, close) for sig in (signal.SIGINT, signal.SIGTERM)}

    def draw():
        if state['closed'] or not rclpy.ok():
            root.quit()
            return
        for _ in range(12):
            rclpy.spin_once(node, timeout_sec=0.)
        image = Image.new('RGB', (1600, 1000), '#f3f5f8')
        pen = ImageDraw.Draw(image)

        def text(x, y, value, fontsize=18, color='#26334a', anchor='la'):
            pen.text((x, y), str(value), font=fonts[fontsize], fill=color, anchor=anchor)

        def card(box, fill='white', radius=18, outline=None):
            pen.rounded_rectangle(box, radius=radius, fill=fill, outline=outline)

        text(42, 23, 'DASE7503  /  NAVIGATION & VISION', 16, '#64748b')
        text(40, 48, '比赛场地 · 导航与扫码', 38)
        card((1205, 43, 1558, 92), '#e5eefc', 24)
        text(1382, 54, '课程布局  ·  平面流程仿真', 18, '#245aa0', 'ma')
        card((36, 132, 855, 948))
        text(61, 151, '场地俯视图', 24)
        text(829, 158, '起点在下 · 目的地在上', 16, '#64748b', 'ra')
        # Grey floor, green destination, blue start and amber rack targets
        # follow the course diagram, without showing costmap inflation bands.
        pen.rectangle(field, fill='#e9edf0', outline='#334155', width=5)
        for x in (i*.5 for i in range(1, math.ceil(size[0]/.5))):
            pen.line([point(x, 0), point(x, size[1])], fill='#dde3e8', width=1)
        for y in (i*.5 for i in range(1, math.ceil(size[1]/.5))):
            pen.line([point(0, y), point(size[0], y)], fill='#dde3e8', width=1)
        pen.rectangle(rect(scene['destination_zone_mm']), fill='#d4efde', outline='#68ae83', width=2)
        pen.rectangle(rect(scene['start_zone_mm']), fill='#d8e8fd', outline='#7da6df', width=2)
        end_rect, start_rect = rect(scene['destination_zone_mm']), rect(scene['start_zone_mm'])
        text((end_rect[0]+end_rect[2])/2, end_rect[1]+28, 'END  目的地区域', 20, '#287248', 'ma')
        text((start_rect[0]+start_rect[2])/2, start_rect[1]+8, 'START', 20, '#285e9e', 'ma')
        for obstacle in scene['obstacles_mm']:
            pen.rectangle(rect(obstacle), fill='#a3acb8', outline='#566378', width=2)
        text((field[0]+field[2])/2, field[3]+15, f'{size[1]:.2f} m', 18, '#64748b', 'ma')
        for edge in (field[0], field[2]):
            pen.line([(edge, field[3]+8), (edge, field[3]+17)], fill='#94a3b8', width=2)
        text(field[0]-19, (field[1]+field[3])/2-14, f'{size[0]:.2f} m', 16, '#64748b', 'ra')
        pen.line([(field[0]-13, field[1]), (field[0]-13, field[3])], fill='#cbd5e1', width=1)
        if len(state['trail']) > 1:
            pen.line([point(*p) for p in state['trail']], fill='#a9c8c5', width=3)
        if len(state['path']) > 1 and state['phase'] in ('NAV_TO_RACK', 'NAV_TO_DESTINATION'):
            pen.line([point(*p) for p in state['path']], fill='#007e87', width=4)
        for index, rack in enumerate(scene['racks']):
            x, y = point(rack['x_mm']/1000., rack['y_mm']/1000.)
            done = index < state['index']
            carrying = index == state['index'] and state['loaded']
            fill, ink = ('#e1e7e5', '#7d9187') if done or carrying else ('#ffdc83', '#6e4b0b')
            card((x-38, y-18, x+38, y+18), fill, 6, '#d0ae58' if not done else '#b8c9c0')
            text(x, y-15, rack['label'], 24, ink, 'ma')
            text(x, y+21, '已送达' if done else ('搬运中' if carrying else '货架 '+rack['label']), 16, ink, 'ma')
        if state['pose'] is not None:
            pose = state['pose']
            if state['loaded']:
                pen.polygon([point(*p) for p in corners(pose, loaded_bounds)],
                            fill='#bbe8da', outline='#29876c', width=2)
            body = [point(*p) for p in corners(pose, empty_bounds)]
            pen.polygon([(x+3, y+4) for x, y in body], fill='#aeb8c5')
            pen.polygon(body, fill='#2867c7', outline='#174386', width=2)
            x, y, yaw = pose
            nose = point(x+.16*math.cos(yaw), y+.16*math.sin(yaw))
            rear = point(x-.04*math.cos(yaw), y-.04*math.sin(yaw))
            pen.line([rear, nose], fill='white', width=4)
            pen.ellipse((nose[0]-4, nose[1]-4, nose[0]+4, nose[1]+4), fill='white')
        pen.line([(699, 821), (732, 821)], fill='#007e87', width=4)
        text(697, 831, '规划路线', 14, '#64748b')
        pen.line([(699, 866), (732, 866)], fill='#a9c8c5', width=3)
        text(697, 876, '已行轨迹', 14, '#64748b')
        card((889, 132, 1564, 393))
        phase = state['phase']
        title = phase_names.get(phase, '正在初始化导航…')
        if phase and phase not in ('COMPLETE', 'ERROR', 'WAIT_START'):
            title += '  ·  '+scene['racks'][min(state['index'], 3)]['label']
        text(918, 153, '任务进度', 16, '#64748b')
        text(918, 181, title, 26, '#16735e' if phase != 'ERROR' else '#b23b3b')
        elapsed = '--' if state['started'] is None else f"{(state['ended'] or time.monotonic())-state['started']:.1f}"
        text(918, 234, f"{state['index']} / 4", 46)
        text(1090, 258, '已送达', 18, '#64748b')
        text(1302, 234, elapsed, 46)
        text(1480, 258, '秒', 18, '#64748b')
        for index, rack in enumerate(scene['racks']):
            x = 918 + index*153
            completed = index < state['index']
            active = index == state['index'] and phase not in (None, 'WAIT_START', 'COMPLETE', 'ERROR')
            card((x, 322, x+133, 366), '#d9f1e6' if completed else ('#e5eefc' if active else '#f1f4f7'), 10)
            text(x+66, 331, rack['label'] + ('  已完成' if completed else ('  进行中' if active else '  待处理')),
                 16, '#297357' if completed else '#40516a', 'ma')
        card((889, 411, 1564, 797))
        text(918, 432, '最近一次扫码画面', 24)
        frame = state['confirmed_frame'] or state['frame']
        card((918, 485, 1190, 757), '#f5f7fa', 12)
        if frame is not None:
            frame = frame.copy()
            frame.thumbnail((256, 256), Image.Resampling.NEAREST)
            image.paste(frame, (1054-frame.width//2, 621-frame.height//2))
        else:
            text(1054, 600, '等待图像', 20, '#94a3b8', 'ma')
        text(1220, 495, '确认结果', 16, '#64748b')
        text(1220, 526, state['codes'][-1] if state['codes'] else '等待 START', 24)
        text(1220, 581, '输入：模拟二维码图片', 16, '#64748b')
        text(1220, 612, '识别：实际视觉节点', 16, '#64748b')
        text(1220, 666, '画面保留至下次扫码', 16, '#64748b')
        card((889, 815, 1564, 948))
        text(918, 835, '扫码确认记录', 20)
        text(1534, 838, f"{len(state['codes'])} 次", 18, '#64748b', 'ra')
        short = [code[4] if code.startswith('RACK') and len(code) > 4 else code for code in state['codes']]
        text(918, 879, ' → '.join(short) if short else '等待任务开始', 18, '#24696c')
        if state['pose_at'] is not None and time.monotonic()-state['pose_at'] > 2.:
            text(1200, 951, '仿真位姿数据已暂停', 16, '#b45309')
        text(40, 965, '课程尺寸；货架位置按示意图估计。货架为任务点，未模拟腿部碰撞、取放接触和相机光照。', 16, '#64748b')
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        ratio = min(width/1600., height/1000.)
        image = image.resize((max(1, round(1600*ratio)), max(1, round(1000*ratio))), Image.Resampling.LANCZOS)
        photo = ImageTk.PhotoImage(image, master=root)
        canvas.configure(image=photo)
        canvas.image = photo
        root.after(70, draw)

    try:
        draw()
        root.mainloop()
    finally:
        root.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
