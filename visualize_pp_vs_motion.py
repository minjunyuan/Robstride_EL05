import math

import matplotlib.pyplot as plt


# 纯仿真：对比 PP 点到点模式和运控/连续跟随模式的曲线感受。
# 不连接电机，不发送任何串口/CAN 数据。

DT = 0.01
T_END = 8.0


def target_at(t):
    # 模拟视觉目标每隔一段时间跳变。
    steps = [
        (0.0, 0.0),
        (1.0, 25.0),
        (2.2, -10.0),
        (3.3, 18.0),
        (4.7, -25.0),
        (6.0, 8.0),
    ]
    value = steps[0][1]
    for ts, val in steps:
        if t >= ts:
            value = val
        else:
            break
    return math.radians(value)


def simulate_pp_like():
    # 简化 PP 感觉：目标变化后，每段都倾向于缓起缓停，到目标附近速度归零。
    pos = 0.0
    vel = 0.0
    max_vel = math.radians(45)
    max_acc = math.radians(100)
    kp_profile = 4.0

    out = []
    t = 0.0
    while t <= T_END:
        target = target_at(t)
        err = target - pos

        # 离目标越近，期望速度越小，形成每段“到点刹车”的感觉。
        vel_des = max(-max_vel, min(max_vel, kp_profile * err))
        dv = max(-max_acc * DT, min(max_acc * DT, vel_des - vel))
        vel += dv
        pos += vel * DT

        out.append((t, target, pos, vel))
        t += DT
    return out


def simulate_motion_like():
    # 简化运控/连续跟随感觉：先对目标低通，再限速限加速度，追的是平滑目标。
    pos = 0.0
    vel = 0.0
    smooth_target = 0.0
    max_vel = math.radians(35)
    max_acc = math.radians(70)
    follow_gain = 3.0
    alpha = 0.04

    out = []
    t = 0.0
    while t <= T_END:
        target = target_at(t)
        smooth_target += alpha * (target - smooth_target)
        err = smooth_target - pos

        vel_des = max(-max_vel, min(max_vel, follow_gain * err))
        dv = max(-max_acc * DT, min(max_acc * DT, vel_des - vel))
        vel += dv
        pos += vel * DT

        out.append((t, target, pos, vel))
        t += DT
    return out


pp = simulate_pp_like()
motion = simulate_motion_like()

times = [x[0] for x in pp]
targets_deg = [math.degrees(x[1]) for x in pp]
pp_pos_deg = [math.degrees(x[2]) for x in pp]
pp_vel_deg = [math.degrees(x[3]) for x in pp]
motion_pos_deg = [math.degrees(x[2]) for x in motion]
motion_vel_deg = [math.degrees(x[3]) for x in motion]

fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

axes[0].plot(times, targets_deg, "k--", label="target")
axes[0].plot(times, pp_pos_deg, label="PP-like position")
axes[0].plot(times, motion_pos_deg, label="motion-control-like position")
axes[0].set_ylabel("position (deg)")
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].plot(times, pp_vel_deg, label="PP-like velocity")
axes[1].plot(times, motion_vel_deg, label="motion-control-like velocity")
axes[1].set_ylabel("velocity (deg/s)")
axes[1].set_xlabel("time (s)")
axes[1].legend()
axes[1].grid(True, alpha=0.3)

fig.suptitle("PP point-to-point vs motion-control-style continuous following")
fig.tight_layout()
plt.show()
