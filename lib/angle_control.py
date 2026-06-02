import math
import time


def deg_to_rad(deg: float) -> float:
    return float(deg) * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    return float(rad) * 180.0 / math.pi


def wrap_to_pi(rad: float) -> float:
    return ((float(rad) + math.pi) % (2.0 * math.pi)) - math.pi


def nearest_equivalent_rad(current_rad: float, target_rad: float) -> float:
    turns = round((float(current_rad) - float(target_rad)) / (2.0 * math.pi))
    return float(target_rad) + turns * 2.0 * math.pi


def read_wrapped_angle(bus, motor_id: int, max_age: float | None = None) -> tuple[float, float]:
    feedback_rad = bus.get_position_rad(motor_id, max_age=max_age)
    if feedback_rad is None:
        raise RuntimeError(f"motor {motor_id}: no fresh feedback")
    return feedback_rad, rad_to_deg(wrap_to_pi(feedback_rad))


def require_angle_in_range(motor_id: int, angle_deg: float, allowed_deg: tuple[float, float], label: str) -> None:
    lo, hi = allowed_deg
    if angle_deg < lo or angle_deg > hi:
        raise RuntimeError(f"motor {motor_id}: {label} {angle_deg:.2f} deg outside [{lo:.2f}, {hi:.2f}] deg")


def require_feedback_jump_ok(motor_id: int, last_rad: float, current_rad: float, max_jump_deg: float) -> None:
    jump_deg = abs(rad_to_deg(current_rad - last_rad))
    if jump_deg > max_jump_deg:
        raise RuntimeError(
            f"motor {motor_id}: feedback jumped {jump_deg:.2f} deg "
            f"({rad_to_deg(last_rad):.2f} -> {rad_to_deg(current_rad):.2f})"
        )


def nearest_command_position(
    motor_id: int,
    feedback_rad: float,
    desired_angle_rad: float,
    max_move_deg: float,
) -> float:
    command_rad = nearest_equivalent_rad(feedback_rad, desired_angle_rad)
    move_deg = abs(rad_to_deg(command_rad - feedback_rad))
    if move_deg > max_move_deg:
        raise RuntimeError(f"motor {motor_id}: target move {move_deg:.2f} deg exceeds {max_move_deg:.2f} deg")
    return command_rad


def return_motors_to_zero(
    bus,
    motor_ids,
    allowed_angle_deg,
    *,
    kp: float = 2.0,
    kd: float = 1.8,
    torque_nm: float = 0.0,
    control_hz: float = 100.0,
    done_deg: float = 1.0,
    max_target_move_deg: float = 200.0,
    max_feedback_jump_deg: float = 30.0,
    feedback_timeout: float = 3.0,
    feedback_max_age: float = 0.2,
    print_status: bool = True,
) -> None:
    """Move one or more motors back to wrapped zero using nearest raw targets.

    The motors are left enabled in motion mode holding zero, so the caller can
    start the next workflow from the initialized position.
    """
    ids = [motor_ids] if isinstance(motor_ids, int) else list(motor_ids)
    if isinstance(allowed_angle_deg, dict):
        angle_range = allowed_angle_deg
    else:
        angle_range = {motor_id: allowed_angle_deg for motor_id in ids}

    try:
        bus.flush_rx()
        for motor_id in ids:
            bus.stop(motor_id)
            bus.set_feedback_active(motor_id, True)
            time.sleep(0.05)

        bus.wait_feedback(ids[0] if len(ids) == 1 else ids, timeout=feedback_timeout)

        last_feedback_rad = {}
        for motor_id in ids:
            feedback_rad, startup_angle_deg = read_wrapped_angle(bus, motor_id, max_age=0.5)
            require_angle_in_range(motor_id, startup_angle_deg, angle_range[motor_id], "startup angle")
            last_feedback_rad[motor_id] = feedback_rad
            if print_status:
                print(
                    f"{motor_id}: startup feedback={rad_to_deg(feedback_rad):.2f} deg, "
                    f"wrapped_angle={startup_angle_deg:.2f} deg"
                )

        for motor_id in ids:
            bus.set_motion_mode(motor_id)
            bus.enable(motor_id)
            time.sleep(0.05)

        while True:
            bus.receive_feedback(0.001)

            done = True
            for motor_id in ids:
                try:
                    feedback_rad, wrapped_angle_deg = read_wrapped_angle(
                        bus,
                        motor_id,
                        max_age=feedback_max_age,
                    )
                except RuntimeError:
                    done = False
                    continue

                require_angle_in_range(motor_id, wrapped_angle_deg, angle_range[motor_id], "wrapped angle")
                require_feedback_jump_ok(motor_id, last_feedback_rad[motor_id], feedback_rad, max_feedback_jump_deg)
                last_feedback_rad[motor_id] = feedback_rad

                command_position_rad = nearest_command_position(
                    motor_id,
                    feedback_rad,
                    0.0,
                    max_target_move_deg,
                )

                bus.motion_control(
                    motor_id,
                    pos_rad=command_position_rad,
                    vel_rad_s=0.0,
                    kp=kp,
                    kd=kd,
                    torque_nm=torque_nm,
                )

                if print_status:
                    print(
                        f"{motor_id}: feedback={rad_to_deg(feedback_rad):.2f} deg, "
                        f"wrapped_angle={wrapped_angle_deg:.2f} deg, "
                        f"command_position={rad_to_deg(command_position_rad):.2f} deg"
                    )

                if abs(wrapped_angle_deg) > done_deg:
                    done = False

            if done:
                return

            time.sleep(1 / control_hz)

    except Exception:
        for motor_id in ids:
            bus.stop(motor_id)
        raise
