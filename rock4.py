import math

def calculate_air_density(alt, temp_c, humidity_pct):
    if alt < 0:
        alt = 0.0
    if alt > 100000:
        return 0.0

    T_k = temp_c + 273.15
    P_0 = 101325.0
    R_d = 287.05

    pressure = P_0 * math.exp(-alt / 8500.0)

    es = 610.78 * math.exp((17.27 * temp_c) / (temp_c + 237.3))
    e = es * (humidity_pct / 100.0)

    p_total = pressure
    if p_total > e:
        tv = T_k / (1.0 - (e / p_total) * (1.0 - 0.622))
    else:
        tv = T_k

    density = (pressure - (0.378 * e)) / (R_d * tv)
    return max(density, 0.0)

def get_wind_velocity(alt, base_wind_speed):
    # wind shear profile
    altitude_multiplier = 1.0 + (alt / 10000.0) if alt < 15000 else 2.5
    return base_wind_speed * altitude_multiplier

def get_drag_coefficient(mach):
    if mach < 0.8:
        return 0.3
    elif mach < 1.2:
        return 0.65  # transonic drag spike
    else:
        return 0.25

class PIDController:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit  # max gimbal angle in radians
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        output = max(-self.output_limit, min(self.output_limit, output))
        self.previous_error = error
        return output

def run_tvc_simulation():
    print("-------------------------------------------------------------")
    print(" rocket simulation rock4: active tvc & pid stabilization ")
    print("-------------------------------------------------------------")

    try:
        dry_mass = float(input("enter dry mass (kg) [default 45.0]: ") or 45.0)
        fuel_mass = float(input("enter fuel mass (kg) [default 250.0]: ") or 250.0)
        thrust = float(input("enter ascent thrust (N) [default 6000.0]: ") or 6000.0)
        burn_time = float(input("enter burn time (s) [default 25.0]: ") or 25.0)
        reference_area = float(input("enter cross-section area (m^2) [default 0.05]: ") or 0.05)

        print("\n--- tvc control & disturbance parameters ---")
        kp = float(input("enter pid proportional gain (kp) [default 1.2]: ") or 1.2)
        ki = float(input("enter pid integral gain (ki) [default 0.05]: ") or 0.05)
        kd = float(input("enter pid derivative gain (kd) [default 0.3]: ") or 0.3)
        max_gimbal_deg = float(input("enter max nozzle gimbal angle (°) [default 7.0]: ") or 7.0)
        base_wind = float(input("enter crosswind speed (m/s) [default 12.0]: ") or 12.0)

    except ValueError:
        dry_mass, fuel_mass, thrust, burn_time, reference_area = 45.0, 250.0, 6000.0, 25.0, 0.05
        kp, ki, kd, max_gimbal_deg, base_wind = 1.2, 0.05, 0.3, 7.0, 12.0

    mass_flow_rate = fuel_mass / burn_time if burn_time > 0 else 0
    max_gimbal_rad = math.radians(max_gimbal_deg)
    tvc_pid = PIDController(kp, ki, kd, max_gimbal_rad)

    dt = 0.05
    t = 0.0
    x = 0.0
    y = 0.0
    vx = 0.0
    vy = 0.0

    theta = 0.0
    omega = 0.0
    moment_of_inertia = 15.0 
    distance_to_cg = 1.5

    current_fuel = fuel_mass
    max_q = 0.0
    max_gimbal_used = 0.0
    max_tilt_deg = 0.0
    apogee = 0.0
    phase = "ASCENT"

    print("\nrunning active tvc stabilisation flight loop...\n")

    while y >= 0.0 or phase == "ASCENT":
        gravity = 9.81
        rho = calculate_air_density(y, 25.0, 60.0)
        wind_vx = get_wind_velocity(y, base_wind)

        if phase == "ASCENT":
            if current_fuel > 0 and t <= burn_time:
                current_mass = dry_mass + current_fuel + 10.0
                current_thrust = thrust
                current_fuel -= mass_flow_rate * dt
            else:
                current_mass = dry_mass + 10.0
                current_thrust = 0.0
                phase = "COAST"
        else:
            current_mass = dry_mass + 10.0
            current_thrust = 0.0
            if vy <= 0:
                phase = "DESCENT"

        target_theta = 0.05 if phase == "ASCENT" and t < 10.0 else 0.0
        attitude_error = target_theta - theta

        gimbal_angle = 0.0
        if phase == "ASCENT" and current_thrust > 0:
            gimbal_angle = tvc_pid.compute(attitude_error, dt)
            if abs(gimbal_angle) > max_gimbal_used:
                max_gimbal_used = abs(gimbal_angle)

        tvc_torque = current_thrust * math.sin(gimbal_angle) * distance_to_cg
        v_rel_x = vx - wind_vx
        v_rel_y = vy
        v_rel = math.sqrt(v_rel_x**2 + v_rel_y**2)
        aero_torque = 0.5 * rho * v_rel**2 * 0.1 * math.sin(theta) * 2.0

        angular_acceleration = (tvc_torque - aero_torque) / moment_of_inertia
        omega += angular_acceleration * dt
        theta += omega * dt

        if math.degrees(abs(theta)) > max_tilt_deg:
            max_tilt_deg = math.degrees(abs(theta))

        total_angle = theta + gimbal_angle
        thrust_x = current_thrust * math.sin(total_angle)
        thrust_y = current_thrust * math.cos(total_angle)

        sound_speed = 340.0
        mach = v_rel / sound_speed
        cd = get_drag_coefficient(mach)
        q = 0.5 * rho * v_rel**2
        if q > max_q:
            max_q = q

        drag_force = q * cd * reference_area 
        drag_x = (drag_force * (v_rel_x / v_rel)) if v_rel > 0.001 else 0.0
        drag_y = (drag_force * (v_rel_y / v_rel)) if v_rel > 0.001 else 0.0

        accel_x = (thrust_x - drag_x) / current_mass
        accel_y = (thrust_y - (current_mass * gravity) - drag_y) / current_mass

        vx += accel_x * dt
        vy += accel_y * dt
        x += vx * dt
        y += vy * dt
        t += dt

        if y > apogee and phase in ["ASCENT", "COAST"]:
            apogee = y
        
        if y <= 0.0:
            y = 0.0
            break

    print("----------------------------------------")
    print("tvc & pid stabilization summary:")
    print(f"  - peak apogee reached : {apogee:.1f} m ({apogee / 1000.0:.2f} km)")
    print(f"  - max dynamic pressure: {max_q:.1f} pa ({max_q / 1000.0:.2f} kpa)")
    print(f"  - max attitude tilt   : {max_tilt_deg:.2f}° (against {base_wind} m/s wind)")
    print(f"  - max nozzle gimbal   : {math.degrees(max_gimbal_used):.2f}° (limit: {max_gimbal_deg}°)")
    print("----------------------------------------")
    if max_tilt_deg < 25.0:
        print("RESULT: success, pid stabilization held strong against wind disturbance.")
    else:
        print("RESULT: tumbled due to excessive wind shear.")

if __name__ == "__main__":
    run_tvc_simulation()