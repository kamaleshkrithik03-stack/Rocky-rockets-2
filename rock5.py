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

    #(humidity changes air density)
    es = 610.78 * math.exp((17.27 * temp_c) / (temp_c + 237.3))
    e = es * (humidity_pct / 100.0)

   
    p_total = pressure
    if p_total > e:
        tv = T_k / (1.0 - (e / p_total) * (1.0 - 0.622))
    else:
        tv = T_k

    # Ideal gas law for moist air density
    density = (pressure - (0.378 * e)) / (R_d * tv)
    return max(density, 0.0)

def get_wind_velocity(alt, base_wind_speed, wind_direction_mode):
    #  winds get stronger with altitude
    altitude_multiplier = 1.0 + (alt / 10000.0) if alt < 15000 else 2.5
    current_wind = base_wind_speed * altitude_multiplier
    
   
    direction_sign = 1.0 if wind_direction_mode.lower() in ["tailwind", "east", "following"] else -1.0
    return current_wind * direction_sign

def get_drag_coefficient(mach):
    # Transonic drag  spike right around Mach 1(stuff that kills most rc planes)
    if mach < 0.8:
        return 0.3
    elif mach < 1.2:
        return 0.65  
    else:
        return 0.25  

class PIDController:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit  # max gimbal angle in radians(i've suffered with this irl in arduino)
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, error, dt):
        
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        
        output = max(-self.output_limit, min(self.output_limit, output))
        self.previous_error = error
        return output

def run_rock5_simulation():
    print("=" * 65)
    print(" ROCKY-ROCKETS: FULL-FLIGHT TVC & LANDING SIMULATOR ")
    print("=" * 65)

    try:
        dry_mass = float(input("enter dry mass (kg) [default 45.0]: ") or 45.0)
        fuel_mass = float(input("enter ascent fuel mass (kg) [default 200.0]: ") or 200.0)
        landing_fuel = float(input("enter landing reserve fuel (kg) [default 35.0]: ") or 35.0)
        payload_mass = float(input("enter payload mass (kg) [default 10.0]: ") or 10.0)
        thrust = float(input("enter ascent thrust (N) [default 5500.0]: ") or 5500.0)
        landing_thrust = float(input("Enter landing retro-thrust (N) [default 4500.0]: ") or 4500.0)
        burn_time = float(input("Enter ascent burn time (s) [default 22.0]: ") or 22.0)
        reference_area = float(input("Enter cross-section area (m^2) [default 0.05]: ") or 0.05)

        print("\n--- Environmental & Guidance Tuning ---")
        temp_c = float(input("enter ground ambient temperature (°C) [default 25.0]: ") or 25.0)
        humidity_pct = float(input("enter relative humidity (%) [default 60.0]: ") or 60.0)
        base_wind = float(input("enter base crosswind speed (m/s) [default 10.0]: ") or 10.0)
        wind_dir = input("enter wind direction ('Tailwind' or 'Headwind') [default Tailwind]: ") or "Tailwind"
        kp = float(input("enter PID Proportional Gain (Kp) [default 1.2]: ") or 1.2)
        ki = float(input("enter PID Integral Gain (Ki) [default 0.05]: ") or 0.05)
        kd = float(input("enter PID Derivative Gain (Kd) [default 0.3]: ") or 0.3)
        max_gimbal_deg = float(input("Enter max nozzle gimbal angle (°) [default 7.0]: ") or 7.0)

    except ValueError:
        dry_mass, fuel_mass, landing_fuel, payload_mass = 45.0, 200.0, 35.0, 10.0
        thrust, landing_thrust, burn_time, reference_area = 5500.0, 4500.0, 22.0, 0.05
        temp_c, humidity_pct, base_wind, wind_dir = 25.0, 60.0, 10.0, "Tailwind"
        kp, ki, kd, max_gimbal_deg = 1.2, 0.05, 0.3, 7.0

    mass_flow_rate = fuel_mass / burn_time if burn_time > 0 else 0
    landing_burn_rate = 6.0
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
    current_landing_fuel = landing_fuel

    max_q = 0.0
    max_drift = 0.0
    max_gimbal_used = 0.0
    max_tilt_deg = 0.0
    apogee = 0.0
    phase = "ASCENT"

    print("\n[INFO] Initializing flight computer loop...\n")

    while y >= 0.0 or phase in ["ASCENT", "COAST", "DESCENT", "LANDING_BURN"]:
        gravity = 9.81
        rho = calculate_air_density(y, temp_c, humidity_pct)
        wind_vx = get_wind_velocity(y, base_wind, wind_dir)

        # State machine managing mission profile
        if phase == "ASCENT":
            if current_fuel > 0 and t <= burn_time:
                current_mass = dry_mass + current_fuel + current_landing_fuel + payload_mass
                current_thrust = thrust
                current_fuel -= mass_flow_rate * dt
            else:
                current_mass = dry_mass + current_landing_fuel + payload_mass
                current_thrust = 0.0
                phase = "COAST"
        elif phase == "COAST":
            current_mass = dry_mass + current_landing_fuel + payload_mass
            current_thrust = 0.0
            if vy <= 0:
                apogee = y
                phase = "DESCENT"
        elif phase == "DESCENT":
            current_mass = dry_mass + current_landing_fuel + payload_mass
            current_thrust = 0.0
            # Fire landing retro-burn if dropping fast close to the ground
            if y < 1500.0 and vy < -15.0 and current_landing_fuel > 0:
                phase = "LANDING_BURN"
        elif phase == "LANDING_BURN":
            if current_landing_fuel > 0 and y > 0:
                current_mass = dry_mass + current_landing_fuel + payload_mass
                current_thrust = landing_thrust
                current_landing_fuel -= landing_burn_rate * dt
            else:
                current_thrust = 0.0
                phase = "FREE_FALL"
        else:
            current_mass = dry_mass + payload_mass
            current_thrust = 0.0

        
        target_theta = 0.03 if phase == "ASCENT" and t < 8.0 else 0.0
        attitude_error = target_theta - theta

        gimbal_angle = 0.0
        if current_thrust > 0 and phase in ["ASCENT", "LANDING_BURN"]:
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

        # 2D Force 
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
        if abs(x) > max_drift:
            max_drift = abs(x)

        if y <= 0.0:
            y = 0.0
            break

    print("=" * 65)
    print(" FLIGHT TELEMETRY REPORT (ROCK5) ")
    print("=" * 65)
    print(f"  - Peak Apogee Reached : {apogee:.1f} m ({apogee / 1000.0:.2f} km)")
    print(f"  - Max Dynamic Pressure: {max_q:.1f} Pa ({max_q / 1000.0:.2f} kPa)")
    print(f"  - Max Horizontal Drift: {max_drift:.1f} m")
    print(f"  - Max Attitude Tilt   : {max_tilt_deg:.2f}°")
    print(f"  - Max Nozzle Gimbal   : {math.degrees(max_gimbal_used):.2f}° (Limit: {max_gimbal_deg}°)")
    print(f"  - Touchdown Velocity  : {abs(vy):.2f} m/s")
    print("=" * 65)

    if abs(vy) < 5.0 and max_tilt_deg < 30.0:
        print("RESULT: Success! Active TVC kept it stable through wind shear and achieved a soft landing.")
    else:
        print("RESULT: HARD LANDING OR LOSS OF CONTROL. Re-tune PID gains.")
    print("=" * 65)

if __name__ == "__main__":
    run_rock5_simulation()